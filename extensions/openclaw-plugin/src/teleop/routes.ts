import type { OpenClawPluginApi, HttpRouteRequest, HttpRouteResponse } from "../plugin-api.js";
import type { RosClawConfig } from "../config.js";
import { getTransport } from "../service.js";
import { toNamespacedTopic } from "../topic-utils.js";
import { getTeleopPageHtml } from "./page.js";

const COMPRESSED_IMAGE_TYPE = "sensor_msgs/msg/CompressedImage";
const IMAGE_TYPE = "sensor_msgs/msg/Image";
const TWIST_TYPE = "geometry_msgs/msg/Twist";

/** Image/CompressedImage type names (for filtering topics). */
const IMAGE_TYPE_PATTERN = /Image|CompressedImage/i;
/** When type is unknown (e.g. Zenoh), treat topic as camera if name looks like one. */
const CAMERA_TOPIC_NAME_PATTERN = /camera|image|compressed/i;

function parseUrl(req: HttpRouteRequest): { pathname: string; searchParams: URLSearchParams } {
  const base = "http://localhost";
  const url = new URL(req.url ?? "", base);
  return { pathname: url.pathname, searchParams: url.searchParams };
}

function readRequestBody(stream: NodeJS.ReadableStream): Promise<string> {
  return new Promise((resolve, reject) => {
    const chunks: Buffer[] = [];
    stream.on("data", (chunk: Buffer | string) => {
      chunks.push(Buffer.isBuffer(chunk) ? chunk : Buffer.from(chunk));
    });
    stream.on("end", () => resolve(Buffer.concat(chunks).toString("utf8")));
    stream.on("error", reject);
  });
}

function getDefaultCameraTopic(config: RosClawConfig): string {
  const t = (config.teleop?.cameraTopic ?? "").trim();
  if (t) return t;
  const r = (config.robot?.cameraTopic ?? "").trim();
  if (r) return r;
  return "/camera/camera/color/image_raw/compressed";
}

function getCmdVelTopic(config: RosClawConfig): string {
  const t = (config.teleop?.cmdVelTopic ?? "").trim();
  if (t) return t;
  return toNamespacedTopic(config, "/cmd_vel");
}

function clampTwist(
  config: RosClawConfig,
  linearX: number,
  linearY: number,
  linearZ: number,
  angularX: number,
  angularY: number,
  angularZ: number,
): { linear: { x: number; y: number; z: number }; angular: { x: number; y: number; z: number } } {
  const maxLin = config.safety?.maxLinearVelocity ?? 1.0;
  const maxAng = config.safety?.maxAngularVelocity ?? 1.5;

  const linMag = Math.sqrt(linearX * linearX + linearY * linearY + linearZ * linearZ);
  const scaleLin = linMag > maxLin && linMag > 0 ? maxLin / linMag : 1;
  const angMag = Math.abs(angularZ);
  const scaleAng = angMag > maxAng && angMag > 0 ? maxAng / angMag : 1;

  return {
    linear: {
      x: linearX * scaleLin,
      y: linearY * scaleLin,
      z: linearZ * scaleLin,
    },
    angular: {
      x: angularX * scaleAng,
      y: angularY * scaleAng,
      z: Math.max(-maxAng, Math.min(maxAng, angularZ)),
    },
  };
}

function imageDataToBuffer(data: unknown): Buffer | null {
  if (data == null) return null;
  if (data instanceof Uint8Array) return Buffer.from(data);
  if (Buffer.isBuffer(data)) return data;
  if (Array.isArray(data)) {
    const bytes = new Uint8Array(data.length);
    for (let i = 0; i < data.length; i++) bytes[i] = Number(data[i]) & 0xff;
    return Buffer.from(bytes);
  }
  if (typeof data === "string") return Buffer.from(data, "base64");
  return null;
}

/**
 * Register Phase 3 teleop HTTP routes (sources, camera, twist, index page).
 * Only registers when api.registerHttpRoute is available.
 */
export function registerTeleopRoutes(api: OpenClawPluginApi, config: RosClawConfig): void {
  const register = api.registerHttpRoute;
  if (typeof register !== "function") {
    api.logger.info("RosClaw teleop: registerHttpRoute not available, skipping routes");
    return;
  }

  // GET /rosclaw/teleop/ping — diagnostic: confirms plugin HTTP routes are mounted
  register({
    path: "/rosclaw/teleop/ping",
    method: "GET",
    handler: (_req, res) => {
      res.setHeader("Content-Type", "application/json");
      res.statusCode = 200;
      res.end(JSON.stringify({ ok: true, rosclaw: "teleop" }));
    },
  });

  // GET /rosclaw/teleop/sources — JSON list of camera topics
  register({
    path: "/rosclaw/teleop/sources",
    method: "GET",
    handler: async (_req, res) => {
      try {
        let list: Array<{ topic: string; label?: string }>;
        const explicit = config.teleop?.cameraTopics ?? [];
        if (explicit.length > 0) {
          list = explicit.map((o) => ({ topic: o.topic, label: o.label }));
        } else {
          const transport = getTransport();
          const topics = await transport.listTopics();
          const imageTopics = topics.filter((t) => {
            if (t.type && IMAGE_TYPE_PATTERN.test(t.type)) return true;
            if (!t.type || t.type === "unknown") {
              return CAMERA_TOPIC_NAME_PATTERN.test(t.name);
            }
            return false;
          });
          imageTopics.sort((a, b) => {
            const aCompressed = /compressed/i.test(a.name) ? 1 : 0;
            const bCompressed = /compressed/i.test(b.name) ? 1 : 0;
            return bCompressed - aCompressed;
          });
          list = imageTopics.map((t) => ({
            topic: t.name,
            label: t.name.replace(/^\//, "").replace(/\//g, " / "),
          }));
        }
        res.setHeader("Content-Type", "application/json");
        res.statusCode = 200;
        res.end(JSON.stringify(list));
      } catch (e) {
        api.logger.warn("Teleop sources error: " + (e instanceof Error ? e.message : String(e)));
        res.setHeader("Content-Type", "application/json");
        res.statusCode = 500;
        res.end(JSON.stringify({ error: "Failed to list camera sources" }));
      }
    },
  });

  // GET /rosclaw/teleop/camera?topic=...&type=compressed|image
  register({
    path: "/rosclaw/teleop/camera",
    method: "GET",
    handler: async (req, res) => {
      const { searchParams } = parseUrl(req);
      let topic = searchParams.get("topic")?.trim();
      if (!topic) topic = getDefaultCameraTopic(config);
      const typeParam = (searchParams.get("type") ?? "compressed").toLowerCase();
      const useImage = typeParam === "image";
      const type = useImage ? IMAGE_TYPE : COMPRESSED_IMAGE_TYPE;
      const resolvedTopic = toNamespacedTopic(config, topic);

      try {
        const transport = getTransport();
        await new Promise((r) => setTimeout(r, 350));
        const result = await new Promise<Record<string, unknown>>((resolve, reject) => {
          const timeout = 4000;
          const sub = transport.subscribe(
            { topic: resolvedTopic, type },
            (msg: Record<string, unknown>) => {
              sub.unsubscribe();
              resolve(msg);
            },
          );
          setTimeout(() => {
            sub.unsubscribe();
            reject(new Error("timeout"));
          }, timeout);
        });

        // CompressedImage: data is raw JPEG/PNG bytes — return as-is
        if (!useImage) {
          const data = result["data"];
          const buf = imageDataToBuffer(data);
          if (!buf || buf.length === 0) {
            res.statusCode = 204;
            res.end();
            return;
          }
          const format = String(result["format"] ?? "jpeg").toLowerCase();
          const mime = format === "png" ? "image/png" : "image/jpeg";
          res.setHeader("Content-Type", mime);
          res.statusCode = 200;
          res.end(buf);
          return;
        }

        // Image (raw): we don't have a JPEG encoder; return 501 and suggest CompressedImage
        res.setHeader("Content-Type", "application/json");
        res.statusCode = 501;
        res.end(
          JSON.stringify({
            error: "Raw Image topics not supported for teleop camera; use a CompressedImage topic (e.g. .../image_raw/compressed)",
          }),
        );
      } catch (e) {
        res.setHeader("Content-Type", "application/json");
        res.statusCode = 500;
        res.end(
          JSON.stringify({
            error: e instanceof Error ? e.message : "Failed to get camera frame",
          }),
        );
      }
    },
  });

  // POST /rosclaw/teleop/twist
  register({
    path: "/rosclaw/teleop/twist",
    method: "POST",
    handler: async (req, res) => {
      let body: Record<string, unknown> = {};
      try {
        if (typeof req.readJsonBody === "function") {
          body = (await req.readJsonBody()) ?? {};
        }
        if (Object.keys(body).length === 0) {
          const raw = (req as { body?: unknown }).body;
          if (typeof raw === "object" && raw !== null) {
            body = raw as Record<string, unknown>;
          } else if (raw && typeof (raw as Promise<unknown>).then === "function") {
            const parsed = await (raw as Promise<Record<string, unknown>>);
            if (parsed && typeof parsed === "object") body = parsed;
          }
        }
        if (Object.keys(body).length === 0 && typeof (req as { on?: (e: string, cb: (c?: Buffer) => void) => void }).on === "function") {
          const rawBody = await readRequestBody(req as unknown as NodeJS.ReadableStream);
          if (rawBody.trim()) {
            try {
              body = JSON.parse(rawBody) as Record<string, unknown>;
            } catch {
              // leave body empty
            }
          }
        }
      } catch {
        res.setHeader("Content-Type", "application/json");
        res.statusCode = 400;
        res.end(JSON.stringify({ error: "Invalid JSON body" }));
        return;
      }

      const lx = Number(body.linear_x ?? (body as Record<string, unknown>).linearX ?? 0);
      const ly = Number(body.linear_y ?? (body as Record<string, unknown>).linearY ?? 0);
      const lz = Number(body.linear_z ?? (body as Record<string, unknown>).linearZ ?? 0);
      const ax = Number(body.angular_x ?? (body as Record<string, unknown>).angularX ?? 0);
      const ay = Number(body.angular_y ?? (body as Record<string, unknown>).angularY ?? 0);
      const az = Number(body.angular_z ?? (body as Record<string, unknown>).angularZ ?? 0);

      const clamped = clampTwist(config, lx, ly, lz, ax, ay, az);
      const topic = getCmdVelTopic(config);

      try {
        const transport = getTransport();
        transport.publish({
          topic,
          type: TWIST_TYPE,
          msg: {
            linear: clamped.linear,
            angular: clamped.angular,
          },
        });
        res.setHeader("Content-Type", "application/json");
        res.statusCode = 200;
        res.end(JSON.stringify({ ok: true, topic }));
      } catch (e) {
        api.logger.warn("Teleop twist error: " + (e instanceof Error ? e.message : String(e)));
        res.setHeader("Content-Type", "application/json");
        res.statusCode = 500;
        res.end(JSON.stringify({ error: "Failed to publish twist" }));
      }
    },
  });

  // GET /rosclaw/teleop/ and /rosclaw/teleop/index.html
  const servePage = (_req: HttpRouteRequest, res: HttpRouteResponse) => {
    const html = getTeleopPageHtml(config);
    res.setHeader("Content-Type", "text/html; charset=utf-8");
    res.statusCode = 200;
    res.end(html);
  };

  register({ path: "/rosclaw/teleop/", method: "GET", handler: servePage });
  register({ path: "/rosclaw/teleop/index.html", method: "GET", handler: servePage });

  api.logger.info("RosClaw teleop routes registered (GET /rosclaw/teleop/, /ping, /sources, /camera; POST /twist)");
}
