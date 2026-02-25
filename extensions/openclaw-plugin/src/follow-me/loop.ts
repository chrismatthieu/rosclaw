/**
 * Native Follow Me loop: camera (ROS2) → Qwen VLM (Ollama) → cmd_vel (ROS2).
 * No external apps; runs inside the plugin.
 */

import type { RosTransport } from "../transport/transport.js";
import type { RosClawConfig } from "../config.js";
import type { PluginLogger } from "../plugin-api.js";
import { getDepthDistance } from "../depth.js";

const COMPRESSED_IMAGE_TYPE = "sensor_msgs/msg/CompressedImage";
const IMAGE_TYPE = "sensor_msgs/msg/Image";
const TWIST_TYPE = "geometry_msgs/msg/Twist";
const SNAPSHOT_TIMEOUT_MS = 5000;
const OLLAMA_TIMEOUT_MS = 15000;
const REALSENSE_DEPTH_TOPIC = "/camera/camera/depth/image_rect_raw";
const DEPTH_TIMEOUT_MS = 2000;
const DEPTH_TIMEOUT_MS_DEFAULT_TOPIC = 800;

function imageDataToBase64(data: unknown): string {
  if (typeof data === "string") return data;
  if (Array.isArray(data)) {
    const bytes = new Uint8Array(data.length);
    for (let i = 0; i < data.length; i++) bytes[i] = Number(data[i]) & 0xff;
    return Buffer.from(bytes).toString("base64");
  }
  throw new Error("Image data must be string (base64) or array of bytes");
}

export interface VlmDetection {
  person_visible: boolean;
  position?: "left" | "center" | "right";
  distance_hint?: "close" | "medium" | "far";
}

const VLM_PROMPT = `Look at this image from a robot's front-facing camera. Is there a person visible (even partially, or from the side)?
Reply with ONLY a JSON object, no other text. Use this exact format:
{"person_visible": true or false, "position": "left" or "center" or "right" (only if person_visible), "distance_hint": "close" or "medium" or "far" (only if person_visible)}
If you see a person at all, set person_visible true. If no person is visible, use: {"person_visible": false}`;

async function getCameraSnapshot(
  transport: RosTransport,
  topic: string,
  messageType: "CompressedImage" | "Image",
): Promise<string> {
  const type = messageType === "Image" ? IMAGE_TYPE : COMPRESSED_IMAGE_TYPE;
  const result = await new Promise<Record<string, unknown>>((resolve, reject) => {
    const subscription = transport.subscribe(
      { topic, type },
      (msg: Record<string, unknown>) => {
        clearTimeout(timer);
        subscription.unsubscribe();
        if (messageType === "Image") {
          const data = msg["data"];
          resolve({ data: imageDataToBase64(data), format: msg["encoding"] ?? "rgb8" });
        } else {
          const data = msg["data"];
          const b64 = typeof data === "string" ? data : imageDataToBase64(data);
          resolve({ data: b64, format: (msg["format"] as string) ?? "jpeg" });
        }
      },
    );
    const timer = setTimeout(() => {
      subscription.unsubscribe();
      reject(new Error(`Camera snapshot timeout on ${topic}`));
    }, SNAPSHOT_TIMEOUT_MS);
  });
  return (result.data as string) ?? "";
}

async function callOllamaVision(
  ollamaUrl: string,
  model: string,
  imageBase64: string,
  prompt: string,
  timeoutMs = OLLAMA_TIMEOUT_MS,
): Promise<string> {
  const url = `${ollamaUrl.replace(/\/$/, "")}/api/generate`;
  const controller = new AbortController();
  const timeoutId = setTimeout(() => controller.abort(), timeoutMs);
  try {
    const res = await fetch(url, {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify({
        model,
        prompt,
        stream: false,
        images: imageBase64 ? [imageBase64] : undefined,
      }),
      signal: controller.signal,
    });
    if (!res.ok) {
      const t = await res.text();
      throw new Error(`Ollama error ${res.status}: ${t.slice(0, 200)}`);
    }
    const json = (await res.json()) as { response?: string };
    return (json.response ?? "").trim();
  } catch (err) {
    if (err instanceof Error && err.name === "AbortError") {
      throw new Error(`Ollama timeout (${timeoutMs}ms). Is the model loaded? Try: ollama run ${model}`);
    }
    throw err;
  } finally {
    clearTimeout(timeoutId);
  }
}

function parseVlmResponse(text: string): VlmDetection {
  const def: VlmDetection = { person_visible: false };
  // Ollama may wrap in ```json ... ``` or return extra text; take last JSON object
  const stripped = text.replace(/```\w*\n?/g, "").trim();
  const match = stripped.match(/\{[\s\S]*\}/);
  if (!match) return def;
  try {
    const o = JSON.parse(match[0]) as Record<string, unknown>;
    def.person_visible = Boolean(o.person_visible ?? o.personVisible ?? false);
    const pos = (o.position as string)?.toLowerCase?.();
    if (pos && ["left", "center", "right"].includes(pos)) {
      def.position = pos as "left" | "center" | "right";
    }
    const hint = (o.distance_hint ?? o.distanceHint) as string | undefined;
    const hintLower = typeof hint === "string" ? hint.toLowerCase() : "";
    if (hintLower && ["close", "medium", "far"].includes(hintLower)) {
      def.distance_hint = hintLower as "close" | "medium" | "far";
    }
  } catch {
    // ignore
  }
  return def;
}

const DEPTH_DEADZONE_M = 0.2;

function computeTwist(
  det: VlmDetection,
  targetDistance: number,
  safety: RosClawConfig["safety"],
  minLinearVelocity: number,
  depthM?: number,
): { linear_x: number; angular_z: number } {
  const maxLin = safety?.maxLinearVelocity ?? 1.0;
  const maxAng = safety?.maxAngularVelocity ?? 1.5;
  const minLin = Math.min(minLinearVelocity, maxLin);

  let linear_x = 0;
  let angular_z = 0;

  // When we have valid depth, use it for approach/retreat regardless of VLM confidence
  if (depthM != null && Number.isFinite(depthM) && depthM > 0) {
    const error = depthM - targetDistance;
    if (error > DEPTH_DEADZONE_M) linear_x = minLin;
    else if (error < -DEPTH_DEADZONE_M) linear_x = -minLin;
  }

  // Angular (and VLM-based linear when no depth): require person_visible
  if (!det.person_visible) {
    if (linear_x === 0) angular_z = Math.min(0.3, maxAng); // search spin when no depth command
    // else keep linear_x from depth, angular stays 0
  } else {
    switch (det.position) {
      case "left":
        angular_z = 0.5;
        break;
      case "right":
        angular_z = -0.5;
        break;
      case "center":
      default:
        // Linear from depth already set above when depth valid; otherwise from VLM distance_hint
        if (linear_x === 0 && (depthM == null || !Number.isFinite(depthM) || depthM <= 0)) {
          switch (det.distance_hint) {
            case "close":
              linear_x = -minLin;
              break;
            case "far":
              linear_x = minLin;
              break;
            case "medium":
            default:
              linear_x = minLin;
              break;
          }
        }
        break;
    }
  }

  // Enforce minimum magnitude so the base actually moves (many need ~0.3 m/s)
  if (linear_x > 0) linear_x = Math.max(minLin, Math.min(maxLin, linear_x));
  else if (linear_x < 0) linear_x = Math.min(-minLin, Math.max(-maxLin, linear_x));
  angular_z = Math.max(-maxAng, Math.min(maxAng, angular_z));
  return { linear_x, angular_z };
}

function buildCmdVelTopic(config: RosClawConfig): string {
  const override = (config.followMe?.cmdVelTopic ?? "").trim();
  if (override) {
    return override.startsWith("/") ? override : `/${override}`;
  }
  const ns = config.robot?.namespace?.trim() ?? "";
  return ns ? `/${ns}/cmd_vel` : "/cmd_vel";
}

/** Return the cmd_vel topic used by Follow Me (for tool response / debugging). */
export function getFollowMeCmdVelTopic(config: RosClawConfig): string {
  return buildCmdVelTopic(config);
}

let loopIntervalId: ReturnType<typeof setInterval> | null = null;
let tickInProgress = false;
let lastError: string | null = null;
let lastDetection: VlmDetection | null = null;
let lastTwist: { linear_x: number; angular_z: number } | null = null;
let tickCount = 0;

export function isFollowMeRunning(): boolean {
  return loopIntervalId != null;
}

/** Last error message, detection, and twist (for status/debugging). */
export function getLastFollowMeState(): {
  last_error: string | null;
  last_detection: VlmDetection | null;
  last_twist: { linear_x: number; angular_z: number } | null;
  tick_count: number;
} {
  return {
    last_error: lastError,
    last_detection: lastDetection,
    last_twist: lastTwist,
    tick_count: tickCount,
  };
}

export function stopFollowMeLoop(
  transport: RosTransport,
  config: RosClawConfig,
): void {
  if (loopIntervalId != null) {
    clearInterval(loopIntervalId);
    loopIntervalId = null;
  }
  const topic = buildCmdVelTopic(config);
  transport.publish({
    topic,
    type: TWIST_TYPE,
    msg: {
      linear: { x: 0, y: 0, z: 0 },
      angular: { x: 0, y: 0, z: 0 },
    },
  });
}

export function startFollowMeLoop(
  transport: RosTransport,
  config: RosClawConfig,
  logger: PluginLogger,
): string {
  if (loopIntervalId != null) {
    return buildCmdVelTopic(config);
  }

  const fm = config.followMe ?? {};
  const ollamaUrl = fm.ollamaUrl ?? "http://localhost:11434";
  const vlmModel = fm.vlmModel ?? "qwen2-vl:7b";
  const cameraTopic = fm.cameraTopic ?? "/camera/image_raw/compressed";
  const cameraMessageType = fm.cameraMessageType ?? "CompressedImage";
  const targetDistance = fm.targetDistance ?? 0.5;
  const rateHz = fm.rateHz ?? 5;
  const minLinearVelocity = fm.minLinearVelocity ?? 0.3;
  const depthTopicConfig = (fm.depthTopic ?? "").trim();
  const depthTopic = depthTopicConfig || REALSENSE_DEPTH_TOPIC;
  const depthTimeoutMs = depthTopicConfig ? DEPTH_TIMEOUT_MS : DEPTH_TIMEOUT_MS_DEFAULT_TOPIC;
  const topic = buildCmdVelTopic(config);

  const intervalMs = Math.round(1000 / rateHz);
  lastError = null;
  lastDetection = null;
  lastTwist = null;
  tickCount = 0;

  // Advertise the topic first so rosbridge creates the publisher with the correct type
  if (typeof transport.advertise === "function") {
    transport.advertise({ topic, type: TWIST_TYPE });
  }

  loopIntervalId = setInterval(async () => {
    if (tickInProgress) return;
    tickInProgress = true;
    try {
      lastError = null;
      // Fetch RGB and depth in parallel (depth topic defaults to RealSense; short timeout if not configured)
      const [base64, depthResult] = await Promise.all([
        getCameraSnapshot(transport, cameraTopic, cameraMessageType),
        getDepthDistance(transport, depthTopic, depthTimeoutMs).catch(() => null),
      ]);
      const responseText = await callOllamaVision(ollamaUrl, vlmModel, base64, VLM_PROMPT);
      const det = parseVlmResponse(responseText);
      lastDetection = det;
      const depthM = depthResult?.valid === true ? depthResult.distance_m : undefined;
      const twist = computeTwist(det, targetDistance, config.safety ?? {}, minLinearVelocity, depthM);
      lastTwist = twist;
      tickCount += 1;
      const { linear_x, angular_z } = twist;

      if (tickCount <= 3 || tickCount % 20 === 0) {
        logger.info(`Follow Me: det=${JSON.stringify(det)} twist=(${linear_x.toFixed(2)}, ${angular_z.toFixed(2)})`);
      }

      transport.publish({
        topic,
        type: TWIST_TYPE,
        msg: {
          linear: { x: linear_x, y: 0, z: 0 },
          angular: { x: 0, y: 0, z: angular_z },
        },
      });
    } catch (err) {
      const msg = err instanceof Error ? err.message : String(err);
      lastError = msg;
      logger.warn(`Follow Me tick error: ${msg}`);
      transport.publish({
        topic,
        type: TWIST_TYPE,
        msg: { linear: { x: 0, y: 0, z: 0 }, angular: { x: 0, y: 0, z: 0 } },
      });
    } finally {
      tickInProgress = false;
    }
  }, intervalMs);

  logger.info(`Follow Me started: cmd_vel=${topic}, camera=${cameraTopic}, depth=${depthTopic || "none"}, VLM=${vlmModel}, target=${targetDistance}m, minLin=${minLinearVelocity}m/s, ${rateHz}Hz`);
  return topic;
}
