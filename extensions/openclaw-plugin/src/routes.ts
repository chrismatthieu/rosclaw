import type { OpenClawPluginApi } from "./plugin-api.js";
import type { RosClawConfig } from "./config.js";
import { getLandingPageHtml } from "./landing-page.js";
import { getConfigPageHtml } from "./config-page.js";
import { registerTeleopRoutes } from "./teleop/routes.js";

/**
 * Returns a JSON-serializable copy of config with sensitive fields redacted.
 */
function configForApi(config: RosClawConfig): Record<string, unknown> {
  const out = JSON.parse(JSON.stringify(config)) as Record<string, unknown>;
  const webrtc = out.webrtc as Record<string, unknown> | undefined;
  if (webrtc && typeof webrtc.robotKey === "string" && webrtc.robotKey.length > 0) {
    webrtc.robotKey = "(set)";
  }
  return out;
}

/**
 * Register all RosClaw HTTP routes: landing, config, config API, and teleop.
 * Only call when api.registerHttpRoute is available.
 */
export function registerRoutes(api: OpenClawPluginApi, config: RosClawConfig): void {
  const register = api.registerHttpRoute;
  if (typeof register !== "function") {
    api.logger.info("RosClaw HTTP: registerHttpRoute not available, skipping routes");
    return;
  }

  register({
    path: "/rosclaw/",
    method: "GET",
    handler: (_req, res) => {
      res.setHeader("Content-Type", "text/html; charset=utf-8");
      res.statusCode = 200;
      res.end(getLandingPageHtml());
    },
  });

  register({
    path: "/rosclaw/config",
    method: "GET",
    handler: (_req, res) => {
      res.setHeader("Content-Type", "text/html; charset=utf-8");
      res.statusCode = 200;
      res.end(getConfigPageHtml());
    },
  });

  register({
    path: "/rosclaw/config.json",
    method: "GET",
    handler: (_req, res) => {
      const payload = configForApi(config);
      res.setHeader("Content-Type", "application/json");
      res.statusCode = 200;
      res.end(JSON.stringify(payload));
    },
  });

  registerTeleopRoutes(api, config);

  api.logger.info("RosClaw HTTP routes registered (GET /rosclaw/, /config, /config.json; teleop routes)");
}
