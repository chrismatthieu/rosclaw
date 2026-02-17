import type { TransportConfig } from "./transport/types.js";
import type { RosTransport } from "./transport/transport.js";
import { createTransport } from "./transport/factory.js";
import type { OpenClawPluginApi } from "./plugin-api.js";
import type { RosClawConfig } from "./config.js";

/** Shared transport instance for all tools. */
let transport: RosTransport | null = null;

/** Get the active transport. Throws if not connected. */
export function getTransport(): RosTransport {
  if (!transport) {
    throw new Error("Transport not initialized. Is the service running?");
  }
  return transport;
}

/**
 * Register the ROS2 transport connection as an OpenClaw managed service.
 * The service handles connection lifecycle (connect on start, disconnect on stop).
 */
export function registerService(api: OpenClawPluginApi, config: RosClawConfig): void {
  const mode = config.transport.mode;

  api.registerService({
    id: "ros2-transport",

    async start(_ctx) {
      let transportCfg: TransportConfig;

      switch (mode) {
        case "rosbridge":
          transportCfg = { mode: "rosbridge", rosbridge: config.rosbridge };
          break;
        case "local":
          transportCfg = { mode: "local", local: config.local };
          break;
        case "webrtc":
          transportCfg = { mode: "webrtc", webrtc: config.webrtc };
          break;
      }

      api.logger.info(`Connecting to ROS2 via ${mode} transport...`);

      transport = await createTransport(transportCfg);

      transport.onConnection((status: string) => {
        api.logger.info(`ROS2 transport status: ${status}`);
      });

      await transport.connect();
      api.logger.info(`ROS2 transport connected (mode: ${mode})`);
    },

    async stop(_ctx) {
      if (transport) {
        await transport.disconnect();
        transport = null;
        api.logger.info("ROS2 transport disconnected");
      }
    },
  });
}
