import type { TransportConfig } from "./transport/types.js";
import type { RosTransport } from "./transport/transport.js";
import { createTransport } from "./transport/factory.js";
import type { OpenClawPluginApi } from "./plugin-api.js";

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
 *
 * Reads transport config from `pluginConfig` to determine which adapter to use:
 * - "rosbridge" (default) — WebSocket to rosbridge_server (Mode B)
 * - "local" — direct DDS on same machine (Mode A)
 * - "webrtc" — WebRTC data channel via signaling server (Mode C)
 */
export function registerService(api: OpenClawPluginApi): void {
  const transportCfg = api.pluginConfig?.["transport"] as { mode?: string } | undefined;
  const mode = transportCfg?.mode ?? "rosbridge";

  api.registerService({
    id: "ros2-transport",

    async start(_ctx) {
      const config = buildTransportConfig(api, mode);
      api.logger.info(`Connecting to ROS2 via ${mode} transport...`);

      transport = await createTransport(config);

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

function buildTransportConfig(api: OpenClawPluginApi, mode: string): TransportConfig {
  switch (mode) {
    case "rosbridge": {
      const rb = api.pluginConfig?.["rosbridge"] as {
        url?: string;
        reconnect?: boolean;
        reconnectInterval?: number;
      } | undefined;
      return {
        mode: "rosbridge",
        rosbridge: {
          url: rb?.url ?? "ws://localhost:9090",
          reconnect: rb?.reconnect ?? true,
          reconnectInterval: rb?.reconnectInterval ?? 3000,
        },
      };
    }

    case "local": {
      const local = api.pluginConfig?.["local"] as { domainId?: number } | undefined;
      return {
        mode: "local",
        local: {
          domainId: local?.domainId ?? 0,
        },
      };
    }

    case "webrtc": {
      const wrtc = api.pluginConfig?.["webrtc"] as {
        signalingUrl?: string;
        apiUrl?: string;
        robotId?: string;
        robotKey?: string;
        iceServers?: Array<{ urls: string | string[]; username?: string; credential?: string }>;
      } | undefined;
      return {
        mode: "webrtc",
        webrtc: {
          signalingUrl: wrtc?.signalingUrl ?? "",
          apiUrl: wrtc?.apiUrl ?? "",
          robotId: wrtc?.robotId ?? "",
          robotKey: wrtc?.robotKey ?? "",
          iceServers: wrtc?.iceServers ?? [
            { urls: "stun:stun.l.google.com:19302" },
          ],
        },
      };
    }

    default:
      throw new Error(`Unknown transport mode: ${mode}`);
  }
}
