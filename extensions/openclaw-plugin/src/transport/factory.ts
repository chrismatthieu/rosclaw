import type { TransportConfig } from "./types.js";
import type { RosTransport } from "./transport.js";

/**
 * Create a RosTransport instance for the given deployment mode.
 *
 * Uses dynamic import() to load the correct adapter so that
 * unused adapters (and their dependencies) are never loaded.
 */
export async function createTransport(config: TransportConfig): Promise<RosTransport> {
  switch (config.mode) {
    case "rosbridge": {
      const { RosbridgeTransport } = await import("./rosbridge/adapter.js");
      return new RosbridgeTransport(config.rosbridge);
    }

    case "local": {
      const { LocalTransport } = await import("./local/transport.js");
      return new LocalTransport(config.local);
    }

    case "webrtc": {
      const { WebRTCTransport } = await import("./webrtc/transport.js");
      return new WebRTCTransport(config.webrtc);
    }

    default: {
      const _exhaustive: never = config;
      throw new Error(`Unknown transport mode: ${(_exhaustive as TransportConfig).mode}`);
    }
  }
}
