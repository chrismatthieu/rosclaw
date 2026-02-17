import type { OpenClawPluginApi } from "../plugin-api.js";

interface SafetyConfig {
  maxLinearVelocity: number;
  maxAngularVelocity: number;
  workspaceLimits: {
    xMin: number;
    xMax: number;
    yMin: number;
    yMax: number;
  };
}

const DEFAULT_SAFETY: SafetyConfig = {
  maxLinearVelocity: 1.0,
  maxAngularVelocity: 1.5,
  workspaceLimits: { xMin: -10, xMax: 10, yMin: -10, yMax: 10 },
};

/**
 * Register the before_tool_call safety validation hook.
 * Intercepts tool calls and validates them against safety limits.
 */
export function registerSafetyHook(api: OpenClawPluginApi): void {
  const safety: SafetyConfig = {
    ...DEFAULT_SAFETY,
    ...(api.pluginConfig?.["safety"] as Partial<SafetyConfig> ?? {}),
  };

  api.on("before_tool_call", async (event, _ctx) => {
    if (event.toolName === "ros2_publish") {
      const msg = event.params["message"] as Record<string, unknown> | undefined;
      if (msg) {
        // Check velocity limits for Twist messages
        const linear = msg["linear"] as Record<string, number> | undefined;
        const angular = msg["angular"] as Record<string, number> | undefined;

        if (linear) {
          const speed = Math.sqrt(
            (linear["x"] ?? 0) ** 2 +
            (linear["y"] ?? 0) ** 2 +
            (linear["z"] ?? 0) ** 2,
          );
          if (speed > safety.maxLinearVelocity) {
            api.logger.warn(`Blocked: linear velocity ${speed} exceeds limit ${safety.maxLinearVelocity}`);
            return { block: true, blockReason: `Linear velocity ${speed.toFixed(2)} m/s exceeds safety limit of ${safety.maxLinearVelocity} m/s` };
          }
        }

        if (angular) {
          const rate = Math.abs(angular["z"] ?? 0);
          if (rate > safety.maxAngularVelocity) {
            api.logger.warn(`Blocked: angular velocity ${rate} exceeds limit ${safety.maxAngularVelocity}`);
            return { block: true, blockReason: `Angular velocity ${rate.toFixed(2)} rad/s exceeds safety limit of ${safety.maxAngularVelocity} rad/s` };
          }
        }
      }
    }

    // TODO: Add workspace limit checks for navigation goals
  });
}
