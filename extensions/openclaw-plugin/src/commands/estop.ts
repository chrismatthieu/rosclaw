import type { OpenClawPluginApi } from "../plugin-api.js";
import type { RosClawConfig } from "../config.js";
import { getTransport } from "../service.js";

/**
 * Register the /estop command.
 * This command bypasses the AI agent and immediately sends a zero-velocity
 * command to stop the robot.
 */
export function registerEstopCommand(api: OpenClawPluginApi, config: RosClawConfig): void {
  const namespace = config.robot.namespace;

  api.registerCommand({
    name: "estop",
    description: "Emergency stop — immediately halt the robot (bypasses AI)",

    async handler(_ctx) {
      try {
        const transport = getTransport();
        const topic = namespace ? `${namespace}/cmd_vel` : "/cmd_vel";

        // Send zero velocity
        transport.publish({
          topic,
          type: "geometry_msgs/msg/Twist",
          msg: {
            linear: { x: 0, y: 0, z: 0 },
            angular: { x: 0, y: 0, z: 0 },
          },
        });

        api.logger.warn("ESTOP: Zero velocity command sent");
        return { text: "Emergency stop activated. Robot halted." };
      } catch (error) {
        api.logger.error(`ESTOP FAILED: ${String(error)}`);
        return { text: "Emergency stop failed — transport may be disconnected!" };
      }
    },
  });
}
