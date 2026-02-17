import { Type } from "@sinclair/typebox";
import type { OpenClawPluginApi } from "../plugin-api.js";
import { getTransport } from "../service.js";

/**
 * Register the ros2_subscribe_once tool with the AI agent.
 * Subscribes to a topic and returns the next message received.
 */
export function registerSubscribeTool(api: OpenClawPluginApi): void {
  api.registerTool({
    name: "ros2_subscribe_once",
    label: "ROS2 Subscribe Once",
    description:
      "Subscribe to a ROS2 topic and return the next message. Use this to read sensor data, " +
      "check robot state, or get the current value of a topic.",
    parameters: Type.Object({
      topic: Type.String({ description: "The ROS2 topic name (e.g., '/battery_state')" }),
      type: Type.Optional(Type.String({ description: "The ROS2 message type (e.g., 'sensor_msgs/msg/BatteryState')" })),
      timeout: Type.Optional(Type.Number({ description: "Timeout in milliseconds (default: 5000)" })),
    }),

    async execute(_toolCallId, params) {
      const topic = params["topic"] as string;
      const msgType = params["type"] as string | undefined;
      const timeout = (params["timeout"] as number | undefined) ?? 5000;

      const transport = getTransport();

      const result = await new Promise<Record<string, unknown>>((resolve, reject) => {
        const subscription = transport.subscribe(
          { topic, type: msgType },
          (msg: Record<string, unknown>) => {
            clearTimeout(timer);
            subscription.unsubscribe();
            resolve({ success: true, topic, message: msg });
          },
        );
        const timer = setTimeout(() => {
          subscription.unsubscribe();
          reject(new Error(`Timeout waiting for message on ${topic}`));
        }, timeout);
      });

      return {
        content: [{ type: "text", text: JSON.stringify(result) }],
        details: result,
      };
    },
  });
}
