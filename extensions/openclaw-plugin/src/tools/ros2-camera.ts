import { Type } from "@sinclair/typebox";
import type { OpenClawPluginApi } from "../plugin-api.js";
import { getTransport } from "../service.js";

/**
 * Register the ros2_camera_snapshot tool with the AI agent.
 * Grabs a single frame from a camera topic.
 */
export function registerCameraTool(api: OpenClawPluginApi): void {
  api.registerTool({
    name: "ros2_camera_snapshot",
    label: "ROS2 Camera Snapshot",
    description:
      "Capture a single image from a ROS2 camera topic. Returns the image as base64-encoded data. " +
      "Use this when the user asks what the robot sees or requests a photo.",
    parameters: Type.Object({
      topic: Type.Optional(Type.String({ description: "The camera image topic (default: '/camera/image_raw/compressed')" })),
      timeout: Type.Optional(Type.Number({ description: "Timeout in milliseconds (default: 10000)" })),
    }),

    async execute(_toolCallId, params) {
      const topic = (params["topic"] as string | undefined) ?? "/camera/image_raw/compressed";
      const timeout = (params["timeout"] as number | undefined) ?? 10000;

      const transport = getTransport();

      const result = await new Promise<Record<string, unknown>>((resolve, reject) => {
        const subscription = transport.subscribe(
          { topic, type: "sensor_msgs/msg/CompressedImage" },
          (msg: Record<string, unknown>) => {
            clearTimeout(timer);
            subscription.unsubscribe();
            resolve({
              success: true,
              topic,
              format: msg["format"] ?? "jpeg",
              data: msg["data"] ?? "",
            });
          },
        );
        const timer = setTimeout(() => {
          subscription.unsubscribe();
          reject(new Error(`Timeout waiting for camera frame on ${topic}`));
        }, timeout);
      });

      return {
        content: [{ type: "text", text: JSON.stringify(result) }],
        details: result,
      };
    },
  });
}
