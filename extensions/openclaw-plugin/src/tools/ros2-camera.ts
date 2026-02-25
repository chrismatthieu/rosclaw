import { Type } from "@sinclair/typebox";
import type { OpenClawPluginApi } from "../plugin-api.js";
import { getTransport } from "../service.js";

/** Known camera topic patterns for common setups (e.g. RealSense). */
export const REALSENSE_CAMERA_TOPICS = {
  color_compressed: "/camera/camera/color/image_raw/compressed",
  color_raw: "/camera/camera/color/image_raw",
  depth_raw: "/camera/camera/depth/image_rect_raw",
  aligned_depth: "/camera/camera/aligned_depth_to_color/image_raw",
} as const;

const COMPRESSED_IMAGE_TYPE = "sensor_msgs/msg/CompressedImage";
const IMAGE_TYPE = "sensor_msgs/msg/Image";

/** Convert sensor_msgs/Image data (byte array) to base64. */
function imageDataToBase64(data: unknown): string {
  if (typeof data === "string") return data;
  if (Array.isArray(data)) {
    const bytes = new Uint8Array(data.length);
    for (let i = 0; i < data.length; i++) bytes[i] = Number(data[i]) & 0xff;
    return Buffer.from(bytes).toString("base64");
  }
  throw new Error("Image data must be string (base64) or array of bytes");
}

/**
 * Register the ros2_camera_snapshot tool with the AI agent.
 * Grabs a single frame from a ROS2 camera topic.
 * Supports CompressedImage (default) and raw Image (e.g. RealSense color/depth).
 */
export function registerCameraTool(api: OpenClawPluginApi): void {
  api.registerTool({
    name: "ros2_camera_snapshot",
    label: "ROS2 Camera Snapshot",
    description:
      "Capture a single image from a ROS2 camera topic. Returns the image as base64-encoded data. " +
      "Use this when the user asks what the robot sees or requests a photo. " +
      "Supports sensor_msgs/CompressedImage (e.g. /camera/image_raw/compressed) and sensor_msgs/Image (e.g. RealSense: /camera/camera/color/image_raw or /camera/camera/depth/image_rect_raw).",
    parameters: Type.Object({
      topic: Type.Optional(
        Type.String({
          description:
            "Camera image topic. Examples: '/camera/image_raw/compressed', RealSense color '/camera/camera/color/image_raw', RealSense depth '/camera/camera/depth/image_rect_raw'. Default: '/camera/image_raw/compressed'.",
        }),
      ),
      message_type: Type.Optional(
        Type.Union([
          Type.Literal("CompressedImage"),
          Type.Literal("Image"),
        ], {
          description:
            "Message type: 'CompressedImage' for JPEG/PNG topics (default), 'Image' for raw sensor_msgs/Image (e.g. RealSense color/depth).",
        }),
      ),
      timeout: Type.Optional(Type.Number({ description: "Timeout in milliseconds (default: 10000)" })),
    }),

    async execute(_toolCallId, params) {
      const topic = (params["topic"] as string | undefined) ?? "/camera/image_raw/compressed";
      const messageType = (params["message_type"] as "CompressedImage" | "Image" | undefined) ?? "CompressedImage";
      const timeout = (params["timeout"] as number | undefined) ?? 10000;

      const transport = getTransport();
      const type = messageType === "Image" ? IMAGE_TYPE : COMPRESSED_IMAGE_TYPE;

      const result = await new Promise<Record<string, unknown>>((resolve, reject) => {
        const subscription = transport.subscribe(
          { topic, type },
          (msg: Record<string, unknown>) => {
            clearTimeout(timer);
            subscription.unsubscribe();
            if (messageType === "Image") {
              const data = msg["data"];
              const encoding = (msg["encoding"] as string) ?? "rgb8";
              resolve({
                success: true,
                topic,
                format: encoding,
                data: imageDataToBase64(data),
                width: msg["width"],
                height: msg["height"],
              });
            } else {
              resolve({
                success: true,
                topic,
                format: msg["format"] ?? "jpeg",
                data: msg["data"] ?? "",
              });
            }
          },
        );
        const timer = setTimeout(() => {
          subscription.unsubscribe();
          reject(new Error(`Timeout waiting for camera frame on ${topic}`));
        }, timeout);
      });

      // Return image as proper image content to avoid sending huge base64 in text (rate limits / token burn).
      const base64 = (result.data as string) ?? "";
      const format = (result.format as string) ?? "jpeg";
      const mimeType = format === "png" ? "image/png" : "image/jpeg";
      const summary = `Captured one frame from ${topic}${result.width != null ? ` (${result.width}×${result.height})` : ""}.`;

      const content: Array<{ type: "text"; text: string } | { type: "image"; data: string; mimeType: string }> = [
        { type: "text", text: summary },
      ];
      if (base64) content.push({ type: "image", data: base64, mimeType });

      return {
        content,
        details: { success: result.success, topic, width: result.width, height: result.height },
      };
    },
  });
}
