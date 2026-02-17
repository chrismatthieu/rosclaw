import { Type } from "@sinclair/typebox";
import type { OpenClawPluginApi } from "../plugin-api.js";
import { getTransport } from "../service.js";

/**
 * Register the ros2_list_topics tool with the AI agent.
 * Allows the agent to discover available ROS2 topics at runtime.
 */
export function registerIntrospectTool(api: OpenClawPluginApi): void {
  api.registerTool({
    name: "ros2_list_topics",
    label: "ROS2 List Topics",
    description:
      "List all available ROS2 topics and their message types. " +
      "Use this to discover what data the robot publishes and what commands it accepts.",
    parameters: Type.Object({}),

    async execute(_toolCallId, _params) {
      const transport = getTransport();
      const topics = await transport.listTopics();

      const result = { success: true, topics };
      return {
        content: [{ type: "text", text: JSON.stringify(result) }],
        details: result,
      };
    },
  });
}
