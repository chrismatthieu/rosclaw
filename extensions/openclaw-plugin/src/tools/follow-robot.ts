/**
 * Follow Me tool — native OpenClaw + RosClaw implementation.
 * Uses camera (ROS2) + Qwen VLM (Ollama) + cmd_vel (ROS2). No external apps.
 */

import { Type } from "@sinclair/typebox";
import type { OpenClawPluginApi } from "../plugin-api.js";
import type { RosClawConfig } from "../config.js";
import { getTransport } from "../service.js";
import {
  startFollowMeLoop,
  stopFollowMeLoop,
  isFollowMeRunning,
  getFollowMeCmdVelTopic,
  getLastFollowMeState,
} from "../follow-me/loop.js";

export function registerFollowRobotTool(
  api: OpenClawPluginApi,
  config: RosClawConfig,
): void {
  api.registerTool({
    name: "follow_robot",
    label: "Follow Me",
    description:
      "Start or stop the native Follow Me mission. Uses the robot's camera (2D or RealSense) and a Qwen VLM (Ollama) to find and follow a person, keeping about 1 m behind. No external apps—runs inside RosClaw. " +
      "Use when the user says 'follow me', 'start following', 'stop following'. Actions: start, stop, status.",

    parameters: Type.Object({
      action: Type.Union(
        [
          Type.Literal("start"),
          Type.Literal("stop"),
          Type.Literal("status"),
        ],
        { description: "Action: start, stop, status" },
      ),
    }),

    async execute(_toolCallId, params) {
      const action = params["action"] as "start" | "stop" | "status";

      if (action === "status") {
        const running = isFollowMeRunning();
        const cmdVelTopic = getFollowMeCmdVelTopic(config);
        const state = getLastFollowMeState();
        let message = running
          ? `Follow Me is active; publishing to ${cmdVelTopic}.`
          : "Follow Me is stopped.";
        if (running && state.last_error) {
          message += ` Last tick failed: ${state.last_error}. If the user sees only zero twist, the loop is failing every tick—check camera topic, Ollama (reachable? model loaded?), and depth topic.`;
        } else if (running && state.last_twist && state.tick_count > 0) {
          message += ` Latest twist: linear_x=${state.last_twist.linear_x}, angular_z=${state.last_twist.angular_z} (tick ${state.tick_count}).`;
        }
        const payload: Record<string, unknown> = {
          success: true,
          running,
          cmd_vel_topic: cmdVelTopic,
          message,
        };
        if (running) {
          payload.last_detection = state.last_detection;
          payload.last_twist = state.last_twist;
          payload.last_error = state.last_error;
          payload.tick_count = state.tick_count;
        }
        return {
          content: [{ type: "text" as const, text: JSON.stringify(payload) }],
          details: { success: true, running, cmd_vel_topic: cmdVelTopic, ...(running ? state : {}) },
        };
      }

      if (action === "stop") {
        try {
          const transport = getTransport();
          stopFollowMeLoop(transport, config);
          return {
            content: [
              {
                type: "text" as const,
                text: JSON.stringify({
                  success: true,
                  message: "Follow Me stopped. Robot halted.",
                }),
              },
            ],
            details: { success: true },
          };
        } catch (err) {
          const message = err instanceof Error ? err.message : String(err);
          return {
            content: [{ type: "text" as const, text: JSON.stringify({ success: false, error: message }) }],
            details: { success: false, error: message },
          };
        }
      }

      if (action === "start") {
        try {
          const transport = getTransport();
          const cmdVelTopic = startFollowMeLoop(transport, config, api.logger);
          return {
            content: [
              {
                type: "text" as const,
                text: JSON.stringify({
                  success: true,
                  cmd_vel_topic: cmdVelTopic,
                  message: `Follow Me started. Publishing Twist to ${cmdVelTopic}. If the wheels don't move, confirm the robot subscribes to this exact topic (e.g. rostopic echo ${cmdVelTopic}). Say 'stop following' to stop.`,
                }),
              },
            ],
            details: { success: true, cmd_vel_topic: cmdVelTopic },
          };
        } catch (err) {
          const message = err instanceof Error ? err.message : String(err);
          return {
            content: [{ type: "text" as const, text: JSON.stringify({ success: false, error: message }) }],
            details: { success: false, error: message },
          };
        }
      }

      return {
        content: [{ type: "text" as const, text: JSON.stringify({ success: false, message: `Unknown action: ${action}` }) }],
        details: { success: false },
      };
    },
  });
}
