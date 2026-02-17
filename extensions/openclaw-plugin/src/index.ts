import type { OpenClawPluginApi } from "./plugin-api.js";
import { registerService } from "./service.js";
import { registerTools } from "./tools/index.js";
import { registerSafetyHook } from "./safety/validator.js";
import { registerRobotContext } from "./context/robot-context.js";
import { registerEstopCommand } from "./commands/estop.js";

/**
 * RosClaw — OpenClaw plugin for ROS2 robot control via natural language.
 */
export default {
  id: "rosclaw",
  name: "RosClaw",

  register(api: OpenClawPluginApi): void {
    api.logger.info("RosClaw plugin loading...");

    // Register the rosbridge WebSocket connection as a managed service
    registerService(api);

    // Register all ROS2 tools with the AI agent
    registerTools(api);

    // Register safety validation hook (before_tool_call)
    registerSafetyHook(api);

    // Register robot capability injection (before_agent_start)
    registerRobotContext(api);

    // Register direct commands (bypass AI)
    registerEstopCommand(api);

    api.logger.info("RosClaw plugin loaded successfully");
  },
};
