# Follow Me Mission (native)

## When to Use

Use when the user wants the robot to follow them:
- "Follow me"
- "Start following"
- "Stop following"

## Prerequisites

- Robot has a camera (2D or RealSense) publishing on a ROS2 topic (e.g. `/camera/image_raw/compressed`).
- OpenClaw gateway can reach **Ollama** with a Qwen vision model (e.g. `qwen2-vl:7b`). Configure **followMe.ollamaUrl** and **followMe.vlmModel** if Ollama is not on localhost or you use a different model.
- Plugin **followMe.cameraTopic** should match the robot’s camera topic.

## Steps

1. **Start** — Use `follow_robot` with `action: "start"`. A loop runs in the plugin: camera → Qwen VLM → cmd_vel. The robot will try to find a person and follow ~0.5 m behind (configurable via followMe.targetDistance).
2. **Stop** — Use `follow_robot` with `action: "stop"`. Stops the loop and sends zero twist.
3. **Status** — Use `follow_robot` with `action: "status"` to see if Follow Me is active.

## Tool: follow_robot

- **action**: `start` | `stop` | `status`

No external apps or ROS2 Follow Me nodes; this is the native OpenClaw + RosClaw implementation.

## Example: "Follow me"

```
Tool: follow_robot
action: start
```

## Example: "Stop following"

```
Tool: follow_robot
action: stop
```

## Depth (RealSense) — recommended if motors don’t move

- If the robot has a RealSense (or other depth camera), set **followMe.depthTopic** (e.g. `/camera/camera/depth/image_rect_raw`). Follow Me will then use **real distance** in meters vs **followMe.targetDistance** to decide forward/back. When depth is configured, the robot will **approach or back off based on depth even if the VLM is uncertain** (person_visible false); the VLM is still used for left/right steering when it sees a person.
- For “how far am I from the robot?” use **ros2_depth_distance**. It returns the median distance in meters at the center of the depth image.

## Tips

- To “stay 2 meters back”, set **followMe.targetDistance** to 2. With **followMe.depthTopic** set, the loop uses real depth vs this target.
- If Follow Me has no detections or the user asks "is Qwen/Ollama running?", use **ollama_status** to check reachability and that the vision model is available. Ensure Ollama is running and the model is pulled (e.g. `ollama run qwen2-vl:7b`).
