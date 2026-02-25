# RosClaw Missions

Missions are higher-level behaviors exposed to the AI agent via **skills** and **mission tools**. The **Follow Me** mission is **native** to OpenClaw + RosClaw: it uses the robot's camera (ROS2), a Qwen vision model (Ollama), and cmd_vel (ROS2)—no external apps or separate ROS2 nodes.

## Follow Me (native)

**Mission:** The robot finds a person in the room (using a 2D or RealSense camera) and follows them, staying about 1 m behind. The logic runs **inside the RosClaw plugin**: a control loop grabs camera frames over ROS2, sends them to Ollama (Qwen VLM) for person detection and position/distance, then publishes Twist commands to cmd_vel.

### Run from OpenClaw webchat

1. **On the robot**: ROS2 and rosbridge running so the gateway can subscribe to the camera and publish to cmd_vel. A camera topic must be available (e.g. `/camera/image_raw/compressed` or RealSense `/camera/camera/color/image_raw`).

2. **Where OpenClaw runs**: Ollama with a vision model (e.g. `ollama pull qwen2-vl:7b`). Configure the RosClaw plugin if needed:
   - **followMe.ollamaUrl** — Ollama API (default `http://localhost:11434`)
   - **followMe.vlmModel** — e.g. `qwen2-vl:7b`
   - **followMe.cameraTopic** — ROS2 camera topic (default `/camera/image_raw/compressed`); use a RealSense topic if that’s what the robot has.
   - **followMe.targetDistance** — follow distance in meters (default 1.0).

3. **In the webchat**, say **"Follow me"** or **"Start following"** — the assistant uses the **follow_robot** tool with action **start**. Say **"Stop following"** to stop. The interface is the webchat; no separate app.

### When to use

- User says "follow me", "start following", "stop following".

### How it works

- **follow_robot** tool (actions: `start`, `stop`, `status`) starts or stops a **background loop** in the plugin.
- The loop: subscribe to the configured camera topic → get one frame → send it to Ollama (Qwen VLM) with a prompt asking for person visibility, position (left/center/right), and distance hint (close/medium/far) → compute linear and angular velocity → publish to cmd_vel. Repeats at the configured rate (default 5 Hz). Velocities are clamped to the plugin’s safety limits.
- No dependency on openclaw-robotics or the rosclaw_follow_me ROS2 package; everything is in the plugin + Ollama + ROS2 transport.

### Requirements

- **ROS2** camera topic (2D or RealSense) reachable via rosbridge.
- **Ollama** with a Qwen vision model (e.g. qwen2-vl:7b) reachable from the machine running the OpenClaw gateway.

### Troubleshooting: motors not moving

1. **Confirm cmd_vel topic**  
   The plugin now sends an **advertise** (topic + type) before publishing, so rosbridge creates the publisher correctly. Ensure **followMe.cmdVelTopic** (or **robot.namespace**) matches the topic the base uses, e.g. `/robot3946b404c33e4aa39a8d16deb1c5c593/cmd_vel`.

2. **See if Twist messages reach the robot**  
   On the robot, with Follow Me running, run:
   ```bash
   ros2 topic echo /robot3946b404c33e4aa39a8d16deb1c5c593/cmd_vel
   ```
   If Twist messages appear, the gateway/rosbridge side is fine; the issue is likely the base driver (QoS or not subscribed to that topic). If no messages appear, check the gateway→rosbridge connection and that the plugin is publishing to that exact topic.

3. **Test the base from the robot**  
   On the robot:
   ```bash
   ros2 topic pub /robot3946b404c33e4aa39a8d16deb1c5c593/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.2, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}" -r 5
   ```
   If the wheels move, the topic and subscriber are correct; the problem is getting messages from the gateway (e.g. rosbridge or QoS). If they don’t move, the base may be subscribed to another topic or the motors may need to be enabled.

---

## Adding more missions

- Add a skill under `extensions/openclaw-plugin/skills/missions/<name>/SKILL.md`.
- Implement the mission (e.g. another plugin loop or tool) and document it here.
