# RosClaw Robot Setup Guide

This guide gets the RosClaw project running on your robot (Ubuntu + ROS2 + OpenClaw) for testing and demos.

## Onboarding (quick path)

For a **new robot**, use the setup scripts after cloning the repo:

```bash
git clone https://github.com/chrismatthieu/rosclaw.git
cd rosclaw
```

- **Interactive wizard** (asks robot vs gateway, then runs the right steps):
  ```bash
  ./scripts/onboard_robot.sh
  ```

- **Robot only** (run on the robot):
  ```bash
  ./scripts/setup_robot.sh
  ```
  Optionally: `--ros-distro jazzy` or `humble`, `--skip-apt` to skip apt and build rosbridge from source.

- **Gateway only** (run where OpenClaw runs):
  ```bash
  ./scripts/setup_gateway_plugin.sh
  ```
  Optionally: `--rosbridge-url ws://ROBOT_IP:9090`, `--robot-namespace YOUR_NAMESPACE` to document config.

Then start the bridges on the robot with `./scripts/run_demo_native.sh` and restart the OpenClaw gateway. See the rest of this doc for manual steps and troubleshooting.

---

## Two ways to run it

| Setup | OpenClaw runs on | Robot runs | Use case |
|-------|------------------|------------|----------|
| **Mode A** (same machine) | The robot | ROS2 + (optional) rosbridge on localhost | Single robot, all-in-one demo |
| **Mode B** (network) | Your laptop or server | ROS2 + rosbridge_server | Dev/testing, multi-robot |

Choose **Mode A** if OpenClaw is installed on the robot. Choose **Mode B** if OpenClaw runs on another machine and connects to the robot over the network.

---

## Prerequisites on the robot

- Ubuntu (22.04 or 24.04 recommended)
- ROS2 **Humble** or **Jazzy** (match your existing install)
- (For Mode A) OpenClaw installed on the robot
- (For Mode B) Robot and OpenClaw machine on the same network

---

## Step 1: Clone and prepare the repo on the robot

On the robot (or on your dev machine if you’ll copy the built workspace to the robot):

```bash
cd ~  # or your preferred path
git clone https://github.com/chrismatthieu/rosclaw.git
cd rosclaw
```

Use `--ros-distro` to match your ROS2 install (e.g. `humble` or `jazzy`):

```bash
./scripts/setup_workspace.sh --ros-distro humble
```

If you already have ROS2 and only need the RosClaw workspace built:

```bash
# Use your distro: humble, jazzy, or kilted
source /opt/ros/humble/setup.bash   # or jazzy
cd rosclaw/ros2_ws
colcon build --symlink-install
source install/setup.bash
```

---

## Step 2: Install the rosbridge WebSocket server (required for plugin)

The plugin talks to ROS2 via **rosbridge** over WebSocket. The robot needs the **rosbridge_suite** (which provides the WebSocket server). Install it for your distro:

```bash
# Ubuntu/Debian — use your ROS2 distro (humble or jazzy)
sudo apt update
sudo apt install -y ros-<DISTRO>-rosbridge-suite
```

Example for Humble:

```bash
sudo apt install -y ros-humble-rosbridge-suite
```

Example for Jazzy:

```bash
sudo apt install -y ros-jazzy-rosbridge-suite
```

---

## Step 3: Run the robot-side stack

Activate the workspace and start **rosbridge** (and optionally **rosclaw_discovery**).

### Option A: Activate once, then run (recommended)

```bash
cd /path/to/rosclaw
source scripts/activate_workspace.sh ros_env humble   # or jazzy
```

In **one terminal**, start the rosbridge WebSocket server (port 9090):

```bash
ros2 launch rosbridge_server rosbridge_websocket_launch.xml
```

In a **second terminal** (optional but useful for the AI agent), start the discovery node so the plugin knows your topics/services/actions:

```bash
source /path/to/rosclaw/scripts/activate_workspace.sh ros_env humble
ros2 run rosclaw_discovery discovery_node
```

Keep both running while you use OpenClaw.

### Option B: Use the launch script

From the repo root:

```bash
./scripts/run_robot_rosbridge.sh [humble|jazzy]
```

This sources the workspace and starts `rosbridge_server`. In another terminal, run the discovery node as above if you want it.

---

## Step 4: Configure OpenClaw and the RosClaw plugin

### If OpenClaw is on the robot (Mode A)

1. Build the plugin (from the repo root, on the robot):
   ```bash
   cd /path/to/rosclaw
   pnpm install
   pnpm build
   ```
2. Install the plugin into OpenClaw:
   ```bash
   openclaw plugins install -l ./extensions/openclaw-plugin
   ```
3. In OpenClaw, set the RosClaw plugin config:
   - **Transport mode:** `rosbridge`
   - **Rosbridge URL:** `ws://localhost:9090`
4. Start OpenClaw and connect your messaging channel (Telegram, WhatsApp, etc.). The plugin will connect to rosbridge on localhost.

### If OpenClaw is on another machine (Mode B)

1. On your **laptop/server** where OpenClaw runs:
   ```bash
   cd /path/to/rosclaw
   pnpm install
   pnpm build
   openclaw plugins install -l ./extensions/openclaw-plugin
   ```
2. Find the robot’s IP (on the robot run `hostname -I` or check your router).
3. In OpenClaw, set the RosClaw plugin config:
   - **Transport mode:** `rosbridge`
   - **Rosbridge URL:** `ws://<ROBOT_IP>:9090`  
     Example: `ws://192.168.1.50:9090`
4. Ensure the robot’s firewall allows TCP port **9090** (rosbridge):
   ```bash
   sudo ufw allow 9090/tcp
   sudo ufw reload
   ```
5. Start OpenClaw and your messaging app; the plugin will connect to the robot at that URL.

---

## Quick checks

- **Rosbridge listening:** On the robot, with rosbridge running:
  ```bash
  ss -tlnp | grep 9090
  ```
  You should see something like `*:9090`.

- **Discovery:** With discovery running, you can inspect what the agent will see:
  ```bash
  ros2 topic echo /rosclaw/capabilities --once
  ```

- **Plugin connection:** In OpenClaw, after starting a chat, the plugin should connect to the robot; check OpenClaw logs for RosClaw connection messages.

---

## Try a demo

Once everything is running:

1. Send a message to your bot, e.g. **“Move forward 1 meter”** (if your robot has `/cmd_vel`).
2. Or **“What do you see?”** if you have a camera topic.
3. Use **`/estop`** for emergency stop (bypasses the AI).

---

## Troubleshooting

| Issue | What to check |
|-------|----------------|
| Plugin won’t connect | Rosbridge URL correct? Robot IP (Mode B)? Firewall allows 9090? |
| “Package not found” (rosbridge_server) | Install `ros-<distro>-rosbridge-suite` (Step 2). |
| “Package not found” (rosclaw_discovery) | Run `source scripts/activate_workspace.sh` and ensure `ros2_ws` is built. |
| Different ROS distro | Use `--ros-distro humble` or `jazzy` in `setup_workspace.sh` and when sourcing. |

### 404 errors when installing rosbridge_suite

If `sudo apt install ros-jazzy-rosbridge-suite` fails with **404 Not Found** (stale ROS or Ubuntu mirror):

1. **Refresh and retry**
   ```bash
   sudo apt update
   sudo apt install -y ros-jazzy-rosbridge-suite
   ```

2. **If it still fails**, install **from source** so you don't depend on the broken packages:
   ```bash
   cd /home/ubuntu/Projects/rosclaw
   ./scripts/install_rosbridge_from_source.sh jazzy
   ```
   Then run rosbridge with:
   ```bash
   source /opt/ros/jazzy/setup.bash
   source ros2_ws/install/setup.bash
   ros2 launch rosbridge_server rosbridge_websocket_launch.xml
   ```

For deployment with OpenClaw in the cloud and the robot behind NAT, see **Mode C** in [Architecture](architecture.md) (WebRTC + `rosclaw_agent` on the robot).
