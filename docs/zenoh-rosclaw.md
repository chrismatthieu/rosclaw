# Zenoh setup for RosClaw (Mode D)

RosClaw can connect to a Zenoh router so you can control a robot that uses **zenoh-bridge-ros2dds** (or ROS 2 with Zenoh RMW). The plugin uses the **zenoh-ts** JavaScript library, which connects **only via WebSocket** to the **zenoh-plugin-remote-api** on the router.

## Two ways to connect to the same router

| Client | Protocol | Endpoint example |
|--------|----------|------------------|
| Native Zenoh (Rust, C, Python, etc.) | TCP | `tcp/127.0.0.1:7447` |
| **zenoh-ts** (RosClaw plugin) | **WebSocket** | `ws://127.0.0.1:10000` |

If you only run `zenohd` with default settings, it listens on `tcp/7447`. Native tools (e.g. `z_sub -e tcp/127.0.0.1:7447`) will see traffic, but **RosClaw will not** until the router also exposes the remote-api WebSocket.

## Install on macOS (Homebrew)

Install the Zenoh router and the WebSocket (remote-api) plugin so RosClaw can connect:

```bash
brew tap eclipse-zenoh/homebrew-zenoh
brew install zenoh
brew install zenoh-plugin-remote-api
```

If the tap or package names differ, see [Eclipse Zenoh installation](https://zenoh.io/docs/getting-started/installation/). Then follow “Run zenohd” below.

## Run zenohd with the remote-api plugin

1. **Install** `zenohd` and `zenoh-plugin-remote-api` (e.g. on macOS: `brew install zenoh zenoh-plugin-remote-api` after tapping `eclipse-zenoh/homebrew-zenoh`; see [Eclipse Zenoh](https://zenoh.io/docs/getting-started/installation/) for other platforms).

2. **Create a config file** that loads the plugin and sets the WebSocket port (e.g. `zenohd-rosclaw.json5`):

   ```json5
   {
     "plugins": {
       "remote_api": {
         "websocket_port": "10000"
       }
     }
   }
   ```

   Or use the one in this repo: `scripts/zenohd-rosclaw.json5`.

3. **Start the router** with that config:

   ```bash
   zenohd -c scripts/zenohd-rosclaw.json5
   ```

   Or, if you use a different path:

   ```bash
   zenohd -c /path/to/zenohd-rosclaw.json5
   ```

4. **Configure RosClaw** with Zenoh transport and WebSocket endpoint:

   ```bash
   ./scripts/configure_rosclaw.sh --mode zenoh --zenoh-endpoint ws://localhost:10000
   ```

   In OpenClaw plugin config: **Zenoh Router Endpoint** = `ws://localhost:10000` (or `ws://<router-ip>:10000` if the router is on another host).

5. **Restart the OpenClaw gateway.** The plugin will connect to the same zenohd via WebSocket and see the same keys as native clients using `tcp/7447`.

## Using zenoh-bridge-ros2dds

- On the **robot**: run ROS 2 (DDS) and **zenoh-bridge-ros2dds**, and point the bridge at the same Zenoh router (e.g. `tcp/<macbook-ip>:7447` if the router runs on your MacBook).
- On the **MacBook**: run **zenohd** with the config above (so it listens on tcp/7447 and ws/10000). RosClaw connects to `ws://localhost:10000`; your Rust `z_sub -e tcp/127.0.0.1:7447` connects to the same router.
- Set RosClaw **zenoh.keyFormat** to **`ros2dds`** (default) so topic keys match the bridge (`cmd_vel`, `camera/camera/...`, etc.).

## Viewing gateway logs (macOS)

The gateway runs as a LaunchAgent. Logs:

- **stdout:** `~/.openclaw/logs/gateway.log`
- **stderr:** `~/.openclaw/logs/gateway.err.log`

Watch live: `tail -f ~/.openclaw/logs/gateway.err.log`. Look for `[RosClaw] Zenoh connected to ...` (success) or `WebSocket has been disconnected from remote-api-plugin: 1006` (Zenoh WebSocket failed).

## Troubleshooting

- **“RosClaw doesn’t see topics” but `z_sub -e tcp/127.0.0.1:7447 -k '**'` does**  
  RosClaw uses zenoh-ts and must connect to the **WebSocket** port of **zenoh-plugin-remote-api** (e.g. 10000). Start zenohd with a config that loads the remote_api plugin and `websocket_port: "10000"`, and set the plugin’s Zenoh Router Endpoint to `ws://localhost:10000`.

- **Connection refused to ws://localhost:10000**  
  zenohd is not running with zenoh-plugin-remote-api, or the plugin is bound to another port. Check the plugin config and the port in RosClaw.

- **"WebSocket has been disconnected from remote-api-plugin: 1006" in gateway.err.log**  
  zenoh-ts could not establish or keep a WebSocket to the router. Start **zenohd with the remote-api config** (port 10000) **before** starting the OpenClaw gateway. Verify: run `zenohd -c scripts/zenohd-rosclaw.json5`, then `lsof -i :10000` or `nc -zv localhost 10000`; you should see the listener. Then start or restart the gateway. On macOS, gateway logs are in `~/.openclaw/logs/gateway.log` and `~/.openclaw/logs/gateway.err.log`; use `tail -f ~/.openclaw/logs/gateway.err.log` to watch.

- **Topics still empty after connecting**  
  Ensure the robot’s zenoh-bridge-ros2dds is connected to this same router and is publishing. Use `z_sub -e tcp/127.0.0.1:7447 -k '**'` to confirm traffic; then restart the gateway so RosClaw reconnects and lists topics.
