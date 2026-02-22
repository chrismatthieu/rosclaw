# @rosclaw/openclaw-canvas

Real-time robot dashboard rendered inside OpenClaw's native apps via the Canvas/A2UI system.

**Status: Phase 3 — not yet implemented.**

## Why This Exists

The primary RosClaw interface is chat (WhatsApp, Telegram, Discord, Slack). Chat works well for supervisory and mission-level commands ("navigate to the kitchen", "check the battery"), but some use cases need more than text:

- **Live camera feeds** — streaming video doesn't fit in a message thread
- **Real-time telemetry** — continuously updating sensor data, battery, IMU
- **Teleoperation** — joystick-style control requiring low-latency, high-frequency updates (>10Hz)
- **Map visualization** — SLAM maps with live robot position overlay

This extension provides a dashboard for **operators using the OpenClaw native app** (macOS, iOS, or Android). Users chatting via WhatsApp/Telegram never see it — they continue getting text replies and inline media from the AI agent.

## Who Sees What

| User | Interface | What They Get |
|------|-----------|---------------|
| Field user on WhatsApp/Telegram | Messaging app chat | AI text replies, inline camera snapshots, status summaries |
| Operator on OpenClaw native app | Chat + Canvas panel | Everything above, plus live dashboard with telemetry, camera stream, controls |

## How Canvas Works in OpenClaw

Canvas is a **WebView panel inside the OpenClaw native apps** (not a standalone web page or browser tab):

| Platform | App Location | Canvas Rendering |
|----------|-------------|------------------|
| macOS | `apps/macos/` | WKWebView with `openclaw-canvas://` URL scheme |
| iOS | `apps/ios/` | SwiftUI + WebView via `RootCanvas.swift` |
| Android | `apps/android/` | WebView via `CanvasController.kt` |

The AI agent controls the Canvas through a built-in `canvas` tool with these actions:

| Action | Description |
|--------|-------------|
| `canvas.present` | Show the canvas panel |
| `canvas.hide` | Hide the canvas panel |
| `canvas.navigate` | Navigate to a URL or local path |
| `canvas.eval` | Execute JavaScript in the canvas |
| `canvas.snapshot` | Capture a screenshot of the canvas |
| `canvas.a2ui.push` | Push A2UI component descriptions |
| `canvas.a2ui.pushJSONL` | Push A2UI JSONL streaming format |
| `canvas.a2ui.reset` | Reset A2UI renderer state |

## A2UI: How the Dashboard Renders

[A2UI](https://a2ui.org) ("Agent-to-UI") is an open standard (Google, Apache 2.0) for agent-driven UIs. It's a declarative JSONL protocol — agents describe UI as JSON, clients render with native components. OpenClaw bundles the A2UI v0.8 Lit renderer at `vendor/a2ui/`.

### Rendering flow

1. Plugin pushes A2UI JSONL via the `canvas` agent tool
2. Gateway forwards to the connected native app
3. Canvas WebView loads `/__openclaw__/a2ui/index.html` (bundles the Lit renderer)
4. Lit renderer maps A2UI components to web components
5. User clicks a button/slider → JavaScript bridge posts `userAction` back to the agent
6. Agent processes the action and pushes updated A2UI

### Key A2UI message types

**`surfaceUpdate`** — define/update the component tree (flat adjacency list):
```json
{
  "surfaceUpdate": {
    "surfaceId": "dashboard",
    "components": [
      { "id": "root", "component": { "Column": { "children": { "explicitList": ["battery_card", "camera_card"] } } } },
      { "id": "battery_card", "component": { "Card": { "child": "battery_text" } } },
      { "id": "battery_text", "component": { "Text": { "text": { "path": "/robot/battery" } } } }
    ]
  }
}
```

**`dataModelUpdate`** — push reactive data (bound components auto-re-render):
```json
{
  "dataModelUpdate": {
    "surfaceId": "dashboard",
    "path": "robot",
    "contents": [
      { "key": "battery", "valueString": "87%" },
      { "key": "status", "valueString": "navigating" }
    ]
  }
}
```

**`userAction`** — received when the operator interacts with the dashboard:
```json
{
  "userAction": {
    "name": "estop",
    "surfaceId": "dashboard",
    "sourceComponentId": "estop_btn"
  }
}
```

### Available A2UI components (v0.8 standard catalog)

| Category | Components |
|----------|-----------|
| Layout | `Row`, `Column`, `List` |
| Display | `Text`, `Image`, `Icon`, `Video`, `AudioPlayer`, `Divider` |
| Interactive | `Button`, `TextField`, `CheckBox`, `DateTimeInput`, `MultipleChoice`, `Slider` |
| Container | `Card`, `Tabs`, `Modal` |

## Data Flow Architecture

The main RosClaw plugin owns the rosbridge connection. This extension doesn't connect to ROS2 directly — it pushes A2UI through the agent, and the agent uses the main plugin's tools to fetch robot data.

```
OpenClaw Native App
    │
    ├── Chat panel (messaging interface)
    │
    └── Canvas panel (A2UI WebView)
            │
            ↕ A2UI JSONL (surfaceUpdate, dataModelUpdate, userAction)
            │
        OpenClaw Gateway
            │
            ├── AI Agent ← canvas tool (push A2UI)
            │       │
            │       └── RosClaw plugin tools (ros2_subscribe, ros2_camera_snapshot, ...)
            │               │
            │               └── rosbridge transport → ROS2 DDS → Robots
            │
            └── Gateway methods (rosclaw.subscribe, rosclaw.getRobotState, ...)
                    │
                    └── rosbridge transport → ROS2 DDS → Robots
```

Two data paths are possible:

1. **Agent-mediated** — the agent calls ros2 tools, formats results as A2UI, and pushes to canvas. Simple but adds latency (LLM round-trip).
2. **Gateway methods** — the main plugin registers `api.registerGatewayMethod()` endpoints. The canvas extension (or its frontend JS) calls these directly for real-time data, bypassing the LLM. Required for high-frequency updates.

Path 2 requires the main plugin to register gateway methods ([Issue #10](../docs/openclaw-plugin-review.md) — currently deferred).

## Planned Features

- Battery and system status cards (`Text` + `dataModelUpdate` for reactive updates)
- Camera feed panel (`Image` component, periodically updated via `dataModelUpdate`)
- Velocity slider/joystick for teleoperation (`Slider` + `userAction` callbacks)
- Emergency stop button (`Button` + `userAction` → triggers estop)
- Topic list and service browser (`List` + `Card` components)
- Nav2 goal progress visualization (`Text` + progress updates)

## Current State

The extension registers with OpenClaw but does nothing:

```typescript
export function register(api) {
  api.log.info("RosClaw Canvas extension loaded (Phase 3 — not yet implemented)");
}
```

## Prerequisites

Before implementing this extension:

1. **Main plugin gateway methods** — `@rosclaw/openclaw-plugin` needs to register `api.registerGatewayMethod()` endpoints for real-time data access (Issue #10, currently deferred)
2. **A2UI v0.8 familiarity** — the dashboard is built entirely as A2UI JSONL, not custom HTML/JS
3. **Native app testing** — Canvas only renders in the OpenClaw macOS, iOS, or Android app

## References

- [A2UI Specification v0.8](https://a2ui.org/specification/v0.8-a2ui/)
- [A2UI Message Reference](https://a2ui.org/reference/messages/)
- [A2UI Component Catalog](https://a2ui.org/concepts/components/)
- [OpenClaw Canvas Docs](https://docs.openclaw.ai/platforms/mac/canvas)
- [OpenClaw Canvas Skill](https://github.com/openclaw/openclaw/blob/main/skills/canvas/SKILL.md)
