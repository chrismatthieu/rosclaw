/**
 * Returns the RosClaw config page HTML.
 * Page fetches /rosclaw/config.json and renders current config with Mode A/B/C/D explanations.
 */
export function getConfigPageHtml(): string {
  return `<!DOCTYPE html>
<html lang="en">
<head>
  <meta charset="UTF-8" />
  <meta name="viewport" content="width=device-width, initial-scale=1" />
  <title>RosClaw Config</title>
  <style>
    * { box-sizing: border-box; }
    body { font-family: system-ui, sans-serif; margin: 12px; background: #1a1a1a; color: #e0e0e0; max-width: 720px; }
    h1 { font-size: 1.5rem; margin: 0 0 8px 0; }
    a { color: #0c9; text-decoration: none; }
    a:hover { text-decoration: underline; }
    .note { background: #2a2a2a; border-radius: 8px; padding: 12px; margin-bottom: 20px; font-size: 0.9rem; color: #aaa; }
    section { margin-bottom: 24px; }
    section h2 { font-size: 1.1rem; margin: 0 0 8px 0; color: #ccc; }
    .mode-grid { display: grid; gap: 8px; margin-bottom: 12px; }
    .mode-card { background: #252525; border-radius: 8px; padding: 10px 12px; border-left: 3px solid #444; }
    .mode-card.current { border-left-color: #0c9; background: #2a2a2a; }
    .mode-card strong { display: block; margin-bottom: 2px; }
    .mode-card span { font-size: 0.85rem; color: #888; }
    dl { margin: 0; display: grid; grid-template-columns: auto 1fr; gap: 4px 16px; font-size: 0.9rem; }
    dt { color: #888; }
    dd { margin: 0; word-break: break-all; }
    .empty { color: #666; font-style: italic; }
  </style>
</head>
<body>
  <h1>RosClaw Config</h1>
  <p><a href="/rosclaw/">Back to RosClaw</a></p>
  <div class="note">To change these values, update the RosClaw plugin configuration in your OpenClaw config and restart the gateway.</div>
  <div id="content">Loading config…</div>

  <script>
(function() {
  fetch('/rosclaw/config.json')
    .then(function(r) { return r.json(); })
    .then(function(c) { render(c); })
    .catch(function(e) { document.getElementById('content').textContent = 'Failed to load config: ' + e.message; });

  var MODES = {
    rosbridge: { label: 'Mode B – Local network', desc: 'WebSocket to rosbridge_server on the robot. Best for: dev, testing, same LAN.' },
    local: { label: 'Mode A – Same machine', desc: 'Plugin talks to ROS2 via local DDS (rclnodejs). Best for: single-robot, edge.' },
    webrtc: { label: 'Mode C – Cloud / remote', desc: 'WebRTC data channel; robot behind NAT runs rosclaw_agent. Best for: production, remote ops.' },
    zenoh: { label: 'Mode D – Zenoh', desc: 'Zenoh RMW; plugin connects to Zenoh router via zenoh-ts. Best for: Zenoh-based stacks.' }
  };

  function esc(s) {
    if (s === undefined || s === null) return '';
    return String(s).replace(/&/g, '&amp;').replace(/</g, '&lt;').replace(/>/g, '&gt;');
  }
  function val(v) {
    if (v === undefined || v === null || v === '') return '<span class="empty">(not set)</span>';
    if (typeof v === 'boolean') return v ? 'true' : 'false';
    if (typeof v === 'object') return esc(JSON.stringify(v));
    return esc(String(v));
  }

  function section(title, body) {
    return '<section><h2>' + esc(title) + '</h2>' + body + '</section>';
  }

  function dl(obj, keys) {
    var rows = keys.map(function(k) {
      var v = obj && obj[k];
      return '<dt>' + esc(k) + '</dt><dd>' + val(v) + '</dd>';
    });
    return '<dl>' + rows.join('') + '</dl>';
  }

  function render(c) {
    var mode = (c.transport && c.transport.mode) || 'rosbridge';
    var modeInfo = MODES[mode] || { label: mode, desc: '' };

    var html = '';

    html += section('Transport / Mode', 
      '<p>Current: <strong>' + esc(modeInfo.label) + '</strong> — ' + esc(modeInfo.desc) + '</p>' +
      '<div class="mode-grid">' +
      Object.keys(MODES).map(function(m) {
        var info = MODES[m];
        var isCurrent = m === mode;
        return '<div class="mode-card' + (isCurrent ? ' current' : '') + '">' +
          '<strong>' + esc(info.label) + '</strong>' +
          '<span>' + esc(info.desc) + '</span>' +
          '</div>';
      }).join('') +
      '</div>'
    );

    html += section('Robot', dl(c.robot || {}, ['name', 'namespace', 'cameraTopic']));

    if (mode === 'rosbridge' && c.rosbridge)
      html += section('Rosbridge (Mode B)', dl(c.rosbridge, ['url', 'reconnect', 'reconnectInterval']));
    if (mode === 'zenoh' && c.zenoh)
      html += section('Zenoh (Mode D)', dl(c.zenoh, ['routerEndpoint', 'domainId', 'keyFormat']));
    if (mode === 'local' && c.local)
      html += section('Local (Mode A)', dl(c.local, ['domainId']));
    if (mode === 'webrtc' && c.webrtc) {
      var w = {};
      for (var k in c.webrtc) {
        if (k === 'robotKey') w[k] = c.webrtc[k] ? '(set)' : '';
        else w[k] = c.webrtc[k];
      }
      html += section('WebRTC (Mode C)', dl(w, ['signalingUrl', 'apiUrl', 'robotId', 'robotKey']));
    }

    html += section('Teleop', dl(c.teleop || {}, ['cameraTopic', 'cmdVelTopic', 'speedDefault', 'cameraPollMs']));
    if (c.teleop && c.teleop.cameraTopics && c.teleop.cameraTopics.length)
      html += '<p>cameraTopics: ' + esc(JSON.stringify(c.teleop.cameraTopics)) + '</p>';

    html += section('Safety', dl(c.safety || {}, ['maxLinearVelocity', 'maxAngularVelocity']));
    if (c.safety && c.safety.workspaceLimits)
      html += '<p>workspaceLimits: ' + esc(JSON.stringify(c.safety.workspaceLimits)) + '</p>';

    html += section('Follow Me', dl(c.followMe || {}, ['useOllama', 'ollamaUrl', 'vlmModel', 'cameraTopic', 'cmdVelTopic', 'targetDistance', 'rateHz', 'minLinearVelocity', 'depthTopic', 'visionCallbackUrl']));

    document.getElementById('content').innerHTML = html;
  }
})();
  </script>
</body>
</html>`;
}
