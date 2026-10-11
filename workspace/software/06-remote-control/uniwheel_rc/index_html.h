/**
 * HTML web page with a single vertical slider (not a 2D thumbstick).
 *
 * Adapted from the reference repo's thumbstick UI, reduced to one axis on purpose:
 * this robot has no yaw actuator (see workspace/software/04-domain-randomization/
 * uniwheel_env_dr.py's module docstring), so a left/right axis would just be a
 * control that silently does nothing - worse than not having it. Up = forward,
 * down = backward, releases snap back to 0 (matching the thumbstick's spring-back).
 */

static const char INDEX_HTML[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html lang="en">
<head>
  <meta charset="UTF-8">
  <meta name="viewport" content="width=device-width, initial-scale=1.0, user-scalable=no">
  <title>UniWheel</title>
  <style>
    * { box-sizing: border-box; margin: 0; padding: 0; }
    body {
      min-height: 100vh;
      display: flex;
      flex-direction: column;
      align-items: center;
      justify-content: center;
      background: #fff;
      user-select: none;
    }
    #status { font-family: monospace; margin-bottom: 1rem; }
    #vel-value { font-family: monospace; margin-top: 1rem; }
    #slider-zone {
      position: relative;
      width: 90px;
      height: 260px;
      border-radius: 45px;
      border: 2px solid #000;
      touch-action: none;
      cursor: grab;
    }
    #slider-zone:active { cursor: grabbing; }
    #knob {
      position: absolute;
      width: 72px;
      height: 72px;
      border-radius: 50%;
      border: 2px solid #000;
      left: 50%;
      top: 50%;
      transform: translate(-50%, -50%);
      pointer-events: none;
    }
  </style>
</head>
<body>
<p id="status">not connected</p>
<div id="slider-zone">
  <div id="knob"></div>
</div>
<p id="vel-value">vel: 0.000</p>
<script>
  const zone     = document.getElementById('slider-zone');
  const knob     = document.getElementById('knob');
  const statusEl = document.getElementById('status');
  const velEl    = document.getElementById('vel-value');

  const ZONE_H = 260;
  const KNOB_R = 36;
  const MAX_D  = ZONE_H / 2 - KNOB_R;

  let active = false;
  let cmdVel = 0;
  let sendInterval = null;
  let ws;

  function connect() {
    ws = new WebSocket('ws://' + location.host + '/ws');

    ws.onopen = () => {
      statusEl.textContent = 'connected';
      sendInterval = setInterval(sendCmd, 50);
    };

    ws.onclose = ws.onerror = () => {
      statusEl.textContent = 'not connected';
      clearInterval(sendInterval);
      cmdVel = 0;
      setTimeout(connect, 1000);
    };
  }

  function sendCmd() {
    if (ws.readyState === WebSocket.OPEN) {
      ws.send(JSON.stringify({ vel: cmdVel }));
    }
  }

  connect();

  function zoneCenterY() {
    const r = zone.getBoundingClientRect();
    return r.top + ZONE_H / 2;
  }

  function updateKnob(clientY) {
    const cy = zoneCenterY();
    let dy = clientY - cy;
    if (dy > MAX_D) dy = MAX_D;
    if (dy < -MAX_D) dy = -MAX_D;
    knob.style.transform = `translate(-50%, calc(-50% + ${dy}px))`;
    // Up (negative dy) = forward = positive cmd_vel
    cmdVel = -parseFloat((dy / MAX_D).toFixed(3));
    velEl.textContent = 'vel: ' + cmdVel.toFixed(3);
  }

  function release() {
    if (!active) return;
    active = false;
    knob.style.transform = 'translate(-50%, -50%)';
    cmdVel = 0;
    velEl.textContent = 'vel: 0.000';
  }

  zone.addEventListener('mousedown', e => { active = true; updateKnob(e.clientY); });
  window.addEventListener('mousemove', e => { if (active) updateKnob(e.clientY); });
  window.addEventListener('mouseup', release);

  zone.addEventListener('touchstart', e => { e.preventDefault(); active = true; updateKnob(e.touches[0].clientY); }, { passive: false });
  window.addEventListener('touchmove', e => { if (active) updateKnob(e.touches[0].clientY); }, { passive: false });
  window.addEventListener('touchend', release);
</script>
</body>
</html>
)rawliteral";
