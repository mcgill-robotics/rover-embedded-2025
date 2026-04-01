"""
can_dashboard.py — Live CAN telemetry dashboard

Reads from the SQLite database written by can_logger.py and serves a
browser UI that auto-updates via Server-Sent Events (SSE).

Usage:
    pip install flask
    python scripts/can_dashboard.py                         # defaults
    python scripts/can_dashboard.py --db can_log.db --port 5000

Then open http://localhost:5000 in your browser.

Run this ALONGSIDE can_logger.py — the logger writes to the DB, and
this dashboard reads from it. SQLite WAL mode allows both to operate
on the same file concurrently without locking issues.

Project structure:
    esc_can/
        __init__.py
        bus.py
        protocol.py
        datalogger.py
    scripts/
        can_logger.py
        can_dashboard.py    ← this file
"""

import argparse
import json
import sqlite3
import time
import threading
from pathlib import Path

from flask import Flask, Response, jsonify

app = Flask(__name__)


# Global config — set by main()

DB_PATH = "can_log.db"
POLL_INTERVAL = 0.20  # seconds between SSE pushes


# Signal metadata — matches your firmware telemetry table

SIGNAL_META = {
    "CALIBRATION": {"label": "Speed",        "unit": "RPM",  "icon": "&#x21BB;", "decimals": 0, "order": 0},
    "POSITION":    {"label": "Position",      "unit": "deg",  "icon": "&#x2316;", "decimals": 2, "order": 1},
    "VOLTAGE":     {"label": "Bus Voltage",   "unit": "V",    "icon": "&#x26A1;", "decimals": 2, "order": 2},
    "CURRENT":     {"label": "Current",       "unit": "A",    "icon": "&#x223F;", "decimals": 3, "order": 3},
    "TEMPERATURE": {"label": "Temperature",   "unit": "°C",   "icon": "&#x1F321;","decimals": 1, "order": 4},
    "CURRENT_STATE":{"label": "Motor State",  "unit": "",     "icon": "&#x2699;", "decimals": 0, "order": 5},
    "PING":        {"label": "Ping",          "unit": "",     "icon": "&#x1F4E1;","decimals": 0, "order": 6},
    "CONTROL_MODE":{"label": "Control Mode",  "unit": "",     "icon": "&#x2638;", "decimals": 0, "order": 7},
    
}


def get_db() -> sqlite3.Connection:
    """Open a read-only connection to the logger database."""
    conn = sqlite3.connect(f"file:{DB_PATH}?mode=ro", uri=True,
                           check_same_thread=False)
    conn.row_factory = sqlite3.Row
    conn.execute("PRAGMA journal_mode=WAL")
    return conn



# API endpoints


@app.route("/api/latest")
def api_latest():
    """Return the latest decoded value for every (device_id, signal) pair."""
    conn = get_db()
    try:
        rows = conn.execute("""
            SELECT device_id, spec_name, decoded_float, timestamp_s,
                   MAX(frame_id) AS latest_frame
            FROM raw_frames
            WHERE decoded_float IS NOT NULL
              AND sender = 'SLAVE'
            GROUP BY device_id, spec_name
            ORDER BY device_id, spec_name
        """).fetchall()

        result = {}
        for r in rows:
            dev = r["device_id"]
            sig = r["spec_name"]
            if dev not in result:
                result[dev] = {}
            result[dev][sig] = {
                "value": r["decoded_float"],
                "timestamp_s": r["timestamp_s"],
            }
        return jsonify(result)
    finally:
        conn.close()


@app.route("/api/history/<int:device_id>/<signal>")
def api_history(device_id: int, signal: str):
    """Return the last 200 data points for a specific signal."""
    conn = get_db()
    try:
        rows = conn.execute("""
            SELECT timestamp_s, decoded_float
            FROM raw_frames
            WHERE device_id = ? AND spec_name = ?
              AND decoded_float IS NOT NULL
              AND sender = 'SLAVE'
            ORDER BY timestamp_s DESC
            LIMIT 200
        """, (device_id, signal.upper())).fetchall()

        data = [{"t": r["timestamp_s"], "v": r["decoded_float"]}
                for r in reversed(rows)]
        return jsonify(data)
    finally:
        conn.close()


@app.route("/api/stats")
def api_stats():
    """Return overall bus statistics."""
    conn = get_db()
    try:
        row = conn.execute("""
            SELECT COUNT(*) AS total,
                   MIN(timestamp_s) AS t_min,
                   MAX(timestamp_s) AS t_max,
                   COUNT(DISTINCT device_id) AS esc_count
            FROM raw_frames
        """).fetchone()

        duration = (row["t_max"] or 0) - (row["t_min"] or 0)
        return jsonify({
            "total_frames": row["total"],
            "duration_s": round(duration, 1),
            "fps": round(row["total"] / duration, 1) if duration > 0 else 0,
            "esc_count": row["esc_count"],
        })
    finally:
        conn.close()


@app.route("/api/stream")
def api_stream():
    """SSE endpoint — pushes latest values every POLL_INTERVAL seconds."""
    def generate():
        conn = get_db()
        last_frame_id = 0
        try:
            while True:
                rows = conn.execute("""
                    SELECT device_id, spec_name, decoded_float, timestamp_s,
                           MAX(frame_id) AS latest_frame
                    FROM raw_frames
                    WHERE decoded_float IS NOT NULL
                      AND sender = 'SLAVE'
                      AND frame_id > ?
                    GROUP BY device_id, spec_name
                """, (last_frame_id,)).fetchall()

                if rows:
                    result = {}
                    for r in rows:
                        dev = str(r["device_id"])
                        sig = r["spec_name"]
                        if dev not in result:
                            result[dev] = {}
                        result[dev][sig] = {
                            "value": r["decoded_float"],
                            "t": r["timestamp_s"],
                        }
                        if r["latest_frame"] and r["latest_frame"] > last_frame_id:
                            last_frame_id = r["latest_frame"]

                    yield f"data: {json.dumps(result)}\n\n"
                else:
                    yield f"data: {json.dumps({})}\n\n"

                time.sleep(POLL_INTERVAL)
        finally:
            conn.close()

    return Response(generate(), mimetype="text/event-stream",
                    headers={"Cache-Control": "no-cache",
                             "X-Accel-Buffering": "no"})



# Serve the dashboard HTML


@app.route("/")
def index():
    return DASHBOARD_HTML



# Dashboard HTML — self-contained, no build step


DASHBOARD_HTML = """<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>CAN FD Telemetry Dashboard</title>
<style>
  @import url('https://fonts.googleapis.com/css2?family=JetBrains+Mono:wght@300;400;500;600;700&family=DM+Sans:wght@400;500;600;700&display=swap');

  :root {
    --bg-primary:    #0a0e14;
    --bg-card:       #111820;
    --bg-card-hover: #161e28;
    --border:        #1e2a3a;
    --border-active: #2a3a4f;
    --text-primary:  #e6edf3;
    --text-secondary:#7d8a9a;
    --text-dim:      #4a5568;
    --accent-blue:   #58a6ff;
    --accent-green:  #3fb950;
    --accent-orange: #d29922;
    --accent-red:    #f85149;
    --accent-purple: #bc8cff;
    --accent-cyan:   #76e4f7;
    --glow-blue:     rgba(88,166,255,0.08);
    --glow-green:    rgba(63,185,80,0.08);
    --glow-orange:   rgba(210,153,34,0.08);
    --radius:        10px;
    --mono:          'JetBrains Mono', monospace;
    --sans:          'DM Sans', system-ui, sans-serif;
  }

  * { margin:0; padding:0; box-sizing:border-box; }

  body {
    background: var(--bg-primary);
    color: var(--text-primary);
    font-family: var(--sans);
    min-height: 100vh;
    overflow-x: hidden;
  }

  /* --- Top bar --- */
  .topbar {
    display: flex;
    align-items: center;
    justify-content: space-between;
    padding: 16px 28px;
    border-bottom: 1px solid var(--border);
    background: linear-gradient(180deg, rgba(17,24,32,0.95) 0%, rgba(10,14,20,0.98) 100%);
    backdrop-filter: blur(12px);
    position: sticky;
    top: 0;
    z-index: 100;
  }

  .topbar-left {
    display: flex;
    align-items: center;
    gap: 14px;
  }

  .topbar-logo {
    width: 32px;
    height: 32px;
    background: linear-gradient(135deg, var(--accent-blue), var(--accent-cyan));
    border-radius: 8px;
    display: flex;
    align-items: center;
    justify-content: center;
    font-size: 16px;
    font-weight: 700;
    color: var(--bg-primary);
    font-family: var(--mono);
  }

  .topbar h1 {
    font-family: var(--sans);
    font-size: 16px;
    font-weight: 600;
    letter-spacing: -0.01em;
    color: var(--text-primary);
  }

  .topbar h1 span {
    color: var(--text-secondary);
    font-weight: 400;
  }

  .topbar-stats {
    display: flex;
    gap: 24px;
    font-family: var(--mono);
    font-size: 12px;
    color: var(--text-secondary);
  }

  .topbar-stat {
    display: flex;
    align-items: center;
    gap: 6px;
  }

  .topbar-stat .val {
    color: var(--text-primary);
    font-weight: 500;
  }

  .status-dot {
    width: 7px;
    height: 7px;
    border-radius: 50%;
    background: var(--accent-green);
    animation: pulse-dot 2s ease-in-out infinite;
  }

  .status-dot.disconnected {
    background: var(--accent-red);
    animation: none;
  }

  @keyframes pulse-dot {
    0%, 100% { opacity: 1; box-shadow: 0 0 0 0 rgba(63,185,80,0.4); }
    50%      { opacity: 0.7; box-shadow: 0 0 0 4px rgba(63,185,80,0); }
  }

  /* --- Main grid --- */
  .main {
    padding: 24px 28px;
  }

  .esc-section {
    margin-bottom: 32px;
  }

  .esc-header {
    display: flex;
    align-items: center;
    gap: 12px;
    margin-bottom: 16px;
  }

  .esc-id-badge {
    font-family: var(--mono);
    font-size: 13px;
    font-weight: 600;
    background: var(--glow-blue);
    color: var(--accent-blue);
    border: 1px solid rgba(88,166,255,0.2);
    border-radius: 6px;
    padding: 4px 10px;
  }

  .esc-label {
    font-size: 14px;
    color: var(--text-secondary);
    font-weight: 500;
  }

  .signals-grid {
    display: grid;
    grid-template-columns: repeat(auto-fill, minmax(220px, 1fr));
    gap: 12px;
  }

  /* --- Signal card --- */
  .signal-card {
    background: var(--bg-card);
    border: 1px solid var(--border);
    border-radius: var(--radius);
    padding: 16px 18px;
    transition: all 0.2s ease;
    position: relative;
    overflow: hidden;
  }

  .signal-card::before {
    content: '';
    position: absolute;
    top: 0;
    left: 0;
    right: 0;
    height: 2px;
    background: var(--card-accent, var(--accent-blue));
    opacity: 0.6;
    transition: opacity 0.2s;
  }

  .signal-card:hover {
    background: var(--bg-card-hover);
    border-color: var(--border-active);
  }

  .signal-card:hover::before {
    opacity: 1;
  }

  .signal-card.flash {
    animation: card-flash 0.3s ease;
  }

  @keyframes card-flash {
    0%   { background: var(--bg-card); }
    30%  { background: rgba(88,166,255,0.06); }
    100% { background: var(--bg-card); }
  }

  .card-top {
    display: flex;
    justify-content: space-between;
    align-items: flex-start;
    margin-bottom: 12px;
  }

  .card-label {
    font-size: 12px;
    font-weight: 500;
    color: var(--text-secondary);
    text-transform: uppercase;
    letter-spacing: 0.04em;
  }

  .card-icon {
    font-size: 16px;
    opacity: 0.5;
  }

  .card-value {
    font-family: var(--mono);
    font-size: 28px;
    font-weight: 600;
    letter-spacing: -0.02em;
    color: var(--text-primary);
    line-height: 1.1;
    margin-bottom: 4px;
    transition: color 0.3s;
  }

  .card-unit {
    font-family: var(--mono);
    font-size: 13px;
    font-weight: 400;
    color: var(--text-dim);
    margin-left: 4px;
  }

  .card-time {
    font-family: var(--mono);
    font-size: 10px;
    color: var(--text-dim);
    margin-top: 8px;
  }

  /* Mini sparkline canvas */
  .card-spark {
    width: 100%;
    height: 32px;
    margin-top: 10px;
    opacity: 0.7;
  }

  /* Per-signal accent colors */
  .signal-card[data-signal="CALIBRATION"]  { --card-accent: var(--accent-blue); }
  .signal-card[data-signal="POSITION"]     { --card-accent: var(--accent-cyan); }
  .signal-card[data-signal="VOLTAGE"]      { --card-accent: var(--accent-orange); }
  .signal-card[data-signal="CURRENT"]      { --card-accent: var(--accent-purple); }
  .signal-card[data-signal="TEMPERATURE"]  { --card-accent: var(--accent-red); }
  .signal-card[data-signal="CURRENT_STATE"]{ --card-accent: var(--accent-green); }
  .signal-card[data-signal="PING"]         { --card-accent: var(--accent-green); }
  .signal-card[data-signal="CONTROL_MODE"] { --card-accent: var(--accent-purple); }

  /* --- Frame log --- */
  .log-section {
    margin-top: 32px;
  }

  .log-header {
    font-size: 14px;
    font-weight: 600;
    color: var(--text-secondary);
    margin-bottom: 12px;
    display: flex;
    align-items: center;
    gap: 8px;
  }

  .log-header .count {
    font-family: var(--mono);
    font-size: 12px;
    color: var(--text-dim);
  }

  .log-table {
    width: 100%;
    border-collapse: collapse;
    font-family: var(--mono);
    font-size: 12px;
  }

  .log-table th {
    text-align: left;
    padding: 8px 12px;
    color: var(--text-dim);
    font-weight: 500;
    text-transform: uppercase;
    letter-spacing: 0.05em;
    font-size: 10px;
    border-bottom: 1px solid var(--border);
    position: sticky;
    top: 0;
    background: var(--bg-primary);
  }

  .log-table td {
    padding: 6px 12px;
    color: var(--text-secondary);
    border-bottom: 1px solid rgba(30,42,58,0.4);
  }

  .log-table tr:hover td {
    color: var(--text-primary);
    background: rgba(88,166,255,0.03);
  }

  .log-table .arb-id {
    color: var(--accent-blue);
    font-weight: 500;
  }

  .log-wrap {
    max-height: 280px;
    overflow-y: auto;
    border: 1px solid var(--border);
    border-radius: var(--radius);
    background: var(--bg-card);
  }

  .log-wrap::-webkit-scrollbar { width: 6px; }
  .log-wrap::-webkit-scrollbar-track { background: transparent; }
  .log-wrap::-webkit-scrollbar-thumb { background: var(--border); border-radius: 3px; }

  /* --- Empty state --- */
  .empty-state {
    text-align: center;
    padding: 80px 20px;
    color: var(--text-dim);
  }

  .empty-state .icon { font-size: 48px; margin-bottom: 16px; }
  .empty-state h2 { font-size: 18px; color: var(--text-secondary); margin-bottom: 8px; }
  .empty-state p { font-size: 14px; max-width: 400px; margin: 0 auto; line-height: 1.6; }

  /* --- Responsive --- */
  @media (max-width: 640px) {
    .topbar { flex-direction: column; gap: 10px; align-items: flex-start; }
    .signals-grid { grid-template-columns: 1fr 1fr; }
    .card-value { font-size: 22px; }
    .main { padding: 16px; }
  }
</style>
</head>
<body>

<div class="topbar">
  <div class="topbar-left">
    <div class="topbar-logo">FD</div>
    <h1>CAN FD Dashboard <span>— B-G431B-ESC1</span></h1>
  </div>
  <div class="topbar-stats">
    <div class="topbar-stat">
      <div class="status-dot" id="statusDot"></div>
      <span id="statusText">Connecting…</span>
    </div>
    <div class="topbar-stat">Frames: <span class="val" id="statFrames">—</span></div>
    <div class="topbar-stat">Rate: <span class="val" id="statFps">—</span> f/s</div>
    <div class="topbar-stat">Uptime: <span class="val" id="statUptime">—</span></div>
  </div>
</div>

<div class="main" id="mainContent">
  <div class="empty-state" id="emptyState">
    <div class="icon">&#x1F50C;</div>
    <h2>Waiting for CAN data…</h2>
    <p>Make sure <code>can_logger.py</code> is running and writing to the same database file. Data will appear here automatically.</p>
  </div>
</div>

<script>
// ---------------------------------------------------------------------------
// Signal metadata (mirrors Python SIGNAL_META)
// ---------------------------------------------------------------------------
const SIGNAL_META = {
  CALIBRATION:   {label:"Speed",       unit:"RPM", icon:"&#x21BB;",  decimals:0, order:0},
  POSITION:      {label:"Position",    unit:"deg", icon:"&#x2316;",  decimals:2, order:1},
  VOLTAGE:       {label:"Bus Voltage", unit:"V",   icon:"&#x26A1;",  decimals:2, order:2},
  CURRENT:       {label:"Current",     unit:"A",   icon:"&#x223F;",  decimals:3, order:3},
  TEMPERATURE:   {label:"Temperature", unit:"°C",  icon:"&#x1F321;", decimals:1, order:4},
  CURRENT_STATE: {label:"Motor State", unit:"",    icon:"&#x2699;",  decimals:0, order:5},
  PING:          {label:"Ping",        unit:"",    icon:"&#x1F4E1;", decimals:0, order:6},
  CONTROL_MODE:  {label:"Control Mode",unit:"",    icon:"&#x2638;",  decimals:0, order:7},
};

const SIGNAL_ORDER = Object.keys(SIGNAL_META).sort((a,b) => SIGNAL_META[a].order - SIGNAL_META[b].order);

// Sparkline history per (device, signal)
const sparkData = {};  // key: "dev_sig" -> array of last 60 values
const SPARK_MAX = 60;

// Track known ESCs
const knownESCs = new Set();
let totalFrames = 0;
let sessionStart = null;

// ---------------------------------------------------------------------------
// Build / update the DOM for an ESC
// ---------------------------------------------------------------------------
function ensureESCSection(devId) {
  if (knownESCs.has(devId)) return;
  knownESCs.add(devId);

  // Remove empty state
  const empty = document.getElementById('emptyState');
  if (empty) empty.remove();

  const main = document.getElementById('mainContent');

  const section = document.createElement('div');
  section.className = 'esc-section';
  section.id = `esc-${devId}`;
  section.innerHTML = `
    <div class="esc-header">
      <div class="esc-id-badge">ESC ${devId}</div>
      <div class="esc-label">Device ID ${devId} · Drive Motor</div>
    </div>
    <div class="signals-grid" id="grid-${devId}"></div>
  `;
  main.appendChild(section);

  // Create signal cards
  const grid = document.getElementById(`grid-${devId}`);
  for (const sig of SIGNAL_ORDER) {
    const meta = SIGNAL_META[sig];
    const card = document.createElement('div');
    card.className = 'signal-card';
    card.id = `card-${devId}-${sig}`;
    card.dataset.signal = sig;
    card.innerHTML = `
      <div class="card-top">
        <div class="card-label">${meta.label}</div>
        <div class="card-icon">${meta.icon}</div>
      </div>
      <div>
        <span class="card-value" id="val-${devId}-${sig}">—</span>
        <span class="card-unit">${meta.unit}</span>
      </div>
      <div class="card-time" id="time-${devId}-${sig}">waiting…</div>
      <canvas class="card-spark" id="spark-${devId}-${sig}" width="220" height="32"></canvas>
    `;
    grid.appendChild(card);
    sparkData[`${devId}_${sig}`] = [];
  }

  // Add frame log section once
  if (!document.getElementById('log-section')) {
    const logSection = document.createElement('div');
    logSection.className = 'log-section';
    logSection.id = 'log-section';
    logSection.innerHTML = `
      <div class="log-header">Recent Frames <span class="count" id="logCount"></span></div>
      <div class="log-wrap">
        <table class="log-table">
          <thead><tr>
            <th>Time</th><th>Arb ID</th><th>Sender</th><th>Action</th>
            <th>Signal</th><th>Dev</th><th>Value</th>
          </tr></thead>
          <tbody id="logBody"></tbody>
        </table>
      </div>
    `;
    main.appendChild(logSection);
  }
}

// ---------------------------------------------------------------------------
// Update a signal card
// ---------------------------------------------------------------------------
function updateCard(devId, sig, value, t) {
  const meta = SIGNAL_META[sig];
  if (!meta) return;

  ensureESCSection(parseInt(devId));

  const valEl = document.getElementById(`val-${devId}-${sig}`);
  const timeEl = document.getElementById(`time-${devId}-${sig}`);
  const card = document.getElementById(`card-${devId}-${sig}`);

  if (!valEl) return;

  const formatted = (meta.decimals > 0)
    ? parseFloat(value).toFixed(meta.decimals)
    : Math.round(value).toString();

  valEl.textContent = formatted;
  timeEl.textContent = `t = ${parseFloat(t).toFixed(2)}s`;

  // Flash animation
  card.classList.remove('flash');
  void card.offsetWidth;  // reflow
  card.classList.add('flash');

  // Sparkline data
  const key = `${devId}_${sig}`;
  if (!sparkData[key]) sparkData[key] = [];
  sparkData[key].push(parseFloat(value));
  if (sparkData[key].length > SPARK_MAX) sparkData[key].shift();
  drawSparkline(devId, sig);
}

// ---------------------------------------------------------------------------
// Sparkline renderer
// ---------------------------------------------------------------------------
function drawSparkline(devId, sig) {
  const canvas = document.getElementById(`spark-${devId}-${sig}`);
  if (!canvas) return;
  const ctx = canvas.getContext('2d');
  const key = `${devId}_${sig}`;
  const data = sparkData[key] || [];
  if (data.length < 2) return;

  const W = canvas.width;
  const H = canvas.height;
  ctx.clearRect(0, 0, W, H);

  const min = Math.min(...data);
  const max = Math.max(...data);
  const range = max - min || 1;

  // Get accent color from CSS
  const style = getComputedStyle(canvas.closest('.signal-card'));
  const accent = style.getPropertyValue('--card-accent').trim() || '#58a6ff';

  // Gradient fill
  const grad = ctx.createLinearGradient(0, 0, 0, H);
  grad.addColorStop(0, accent.replace(')', ',0.25)').replace('rgb', 'rgba'));
  grad.addColorStop(1, 'transparent');

  ctx.beginPath();
  ctx.moveTo(0, H);
  for (let i = 0; i < data.length; i++) {
    const x = (i / (data.length - 1)) * W;
    const y = H - ((data[i] - min) / range) * (H - 4) - 2;
    ctx.lineTo(x, y);
  }
  ctx.lineTo(W, H);
  ctx.closePath();
  ctx.fillStyle = grad;
  ctx.fill();

  // Line
  ctx.beginPath();
  for (let i = 0; i < data.length; i++) {
    const x = (i / (data.length - 1)) * W;
    const y = H - ((data[i] - min) / range) * (H - 4) - 2;
    if (i === 0) ctx.moveTo(x, y);
    else ctx.lineTo(x, y);
  }
  ctx.strokeStyle = accent;
  ctx.lineWidth = 1.5;
  ctx.stroke();

  // Dot at latest value
  const lastX = W;
  const lastY = H - ((data[data.length-1] - min) / range) * (H - 4) - 2;
  ctx.beginPath();
  ctx.arc(lastX - 1, lastY, 2.5, 0, Math.PI * 2);
  ctx.fillStyle = accent;
  ctx.fill();
}

// ---------------------------------------------------------------------------
// Frame log (latest 30 frames)
// ---------------------------------------------------------------------------
const logEntries = [];
const LOG_MAX = 30;

function addLogEntry(devId, sig, value, t) {
  logEntries.unshift({devId, sig, value, t});
  if (logEntries.length > LOG_MAX) logEntries.pop();

  const body = document.getElementById('logBody');
  if (!body) return;

  // Rebuild (simple and fast enough for 30 rows)
  body.innerHTML = logEntries.map(e => `
    <tr>
      <td>${parseFloat(e.t).toFixed(3)}s</td>
      <td class="arb-id">—</td>
      <td>SLAVE</td>
      <td>READ</td>
      <td>${e.sig}</td>
      <td>${e.devId}</td>
      <td>${parseFloat(e.value).toFixed(3)}</td>
    </tr>
  `).join('');

  const countEl = document.getElementById('logCount');
  if (countEl) countEl.textContent = `(${logEntries.length})`;
}

// ---------------------------------------------------------------------------
// SSE connection
// ---------------------------------------------------------------------------
function connectSSE() {
  const dot = document.getElementById('statusDot');
  const txt = document.getElementById('statusText');
  const es = new EventSource('/api/stream');

  es.onopen = () => {
    dot.classList.remove('disconnected');
    txt.textContent = 'Live';
  };

  es.onmessage = (ev) => {
    const data = JSON.parse(ev.data);
    if (!data || Object.keys(data).length === 0) return;

    for (const [devId, signals] of Object.entries(data)) {
      for (const [sig, info] of Object.entries(signals)) {
        updateCard(devId, sig, info.value, info.t);
        addLogEntry(devId, sig, info.value, info.t);
        totalFrames++;
      }
    }

    // Update topbar stats
    document.getElementById('statFrames').textContent = totalFrames.toLocaleString();
  };

  es.onerror = () => {
    dot.classList.add('disconnected');
    txt.textContent = 'Reconnecting…';
    es.close();
    setTimeout(connectSSE, 2000);
  };
}

// ---------------------------------------------------------------------------
// Stats polling & uptime
// ---------------------------------------------------------------------------
async function pollStats() {
  try {
    const res = await fetch('/api/stats');
    const s = await res.json();
    document.getElementById('statFrames').textContent = s.total_frames.toLocaleString();
    document.getElementById('statFps').textContent = s.fps;
    document.getElementById('statUptime').textContent = s.duration_s + 's';
  } catch {}
}

// Load initial data, then connect SSE
async function init() {
  // Fetch current state
  try {
    const res = await fetch('/api/latest');
    const data = await res.json();
    for (const [devId, signals] of Object.entries(data)) {
      for (const [sig, info] of Object.entries(signals)) {
        updateCard(devId, sig, info.value, info.timestamp_s);
      }
    }
  } catch {}

  await pollStats();
  connectSSE();

  // Refresh stats every 5s
  setInterval(pollStats, 5000);
}

init();
</script>
</body>
</html>
"""



# Entry point
def main():
    global DB_PATH, POLL_INTERVAL

    parser = argparse.ArgumentParser(description="Live CAN FD telemetry dashboard")
    parser.add_argument("--db", default="can_log.db",
                        help="SQLite database path (must match can_logger.py)")
    parser.add_argument("--port", type=int, default=5000,
                        help="HTTP port (default: 5000)")
    parser.add_argument("--host", default="127.0.0.1",
                        help="Bind address (default: 127.0.0.1)")
    parser.add_argument("--poll", type=float, default=0.20,
                        help="SSE poll interval in seconds (default: 0.20)")
    args = parser.parse_args()

    DB_PATH = args.db
    POLL_INTERVAL = args.poll

    if not Path(DB_PATH).exists():
        print(f"WARNING: Database '{DB_PATH}' does not exist yet.")
        print(f"  Start can_logger.py first, or the dashboard will show no data.")
        print()

    print(f"CAN FD Dashboard")
    print(f"  Database : {DB_PATH}")
    print(f"  URL      : http://{args.host}:{args.port}")
    print(f"  SSE poll : {POLL_INTERVAL}s")
    print()

    app.run(host=args.host, port=args.port, debug=False, threaded=True)


if __name__ == "__main__":
    main()

# # Terminal 1 — capture from bus
# python scripts/can_logger.py --port COM4 --db can_log.db

# # Terminal 2 — serve dashboard
# python scripts/can_dashboard.py --db can_log.db