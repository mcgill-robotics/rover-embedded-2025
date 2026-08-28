#!/usr/bin/env bash
# ============================================================
#  launch_arm.sh — Arm test launcher (Raspberry Pi / Linux / macOS)
#  Every process is started with the project venv's python by
#  ABSOLUTE PATH, so it works regardless of shell activation.
#
#  Launches THREE processes, each in its own terminal:
#    1. CAN logger      — owns the serial port + CmdServer
#    2. Camera server    — standalone, port 5001 (own process/GIL)
#    3. CAN dashboard     — GUI on port 5000 (camera-free)
# ============================================================

# --- Resolve project root (directory containing this script) ---
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# --- Edit these to match your setup ---
PORT="/dev/ttyACM0"      # CANable on the Pi (was COM4 on Windows)
DB="can_log.db"
CAM_PORT=5001            # must match CAM_PORT in the dashboard HTML
VENV_PY="$SCRIPT_DIR/venv/bin/python"

# --- Sanity checks before launching anything ---
if [[ ! -x "$VENV_PY" ]]; then
    echo "ERROR: venv python not found at $VENV_PY"
    echo "Create it with:  python3 -m venv --system-site-packages venv"
    exit 1
fi

if ! "$VENV_PY" -c "import flask, can, serial" 2>/dev/null; then
    echo "ERROR: venv is missing dependencies. Run:"
    echo "  $SCRIPT_DIR/venv/bin/pip install flask python-can pyserial"
    echo "  (optional, recommended:  $SCRIPT_DIR/venv/bin/pip install waitress)"
    exit 1
fi

if ! "$VENV_PY" -c "import gi; gi.require_version('Gst','1.0'); from gi.repository import Gst" 2>/dev/null; then
    echo "WARNING: GStreamer bindings not visible to the venv — the camera"
    echo "server will fail to start (logging + dashboard are unaffected). Fix with:"
    echo "  sudo apt install python3-gi python3-gst-1.0 gir1.2-gstreamer-1.0 gstreamer1.0-plugins-good"
    echo "  (and ensure the venv was created with --system-site-packages)"
    echo "Continuing anyway — CAN logging and the GUI will still run."
fi

# --- Detect terminal emulator ---
if command -v lxterminal &> /dev/null; then
    # Raspberry Pi OS default desktop terminal
    run_in_term() { lxterminal --title="$1" -e bash -c "$2; exec bash" & }
elif command -v gnome-terminal &> /dev/null; then
    run_in_term() { gnome-terminal --title="$1" -- bash -c "$2; exec bash"; }
elif command -v xterm &> /dev/null; then
    run_in_term() { xterm -T "$1" -e bash -c "$2; exec bash" & }
elif command -v osascript &> /dev/null; then
    # macOS — open a new Terminal window
    run_in_term() {
        osascript -e "tell application \"Terminal\" to do script \"cd $SCRIPT_DIR; $2\"" \
                  -e "tell application \"Terminal\" to set custom title of front window to \"$1\""
    }
else
    echo "No supported terminal emulator found."
    echo "Run the commands manually instead:"
    echo "  $VENV_PY scripts/can_logger.py --port $PORT --db $DB"
    echo "  $VENV_PY scripts/camera_server.py --port $CAM_PORT"
    echo "  $VENV_PY scripts/can_dashboard.py --db $DB"
    exit 1
fi

# --- Terminal 1: CAN logger (owns the serial port) ---
echo "Starting CAN Logger on port $PORT..."
run_in_term "CAN Logger" "cd '$SCRIPT_DIR' && '$VENV_PY' scripts/can_logger.py --port $PORT --db $DB"

# Give the logger a moment to bind its CmdServer before the others start
sleep 2

# --- Terminal 2: Camera server (separate process, own port) ---
echo "Starting Camera Server on port $CAM_PORT..."
run_in_term "Camera Server" "cd '$SCRIPT_DIR' && '$VENV_PY' scripts/camera_server.py --port $CAM_PORT"

# --- Terminal 3: Dashboard (camera-free GUI) ---
echo "Starting CAN Dashboard..."
run_in_term "CAN Dashboard" "cd '$SCRIPT_DIR' && '$VENV_PY' scripts/can_dashboard.py --db $DB"

echo ""
echo "All processes launched."
echo "Dashboard:  http://localhost:5000   (camera panel pulls from :$CAM_PORT)"
echo "Camera:     http://localhost:$CAM_PORT   (standalone test viewer)"
echo "From your laptop:  ssh -L 5000:localhost:5000 -L $CAM_PORT:localhost:$CAM_PORT pi@<pi-ip>"
echo "                   then open http://localhost:5000"
