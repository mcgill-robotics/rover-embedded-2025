#!/usr/bin/env bash
# ============================================================
#  launch_arm.sh — Arm test launcher (Linux/macOS)
#  Edit the variables below to match your setup.
# ============================================================

PORT="COM4"          # e.g. /dev/ttyUSB0 on Linux, /dev/tty.usbmodem* on macOS
DB="can_log.db"
SHOULDER_SPEED="0.5"
ELBOW_SPEED="0.6"
WAIST_SPEED="0.4"

# --- Detect terminal emulator ---
if command -v gnome-terminal &> /dev/null; then
    TERM_CMD="gnome-terminal --title"
    run_in_term() { gnome-terminal --title="$1" -- bash -c "$2; exec bash"; }
elif command -v xterm &> /dev/null; then
    run_in_term() { xterm -T "$1" -e bash -c "$2; exec bash" &; }
elif command -v osascript &> /dev/null; then
    # macOS — open a new Terminal tab
    run_in_term() {
        osascript -e "tell application \"Terminal\" to do script \"$2\"" \
                  -e "tell application \"Terminal\" to set custom title of front window to \"$1\""
    }
else
    echo "No supported terminal emulator found (gnome-terminal, xterm, or macOS Terminal)."
    exit 1
fi

# --- Terminal 1: CAN logger ---
echo "Starting CAN Logger on port $PORT..."
run_in_term "CAN Logger" "python scripts/can_logger.py --port $PORT --db $DB"

# Give the logger a moment to bind before the dashboard connects
sleep 2

# --- Terminal 2: Dashboard ---
echo "Starting CAN Dashboard..."
run_in_term "CAN Dashboard" "python scripts/can_dashboard.py --db $DB"

# --- Optional Terminal 3: PS4 controller ---
read -rp "Launch PS4 controller? (y/n): " USE_CONTROLLER
if [[ "$USE_CONTROLLER" =~ ^[Yy]$ ]]; then
    echo "Starting PS4 controller..."
    run_in_term "PS4 Controller" "python scripts/ps4_arm_controller.py --shoulder-speed $SHOULDER_SPEED --elbow-speed $ELBOW_SPEED --waist-speed $WAIST_SPEED"
fi

echo ""
echo "All processes launched."
