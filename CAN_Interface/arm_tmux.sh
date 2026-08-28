#!/usr/bin/env bash
# ============================================================
#  arm_tmux.sh — launch the whole stack in one tmux session:
#    pane 0: CAN logger      (owns the serial port + CmdServer)
#    pane 1: camera server   (own process/port 5001)
#    pane 2: CAN dashboard    (reads DB, relays commands)
#  All three bind 0.0.0.0 so you can reach them over the LAN,
#  or forward ports 5000 + 5001 over SSH.
# ============================================================
TERM=xterm
SESSION=arm

# Start fresh if a previous session is still around.
tmux kill-session -t "$SESSION" 2>/dev/null

# Keep dead panes visible so a crash/error message doesn't vanish.
tmux new-session -d -s "$SESSION" -n main
tmux set-option -t "$SESSION" remain-on-exit on

# Pane 0: logger first — it owns the serial port and the CmdServer.
tmux send-keys -t "$SESSION":0.0 "./start_logger.sh" C-m

# Pane 1: camera server (separate process; 5001).
tmux split-window -h -t "$SESSION":0.0
tmux send-keys -t "$SESSION":0.1 "./start_camera.sh --host 0.0.0.0" C-m

# Pane 2: dashboard — give the logger ~2s to bind its CmdServer first.
tmux split-window -v -t "$SESSION":0.1
tmux send-keys -t "$SESSION":0.2 "sleep 2 && ./start_dashboard.sh --host 0.0.0.0" C-m

tmux select-layout -t "$SESSION" tiled
tmux attach -t "$SESSION"

# Detach with:  Ctrl-b then d      Kill everything with:  tmux kill-session -t arm
