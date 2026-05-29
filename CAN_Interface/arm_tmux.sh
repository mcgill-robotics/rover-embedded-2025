TERM=xterm
tmux new-session -d -s logger "python3 scripts/can_logger.py --port /dev/ttyACM0 --db can_log.db"
tmux split-window -h
tmux send-keys -t 1 "python3 scripts/can_dashboard.py --db can_log.db" C-m
tmux attach -t logger
