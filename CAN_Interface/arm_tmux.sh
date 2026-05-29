TERM=xterm
tmux new-session -d -s logger "./start_dashboard.sh --host 0.0.0.0"
tmux split-window -h
tmux send-keys -t 1 "./start_logger.sh" C-m
tmux attach -t logger
