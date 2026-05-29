TERM=xterm
tmux new-session -L -d -s logger "./start_dashboard.sh"
tmux split-window -h
tmux send-keys -t 1 "./start_logger.sh" C-m
tmux attach -t logger
