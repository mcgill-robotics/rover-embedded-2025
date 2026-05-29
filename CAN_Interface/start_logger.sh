if [ -d "./.venv/bin" ]; then
  source .venv/bin/activate
fi
if [ -v $0 ]; then
    python3 scripts/can_logger.py --port $0 --db can_log.db
else
	python3 scripts/can_logger.py --port /dev/ttyACM0 --db can_log.db
fi
