if [ -d "./.venv/bin" ]; then
  source .venv/bin/activate
fi
if [ -n "$1" ]; then
    python3 scripts/can_logger.py --port "$1" --db can_log.db
else
    python3 scripts/can_logger.py --port /dev/ttyACM0 --db can_log.db
fi
