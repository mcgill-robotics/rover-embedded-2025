if [ -d "./.venv/bin" ]; then
  source .venv/bin/activate
fi
python3 scripts/can_dashboard.py --db can_log.db
