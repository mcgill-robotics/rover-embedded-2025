if [ -d "./.venv/bin" ]; then
  source .venv/bin/activate
fi
# Standalone camera server — own process/port, never touches the dashboard GIL.
# Needs GStreamer bindings visible to this interpreter (venv created with
# --system-site-packages). --host/--port and camera flags pass through via $@.
python3 scripts/camera_server.py --port 5001 "$@"
