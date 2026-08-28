"""
camera_server.py — Standalone webcam MJPEG server (separate from the dashboard)

Runs the camera pipeline in its OWN process on its OWN port, so its GStreamer
capture/encode work and its HTTP streaming never compete for the CAN
dashboard's GIL. This is what removes the "camera makes the dashboard laggy
over time" problem: the control app (can_dashboard.py) and the video app
(this file) no longer share a Python process.

It reuses the exact CameraStreamer class from can_dashboard.py, unchanged, so
there is nothing new to debug on the camera side.

Endpoints (mirror the dashboard's camera API so the dashboard JS can point
straight at them):
    GET /                       tiny standalone viewer (handy for testing)
    GET /api/camera/status      JSON status
    GET /api/camera/stream      multipart MJPEG stream (lazy-starts pipeline)
    GET /api/camera/snapshot    single JPEG

CORS is wide open (Access-Control-Allow-Origin: *) so the dashboard served
from a different port can fetch /api/camera/status without a cross-origin
error. The MJPEG <img> itself doesn't need CORS.

Usage (on the Pi):
    pip install flask waitress          # waitress optional but recommended
    python scripts/camera_server.py --port 5001 \
        --camera-device /dev/video0 --camera-width 1280 \
        --camera-height 720 --camera-fps 30

Then forward BOTH ports over SSH from your laptop:
    ssh -L 5000:localhost:5000 -L 5001:localhost:5001 pi@<pi-address>

Run can_dashboard.py with --no-camera and set CAM_BASE in its HTML to
'http://localhost:5001' (see the notes that came with this file).
"""

from __future__ import annotations

import argparse
import sys
import time
import threading

from flask import Flask, Response, jsonify

app = Flask(__name__)

_camera = None  # CameraStreamer instance — created in main()


# ---------------------------------------------------------------------------
# CameraStreamer — copied verbatim from can_dashboard.py (same tested code)
# ---------------------------------------------------------------------------
class CameraStreamer:
    def __init__(self, device: str = "/dev/video0",
                 width: int = 1280, height: int = 720, fps: int = 30,
                 jpeg_quality: int = 80,
                 idle_timeout_s: float = 10.0,
                 startup_timeout_s: float = 4.0) -> None:
        self.device = device
        self.width = width
        self.height = height
        self.fps = fps
        self.jpeg_quality = jpeg_quality
        self.idle_timeout_s = idle_timeout_s
        self.startup_timeout_s = startup_timeout_s

        self._cond = threading.Condition()
        self._frame = None          # latest JPEG bytes
        self._seq = 0               # frame sequence number
        self._running = False
        self._error = None
        self._mode = None           # "mjpeg-passthrough" | "sw-encode"
        self._viewers = 0
        self._frames = 0
        self._t_start = None
        self._pipeline = None
        self._sink = None
        self._cap_thread = None
        self._idle_timer = None
        self._state_lock = threading.Lock()

    # -- public API ---------------------------------------------------------

    @property
    def running(self) -> bool:
        return self._running

    def status(self) -> dict:
        with self._cond:
            measured = 0.0
            if self._running and self._t_start:
                dt = time.monotonic() - self._t_start
                measured = self._frames / dt if dt > 0.5 else 0.0
            return {
                "available": True,
                "running": self._running,
                "mode": self._mode,
                "device": self.device,
                "resolution": f"{self.width}x{self.height}",
                "fps_target": self.fps,
                "fps_measured": round(measured, 1),
                "viewers": self._viewers,
                "frames": self._frames,
                "error": self._error,
            }

    def ensure_started(self) -> tuple:
        """Start the pipeline if idle; block until the first frame or
        timeout.  Returns (ok, error_message)."""
        with self._state_lock:
            if self._running:
                return True, None
            self._error = None
            ok, err = self._start_locked()
            if not ok:
                self._error = err
                return False, err
        # Wait for the first frame (spin-up delay)
        with self._cond:
            deadline = time.monotonic() + self.startup_timeout_s
            while self._seq == 0 and self._running:
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    break
                self._cond.wait(remaining)
            if self._seq > 0:
                return True, None
        self.stop()
        err = ("Camera started but produced no frames within "
               f"{self.startup_timeout_s:.0f}s")
        self._error = err
        return False, err

    def stop(self) -> None:
        with self._state_lock:
            self._running = False
            if self._cap_thread:
                self._cap_thread.join(timeout=2.0)
                self._cap_thread = None
            if self._pipeline is not None:
                try:
                    Gst = self._gst()
                    self._pipeline.set_state(Gst.State.NULL)
                except Exception:
                    pass
                self._pipeline = None
                self._sink = None
        with self._cond:
            self._cond.notify_all()
        print("[camera] pipeline stopped")

    def add_viewer(self) -> None:
        with self._cond:
            self._viewers += 1
            if self._idle_timer:
                self._idle_timer.cancel()
                self._idle_timer = None

    def remove_viewer(self) -> None:
        with self._cond:
            self._viewers = max(0, self._viewers - 1)
            if self._viewers == 0 and self._running:
                if self._idle_timer:
                    self._idle_timer.cancel()
                self._idle_timer = threading.Timer(
                    self.idle_timeout_s, self._idle_stop)
                self._idle_timer.daemon = True
                self._idle_timer.start()

    def wait_frame(self, last_seq: int, timeout: float = 5.0) -> tuple:
        """Block until a frame newer than last_seq exists.
        Returns (jpeg_bytes, seq) or (None, last_seq) on timeout/stop."""
        with self._cond:
            deadline = time.monotonic() + timeout
            while self._running and self._seq <= last_seq:
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    return None, last_seq
                self._cond.wait(remaining)
            if not self._running or self._frame is None:
                return None, last_seq
            return self._frame, self._seq

    # -- internals ----------------------------------------------------------

    def _gst(self):
        import gi
        gi.require_version("Gst", "1.0")
        from gi.repository import Gst
        if not Gst.is_initialized():
            Gst.init(None)
        return Gst

    def _pipeline_desc(self, passthrough: bool) -> str:
        common = (f"appsink name=sink drop=true max-buffers=1 "
                  f"sync=false emit-signals=false")
        if passthrough:
            return (f"v4l2src device={self.device} ! "
                    f"image/jpeg,width={self.width},height={self.height},"
                    f"framerate={self.fps}/1 ! {common}")
        return (f"v4l2src device={self.device} ! videoconvert ! "
                f"video/x-raw,width={self.width},height={self.height},"
                f"framerate={self.fps}/1 ! "
                f"jpegenc quality={self.jpeg_quality} ! {common}")

    def _open_any_pipeline(self):
        """Try MJPEG passthrough, fall back to software encode.
        Returns (pipeline, sink, mode) or raises RuntimeError."""
        Gst = self._gst()
        last_err = None
        for passthrough, mode in ((True, "mjpeg-passthrough"),
                                  (False, "sw-encode")):
            desc = self._pipeline_desc(passthrough)
            try:
                pipeline = Gst.parse_launch(desc)
            except Exception as e:
                last_err = f"parse failed: {e}"
                continue
            sink = pipeline.get_by_name("sink")
            ret = pipeline.set_state(Gst.State.PLAYING)
            if ret == Gst.StateChangeReturn.FAILURE:
                pipeline.set_state(Gst.State.NULL)
                last_err = f"{mode}: pipeline refused to start"
                continue
            # Confirm it actually delivers a frame in this format
            sample = sink.emit("try-pull-sample", 2 * Gst.SECOND)
            if sample is None:
                pipeline.set_state(Gst.State.NULL)
                last_err = f"{mode}: no frames (unsupported caps?)"
                continue
            # Push the probe frame into the buffer so ensure_started
            # returns immediately
            self._store_frame(self._sample_bytes(Gst, sample))
            return pipeline, sink, mode
        raise RuntimeError(last_err or "no pipeline could be built")

    @staticmethod
    def _sample_bytes(Gst, sample) -> bytes:
        buf = sample.get_buffer()
        ok, mapinfo = buf.map(Gst.MapFlags.READ)
        if not ok:
            return b""
        try:
            return bytes(mapinfo.data)
        finally:
            buf.unmap(mapinfo)

    def _pull_frame(self) -> bytes:
        """Pull one JPEG frame from the sink (blocking up to 100 ms).
        Returns b'' when no frame was available."""
        Gst = self._gst()
        sample = self._sink.emit("try-pull-sample", 100 * Gst.MSECOND)
        if sample is None:
            return b""
        return self._sample_bytes(Gst, sample)

    def _store_frame(self, data: bytes) -> None:
        if not data:
            return
        with self._cond:
            self._frame = data
            self._seq += 1
            self._frames += 1
            self._cond.notify_all()

    def _start_locked(self) -> tuple:
        try:
            self._gst()
        except ImportError:
            return False, (
                f"Python 'gi' bindings not visible to this interpreter "
                f"({sys.executable}). Install via apt, NOT pip: "
                f"sudo apt install python3-gi python3-gst-1.0 "
                f"gstreamer1.0-plugins-good. If this runs in a virtualenv, "
                f"recreate it with --system-site-packages — pip's PyGObject "
                f"does not include the GStreamer typelibs.")
        except ValueError as e:
            return False, (
                f"'gi' is installed but the GStreamer typelib is missing "
                f"({e}). Run: sudo apt install python3-gst-1.0 "
                f"gir1.2-gstreamer-1.0 gstreamer1.0-plugins-good")
        try:
            self._pipeline, self._sink, self._mode = self._open_any_pipeline()
        except Exception as e:
            return False, f"Could not open camera {self.device}: {e}"

        self._running = True
        self._frames = 0
        self._t_start = time.monotonic()
        self._cap_thread = threading.Thread(
            target=self._capture_loop, name="CameraCapture", daemon=True)
        self._cap_thread.start()
        print(f"[camera] pipeline started ({self._mode}, "
              f"{self.width}x{self.height}@{self.fps})")
        return True, None

    def _capture_loop(self) -> None:
        try:
            while self._running:
                self._store_frame(self._pull_frame())
        except Exception as e:
            print(f"[camera] capture error: {e}")
            with self._cond:
                self._error = f"capture error: {e}"
                self._running = False
                self._cond.notify_all()

    def _idle_stop(self) -> None:
        with self._cond:
            if self._viewers > 0 or not self._running:
                return
        print(f"[camera] no viewers for {self.idle_timeout_s:.0f}s — "
              f"stopping pipeline")
        self.stop()


# ---------------------------------------------------------------------------
# CORS — allow the dashboard (different port) to read /api/camera/status
# ---------------------------------------------------------------------------
@app.after_request
def _add_cors(resp):
    resp.headers["Access-Control-Allow-Origin"] = "*"
    resp.headers["Cache-Control"] = "no-cache"
    return resp


# ---------------------------------------------------------------------------
# Endpoints — same paths as the dashboard's camera API
# ---------------------------------------------------------------------------
@app.route("/api/camera/status")
def api_camera_status():
    if _camera is None:
        return jsonify({"available": False})
    return jsonify(_camera.status())


@app.route("/api/camera/stream")
def api_camera_stream():
    """Multipart MJPEG stream.  Lazily starts the pipeline on the first
    viewer; the pipeline stops itself after the last viewer leaves."""
    if _camera is None:
        return jsonify({"error": "Camera disabled"}), 503
    ok, err = _camera.ensure_started()
    if not ok:
        return jsonify({"error": err}), 503

    def gen():
        _camera.add_viewer()
        last = 0
        try:
            while True:
                frame, last = _camera.wait_frame(last, timeout=5.0)
                if frame is None:
                    if not _camera.running:
                        break
                    continue
                yield (b"--frame\r\n"
                       b"Content-Type: image/jpeg\r\n"
                       + f"Content-Length: {len(frame)}\r\n\r\n".encode()
                       + frame + b"\r\n")
        finally:
            _camera.remove_viewer()

    return Response(gen(),
                    mimetype="multipart/x-mixed-replace; boundary=frame")


@app.route("/api/camera/snapshot")
def api_camera_snapshot():
    """Single JPEG frame (also lazy-starts, then idles back out)."""
    if _camera is None:
        return jsonify({"error": "Camera disabled"}), 503
    ok, err = _camera.ensure_started()
    if not ok:
        return jsonify({"error": err}), 503
    _camera.add_viewer()
    try:
        frame, _ = _camera.wait_frame(0, timeout=3.0)
    finally:
        _camera.remove_viewer()
    if frame is None:
        return jsonify({"error": "No frame available"}), 503
    return Response(frame, mimetype="image/jpeg")


@app.route("/")
def index():
    """Minimal standalone viewer — handy for testing the camera alone."""
    return """<!DOCTYPE html>
<html><head><meta charset="utf-8"><title>Camera Server</title>
<style>
  body { background:#0a0e14; color:#e6edf3; font-family:sans-serif;
         margin:0; padding:24px; text-align:center; }
  img { max-width:100%; border:1px solid #1e2a3a; border-radius:8px; }
  #info { margin-top:12px; font-size:14px; color:#8b98a9; }
</style></head><body>
<h2>Standalone Camera Server</h2>
<img id="cam" src="/api/camera/stream" alt="camera stream">
<div id="info">connecting…</div>
<script>
  async function poll() {
    try {
      const s = await (await fetch('/api/camera/status')).json();
      document.getElementById('info').textContent = s.running
        ? `${s.device} · ${s.resolution} · ${s.fps_measured} fps `
          + `(${s.mode}) · viewers: ${s.viewers}`
        : (s.error ? '✗ ' + s.error : 'idle');
    } catch {}
  }
  poll(); setInterval(poll, 2000);
</script>
</body></html>"""


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------
def main():
    global _camera

    parser = argparse.ArgumentParser(
        description="Standalone webcam MJPEG server (independent of the "
                    "CAN dashboard)")
    parser.add_argument("--host", default="127.0.0.1",
                        help="Bind address (default: 127.0.0.1)")
    parser.add_argument("--port", type=int, default=5001,
                        help="HTTP port (default: 5001)")
    parser.add_argument("--camera-device", default="/dev/video0",
                        help="V4L2 device (default: /dev/video0)")
    parser.add_argument("--camera-width", type=int, default=1280,
                        help="Capture width (default: 1280)")
    parser.add_argument("--camera-height", type=int, default=720,
                        help="Capture height (default: 720)")
    parser.add_argument("--camera-fps", type=int, default=30,
                        help="Capture framerate (default: 30)")
    parser.add_argument("--camera-idle-timeout", type=float, default=10.0,
                        help="Seconds after last viewer before the pipeline "
                             "stops (default: 10)")
    parser.add_argument("--threads", type=int, default=8,
                        help="waitress thread count (default: 8)")
    args = parser.parse_args()

    _camera = CameraStreamer(
        device=args.camera_device,
        width=args.camera_width,
        height=args.camera_height,
        fps=args.camera_fps,
        idle_timeout_s=args.camera_idle_timeout,
    )

    print("Standalone Camera Server")
    print(f"  URL     : http://{args.host}:{args.port}")
    print(f"  Camera  : {args.camera_device} "
          f"{args.camera_width}x{args.camera_height}@{args.camera_fps} "
          f"(lazy — starts on first viewer)")
    print()

    try:
        from waitress import serve
        print(f"  Server  : waitress (threads={args.threads})")
        print()
        serve(app, host=args.host, port=args.port,
              threads=args.threads, channel_timeout=300)
    except ImportError:
        print("  Server  : flask dev server "
              "(pip install waitress for a bounded thread pool)")
        print()
        app.run(host=args.host, port=args.port, debug=False, threaded=True)


if __name__ == "__main__":
    main()
