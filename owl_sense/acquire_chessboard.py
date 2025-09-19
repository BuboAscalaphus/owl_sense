#!/usr/bin/env python3
"""
Simple shooter for AR0822 via PyVizionSDK:
- Starts streaming
- Takes one photo every N seconds with a console countdown
- Saves JPEGs under a directory named after the camera (camera_name/)

Usage:
  python timed_shooter.py --camera-name MyCam

Optional args:
  --interval 2           Seconds between shots (default 2)
  --resolution 4K        Resolution name for the SDK (default "4K")
  --fps 12               Stream fps for the SDK (default 12)
  --fps-format 15        SDK fps format (default 15)
  --exposure-mode 1      SDK exposure mode (default 1)
  --exposure-time 5000   Exposure time in microseconds (default 5000)
  --jpeg-quality 205     SDK JPEG quality (default 205)
  --camera-id -1         SDK camera id (default -1)
"""

import argparse
import os
import sys
import time
from datetime import datetime
from pathlib import Path

from owl_sense.camera_ar082_generic import CameraAR082x_generic


DEFAULTS = dict(
    resolution="4K",
    fps=12,
    fps_format=15,
    exposure_mode=1,
    exposure_time=5000,       # us
    jpeg_quality=205,
    camera_id=-1,
    interval=2,
)


def parse_args():
    p = argparse.ArgumentParser(description="Timed shooter for AR0822 via PyVizionSDK")
    p.add_argument("--resolution", default=DEFAULTS["resolution"])
    p.add_argument("--fps", type=int, default=DEFAULTS["fps"])
    p.add_argument("--fps-format", type=int, default=DEFAULTS["fps_format"])
    p.add_argument("--exposure-mode", type=int, default=DEFAULTS["exposure_mode"])
    p.add_argument("--exposure-time", type=int, default=DEFAULTS["exposure_time"])
    p.add_argument("--jpeg-quality", type=int, default=DEFAULTS["jpeg_quality"])
    p.add_argument("--interval", type=float, default=DEFAULTS["interval"], help="Seconds between shots")
    return p.parse_args()


def ensure_outdir(camera_name: str) -> Path:
    out = Path(camera_name)
    out.mkdir(parents=True, exist_ok=True)
    return out


def timestamp_filename(prefix="img", ext=".jpg") -> str:
    # e.g., img_2025-09-11_14-03-27.123.jpg
    now = datetime.now()
    return f"{prefix}_{now.strftime('%Y-%m-%d_%H-%M-%S')}.{int(now.microsecond/1000):03d}{ext}"


def main():
    args = parse_args()

    cam = None
    try:
        cam = CameraAR082x_generic()
        
        cam.set_streaming_parameters(
            fps=args.fps,
            fps_format=args.fps_format,
            jpeg_quality=args.jpeg_quality,
            resolution_name=args.resolution,
            exposure_time=args.exposure_time,
            exposure_mode=args.exposure_mode
        )
        cam.configure()
        print(cam.camera_name)
        outdir = ensure_outdir(cam.camera_name)

        print(
            f"[INFO] Starting camera stream\n"
            f"       Camera: name='{cam.camera_name}'\n"
            f"       Res={args.resolution}, FPS={args.fps} (fmt {args.fps_format}), "
            f"Exposure(mode={args.exposure_mode}, time_us={args.exposure_time}), JPEG Q={args.jpeg_quality}\n"
            f"       Shooting every {args.interval:.1f}s. Saving to: {outdir.resolve()}\n"
            f"       Press Ctrl-C to stop.\n"
        )
        
        cam.start_streaming()

        # timeout slightly larger than 1/fps to avoid premature timeouts
        timeout_ms = int(1000 / max(args.fps, 1)) + 200

        while True:
            # Countdown in the console
            whole = int(max(1, round(args.interval)))
            start_count = whole
            start_t = time.time()
            for sec in range(start_count, 0, -1):
                # Print on one line and flush
                print(f"\r[COUNTDOWN] {sec}...", end="", flush=True)
                time.sleep(1)
            # Align with desired interval (handles non-integer intervals)
            elapsed = time.time() - start_t
            if elapsed < args.interval:
                time.sleep(args.interval - elapsed)

            print("\r[SNAP] Capturing frame...        ", end="", flush=True)
            t0 = time.time()
            code, img = cam.acquire(timeout_ms)
            t1 = time.time()

            if img is None:
                print(f"\r[WARN] Capture failed (code={code}). Retrying...", flush=True)
                continue

            # The SDK delivers a compressed JPEG buffer; persist as-is
            try:
                img_bytes = img.tobytes() if hasattr(img, "tobytes") else bytes(img)
            except Exception:
                # Fall back: treat as bytes-like
                img_bytes = bytes(img)

            fname = timestamp_filename(prefix=cam.camera_name, ext=".jpg")
            fpath = outdir / fname
            with open(fpath, "wb") as f:
                f.write(img_bytes)

            size_kb = len(img_bytes) / 1024.0
            print(
                f"\r[SAVED] {fpath} | {size_kb:.1f} KB | acquire {(t1 - t0)*1000:.1f} ms",
                flush=True,
            )

    except KeyboardInterrupt:
        print("\n[INFO] Stopping (user interrupt)...")
    except Exception as ex:
        print(f"\n[ERROR] {ex}", file=sys.stderr)
    finally:
        if cam is not None:
            try:
                cam.stop_streaming()
            except Exception:
                pass
            try:
                cam.close()
            except Exception:
                pass
        print("[INFO] Done.")


if __name__ == "__main__":
    main()
