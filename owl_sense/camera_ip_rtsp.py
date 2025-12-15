import time
from typing import Any, Dict, Tuple, Optional

import cv2

from .camera_base import CameraBase, FrameLike


class CameraIPRTSP(CameraBase):
    def __init__(
        self,
        camera_name: str = "",
        url: str = "",
        fps: float = 30.0,
        output: str = "compressed",
        reconnect_delay_s: float = 2.0,
    ):
        self.camera_name = camera_name
        self.url = url
        self.target_fps = fps
        self.output = output
        self.reconnect_delay_s = reconnect_delay_s

        self.cap: Optional[cv2.VideoCapture] = None
        self._last_ts: Optional[float] = None
        self._last_reconnect_log: float = 0.0

    def capabilities(self) -> Dict[str, set]:
        return {
            "pre_stream_only": set(),
            "runtime_safe": {"fps"},
            "sticky_on_start": set(),
        }

    # ------------------------------------------------------------------ #
    # Helpers                                                            #
    # ------------------------------------------------------------------ #
    def _open_internal(self) -> None:
        # Prova esplicitamente CAP_FFMPEG (più affidabile su RTSP)
        self.cap = cv2.VideoCapture(self.url, cv2.CAP_FFMPEG)
        if not self.cap.isOpened():
            # Fallback: prova default backend
            self.cap.release()
            self.cap = cv2.VideoCapture(self.url)

        if not self.cap.isOpened():
            raise RuntimeError(f"CameraIPRTSP: impossibile aprire stream {self.url}")
        self._last_ts = None

    # ------------------------------------------------------------------ #
    # Lifecycle                                                          #
    # ------------------------------------------------------------------ #
    def open(self) -> None:
        if not self.url:
            raise RuntimeError("CameraIPRTSP: 'url' non impostato")
        self._open_internal()

    def configure_pre_stream(self, cfg: Dict[str, Any]) -> None:
        pass

    def start_streaming(self) -> None:
        self._last_ts = None

    def configure_runtime(self, cfg: Dict[str, Any]) -> None:
        fps = cfg.get("fps")
        if fps:
            self.target_fps = float(fps)

    def stop_streaming(self) -> None:
        pass

    def close(self) -> None:
        if self.cap is not None:
            self.cap.release()
            self.cap = None

    def __del__(self):
        try:
            self.close()
        except Exception:
            pass

    # ------------------------------------------------------------------ #
    # Acquisizione frame                                                 #
    # ------------------------------------------------------------------ #
    def acquire(self, timeout_ms: int) -> Tuple[str, Optional[FrameLike]]:
        if self.cap is None:
            # tenta di riaprire
            try:
                self._open_internal()
            except Exception:
                return "EOF", None

        # pacing sul target_fps
        if self._last_ts is not None and self.target_fps > 0:
            min_dt = 1.0 / self.target_fps
            now = time.time()
            if now - self._last_ts < min_dt:
                time.sleep(min_dt - (now - self._last_ts))
        self._last_ts = time.time()

        ok, frame = self.cap.read()
        if not ok or frame is None:
            # prova a riconnettere ogni tanto
            now = time.time()
            if now - self._last_reconnect_log > 5.0:
                print("CameraIPRTSP: read() fallita, provo a riaprire lo stream...")
                self._last_reconnect_log = now

            self.close()
            time.sleep(self.reconnect_delay_s)
            try:
                self._open_internal()
            except Exception:
                return "EOF", None

            ok, frame = self.cap.read()
            if not ok or frame is None:
                return "EOF", None

        if self.output == "compressed":
            ok_enc, buf = cv2.imencode(".jpg", frame)
            if not ok_enc:
                return "ENCODE_ERR", None
            data = buf.tobytes()
            frame_obj: FrameLike = {
                "kind": "compressed",
                "data": data,
            }
            return "0", frame_obj

        return "UNSUPPORTED_OUTPUT", None

