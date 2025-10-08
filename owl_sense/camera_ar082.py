import pyvizionsdk as sdk, time
from typing import Optional, Tuple, Dict, Any, List
from .camera_base import CameraBase, Frame, FrameLike

MP_RESOLUTIONS = {
    "5MP": (2592, 1944),
    "4K":  (3840, 2160),
    "1440p": (2560, 1440),
    "1080p": (1920, 1080),
    "720p":  (1280, 720),
    "VGA":    (640, 480),
}

class CameraAR082x(CameraBase):
    def __init__(self, camera_name: str = "", output: str = "compressed"):
        self.camera_name = camera_name
        self.output = output  # "compressed" | "raw" | "both"
        self.is_open = False
        self.vxcam = None
        self.format = None
        self.width = 0; self.height = 0
        # optional internal fallback defaults (used only if YAML omits them)
        self._defaults = dict(resolution="4K", fps_format=15, fps=12,
                              exposure_mode=1, exposure_time_us=1000, jpeg_quality=205)

    def capabilities(self) -> Dict[str, set]:
        return {
            "pre_stream_only": {"resolution", "pixel_format", "fps_format"},
            "runtime_safe": {"fps", "exposure_mode", "exposure_time_us", "jpeg_quality"},
            "sticky_on_start": {"auto_exposure", "auto_white_balance"},
        }

    def open(self):
        if self.is_open: return
        _, cameras = sdk.VxDiscoverCameraDevices()
        cam_id = None
        for idx, name in enumerate(cameras):
            if name == self.camera_name:
                cam_id = idx; break
        if cam_id is None:
            raise RuntimeError(f"Camera '{self.camera_name}' not found")
        self.vxcam = sdk.VxInitialCameraDevice(cam_id)
        sdk.VxOpen(self.vxcam)
        self.is_open = True

    def _resolve_res(self, name: str):
        if isinstance(name, (list, tuple)) and len(name) == 2:
            self.width, self.height = int(name[0]), int(name[1]); return
        w_h = MP_RESOLUTIONS.get(name)
        if not w_h: raise ValueError(f"Unknown resolution: {name}")
        self.width, self.height = w_h

    def configure_pre_stream(self, cfg: Dict[str, Any]):
        # Merge fallbacks with YAML
        local = {**self._defaults, **cfg}
        self._resolve_res(local["resolution"])
        sdk.VxSetISPImageProcessingDefault(self.vxcam)
        sdk.VxSetOSPProfileFlag(self.vxcam, sdk.VX_OSP_PROFILE_FLAG.ENABLED)

        want_raw = (self.output in ("raw", "both"))
        fmt_target = sdk.VX_IMAGE_FORMAT.VX_IMAGE_FORMAT_RAW8 if want_raw else sdk.VX_IMAGE_FORMAT.VX_IMAGE_FORMAT_MJPG

        _, formats = sdk.VxGetFormatList(self.vxcam)
        chosen = None
        for fmt in formats:
            if (fmt.format == fmt_target and fmt.width == self.width and
                fmt.height == self.height and fmt.framerate == local["fps_format"]):
                sdk.VxSetFormat(self.vxcam, fmt); chosen = fmt; break
        if not chosen:
            raise RuntimeError(f"No matching format for {self.width}x{self.height}@{local['fps_format']} ({'RAW' if want_raw else 'MJPG'})")
        self.format = chosen

        # stash for runtime config
        self._fps          = int(local.get("fps", self._defaults["fps"]))
        self._exp_mode     = int(local.get("exposure_mode", self._defaults["exposure_mode"]))
        self._exp_time_us  = int(local.get("exposure_time_us", self._defaults["exposure_time_us"]))
        self._jpeg_quality = int(local.get("jpeg_quality", self._defaults["jpeg_quality"]))

    def start_streaming(self):
        sdk.VxStartStreaming(self.vxcam)
        time.sleep(0.3)

    def configure_runtime(self, cfg: Dict[str, Any]):
        # Merge and apply runtime values (re-apply after start)
        local = {
            "fps": self._fps,
            "exposure_mode": self._exp_mode,
            "exposure_time_us": self._exp_time_us,
            "jpeg_quality": self._jpeg_quality,
            **cfg
        }
        sdk.VxSetMaxFPS(self.vxcam, int(local["fps"]))
        sdk.VxSetISPImageProcessing(self.vxcam, sdk.VX_ISP_IMAGE_PROPERTIES.ISP_IMAGE_EXPOSURE_MODE, int(local["exposure_mode"]))
        sdk.VxSetISPImageProcessing(self.vxcam, sdk.VX_ISP_IMAGE_PROPERTIES.ISP_EHDR_EXPOSURE_MAX_NUMBER, int(local["ehdr_exposure_max_number"]))
        if int(local["exposure_mode"]) == 1:
            sdk.VxSetISPImageProcessing(self.vxcam, sdk.VX_ISP_IMAGE_PROPERTIES.ISP_IMAGE_EXPOSURE_MAX_TIME, int(local["exposure_time_us"]))
            sdk.VxSetISPImageProcessing(self.vxcam, sdk.VX_ISP_IMAGE_PROPERTIES.ISP_IMAGE_EXPOSURE_MIN_TIME, 10)
        else:
            sdk.VxSetISPImageProcessing(self.vxcam, sdk.VX_ISP_IMAGE_PROPERTIES.ISP_IMAGE_EXPOSURE_TIME, int(local["exposure_time_us"]))
        sdk.VxSetISPImageProcessing(self.vxcam, sdk.VX_ISP_IMAGE_PROPERTIES.ISP_IMAGE_JPEG_QUALITY, int(local["jpeg_quality"]))

        self._fps = int(local["fps"])
        self._exp_mode = int(local["exposure_mode"])
        self._exp_time_us = int(local["exposure_time_us"])
        self._jpeg_quality = int(local["jpeg_quality"])
        self._ehdr_exposure_max_number = int(local["ehdr_exposure_max_number"])

    def stop_streaming(self):
        sdk.VxStopStreaming(self.vxcam)

    def close(self):
        if not self.is_open: return
        sdk.VxClose(self.vxcam)
        self.is_open = False

    def __del__(self):
        try: self.close()
        except Exception: pass

    def acquire(self, timeout_ms: int) -> Tuple[str, Optional[FrameLike]]:
        if not self.is_open:
            raise RuntimeError("Camera is not open")
        code, img = sdk.VxGetImage(self.vxcam, timeout_ms, self.format)
        if code != sdk.VX_CAPTURE_RESULT.VX_SUCCESS:
            return str(code), None

        if self.output == "compressed":
            return "0", {"kind": "compressed", "data": img.tobytes()}
        if self.output == "raw":
            return "0", {"kind": "raw", "data": img.tobytes(),
                        "encoding": "bayer_rg8", "width": self.width, "height": self.height}
        # both: RAW from camera + on-device JPEG (if available) OR just reuse the same buffer (SDK-dependent).
        # If your SDK can only deliver one format at a time, consider encoding JPEG here from RAW.
        return "0", [
            {"kind": "raw", "data": img.tobytes(),
             "encoding": "bayer_rg8", "width": self.width, "height": self.height}
            # Optionally append a "compressed" frame if you encode one here.
        ]

