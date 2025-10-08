import pyvizionsdk as sdk
import time

# Common resolutions dictionary
MP_RESOLUTIONS = {
    "5MP":   (2592, 1944),
    "4K":    (3840, 2160),
    "1440p": (2560, 1440),
    "1080p": (1920, 1080),
    "720p":  (1280, 720),
    "VGA":   (640, 480),
}

class CameraAR082x:
    def __init__(self, camera_name: str, fps: int,fps_format : int, jpeg_quality: int,resolution_name: str, exposure_time: int, exposure_mode: int):
        """
        Initialize a Camera object with a given camera_id.
        The device is not opened until configure() is called.
        """
        _, cameras = sdk.VxDiscoverCameraDevices()
        for idx,name in enumerate(cameras):
            if name == camera_name:
                camera_id = idx
                break
        self.vxcam = sdk.VxInitialCameraDevice(camera_id)
        
        self.format = None
        self.is_open = False
        self.resolution_name = resolution_name
        self.fps = fps
        self.fps_format = fps_format
        self.jpeg_quality = jpeg_quality
        self.width = 0
        self.height = 0
        self.camera_name = camera_name
        self.exposure_time = exposure_time
        self.exposure_mode = exposure_mode
        self.jpeg_quality = jpeg_quality
	self.EHDR_EXPOSURE_MAX_NUMBER = 3
	
    def open(self):
        """Open the camera device."""
        if not self.is_open:
            sdk.VxOpen(self.vxcam)
            self.is_open = True

    def get_resolution(self):
        """Get the current resolution of the camera."""
        res = MP_RESOLUTIONS.get(self.resolution_name)
        if not res:
            raise ValueError(f"Unknown resolution name: {self.resolution_name}")
        self.width, self.height = res

        return self.width, self.height
    
    def configure(self):
        """
        Configure the camera with the given resolution and framerate.
        """
        
        self.open()
        self.get_resolution()
        # Apply default ISP settings
        sdk.VxSetISPImageProcessingDefault(self.vxcam)

        # Enable OSP profile
        sdk.VxSetOSPProfileFlag(self.vxcam, sdk.VX_OSP_PROFILE_FLAG.ENABLED)

        # Search for supported format
        _, formats = sdk.VxGetFormatList(self.vxcam)
        chosen_fmt = None
        for fmt in formats:
            if (fmt.format == sdk.VX_IMAGE_FORMAT.VX_IMAGE_FORMAT_MJPG and
                fmt.width == self.width and fmt.height == self.height and
                fmt.framerate == self.fps_format):
                sdk.VxSetFormat(self.vxcam, fmt)
                print(f"[Camera {self.camera_name}] Format set: {fmt.width}x{fmt.height}@{fmt.framerate}")
                chosen_fmt = fmt
                break

        if not chosen_fmt:
            raise RuntimeError(f"No matching format found for {self.width}x{self.height}@{self.fps}")

        self.format = chosen_fmt
        #sdk.VxActivateProfileImageProcessing(self.vxcam)

    def start_streaming(self):
        """
        Start streaming from the camera.
        This method should be called after configure().
        """
        sdk.VxStartStreaming(self.vxcam)
        time.sleep(0.5)

        sdk.VxSetISPImageProcessing(self.vxcam,
            sdk.VX_ISP_IMAGE_PROPERTIES.ISP_IMAGE_JPEG_QUALITY,
            self.jpeg_quality)
        sdk.VxSetMaxFPS(self.vxcam, self.fps)
        
        sdk.VxSetISPImageProcessing(self.vxcam, sdk.VX_ISP_IMAGE_PROPERTIES.ISP_IMAGE_EXPOSURE_MODE, self.exposure_mode)
        sdk.VxSetISPImageProcessing(self.vxcam, sdk.VX_ISP_IMAGE_PROPERTIES.ISP_EHDR_EXPOSURE_MAX_NUMBER, self.EHDR_EXPOSURE_MAX_NUMBER)
        
        if self.exposure_mode == 1:  # Auto exposure
            sdk.VxSetISPImageProcessing(self.vxcam, sdk.VX_ISP_IMAGE_PROPERTIES.ISP_IMAGE_EXPOSURE_MAX_TIME, self.exposure_time)
            sdk.VxSetISPImageProcessing(self.vxcam, sdk.VX_ISP_IMAGE_PROPERTIES.ISP_IMAGE_EXPOSURE_MIN_TIME, 10)
        elif self.exposure_mode == 2:  # Fixed exposure, auto gain
            sdk.VxSetISPImageProcessing(self.vxcam, sdk.VX_ISP_IMAGE_PROPERTIES.ISP_IMAGE_EXPOSURE_TIME, self.exposure_time)
    def stop_streaming(self):
        sdk.VxStopStreaming(self.vxcam)

    def close(self):
        """Close the camera device."""
        if self.is_open:
            sdk.VxClose(self.vxcam)
            self.is_open = False

    def __del__(self):
        """Ensure the camera is closed when the object is destroyed."""
        self.close()

    def acquire(self,timeout_ms: int):
        """
        Acquire an image from the camera.
        Returns a tuple (code, image).
        """
        if not self.is_open:
            raise RuntimeError("Camera is not open. Call configure() first.")
        
        code, img = sdk.VxGetImage(self.vxcam, timeout_ms, self.format)
        
        if code != sdk.VX_CAPTURE_RESULT.VX_SUCCESS:
            return str(code), None
        
        return str(code), img
    
    '''
    VX_SUCCESS = 0
    VX_TIMEOUT = -1
    VX_CAM_OCCUPIED = -2
    VX_OTHER_ERROR = -3
    VX_BUFFER_CORRUPTED = -4
    '''
