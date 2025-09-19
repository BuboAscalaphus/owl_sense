#!/usr/bin/env python3
"""
Standalone AR0822 MJPEG capture script using PyVizionSDK.
No ROS 2 dependencies — useful for measuring camera-side FPS vs JPEG quality.
"""
import time
import json
import pyvizionsdk as sdk

# --- Default settings ---
RES = "1080p"
FPS = 15
JPEG_QUALITY = 20  # 0-200
FPS_FORMAT = 30
MP_RESOLUTIONS = {
    "5MP":   (2592, 1944),
    "4K":    (3840, 2160),
    "1440p": (2560, 1440),
    "1080p": (1920, 1080),
    "720p":  (1280, 720),
    "VGA":   (640, 480),
}

def init_camera():
    width, height = MP_RESOLUTIONS.get(RES, MP_RESOLUTIONS["1080p"])
    _, cameras = sdk.VxDiscoverCameraDevices()
    for idx, cam in enumerate(cameras):
        if 'AR082' in cam:  
            vxcam = sdk.VxInitialCameraDevice(idx)
            sdk.VxOpen(vxcam)
            sdk.VxSetISPImageProcessingDefault(vxcam)

            sdk.VxSetOSPProfileFlag(vxcam, sdk.VX_OSP_PROFILE_FLAG.ENABLED)
            _, profile_json = sdk.VxGetOSPProfileConfig(vxcam)
            profile = json.loads(profile_json)
            sdk.VxLoadProfileConfig(vxcam, json.dumps(profile))

            _, formats = sdk.VxGetFormatList(vxcam)
            for fmt in formats:
                if (fmt.format == sdk.VX_IMAGE_FORMAT.VX_IMAGE_FORMAT_MJPG and
                    fmt.width == width and fmt.height == height and
                    fmt.framerate == FPS_FORMAT):
                    sdk.VxSetFormat(vxcam, fmt)
                    chosen_fmt = fmt
                    break

            sdk.VxActivateProfileImageProcessing(vxcam)
            sdk.VxActivateProfileStreaming(vxcam)
            sdk.VxStartStreaming(vxcam)
            time.sleep(0.5)

            sdk.VxSetISPImageProcessing(vxcam,
                sdk.VX_ISP_IMAGE_PROPERTIES.ISP_IMAGE_JPEG_QUALITY,
                JPEG_QUALITY)
            sdk.VxSetMaxFPS(vxcam, FPS)

            return vxcam, chosen_fmt

    raise RuntimeError("AR0822 camera not found")

def main():
    vxcam, fmt = init_camera()
    print(f"Capturing {RES} @ {FPS} FPS, JPEG quality {JPEG_QUALITY}")

    frame_count = 0
    last_log = time.time()

    try:
        while True:
            t0 = time.time()
            code, img = sdk.VxGetImage(vxcam, int(1000 / FPS) + 50, fmt)
            t1 = time.time()

            if code == sdk.VX_CAPTURE_RESULT.VX_SUCCESS:
                jpeg_size = len(img)
                frame_count += 1

                now = time.time()
                if now - last_log >= 2.0:
                    avg_fps = frame_count / (now - last_log)
                    print(f"FPS: {avg_fps:.2f} | JPEG size: {jpeg_size/1024:.1f} KB | "
                          f"SDK capture: {(t1-t0)*1000:.1f} ms")
                    frame_count = 0
                    last_log = now
            else:
                print(f"Frame capture failed with code: {code}")

    except KeyboardInterrupt:
        print("Stopping capture...")
    finally:
        sdk.VxStopStreaming(vxcam)
        sdk.VxClose(vxcam)

if __name__ == "__main__":
    main()


