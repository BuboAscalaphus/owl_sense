"""
ROS 2 node for publishing compressed MJPEG frames from an AR0822 camera via PyVizionSDK,
with timing measurements for performance analysis.
"""
import time
import threading

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from rclpy.utilities import try_shutdown
from sensor_msgs.msg import CompressedImage
from owl_sense.camera_ar082 import CameraAR082x

# --- Default settings ---
DEFAULT_RES = "4K"
DEFAULT_FPS = 12
DEFAULT_EXP_MODE = 1
DEFAULT_EXP_TIME_US = 1000
DEFAULT_JPEG_QUALITY = 205
DEFAULT_FPS_FORMAT = 15

class CameraNode(Node):
    def __init__(self):
        super().__init__('camera')

        # Stop coordination
        self._stop_evt = threading.Event()
        # Se il contesto va in shutdown (Ctrl-C), sveglia i thread
        rclpy.get_default_context().on_shutdown(self._stop_evt.set)

        # Declare and read parameters
        self.declare_parameter('resolution', DEFAULT_RES)
        self.declare_parameter('fps', DEFAULT_FPS)
        self.declare_parameter('fps_format', DEFAULT_FPS_FORMAT)
        self.declare_parameter('exposure_mode', DEFAULT_EXP_MODE)
        self.declare_parameter('exposure_time', DEFAULT_EXP_TIME_US)
        self.declare_parameter('jpeg_quality', DEFAULT_JPEG_QUALITY)
        self.declare_parameter('camera_id', -1)
        self.declare_parameter('camera_name', '')

        res = self.get_parameter('resolution').get_parameter_value().string_value
        self.fps = self.get_parameter('fps').get_parameter_value().integer_value
        self.fps_format = self.get_parameter('fps_format').get_parameter_value().integer_value
        self.jpeg_quality = self.get_parameter('jpeg_quality').get_parameter_value().integer_value
        self.camera_id = self.get_parameter('camera_id').get_parameter_value().integer_value
        self.camera_name = self.get_parameter('camera_name').get_parameter_value().string_value
        self.exposure_time = self.get_parameter('exposure_time').get_parameter_value().integer_value
        self.exposure_mode = self.get_parameter('exposure_mode').get_parameter_value().integer_value

        self.topic_name = 'image/compressed'

        self.timeout_ms = int(1000 / max(self.fps, 1)) + 100
        self.frame_counter = 0
        self.last_fps_time = time.time()

        qos = QoSProfile(depth=1)
        self.publisher = self.create_publisher(CompressedImage, self.topic_name, qos)

        self.get_logger().info(
            f"Camera ID: {self.camera_id}, Name: {self.camera_name}, "
            f"Resolution: {res}, FPS: {self.fps}, JPEG Quality: {self.jpeg_quality}"
        )

        self.camera = CameraAR082x(
            camera_name=self.camera_name,
            fps=self.fps,
            fps_format=self.fps_format,
            jpeg_quality=self.jpeg_quality,
            resolution_name=res,
            exposure_time=self.exposure_time,
            exposure_mode=self.exposure_mode
        )
        self.get_logger().info(f"Publishing compressed images on {self.topic_name} with QoS depth {qos.depth}")

        self.camera.configure()
        self.camera.start_streaming()

        # Start background capture thread
        self.capture_thread = threading.Thread(target=self._capture_loop, name="capture_loop", daemon=False)
        self.capture_thread.start()

    def _capture_loop(self):
        while not self._stop_evt.is_set() and self.context.ok():
            t0 = time.time()
            code, img = self.camera.acquire(self.timeout_ms)
            t1 = time.time()

            if self._stop_evt.is_set() or not self.context.ok():
                break

            if img is None:
                # Logga solo se il contesto è ancora valido (evita rosout error)
                if self.context.ok():
                    try:
                        self.get_logger().warn(f'Frame capture failed with code: {code}')
                    except Exception:
                        pass
                continue

            jpeg_size = len(img)
            t2 = time.time()
            img_bytes = img.tobytes()
            t3 = time.time()

            msg = CompressedImage()
            msg.header.frame_id = self.camera_name
            # Evita usare il clock se il contesto non è ok
            if self.context.ok():
                msg.header.stamp = self.get_clock().now().to_msg()
            msg.format = 'jpeg'
            msg.data.frombytes(img_bytes)

            t4 = time.time()
            try:
                # Se il contesto sta chiudendo, publish può fallire: proteggi
                self.publisher.publish(msg)
            except Exception:
                break
            t5 = time.time()

            # FPS/logging periodico solo con contesto valido
            self.frame_counter += 1
            now = time.time()
            if self.context.ok() and (now - self.last_fps_time >= 2.0):
                try:
                    avg_fps = self.frame_counter / (now - self.last_fps_time)
                    self.get_logger().info(
                        f"FPS: {avg_fps:.2f} (req: {self.fps}) | "
                        f"JPEG size: {jpeg_size/1024:.1f} KB | "
                        f"SDK: {(t1-t0)*1000:.1f} ms | "
                        f"tobytes: {(t3-t2)*1000:.1f} ms | "
                        f"publish: {(t5-t4)*1000:.1f} ms | "
                        f"total: {(time.time()-t0)*1000:.1f} ms"
                    )
                finally:
                    self.frame_counter = 0
                    self.last_fps_time = now

    def request_stop(self):
        self._stop_evt.set()

    def destroy_node(self):
        # 1) segnala stop
        self._stop_evt.set()
        # 2) ferma lo streaming per sbloccare acquire() ASAP
        try:
            self.camera.stop_streaming()
        except Exception:
            pass
        # 3) attendi il thread
        if self.capture_thread.is_alive():
            self.capture_thread.join(timeout=2.0)
        # 4) chiudi la camera
        try:
            self.camera.close()
        except Exception:
            pass
        # 5) distruggi entità ROS
        return super().destroy_node()

def main():
    rclpy.init()
    node = None
    log = rclpy.logging.get_logger('main_log')

    try:
        node = CameraNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        # Potrebbe già essere tardi per rosout: fallback a print
        try:
            node.get_logger().info('Shutdown requested by user.')
        except Exception:
            print('Shutdown requested by user.')
    except Exception as ex:
        try:
            log.error(str(ex))
        except Exception:
            print(ex)
    finally:
        if node is not None:
            node.request_stop()
            node.destroy_node()
        # idempotente: non esplode se già shutdown
        try_shutdown()

if __name__ == '__main__':
    main()
