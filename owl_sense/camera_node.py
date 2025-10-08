import time, threading
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from rclpy.utilities import try_shutdown
from sensor_msgs.msg import CompressedImage, Image
from std_msgs.msg import Header

from owl_sense.camera_factory import load_camera
from owl_sense.camera_base import Frame, FrameLike

class CameraNode(Node):
    def __init__(self):
        super().__init__('camera')

        # Only truly generic params
        self.declare_parameter('camera_impl', 'owl_sense.camera_ar082:CameraAR082x')
        self.declare_parameter('driver_params_file', '')
        self.declare_parameter('camera_name', '')
        self.declare_parameter('frame_timeout_ms', 120)  # generic read timeout

        camera_impl  = self.get_parameter('camera_impl').value
        params_file  = self.get_parameter('driver_params_file').value
        self.camera_name = self.get_parameter('camera_name').value
        self.timeout_ms  = int(self.get_parameter('frame_timeout_ms').value)

        self._stop_evt = threading.Event()
        rclpy.get_default_context().on_shutdown(self._stop_evt.set)

        qos = QoSProfile(depth=1)
        self.pub_compressed = self.create_publisher(CompressedImage, 'image/compressed', qos)
        self.pub_raw        = self.create_publisher(Image,           'image/raw',        qos)

        self.get_logger().info(
            f"Impl: {camera_impl} | Name: {self.camera_name} | YAML: {params_file}"
        )

        # Instantiate driver via factory (YAML is the source of truth)
        self.camera = load_camera(
            camera_impl,
            driver_params_file=params_file,
            camera_name=self.camera_name,
        )

        # Universal bring-up sequence
        self.camera.open()
        self.camera.configure_pre_stream(getattr(self.camera, "_pre_cfg", {}))
        self.camera.start_streaming()
        self.camera.configure_runtime(getattr(self.camera, "_run_cfg", {}))

        self.frame_counter = 0
        self.last_fps_time = time.time()

        self.capture_thread = threading.Thread(target=self._capture_loop, name="capture_loop", daemon=False)
        self.capture_thread.start()

    def _stamp(self) -> Header:
        h = Header()
        h.frame_id = self.camera_name
        if self.context.ok():
            h.stamp = self.get_clock().now().to_msg()
        return h

    def _publish_frame(self, frame: Frame):
        if frame["kind"] == "compressed":
            msg = CompressedImage()
            msg.header = self._stamp()
            msg.format = 'jpeg'
            msg.data.frombytes(frame["data"])
            self.pub_compressed.publish(msg)
        elif frame["kind"] == "raw":
            msg = Image()
            msg.header = self._stamp()
            msg.width  = int(frame["width"])
            msg.height = int(frame["height"])
            msg.encoding = frame["encoding"]          # e.g., "bayer_rg8","rgb8","mono8"
            msg.is_bigendian = 0
            # conservative step: mono8/bayer -> 1 byte per pixel; rgb8/bgr8 -> 3 bytes
            bpp = 3 if msg.encoding in ("rgb8", "bgr8") else 1
            msg.step = msg.width * bpp
            msg.data.frombytes(frame["data"])
            self.pub_raw.publish(msg)
        else:
            self.get_logger().warn(f"Unknown frame kind: {frame.get('kind')}")

    def _capture_loop(self):
        while not self._stop_evt.is_set() and self.context.ok():
            t0 = time.time()
            code, out = self.camera.acquire(self.timeout_ms)
            t1 = time.time()

            if self._stop_evt.is_set() or not self.context.ok():
                break
            if out is None:
                self.get_logger().warn(f'Frame capture failed: {code}')
                continue

            # Accept single frame or list of frames (for 'both' output)
            if isinstance(out, list):
                for f in out:
                    self._publish_frame(f)
            else:
                self._publish_frame(out)

            self.frame_counter += 1
            now = time.time()
            if now - self.last_fps_time >= 2.0:
                try:
                    avg_fps = self.frame_counter / (now - self.last_fps_time)
                    self.get_logger().info(f"FPS: {avg_fps:.2f} | SDK: {(t1-t0)*1000:.1f} ms")
                finally:
                    self.frame_counter = 0
                    self.last_fps_time = now

    def request_stop(self):
        self._stop_evt.set()

    def destroy_node(self):
        self._stop_evt.set()
        try: self.camera.stop_streaming()
        except Exception: pass
        if self.capture_thread.is_alive():
            self.capture_thread.join(timeout=2.0)
        try: self.camera.close()
        except Exception: pass
        return super().destroy_node()

def main():
    rclpy.init()
    node = None
    try:
        node = CameraNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        try: node.get_logger().info('Shutdown requested by user.')
        except Exception: print('Shutdown requested by user.')
    except Exception as ex:
        try: rclpy.logging.get_logger('main_log').error(str(ex))
        except Exception: print(ex)
    finally:
        if node is not None:
            node.request_stop()
            node.destroy_node()
        try_shutdown()

