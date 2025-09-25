#!/usr/bin/env python3
"""
multi_camera_image_saver.py  (time-aware + optimized)

Subscriptions:
  - /velocity (geometry_msgs/TwistStamped)    # linear.x in m/s (stamped)
  - /enabled  (owl_msgs/BoolStamped)          # stamped; 0->1 starts a NEW session + first-frame-per-camera
  - /*/image/compressed (sensor_msgs/CompressedImage)  # any number of cameras

Folders:
  <base_dir>/<DD-MM-YYYY>/<camera>/<HH-MM-SS>/img_<YYYYmmdd_HHMMSS_mmm>_<seq>.{jpg|png}

Performance:
  - Background writer threads with bounded queue (no blocking in callbacks)
  - Large QoS depth, best-effort
  - Reentrant callbacks + MultiThreadedExecutor friendly
  - Throttled stats logging (no per-image prints)

Time-correctness:
  - enabled is evaluated at the **image timestamp**
  - distance since last save is ∫ v(t) dt from last_save_time[camera] to image_time,
    using a timestamped velocity history (linear interpolation; trapezoidal integral)
"""

from pathlib import Path
from datetime import datetime
from functools import partial
from typing import Dict, Optional
import threading
import queue
import bisect

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from rclpy.callback_groups import ReentrantCallbackGroup

from geometry_msgs.msg import TwistStamped
from sensor_msgs.msg import CompressedImage
from owl_msgs.msg import BoolStamped


class MultiCameraImageSaver(Node):
    def __init__(self):
        super().__init__("multi_camera_image_saver")

        # ---- Parameters ----
        gp = self.declare_parameter
        self.base_dir = Path(gp("base_dir", "/home/dev/bags").value)
        self.min_distance = float(gp("min_distance", 0.5).value)
        self.velocity_topic = gp("velocity_topic", "/velocity").value
        self.enabled_topic = gp("enabled_topic", "/enabled").value
        self.image_pattern = gp("image_topic_pattern", "/*/image/compressed").value
        self.save_only_when_enabled = bool(gp("save_only_when_enabled", True).value)
        self.file_prefix = gp("file_prefix", "img").value
        self.discover_period = float(gp("discover_period", 1.0).value)
        self.continuous_discovery = bool(gp("continuous_discovery", True).value)
        self.qos_depth = int(gp("qos_depth", 64).value)
        self.writer_threads = int(gp("writer_threads", 2).value)
        self.write_queue_size = int(gp("write_queue_size", 512).value)
        self.drop_when_queue_full = bool(gp("drop_when_queue_full", True).value)
        self.stats_period = float(gp("stats_period", 2.0).value)

        # Velocity history config
        self.vel_hist_seconds = float(gp("vel_hist_seconds", 30.0).value)
        self.vel_hist_max_samples = int(gp("vel_hist_max_samples", 5000).value)

        self.base_dir.mkdir(parents=True, exist_ok=True)

        # ---- State ----
        # Current enabled (for info); time-aware gating uses _enabled_at(t)
        self.enabled: bool = False
        self._prev_enabled: Optional[bool] = None
        self._last_enable_true: Optional[Time] = None
        self._last_enable_false: Optional[Time] = None

        # New session marker after 0->1
        self._pending_new_session: bool = False
        self._session_start_time: Optional[Time] = None
        self._current_session_date: Optional[str] = None   # "DD-MM-YYYY"
        self._current_session_label: Optional[str] = None  # "HH-MM-SS"

        # Velocity history (sorted)
        self._vel_t_ns: list[int] = []     # nanoseconds
        self._vel_v: list[float] = []      # m/s
        self._current_velocity: float = 0.0  # last seen (for edges/fallback)

        # Per-camera state
        self._subs_by_topic: Dict[str, object] = {}      # topic -> Subscription
        self._camera_name_by_topic: Dict[str, str] = {}  # topic -> camera
        self._last_saved_time: Dict[str, Time] = {}      # camera -> Time
        self._session_dir_by_cam: Dict[str, Path] = {}   # camera -> Path
        self._seq_by_cam: Dict[str, int] = {}            # camera -> int
        self._first_frame_pending: Dict[str, bool] = {}  # camera -> bool

        # Stats
        self._saved_by_cam: Dict[str, int] = {}
        self._dropped_by_cam: Dict[str, int] = {}

        # Background writers
        self._write_q: queue.Queue = queue.Queue(maxsize=self.write_queue_size)
        self._writer_threads_list: list[threading.Thread] = []
        self._writers_alive = True
        for i in range(max(1, self.writer_threads)):
            t = threading.Thread(target=self._writer_loop, name=f"img-writer-{i}", daemon=True)
            t.start()
            self._writer_threads_list.append(t)

        # Reentrant callbacks + big QoS for images
        cbg = ReentrantCallbackGroup()
        img_qos = QoSProfile(
            depth=self.qos_depth,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            durability=DurabilityPolicy.VOLATILE,
        )

        # Subscriptions
        self.create_subscription(TwistStamped, self.velocity_topic, self._on_velocity, 10, callback_group=cbg)
        self.create_subscription(BoolStamped,  self.enabled_topic,  self._on_enabled,  10, callback_group=cbg)

        # Discovery
        if self.continuous_discovery and self.discover_period > 0.0:
            self.create_timer(max(0.1, self.discover_period),
                              lambda: self._discover_cameras(img_qos),
                              callback_group=cbg)
        self._discover_cameras(img_qos)  # initial pass

        # Periodic stats logging
        if self.stats_period > 0:
            self.create_timer(self.stats_period, self._log_stats, callback_group=cbg)

        self.get_logger().info(
            "Multi-camera saver (time-aware) started\n"
            f"  base_dir: {self.base_dir}\n"
            f"  min_distance: {self.min_distance} m | save_only_when_enabled: {self.save_only_when_enabled}\n"
            f"  qos_depth: {self.qos_depth} | writer_threads: {self.writer_threads} | queue: {self.write_queue_size}\n"
            f"  vel_hist: {self.vel_hist_seconds}s window, max {self.vel_hist_max_samples} samples"
        )

    # ---------- Hooks ----------
    def should_save_image(self, distance_m: float, enabled_at_img: bool, first_frame: bool, camera: str) -> bool:
        if first_frame:
            return enabled_at_img if self.save_only_when_enabled else True
        if self.save_only_when_enabled and not enabled_at_img:
            return False
        return distance_m >= self.min_distance

    # ---------- Time helpers ----------
    def _valid_time(self, t: Optional[Time]) -> bool:
        return t is not None and t.nanoseconds > 0

    def _time_from_header(self, header) -> Time:
        try:
            if header is not None and (header.stamp.sec != 0 or header.stamp.nanosec != 0):
                return Time(seconds=header.stamp.sec, nanoseconds=header.stamp.nanosec)
        except Exception:
            pass
        return self.get_clock().now()

    def _ns(self, t: Optional[Time]) -> int:
        return int(t.nanoseconds if self._valid_time(t) else self.get_clock().now().nanoseconds)

    # ---------- enabled(t) ----------
    def _enabled_at(self, t: Time) -> bool:
        tn = self._ns(t)
        lt = self._last_enable_true
        if lt is None:
            return False
        ltn = self._ns(lt)
        lf = self._last_enable_false
        lfn = self._ns(lf) if lf is not None else None
        if lfn is None or lfn < ltn:
            return tn >= ltn
        return ltn <= tn < lfn

    # ---------- Velocity history ----------
    def _insert_velocity_sample(self, t: Time, v: float):
        ns = self._ns(t)
        i = bisect.bisect_left(self._vel_t_ns, ns)
        if i < len(self._vel_t_ns) and self._vel_t_ns[i] == ns:
            self._vel_v[i] = v
        else:
            self._vel_t_ns.insert(i, ns)
            self._vel_v.insert(i, v)
        # prune by time window
        if self._vel_t_ns:
            cutoff = ns - int(self.vel_hist_seconds * 1e9)
            j = bisect.bisect_left(self._vel_t_ns, cutoff)
            if j > 0:
                del self._vel_t_ns[:j]
                del self._vel_v[:j]
        # prune by max samples
        if len(self._vel_t_ns) > self.vel_hist_max_samples:
            extra = len(self._vel_t_ns) - self.vel_hist_max_samples
            del self._vel_t_ns[:extra]
            del self._vel_v[:extra]

    def _velocity_at_ns(self, ns: int) -> float:
        if not self._vel_t_ns:
            return self._current_velocity
        i = bisect.bisect_left(self._vel_t_ns, ns)
        if i == 0:
            return self._vel_v[0]
        if i >= len(self._vel_t_ns):
            return self._vel_v[-1]
        t0, t1 = self._vel_t_ns[i-1], self._vel_t_ns[i]
        v0, v1 = self._vel_v[i-1], self._vel_v[i]
        if t1 == t0:
            return v1
        alpha = (ns - t0) / float(t1 - t0)
        return v0 + alpha * (v1 - v0)

    def _integrate_velocity(self, t0: Time, t1: Time) -> float:
        ns0, ns1 = self._ns(t0), self._ns(t1)
        if ns1 <= ns0:
            return 0.0
        if not self._vel_t_ns:
            return self._current_velocity * ((ns1 - ns0) * 1e-9)
        i0 = bisect.bisect_left(self._vel_t_ns, ns0)
        i1 = bisect.bisect_left(self._vel_t_ns, ns1)

        knots_t = [ns0]
        knots_v = [self._velocity_at_ns(ns0)]
        for k in range(i0, i1):
            tk = self._vel_t_ns[k]
            if ns0 < tk < ns1:
                knots_t.append(tk)
                knots_v.append(self._vel_v[k])
        knots_t.append(ns1)
        knots_v.append(self._velocity_at_ns(ns1))

        dist = 0.0
        for a in range(len(knots_t) - 1):
            dt = (knots_t[a+1] - knots_t[a]) * 1e-9
            dist += 0.5 * (knots_v[a] + knots_v[a+1]) * dt
        return dist

    # ---------- Discovery ----------
    def _discover_cameras(self, img_qos: Optional[QoSProfile] = None):
        # Only supports /*/image/compressed
        for topic_name, types in self.get_topic_names_and_types():
            if not (topic_name.startswith("/") and topic_name.endswith("/image/compressed")):
                continue
            parts = topic_name.strip("/").split("/")
            if len(parts) != 3 or parts[1] != "image" or parts[2] != "compressed":
                continue
            if "sensor_msgs/msg/CompressedImage" not in types:
                continue
            if topic_name in self._subs_by_topic:
                continue

            camera = parts[0]
            qos_profile = img_qos or QoSProfile(
                depth=self.qos_depth,
                reliability=ReliabilityPolicy.BEST_EFFORT,
                history=HistoryPolicy.KEEP_LAST,
                durability=DurabilityPolicy.VOLATILE,
            )
            sub = self.create_subscription(
                CompressedImage,
                topic_name,
                partial(self._on_image, camera=camera),
                qos_profile=qos_profile,
            )
            self._subs_by_topic[topic_name] = sub
            self._camera_name_by_topic[topic_name] = camera
            self._seq_by_cam.setdefault(camera, 0)
            self._saved_by_cam.setdefault(camera, 0)
            self._dropped_by_cam.setdefault(camera, 0)
            self._first_frame_pending.setdefault(camera, False)
            self.get_logger().info(f"Discovered camera: '{camera}' at {topic_name}")

    # ---------- Background writer ----------
    def _writer_loop(self):
        while self._writers_alive:
            try:
                item = self._write_q.get(timeout=0.2)
            except queue.Empty:
                continue
            if item is None:
                break
            out_path, data, camera = item
            try:
                with open(out_path, "wb", buffering=65536) as f:
                    f.write(data)
            except Exception as e:
                self.get_logger().error(f"[{camera}] Write failed: {out_path} | {e}")
            finally:
                self._write_q.task_done()

    # ---------- Callbacks ----------
    def _on_velocity(self, msg: TwistStamped):
        t = self._time_from_header(msg.header)
        v = float(msg.twist.linear.x)
        self._current_velocity = v
        self._insert_velocity_sample(t, v)

    def _on_enabled(self, msg: BoolStamped):
        t = self._time_from_header(msg.header)
        self.enabled = bool(msg.data)

        if self.enabled:
            self._last_enable_true = t
            # Rising edge: open new session + force first frame for known cameras
            if self._prev_enabled is not None and (not self._prev_enabled):
                self._pending_new_session = True
                self._session_start_time = t
                self._current_session_date = None
                self._current_session_label = None
                self._session_dir_by_cam.clear()
                for cam in list(self._camera_name_by_topic.values()):
                    self._first_frame_pending[cam] = True
                self.get_logger().info("Enabled 0->1: next image from each known camera will be saved immediately.")
                if not self.continuous_discovery:
                    self._discover_cameras()
        else:
            self._last_enable_false = t

        self._prev_enabled = self.enabled

    def _ensure_session_labels(self, t: Time):
        if self._session_start_time is None:
            self._session_start_time = t if self._valid_time(t) else self.get_clock().now()
        dt_wall = datetime.fromtimestamp(self._session_start_time.nanoseconds * 1e-9)
        self._current_session_date = dt_wall.strftime("%d-%m-%Y")
        self._current_session_label = dt_wall.strftime("%H-%M-%S")

    def _camera_session_dir(self, camera: str) -> Path:
        assert self._current_session_date and self._current_session_label
        cam_dir = self.base_dir / self._current_session_date / camera
        cam_dir.mkdir(parents=True, exist_ok=True)
        session_dir = cam_dir / self._current_session_label
        session_dir.mkdir(parents=True, exist_ok=True)
        return session_dir

    def _on_image(self, msg: CompressedImage, camera: str):
        # Keep FAST: no blocking I/O here
        t_img = self._time_from_header(msg.header)

        enabled_at_img = self._enabled_at(t_img)

        # Distance since last saved time for this camera, evaluated on image time
        t0 = self._last_saved_time.get(camera)
        if t0 is None:
            # If never saved, use session start (if any) to integrate; otherwise same as t_img (=> 0 distance)
            t0 = self._session_start_time or t_img
        dist_since = self._integrate_velocity(t0, t_img)

        # Only consider "first frame" if we're currently in a new-session phase AND enabled at image time
        first_pending = bool(self._first_frame_pending.get(camera, False)) and enabled_at_img

        if not self.should_save_image(dist_since, enabled_at_img, first_pending, camera):
            return

        # Ensure session labels/dirs (use enable time if we have it; else image time)
        if self._pending_new_session:
            self._ensure_session_labels(self._session_start_time or t_img)
        if self._current_session_date is None or self._current_session_label is None:
            self._ensure_session_labels(t_img)

        session_dir = self._session_dir_by_cam.get(camera)
        if session_dir is None or self._pending_new_session:
            session_dir = self._camera_session_dir(camera)
            self._session_dir_by_cam[camera] = session_dir
        self._pending_new_session = False

        # Filename from image time
        fmt = (msg.format or "").lower()
        ext = "jpg" if ("jpeg" in fmt or "jpg" in fmt) else ("png" if "png" in fmt else "bin")
        dt = datetime.fromtimestamp(t_img.nanoseconds * 1e-9)
        ts = dt.strftime("%Y%m%d_%H%M%S_%f")[:-3]
        self._seq_by_cam[camera] = self._seq_by_cam.get(camera, 0) + 1
        filename = f"{self.file_prefix}_{ts}_{self._seq_by_cam[camera]:06d}.{ext}"
        out_path = session_dir / filename

        data = msg.data
        if not isinstance(data, (bytes, bytearray, memoryview)):
            data = bytes(data)

        try:
            self._write_q.put_nowait((out_path, data, camera))
        except queue.Full:
            if self.drop_when_queue_full:
                self._dropped_by_cam[camera] = self._dropped_by_cam.get(camera, 0) + 1
                return
            else:
                self._write_q.put((out_path, data, camera))

        # Update per-camera state quickly
        self._last_saved_time[camera] = t_img
        if first_pending:
            self._first_frame_pending[camera] = False
        self._saved_by_cam[camera] = self._saved_by_cam.get(camera, 0) + 1

    # ---------- Stats ----------
    def _log_stats(self):
        cams = sorted(set(self._saved_by_cam) | set(self._dropped_by_cam))
        if not cams:
            return
        parts = []
        for c in cams:
            saved = self._saved_by_cam.get(c, 0)
            dropped = self._dropped_by_cam.get(c, 0)
            parts.append(f"{c}: saved={saved}, dropped={dropped}")
        self.get_logger().info(" | ".join(parts))

    # ---------- Cleanup ----------
    def destroy_node(self):
        self._writers_alive = False
        try:
            for _ in self._writer_threads_list:
                self._write_q.put_nowait(None)
        except queue.Full:
            pass
        for t in self._writer_threads_list:
            t.join(timeout=1.0)
        super().destroy_node()


def main():
    rclpy.init()
    node = MultiCameraImageSaver()

    from rclpy.executors import MultiThreadedExecutor
    exec = MultiThreadedExecutor(num_threads=max(2, node.writer_threads + 1))
    exec.add_node(node)
    try:
        exec.spin()
    except KeyboardInterrupt:
        pass
    finally:
        exec.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()


