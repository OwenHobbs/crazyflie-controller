from dataclasses import dataclass
import math
import threading
import time

import motioncapture
import numpy as np


@dataclass(frozen=True)
class Pose:
    x: float
    y: float
    z: float
    yaw: float
    timestamp: float


class ViconMotionClient:
    """Single-reader mocap client with a background frame thread."""

    def __init__(self, host_name: str, mocap_system_type: str = 'vicon'):
        self._mc = motioncapture.connect(mocap_system_type, {'hostname': host_name})

        self._lock = threading.Lock()
        self._condition = threading.Condition(self._lock)

        self._latest_frame: dict[str, Pose] = {}
        self._frame_id: int = 0

        self._thread: threading.Thread | None = None
        self._stop_event = threading.Event()
        self._started = False

    def start(self) -> None:
        if self._started:
            return

        self._stop_event.clear()
        self._thread = threading.Thread(target=self._run, name='ViconMotionThread', daemon=True)
        self._thread.start()
        self._started = True

    def stop(self) -> None:
        self._stop_event.set()
        with self._condition:
            self._condition.notify_all()

        if self._thread is not None:
            self._thread.join(timeout=2.0)

        self._started = False

    def get_latest_frame(self) -> tuple[int, dict[str, Pose]]:
        with self._lock:
            return self._frame_id, dict(self._latest_frame)

    def wait_for_new_frame(
        self,
        last_frame_id: int,
        timeout: float | None = None,
    ) -> tuple[int, dict[str, Pose]] | None:
        with self._condition:
            ok = self._condition.wait_for(
                lambda: self._frame_id > last_frame_id or self._stop_event.is_set(),
                timeout=timeout,
            )

            if not ok or self._stop_event.is_set():
                return None

            return self._frame_id, dict(self._latest_frame)

    def get_latest_pose(self, rigid_body_name: str) -> Pose | None:
        with self._lock:
            return self._latest_frame.get(rigid_body_name)

    def wait_for_pose(
        self,
        rigid_body_name: str,
        last_frame_id: int = 0,
        timeout: float | None = None,
    ) -> tuple[int, Pose] | None:
        result = self.wait_for_new_frame(last_frame_id=last_frame_id, timeout=timeout)
        if result is None:
            return None

        frame_id, frame = result
        pose = frame.get(rigid_body_name)
        if pose is None:
            return None

        return frame_id, pose

    def _run(self) -> None:
        while not self._stop_event.is_set():
            self._mc.waitForNextFrame()
            timestamp = time.time()

            poses: dict[str, Pose] = {}
            for name, obj in self._mc.rigidBodies.items():
                pos = obj.position
                if not self._is_valid_position(pos):
                    continue

                yaw = self._quaternion_to_yaw(
                    obj.rotation.w,
                    obj.rotation.x,
                    obj.rotation.y,
                    obj.rotation.z,
                )

                poses[name] = Pose(
                    x=pos[0],
                    y=pos[1],
                    z=pos[2],
                    yaw=yaw,
                    timestamp=timestamp,
                )

            with self._condition:
                self._latest_frame = poses
                self._frame_id += 1
                self._condition.notify_all()

    @staticmethod
    def _is_valid_position(pos) -> bool:
        return (
            np.abs(pos[0]) > 1.0e-9 and np.abs(pos[0]) < 1.0e4 and
            np.abs(pos[1]) > 1.0e-9 and np.abs(pos[1]) < 1.0e4 and
            np.abs(pos[2]) > 1.0e-9 and np.abs(pos[2]) < 1.0e4
        )

    @staticmethod
    def _quaternion_to_yaw(q_w: float, q_x: float, q_y: float, q_z: float) -> float:
        return math.atan2(
            2 * (q_w * q_z + q_x * q_y),
            1 - 2 * (q_y * q_y + q_z * q_z),
        )