# coding=utf-8
import argparse
import os
import threading
import time

import cv2
import h5py
import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import Pose
from piper_msgs.msg import PosCmd
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile, qos_profile_sensor_data
from sensor_msgs.msg import Image, JointState


def load_hdf5(dataset_dir, task_name, episode_idx):
    dataset_path = os.path.join(dataset_dir, task_name, f'episode_{episode_idx}.hdf5')
    if not os.path.isfile(dataset_path):
        raise FileNotFoundError(f'Dataset does not exist: {dataset_path}')

    data = {}
    with h5py.File(dataset_path, 'r') as root:
        data['timestamps'] = root['/timestamps'][()] if '/timestamps' in root else None

        # RGB / Depth images
        data['images'] = {}
        data['images_depth'] = {}
        for cam_name in root['/observations/images'].keys():
            data['images'][cam_name] = root[f'/observations/images/{cam_name}'][()]
        if '/observations/images_depth' in root:
            for cam_name in root['/observations/images_depth'].keys():
                data['images_depth'][cam_name] = root[f'/observations/images_depth/{cam_name}'][()]

        # Arm datasets (new format)
        if '/arm/joint_states_left/position' in root:
            def _read_joint(prefix):
                return {
                    'position': root[f'/arm/{prefix}/position'][()],
                    'velocity': root[f'/arm/{prefix}/velocity'][()],
                    'effort': root[f'/arm/{prefix}/effort'][()],
                }

            data['joint_states_left'] = _read_joint('joint_states_left')
            data['joint_states_right'] = _read_joint('joint_states_right')

            if '/arm/joint_left/position' in root and '/arm/joint_right/position' in root:
                data['joint_left'] = _read_joint('joint_left')
                data['joint_right'] = _read_joint('joint_right')
            else:
                # Compatibility: some datasets only keep joint_states + /action.
                action = root['/action'][()] if '/action' in root else None
                left_pos = data['joint_states_left']['position']
                right_pos = data['joint_states_right']['position']
                if action is not None:
                    split = action.shape[1] // 2
                    left_cmd = action[:, :split]
                    right_cmd = action[:, split:]
                else:
                    left_cmd = left_pos
                    right_cmd = right_pos
                data['joint_left'] = {
                    'position': left_cmd,
                    'velocity': np.zeros_like(left_cmd),
                    'effort': np.zeros_like(left_cmd),
                }
                data['joint_right'] = {
                    'position': right_cmd,
                    'velocity': np.zeros_like(right_cmd),
                    'effort': np.zeros_like(right_cmd),
                }

            n = data['joint_left']['position'].shape[0]
            if '/arm/end_pose_left' in root and '/arm/end_pose_right' in root:
                data['end_pose_left'] = root['/arm/end_pose_left'][()]
                data['end_pose_right'] = root['/arm/end_pose_right'][()]
            else:
                data['end_pose_left'] = np.zeros((n, 7), dtype=np.float32)
                data['end_pose_right'] = np.zeros((n, 7), dtype=np.float32)

        else:
            # Fallback for older files
            print('Using fallback dataset loader (legacy joint layout).')
            qpos = root['/observations/qpos'][()]
            qvel = root['/observations/qvel'][()] if '/observations/qvel' in root else np.zeros_like(qpos)
            effort = root['/observations/effort'][()] if '/observations/effort' in root else np.zeros_like(qpos)
            action = root['/action'][()] if '/action' in root else qpos

            q_split = qpos.shape[1] // 2
            a_split = action.shape[1] // 2
            data['joint_states_left'] = {
                'position': qpos[:, :q_split], 'velocity': qvel[:, :q_split], 'effort': effort[:, :q_split]
            }
            data['joint_states_right'] = {
                'position': qpos[:, q_split:], 'velocity': qvel[:, q_split:], 'effort': effort[:, q_split:]
            }
            data['joint_left'] = {
                'position': action[:, :a_split], 'velocity': np.zeros_like(action[:, :a_split]), 'effort': np.zeros_like(action[:, :a_split])
            }
            data['joint_right'] = {
                'position': action[:, a_split:], 'velocity': np.zeros_like(action[:, a_split:]), 'effort': np.zeros_like(action[:, a_split:])
            }
            n = action.shape[0]
            if '/arm/end_pose_left' in root and '/arm/end_pose_right' in root:
                data['end_pose_left'] = root['/arm/end_pose_left'][()]
                data['end_pose_right'] = root['/arm/end_pose_right'][()]
            else:
                data['end_pose_left'] = np.zeros((n, 7), dtype=np.float32)
                data['end_pose_right'] = np.zeros((n, 7), dtype=np.float32)

    return data, dataset_path


class LiveVideoRecorder(Node):
    """Record the three physical camera topics while robot actions are replayed."""

    def __init__(self, args):
        super().__init__('live_replay_video_recorder')
        self.args = args
        self.bridge = CvBridge()
        self.output_dir = self._resolve_output_dir()
        self.writers = {}
        self.frame_counts = {'top': 0, 'left': 0, 'right': 0}
        self.frame_sizes = {}
        self.callback_groups = {}
        self.ready_events = {
            camera_name: threading.Event()
            for camera_name in self.frame_counts
        }
        self.callback_error = None

        os.makedirs(self.output_dir, exist_ok=True)
        camera_topics = {
            'top': args.img_top_topic,
            'left': args.img_left_topic,
            'right': args.img_right_topic,
        }
        for camera_name, topic in camera_topics.items():
            callback_group = MutuallyExclusiveCallbackGroup()
            self.callback_groups[camera_name] = callback_group
            self.create_subscription(
                Image,
                topic,
                lambda msg, name=camera_name: self._image_callback(name, msg),
                qos_profile_sensor_data,
                callback_group=callback_group,
            )
        print(f'Live camera videos: {self.output_dir}')
        for camera_name, topic in camera_topics.items():
            print(f'  waiting for {camera_name}: {topic}')

    def _resolve_output_dir(self):
        output_dir = self.args.video_output_dir
        if not output_dir:
            timestamp = time.strftime('%Y%m%d_%H%M%S')
            output_dir = os.path.join(
                self.args.dataset_dir,
                self.args.task_name,
                'replay_videos',
                f'episode_{self.args.episode_idx}_live_{timestamp}',
            )
        return os.path.abspath(os.path.expanduser(output_dir))

    def _image_callback(self, camera_name, msg):
        if self.callback_error is not None:
            return
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            frame = np.ascontiguousarray(frame)
            if camera_name not in self.writers:
                self._open_writer(camera_name, frame)
            expected_size = self.frame_sizes[camera_name]
            actual_size = (frame.shape[1], frame.shape[0])
            if actual_size != expected_size:
                raise RuntimeError(
                    f'Live camera {camera_name} resolution changed from '
                    f'{expected_size} to {actual_size}'
                )
            self.writers[camera_name].write(frame)
            self.frame_counts[camera_name] += 1
            self.ready_events[camera_name].set()
        except Exception as exc:
            self.callback_error = exc

    def _open_writer(self, camera_name, frame):
        if frame.dtype != np.uint8 or frame.ndim != 3 or frame.shape[2] != 3:
            raise RuntimeError(
                f'Cannot record live camera {camera_name}: expected uint8 HxWx3, '
                f'got dtype={frame.dtype}, shape={frame.shape}'
            )
        height, width = frame.shape[:2]
        output_path = os.path.join(self.output_dir, f'{camera_name}.mkv')
        writer = cv2.VideoWriter(
            output_path,
            cv2.VideoWriter_fourcc(*'FFV1'),
            float(self.args.live_video_fps),
            (width, height),
        )
        if not writer.isOpened():
            writer.release()
            raise RuntimeError(f'Failed to open lossless video writer: {output_path}')
        self.writers[camera_name] = writer
        self.frame_sizes[camera_name] = (width, height)
        print(
            f'Live camera ready [{camera_name}]: {width}x{height}, '
            f'{self.args.live_video_fps:g} fps, FFV1 lossless'
        )

    def wait_until_ready(self, timeout_s):
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            if self.callback_error is not None:
                raise RuntimeError(f'Live video recorder failed: {self.callback_error}')
            if all(event.is_set() for event in self.ready_events.values()):
                print('All three live cameras are ready; robot replay may start.')
                return
            time.sleep(0.05)
        missing = [
            name for name, event in self.ready_events.items()
            if not event.is_set()
        ]
        raise RuntimeError(
            f'Timed out after {timeout_s:g}s waiting for live cameras: '
            f'{", ".join(missing)}. Start camera.sh before replaying.'
        )

    def close(self):
        for writer in self.writers.values():
            writer.release()
        if self.writers:
            counts = ', '.join(
                f'{name}={self.frame_counts[name]}'
                for name in ['top', 'left', 'right']
            )
            print(f'Saved live camera videos to: {self.output_dir}')
            print(f'Live video frame counts: {counts}')


class Ros2Replayer(Node):
    def __init__(self, args):
        super().__init__('replay_node_ros2')
        self.args = args
        self.bridge = CvBridge()

        sensor_qos = qos_profile_sensor_data
        default_qos = QoSProfile(depth=10)

        # RGB
        self.pub_rgb_left = self.create_publisher(Image, args.img_left_topic, sensor_qos)
        self.pub_rgb_right = self.create_publisher(Image, args.img_right_topic, sensor_qos)
        self.pub_rgb_top = self.create_publisher(Image, args.img_top_topic, sensor_qos)

        # Depth
        self.pub_depth_left = self.create_publisher(Image, args.img_left_depth_topic, sensor_qos)
        self.pub_depth_right = self.create_publisher(Image, args.img_right_depth_topic, sensor_qos)
        self.pub_depth_top = self.create_publisher(Image, args.img_top_depth_topic, sensor_qos)

        # Arm topics
        self.pub_joint_states_left = self.create_publisher(JointState, args.joint_states_left_topic, default_qos)
        self.pub_joint_states_right = self.create_publisher(JointState, args.joint_states_right_topic, default_qos)
        self.pub_joint_left = self.create_publisher(JointState, args.joint_left_topic, default_qos)
        self.pub_joint_right = self.create_publisher(JointState, args.joint_right_topic, default_qos)
        self.pub_joint_ctrl_cmd_left = self.create_publisher(JointState, args.joint_ctrl_cmd_left_topic, default_qos)
        self.pub_joint_ctrl_cmd_right = self.create_publisher(JointState, args.joint_ctrl_cmd_right_topic, default_qos)

        self.pub_end_pose_left = self.create_publisher(Pose, args.end_pose_left_topic, default_qos)
        self.pub_end_pose_right = self.create_publisher(Pose, args.end_pose_right_topic, default_qos)
        self.pub_pos_cmd_left = self.create_publisher(PosCmd, args.pos_cmd_left_topic, default_qos)
        self.pub_pos_cmd_right = self.create_publisher(PosCmd, args.pos_cmd_right_topic, default_qos)

    @staticmethod
    def _align_vec_len(vec, target_len):
        arr = np.asarray(vec, dtype=np.float64)
        if arr.shape[0] == target_len:
            return arr
        if arr.shape[0] > target_len:
            return arr[:target_len]
        out = np.zeros((target_len,), dtype=np.float64)
        out[:arr.shape[0]] = arr
        return out

    def _build_joint_msg(self, position, velocity, effort):
        pos = np.asarray(position, dtype=np.float64)
        vel = self._align_vec_len(velocity, pos.shape[0])
        eff = self._align_vec_len(effort, pos.shape[0])
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = [f'joint{i + 1}' for i in range(pos.shape[0])]
        msg.position = pos.tolist()
        msg.velocity = vel.tolist()
        msg.effort = eff.tolist()
        return msg

    @staticmethod
    def _build_pose_msg(vec7):
        msg = Pose()
        msg.position.x = float(vec7[0])
        msg.position.y = float(vec7[1])
        msg.position.z = float(vec7[2])
        msg.orientation.x = float(vec7[3])
        msg.orientation.y = float(vec7[4])
        msg.orientation.z = float(vec7[5])
        msg.orientation.w = float(vec7[6])
        return msg

    @staticmethod
    def _quat_to_rpy(qx, qy, qz, qw):
        # Quaternion (x, y, z, w) -> Euler (roll, pitch, yaw), radians.
        sinr_cosp = 2.0 * (qw * qx + qy * qz)
        cosr_cosp = 1.0 - 2.0 * (qx * qx + qy * qy)
        roll = np.arctan2(sinr_cosp, cosr_cosp)

        sinp = 2.0 * (qw * qy - qz * qx)
        if abs(sinp) >= 1.0:
            pitch = np.pi / 2.0 * np.sign(sinp)
        else:
            pitch = np.arcsin(sinp)

        siny_cosp = 2.0 * (qw * qz + qx * qy)
        cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
        yaw = np.arctan2(siny_cosp, cosy_cosp)
        return float(roll), float(pitch), float(yaw)

    def _build_pos_cmd(self, pose_vec7, gripper, mode1, mode2):
        msg = PosCmd()
        msg.x = float(pose_vec7[0])
        msg.y = float(pose_vec7[1])
        msg.z = float(pose_vec7[2])
        roll, pitch, yaw = self._quat_to_rpy(
            float(pose_vec7[3]), float(pose_vec7[4]), float(pose_vec7[5]), float(pose_vec7[6])
        )
        msg.roll = roll
        msg.pitch = pitch
        msg.yaw = yaw
        msg.gripper = float(gripper)
        msg.mode1 = int(mode1)
        msg.mode2 = int(mode2)
        return msg

    @staticmethod
    def _fmt_vec(vec, precision=4):
        arr = np.asarray(vec, dtype=np.float64)
        return np.array2string(arr, precision=precision, separator=', ', suppress_small=False)

    def replay(self, data):
        cam_top, cam_left, cam_right = self.args.camera_names
        for cam_name in [cam_top, cam_left, cam_right]:
            if cam_name not in data['images']:
                raise RuntimeError(f'Missing RGB camera stream in hdf5: {cam_name}')

        total = data['joint_left']['position'].shape[0]
        print(f'Replay frames: {total}')

        prev_ts = None
        for i in range(total):
            if not rclpy.ok():
                break

            # Live cameras publish on these same topics. Do not mix saved HDF5
            # images into the physical-camera streams while live recording.
            if not self.args.record_live_videos:
                self.pub_rgb_top.publish(self.bridge.cv2_to_imgmsg(data['images'][cam_top][i], encoding='passthrough'))
                self.pub_rgb_left.publish(self.bridge.cv2_to_imgmsg(data['images'][cam_left][i], encoding='passthrough'))
                self.pub_rgb_right.publish(self.bridge.cv2_to_imgmsg(data['images'][cam_right][i], encoding='passthrough'))
                if all(cam in data['images_depth'] for cam in [cam_top, cam_left, cam_right]):
                    self.pub_depth_top.publish(self.bridge.cv2_to_imgmsg(data['images_depth'][cam_top][i], encoding='passthrough'))
                    self.pub_depth_left.publish(self.bridge.cv2_to_imgmsg(data['images_depth'][cam_left][i], encoding='passthrough'))
                    self.pub_depth_right.publish(self.bridge.cv2_to_imgmsg(data['images_depth'][cam_right][i], encoding='passthrough'))

            # Publish joints
            self.pub_joint_states_left.publish(
                self._build_joint_msg(
                    data['joint_states_left']['position'][i],
                    data['joint_states_left']['velocity'][i],
                    data['joint_states_left']['effort'][i],
                )
            )
            self.pub_joint_states_right.publish(
                self._build_joint_msg(
                    data['joint_states_right']['position'][i],
                    data['joint_states_right']['velocity'][i],
                    data['joint_states_right']['effort'][i],
                )
            )
            self.pub_joint_left.publish(
                self._build_joint_msg(
                    data['joint_left']['position'][i],
                    data['joint_left']['velocity'][i],
                    data['joint_left']['effort'][i],
                )
            )
            self.pub_joint_right.publish(
                self._build_joint_msg(
                    data['joint_right']['position'][i],
                    data['joint_right']['velocity'][i],
                    data['joint_right']['effort'][i],
                )
            )
            # Replay control inputs for controller subscribers.
            self.pub_joint_ctrl_cmd_left.publish(
                self._build_joint_msg(
                    data['joint_left']['position'][i],
                    data['joint_left']['velocity'][i],
                    data['joint_left']['effort'][i],
                )
            )
            self.pub_joint_ctrl_cmd_right.publish(
                self._build_joint_msg(
                    data['joint_right']['position'][i],
                    data['joint_right']['velocity'][i],
                    data['joint_right']['effort'][i],
                )
            )

            # Publish end poses
            self.pub_end_pose_left.publish(self._build_pose_msg(data['end_pose_left'][i]))
            self.pub_end_pose_right.publish(self._build_pose_msg(data['end_pose_right'][i]))
            left_pos = data['joint_left']['position'][i]
            right_pos = data['joint_right']['position'][i]
            left_gripper = float(left_pos[6]) if len(left_pos) > 6 else (float(left_pos[-1]) if len(left_pos) > 0 else 0.0)
            right_gripper = float(right_pos[6]) if len(right_pos) > 6 else (float(right_pos[-1]) if len(right_pos) > 0 else 0.0)
            self.pub_pos_cmd_left.publish(
                self._build_pos_cmd(
                    data['end_pose_left'][i],
                    left_gripper,
                    self.args.pos_cmd_mode1,
                    self.args.pos_cmd_mode2,
                )
            )
            self.pub_pos_cmd_right.publish(
                self._build_pos_cmd(
                    data['end_pose_right'][i],
                    right_gripper,
                    self.args.pos_cmd_mode1,
                    self.args.pos_cmd_mode2,
                )
            )

            if self.args.print_data_info and (i % self.args.print_every_n == 0 or i == total - 1):
                print(
                    f"[frame {i}/{total}] "
                    f"joint_left={self._fmt_vec(left_pos)} "
                    f"joint_right={self._fmt_vec(right_pos)} "
                    f"end_pose_left={self._fmt_vec(data['end_pose_left'][i])} "
                    f"end_pose_right={self._fmt_vec(data['end_pose_right'][i])}"
                )

            # Replay timing
            if self.args.use_saved_timestamps and data['timestamps'] is not None:
                cur_ts = float(data['timestamps'][i])
                if prev_ts is not None:
                    dt = max(0.0, min(cur_ts - prev_ts, self.args.max_sleep_s))
                    time.sleep(dt)
                prev_ts = cur_ts
            else:
                if self.args.frame_rate > 0:
                    time.sleep(1.0 / self.args.frame_rate)

            completed_steps = i + 1
            if (
                self.args.pause_every_n > 0
                and self.args.pause_seconds > 0
                and completed_steps % self.args.pause_every_n == 0
                and completed_steps < total
            ):
                print(
                    f'Completed {completed_steps} steps; '
                    f'pausing for {self.args.pause_seconds:g} seconds...',
                    flush=True,
                )
                time.sleep(self.args.pause_seconds)

            # if i % 50 == 0:
            #     print(f'Replay {i}/{total}')
            print(f'Replay {i}/{total}')


def main(args):
    data, path = load_hdf5(args.dataset_dir, args.task_name, args.episode_idx)
    print(f'Loaded: {path}')

    rclpy.init(args=None)
    node = None
    recorder = None
    recorder_executor = None
    recorder_thread = None
    try:
        if args.record_live_videos:
            recorder = LiveVideoRecorder(args)
            # FFV1 is CPU-intensive. Give each camera callback its own worker so
            # one stream cannot starve the other two while frames are encoded.
            recorder_executor = MultiThreadedExecutor(num_threads=3)
            recorder_executor.add_node(recorder)
            recorder_thread = threading.Thread(
                target=recorder_executor.spin,
                name='live-video-recorder',
                daemon=True,
            )
            recorder_thread.start()
            recorder.wait_until_ready(args.live_camera_timeout)

        node = Ros2Replayer(args)
        node.replay(data)
        if recorder is not None and recorder.callback_error is not None:
            raise RuntimeError(f'Live video recorder failed: {recorder.callback_error}')
    finally:
        if node is not None:
            node.destroy_node()
        if recorder_executor is not None:
            recorder_executor.shutdown()
        if recorder_thread is not None:
            recorder_thread.join(timeout=2.0)
        if recorder is not None:
            recorder.close()
            recorder.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--dataset_dir', type=str, required=True, help='Dataset root dir')
    parser.add_argument('--task_name', type=str, default='aloha_mobile_dummy')
    parser.add_argument('--episode_idx', type=int, default=0)

    parser.add_argument('--frame_rate', type=int, default=30)
    parser.add_argument('--use_saved_timestamps', action='store_true')
    parser.add_argument('--max_sleep_s', type=float, default=0.2)
    parser.add_argument('--pause_every_n', type=int, default=0, help='Pause after every N replayed steps; 0 disables it.')
    parser.add_argument('--pause_seconds', type=float, default=0.0, help='Extra pause duration in seconds.')
    parser.add_argument('--camera_names', nargs=3, default=['cam_high', 'cam_left_wrist', 'cam_right_wrist'])
    parser.add_argument(
        '--record_live_videos',
        action='store_true',
        help='Record physical top/left/right camera topics during robot action replay.',
    )
    parser.add_argument(
        '--video_output_dir',
        type=str,
        default=None,
        help='Directory for top.mkv, left.mkv and right.mkv.',
    )
    parser.add_argument(
        '--live_video_fps',
        type=float,
        default=30.0,
        help='Output fps for physical-camera videos.',
    )
    parser.add_argument(
        '--live_camera_timeout',
        type=float,
        default=15.0,
        help='Seconds to wait for all three physical camera topics before robot replay.',
    )

    parser.add_argument('--img_left_topic', type=str, default='/left/color/image_raw')
    parser.add_argument('--img_right_topic', type=str, default='/right/color/image_raw')
    parser.add_argument('--img_top_topic', type=str, default='/top/color/image_raw')

    parser.add_argument('--img_left_depth_topic', type=str, default='/left/depth/image_rect_raw')
    parser.add_argument('--img_right_depth_topic', type=str, default='/right/depth/image_rect_raw')
    parser.add_argument('--img_top_depth_topic', type=str, default='/top/depth/image_rect_raw')

    parser.add_argument('--joint_states_left_topic', type=str, default='/joint_states_left')
    parser.add_argument('--joint_states_right_topic', type=str, default='/joint_states_right')
    parser.add_argument('--joint_left_topic', type=str, default='/joint_left')
    parser.add_argument('--joint_right_topic', type=str, default='/joint_right')
    parser.add_argument('--joint_ctrl_cmd_left_topic', type=str, default='/joint_ctrl_cmd_left')
    parser.add_argument('--joint_ctrl_cmd_right_topic', type=str, default='/joint_ctrl_cmd_right')
    parser.add_argument('--end_pose_left_topic', type=str, default='/end_pose_left')
    parser.add_argument('--end_pose_right_topic', type=str, default='/end_pose_right')
    parser.add_argument('--pos_cmd_left_topic', type=str, default='/pos_cmd_left')
    parser.add_argument('--pos_cmd_right_topic', type=str, default='/pos_cmd_right')
    parser.add_argument('--pos_cmd_mode1', type=int, default=0)
    parser.add_argument('--pos_cmd_mode2', type=int, default=0)
    parser.add_argument('--print_data_info', action='store_true', help='Print replayed joint/end-pose data.')
    parser.add_argument('--print_every_n', type=int, default=1, help='Print data info every N frames.')

    args = parser.parse_args()
    args.print_every_n = max(1, int(args.print_every_n))
    if args.pause_every_n < 0:
        parser.error('--pause_every_n must be greater than or equal to 0')
    if not np.isfinite(args.pause_seconds) or args.pause_seconds < 0:
        parser.error('--pause_seconds must be a finite number greater than or equal to 0')
    if not np.isfinite(args.live_video_fps) or args.live_video_fps <= 0:
        parser.error('--live_video_fps must be a finite number greater than 0')
    if not np.isfinite(args.live_camera_timeout) or args.live_camera_timeout <= 0:
        parser.error('--live_camera_timeout must be a finite number greater than 0')
    main(args)
