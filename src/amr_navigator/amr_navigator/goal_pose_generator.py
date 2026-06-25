import os
import re
import tempfile
from typing import Any, Dict

import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformException, TransformListener
import yaml


class GoalPoseGenerator(Node):
    def __init__(self):
        super().__init__('goal_pose_generator')

        self.declare_parameter('output_file', '')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('name_prefix', 'work_station')
        self.declare_parameter('lookup_timeout_sec', 1.0)
        self.declare_parameter('float_precision', 6)

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self._save_service = self.create_service(
            Trigger,
            '~/save_current',
            self._save_current_callback,
        )

        self.get_logger().info(
            'goal_pose_generator ready. '
            'Call /goal_pose_generator/save_current to save map -> base_link.'
        )

    def _save_current_callback(self, request, response):
        del request

        map_frame = str(self.get_parameter('map_frame').value)
        base_frame = str(self.get_parameter('base_frame').value)
        timeout = float(self.get_parameter('lookup_timeout_sec').value)

        try:
            transform = self._tf_buffer.lookup_transform(
                map_frame,
                base_frame,
                Time(),
                timeout=Duration(seconds=timeout),
            )
        except TransformException as exc:
            response.success = False
            response.message = (
                f'failed to lookup transform {map_frame} -> {base_frame}: {exc}'
            )
            self.get_logger().warn(response.message)
            return response

        try:
            output_file = self._resolve_output_file()
            data = self._load_waypoint_yaml(output_file, map_frame)
            waypoint_name = self._next_waypoint_name(data)
            self._append_transform(data, waypoint_name, transform)
            self._write_waypoint_yaml(output_file, data)
        except Exception as exc:
            response.success = False
            response.message = f'failed to save waypoint: {exc}'
            self.get_logger().error(response.message)
            return response

        x = data['waypoints'][waypoint_name]['position']['x']
        y = data['waypoints'][waypoint_name]['position']['y']
        response.success = True
        response.message = f'saved {waypoint_name}: x={x}, y={y}'
        self.get_logger().info(f'{response.message} -> {output_file}')
        return response

    def _resolve_output_file(self) -> str:
        configured = str(self.get_parameter('output_file').value).strip()
        if configured:
            return os.path.abspath(os.path.expanduser(os.path.expandvars(configured)))

        source_candidate = os.path.abspath(
            os.path.join(os.getcwd(), 'src', 'amr_navigator', 'params', 'waypoints.yaml')
        )
        if os.path.exists(source_candidate):
            return source_candidate

        share_dir = get_package_share_directory('amr_navigator')
        return os.path.join(share_dir, 'params', 'waypoints.yaml')

    def _load_waypoint_yaml(self, path: str, frame_id: str) -> Dict[str, Any]:
        if os.path.exists(path):
            with open(path, 'r', encoding='utf-8') as stream:
                loaded = yaml.safe_load(stream) or {}
            if not isinstance(loaded, dict):
                raise ValueError(f'YAML root must be a map: {path}')
            data = loaded
        else:
            data = {}

        waypoints = data.get('waypoints')
        if waypoints is None:
            data['waypoints'] = {}
        elif not isinstance(waypoints, dict):
            raise ValueError('waypoints must be a map')

        sequence = data.get('sequence')
        if sequence is None:
            data['sequence'] = list(data['waypoints'].keys())
        elif not isinstance(sequence, list):
            raise ValueError('sequence must be a list')

        data['frame_id'] = data.get('frame_id') or frame_id
        return data

    def _next_waypoint_name(self, data: Dict[str, Any]) -> str:
        prefix = str(self.get_parameter('name_prefix').value)
        start_index = 1
        pattern = re.compile(rf'^{re.escape(prefix)}(\d+)$')
        max_index = start_index - 1

        names = set(data.get('waypoints', {}).keys())
        names.update(str(name) for name in data.get('sequence', []))
        for name in names:
            match = pattern.match(str(name))
            if match:
                max_index = max(max_index, int(match.group(1)))

        return f'{prefix}{max_index + 1}'

    def _append_transform(self, data: Dict[str, Any], name: str, transform) -> None:
        precision = int(self.get_parameter('float_precision').value)
        translation = transform.transform.translation
        rotation = transform.transform.rotation

        data['waypoints'][name] = {
            'position': {
                'x': self._round_float(translation.x, precision),
                'y': self._round_float(translation.y, precision),
                'z': self._round_float(translation.z, precision),
            },
            'orientation': {
                'x': self._round_float(rotation.x, precision),
                'y': self._round_float(rotation.y, precision),
                'z': self._round_float(rotation.z, precision),
                'w': self._round_float(rotation.w, precision),
            },
        }

        if name not in data['sequence']:
            data['sequence'].append(name)

    def _write_waypoint_yaml(self, path: str, data: Dict[str, Any]) -> None:
        directory = os.path.dirname(path)
        if directory:
            os.makedirs(directory, exist_ok=True)

        fd, tmp_path = tempfile.mkstemp(
            prefix='.waypoints.',
            suffix='.yaml',
            dir=directory or None,
            text=True,
        )
        try:
            with os.fdopen(fd, 'w', encoding='utf-8') as stream:
                yaml.safe_dump(
                    data,
                    stream,
                    sort_keys=False,
                    allow_unicode=True,
                    default_flow_style=False,
                )
            os.replace(tmp_path, path)
        except Exception:
            if os.path.exists(tmp_path):
                os.unlink(tmp_path)
            raise

    @staticmethod
    def _round_float(value: float, precision: int) -> float:
        return round(float(value), precision)


def main(args=None):
    rclpy.init(args=args)
    node = GoalPoseGenerator()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
