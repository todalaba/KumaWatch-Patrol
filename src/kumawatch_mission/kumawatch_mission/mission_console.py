"""mission_console ノード: ミッションの進み具合を短い英語の行で表示する(動画撮影用).

ROS 2 の長いログの代わりに、別の端末でこれだけを大きく表示すると、審査者に進行が伝わる。
正常な出来事は緑、熊の検知は黄、失敗は赤で表示する。

  ros2 run kumawatch_mission mission_console
"""

import json
import sys
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from kumawatch_mission.mission_text import format_event

GREEN, YELLOW, RED, BOLD, RESET = '\033[92m', '\033[93m', '\033[91m', '\033[1m', '\033[0m'
WARN_EVENTS = {'bear_detected', 'retreat'}
FAIL_EVENTS = {'unreachable', 'timeout', 'retreat_failed', 'return_failed'}


class MissionConsole(Node):

    def __init__(self):
        super().__init__('mission_console')
        self.t0 = None
        self.color = sys.stdout.isatty()
        self.create_subscription(String, '/kumawatch/mission_events', self._on_event, 50)
        self._print('KumaWatch Patrol - mission log (waiting for the mission to start)', BOLD)

    def _print(self, text, color=''):
        if self.color and color:
            text = color + text + RESET
        print(text, flush=True)

    def _on_event(self, msg):
        try:
            ev = json.loads(msg.data)
        except ValueError:
            return
        line = format_event(ev)
        if line is None:
            return
        wall = ev.get('wall_time') or time.time()
        if self.t0 is None:
            self.t0 = wall
        t = wall - self.t0
        kind = ev.get('event')
        color = YELLOW + BOLD if kind in WARN_EVENTS else RED if kind in FAIL_EVENTS else GREEN
        if kind == 'mission_completed':
            color = GREEN + BOLD
        self._print('[%02d:%02d] %s' % (t // 60, t % 60, line), color)


def main(args=None):
    rclpy.init(args=args)
    node = MissionConsole()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
