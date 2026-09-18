#!/usr/bin/env python3
"""
sweep_runner.py
===============
Phase 7: The Benchmarking Harness.

Runs N cycles across a list of target cycle times.
Sends /waferflow/run_cycle commands to the PickPlaceFSM, waits for
/waferflow/cycle_metrics, and triggers metrics_logger to save the results.

Usage:
  ros2 run wafer_benchmark sweep_runner
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from wafer_msgs.msg import CycleMetrics, PlacementResult


class SweepRunner(Node):
    def __init__(self):
        super().__init__('sweep_runner')

        self.declare_parameter('cycle_times', [1.5, 1.0, 0.75, 0.5])
        self.declare_parameter('cycles_per_time', 20)

        self.cycle_times = self.get_parameter('cycle_times').value
        self.cycles_per_time = self.get_parameter('cycles_per_time').value

        self.cmd_pub = self.create_publisher(String, '/waferflow/run_cycle', 10)
        self.metrics_sub = self.create_subscription(CycleMetrics, '/waferflow/cycle_metrics', self.metrics_cb, 10)

        self.current_time_idx = 0
        self.current_cycle = 0

        # Start the sweep after a short delay
        self.timer = self.create_timer(2.0, self.start_sweep)

    def start_sweep(self):
        self.timer.cancel()
        self.get_logger().info('Starting benchmark sweep...')
        self.trigger_next()

    def trigger_next(self):
        if self.current_time_idx >= len(self.cycle_times):
            self.get_logger().info('Sweep complete!')
            rclpy.shutdown()
            return

        ct = self.cycle_times[self.current_time_idx]
        msg = String()
        # Alternate A1->B1 and B1->A1 just for simplicity
        if self.current_cycle % 2 == 0:
            msg.data = f'A1->B1:cycle_time={ct}'
        else:
            msg.data = f'B1->A1:cycle_time={ct}'

        self.get_logger().info(f'Triggering: {msg.data} (cycle {self.current_cycle+1}/{self.cycles_per_time})')
        self.cmd_pub.publish(msg)

    def metrics_cb(self, msg: CycleMetrics):
        self.current_cycle += 1
        if self.current_cycle >= self.cycles_per_time:
            self.current_cycle = 0
            self.current_time_idx += 1
            self.get_logger().info('Moving to next cycle time setting.')

        # One-shot timer: settle 0.5s then trigger next cycle
        self._settle_timer = self.create_timer(0.5, self.trigger_delayed)

    def trigger_delayed(self):
        # Cancel immediately so this acts as a one-shot
        self._settle_timer.cancel()
        self.trigger_next()


def main(args=None):
    rclpy.init(args=args)
    node = SweepRunner()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()


if __name__ == '__main__':
    main()
