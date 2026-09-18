#!/usr/bin/env python3
"""
metrics_logger.py
=================
Listens to PlacementResult and CycleMetrics and writes them to a CSV file.
"""
import rclpy
from rclpy.node import Node
import csv
import os
from datetime import datetime
from wafer_msgs.msg import PlacementResult, CycleMetrics
import collections

class MetricsLogger(Node):
    def __init__(self):
        super().__init__('metrics_logger')
        
        # Save path
        pkg_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
        res_dir = os.path.join(pkg_dir, 'results')
        os.makedirs(res_dir, exist_ok=True)
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        self.csv_path = os.path.join(res_dir, f'sweep_{timestamp}.csv')
        
        self.file = open(self.csv_path, 'w', newline='')
        self.writer = csv.writer(self.file)
        self.writer.writerow([
            'timestamp', 'cycle_index', 'target_cycle_time', 'actual_cycle_time',
            'peak_jerk', 'pos_error_mm', 'rot_error_deg', 'success', 'phase_failed'
        ])
        
        # Buffer to match metrics with placement results
        self.buffer = collections.defaultdict(dict)

        self.sub_metrics = self.create_subscription(CycleMetrics, '/waferflow/cycle_metrics', self.metrics_cb, 10)
        self.sub_placement = self.create_subscription(PlacementResult, '/waferflow/placement_result', self.placement_cb, 10)
        
        self.get_logger().info(f'Logging metrics to {self.csv_path}')

    def metrics_cb(self, msg: CycleMetrics):
        idx = msg.cycle_index
        self.buffer[idx]['actual_time'] = msg.cycle_time
        self.buffer[idx]['peak_jerk'] = msg.peak_jerk
        self.buffer[idx]['phase_failed'] = msg.phase_failed
        self.check_write(idx)

    def placement_cb(self, msg: PlacementResult):
        idx = msg.cycle_index
        self.buffer[idx]['target_time'] = msg.cycle_time_target
        self.buffer[idx]['pos_error_mm'] = msg.position_error * 1000.0
        import math
        self.buffer[idx]['rot_error_deg'] = math.degrees(msg.orientation_error)
        self.buffer[idx]['success'] = msg.success
        self.check_write(idx)

    def check_write(self, idx):
        b = self.buffer[idx]
        if 'actual_time' in b and 'target_time' in b:
            self.writer.writerow([
                datetime.now().isoformat(),
                idx,
                b['target_time'],
                b['actual_time'],
                b['peak_jerk'],
                b['pos_error_mm'],
                b['rot_error_deg'],
                b['success'],
                b['phase_failed']
            ])
            self.file.flush()
            del self.buffer[idx]

def main(args=None):
    rclpy.init(args=args)
    node = MetricsLogger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.file.close()
        node.destroy_node()

if __name__ == '__main__':
    main()
