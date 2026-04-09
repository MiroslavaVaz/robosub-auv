#!/usr/bin/env python3

import csv
import os
import matplotlib.pyplot as plt
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from nav_msgs.msg import Path, Odometry


class TrajectoryPlotter(Node):
    def __init__(self):
        super().__init__('trajectory_plotter_node')

        self.planned_path = []
        self.actual_path = []

        self.path_received = False
        self.path_complete = False
        self.plot_saved = False

        self.path_sub = self.create_subscription(
            Path, '/planned_path', self.path_callback, 10
        )
        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10
        )
        self.complete_sub = self.create_subscription(
            Bool, '/path_complete', self.complete_callback, 10
        )

        self.timer = self.create_timer(0.5, self.check_and_plot)

        self.get_logger().info('Trajectory plotter node started.')

    def path_callback(self, msg: Path):
        self.planned_path = []
        for pose_stamped in msg.poses:
            x = pose_stamped.pose.position.x
            y = pose_stamped.pose.position.y
            self.planned_path.append((x, y))

        if len(self.planned_path) > 0:
            self.path_received = True
            self.path_complete = False
            self.plot_saved = False
            self.actual_path = []
            self.get_logger().info(
                f'Received planned path with {len(self.planned_path)} points.'
            )

    def odom_callback(self, msg: Odometry):
        if not self.path_received or self.plot_saved:
            return

        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        self.actual_path.append((x, y))

    def complete_callback(self, msg: Bool):
        self.path_complete = msg.data
        if self.path_complete:
            self.get_logger().info('Received path complete signal.')

    def check_and_plot(self):
        if not self.path_received or not self.path_complete or self.plot_saved:
            return

        self.save_csv()
        self.make_plot()
        self.plot_saved = True

        self.get_logger().info('Plot saved after path completion. Shutting down plotter node.')
        rclpy.shutdown()

    def save_csv(self):
        output_dir = os.path.expanduser('~/auv_ws')
        planned_file = os.path.join(output_dir, 'planned_path.csv')
        actual_file = os.path.join(output_dir, 'actual_path.csv')

        with open(planned_file, 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(['x', 'y'])
            writer.writerows(self.planned_path)

        with open(actual_file, 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(['x', 'y'])
            writer.writerows(self.actual_path)

        self.get_logger().info(f'Saved {planned_file}')
        self.get_logger().info(f'Saved {actual_file}')

    def make_plot(self):
        planned_x = [p[0] for p in self.planned_path]
        planned_y = [p[1] for p in self.planned_path]

        actual_x = [p[0] for p in self.actual_path]
        actual_y = [p[1] for p in self.actual_path]

        print("First 10 actual points:", self.actual_path[:10])
        print("Last 10 actual points:", self.actual_path[-10:])
        print("Total actual samples:", len(self.actual_path))

        plt.figure(figsize=(13.6, 7.9))
        plt.tight_layout() 

        plt.plot(planned_x, planned_y, '--', linewidth=2.5, label='Planned Path')
        plt.plot(actual_x, actual_y, '-', linewidth=2.5, label='Simulated Path')

        if len(self.planned_path) > 0:
            plt.scatter(planned_x[0], planned_y[0], s=120, label='Start')
            plt.scatter(planned_x[-1], planned_y[-1], marker='x', s=100, label='Goal/End')

        if len(self.planned_path) > 2:
            landmark_x = [p[0] for p in self.planned_path[1:-1]]
            landmark_y = [p[1] for p in self.planned_path[1:-1]]
            plt.scatter(landmark_x, landmark_y, marker='*', s=220, label='Landmarks')
        
        labels = ['Start/End', 'P1','P2','P3']
        for i, (lx, ly) in enumerate(self.planned_path[:-1]):
            plt.text(lx + 0.03, ly + 0.03, labels[i], fontsize=28)

        plt.xlabel('X Position [m]', fontsize=28, fontname='Arial')
        plt.ylabel('Y Position [m]', fontsize=28, fontname='Arial')
        plt.title('\nPlanned vs. Simulated Trajectory\n', fontsize = 40, fontname = 'Arial')
        plt.legend(prop={'family': 'Arial', 'size': 28}, bbox_to_anchor=(1.02, 1), loc='upper left')
        plt.grid(True, linewidth=1.0)
        plt.xticks(fontsize=18)
        plt.yticks(fontsize=18)
        x_all = planned_x + actual_x
        y_all = planned_y + actual_y

        plt.xlim(min(x_all) - 1.0, max(x_all) + 1.0)
        plt.ylim(min(y_all) - 0.8, max(y_all) + 0.8)
        plt.gca().set_aspect('equal', adjustable='box')       
        output_file = os.path.expanduser('~/auv_ws/trajectory_plot.png')
        plt.savefig(output_file, dpi=600, bbox_inches='tight', pad_inches=0.25)
        plt.subplots_adjust(left=0.12, right=0.82, top=0.88, bottom=0.12)

        plt.show()

        self.get_logger().info(f'Saved plot to {output_file}')


def main(args=None):
    rclpy.init(args=args)
    node = TrajectoryPlotter()
    rclpy.spin(node)


if __name__ == '__main__':
    main()