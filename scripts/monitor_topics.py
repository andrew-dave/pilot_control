#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
import time
from collections import defaultdict, deque
import threading

class TopicMonitor(Node):
    def __init__(self):
        super().__init__('topic_monitor')
        
        # Store message counts and sizes
        self.topic_stats = defaultdict(lambda: {
            'count': 0,
            'last_time': time.time(),
            'rate': 0.0,
            'sizes': deque(maxlen=100)  # Keep last 100 message sizes
        })
        
        # Start monitoring thread
        self.monitor_thread = threading.Thread(target=self.monitor_loop, daemon=True)
        self.monitor_thread.start()
        
        self.get_logger().info("Topic monitor started. Press Ctrl+C to stop.")
    
    def monitor_loop(self):
        """Monitor and print statistics every 5 seconds"""
        while rclpy.ok():
            time.sleep(5.0)
            self.print_stats()
    
    def print_stats(self):
        """Print current topic statistics"""
        self.get_logger().info("=" * 60)
        self.get_logger().info("TOPIC MONITORING STATISTICS")
        self.get_logger().info("=" * 60)
        
        # Sort topics by message rate
        sorted_topics = sorted(
            self.topic_stats.items(),
            key=lambda x: x[1]['rate'],
            reverse=True
        )
        
        for topic_name, stats in sorted_topics:
            if stats['rate'] > 0.1:  # Only show topics with rate > 0.1 Hz
                avg_size = sum(stats['sizes']) / len(stats['sizes']) if stats['sizes'] else 0
                self.get_logger().info(
                    f"{topic_name:<30} | "
                    f"Rate: {stats['rate']:>6.1f} Hz | "
                    f"Count: {stats['count']:>6d} | "
                    f"Avg Size: {avg_size:>8.0f} bytes"
                )
        
        self.get_logger().info("=" * 60)
    
    def message_callback(self, topic_name, msg):
        """Generic message callback to track statistics"""
        stats = self.topic_stats[topic_name]
        current_time = time.time()
        
        # Calculate rate (messages per second)
        time_diff = current_time - stats['last_time']
        if time_diff > 0:
            stats['rate'] = 1.0 / time_diff
        
        stats['count'] += 1
        stats['last_time'] = current_time
        
        # Estimate message size (rough approximation)
        try:
            msg_size = len(str(msg))
            stats['sizes'].append(msg_size)
        except:
            stats['sizes'].append(1000)  # Default size if we can't measure
    
    def subscribe_to_topic(self, topic_name, msg_type):
        """Subscribe to a topic to monitor it"""
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            depth=1
        )
        
        # Create a callback that captures the topic name
        def callback(msg):
            self.message_callback(topic_name, msg)
        
        try:
            self.create_subscription(
                msg_type,
                topic_name,
                callback,
                qos
            )
            self.get_logger().info(f"Monitoring topic: {topic_name}")
        except Exception as e:
            self.get_logger().warn(f"Failed to monitor {topic_name}: {e}")

def main():
    rclpy.init()
    
    monitor = TopicMonitor()
    
    # Monitor common high-frequency topics
    from sensor_msgs.msg import PointCloud2, Imu
    from nav_msgs.msg import Odometry, Path
    from geometry_msgs.msg import Twist
    from livox_ros_driver2.msg import CustomMsg
    
    topics_to_monitor = [
        ('/livox/lidar', CustomMsg),
        ('/livox/imu', Imu),
        ('/cloud_registered', PointCloud2),
        ('/cloud_registered_body', PointCloud2),
        ('/cloud_effected', PointCloud2),
        ('/Laser_map', PointCloud2),
        ('/Odometry', Odometry),
        ('/path', Path),
        ('/cmd_vel', Twist),
        ('/odom', Odometry),
    ]
    
    for topic_name, msg_type in topics_to_monitor:
        monitor.subscribe_to_topic(topic_name, msg_type)
    
    try:
        rclpy.spin(monitor)
    except KeyboardInterrupt:
        monitor.get_logger().info("Monitoring stopped by user")
    finally:
        monitor.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main() 