#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Launch arguments
        DeclareLaunchArgument(
            'fastlio_odom_topic',
            default_value='/Odometry',
            description='Odometry topic name'
        ),
        DeclareLaunchArgument(
            'log_directory',
            default_value='~/unified_scans',
            description='Base directory for data logging'
        ),
        DeclareLaunchArgument(
            'use_seekvision_mode',
            default_value='true',
            description='Use SeekVision pipeline for thermal camera'
        ),
        DeclareLaunchArgument(
            'save_color_png',
            default_value='true',
            description='Save colorized thermal images as PNG'
        ),
        DeclareLaunchArgument(
            'csv_flush_every_rows',
            default_value='50',
            description='Flush CSV every N rows'
        ),
        
        # Video streaming parameters
        DeclareLaunchArgument(
            'left_device',
            default_value='/dev/v4l/by-id/See3CAM_Left-video-index0',
            description='Left camera device path'
        ),
        DeclareLaunchArgument(
            'right_device',
            default_value='/dev/v4l/by-id/See3CAM_Right-video-index0',
            description='Right camera device path'
        ),
        DeclareLaunchArgument(
            'stream_host',
            default_value='172.16.10.121',
            description='Streaming target host IP'
        ),
        DeclareLaunchArgument(
            'stream_port',
            default_value='5600',
            description='Streaming target port'
        ),
        DeclareLaunchArgument(
            'stream_bitrate_kbps',
            default_value='800',
            description='Streaming bitrate in kbps'
        ),
        DeclareLaunchArgument(
            'rtp_mtu',
            default_value='1200',
            description='RTP MTU size'
        ),
        DeclareLaunchArgument(
            'use_mjpeg_pipeline',
            default_value='true',
            description='Use MJPEG pipeline for cameras'
        ),
        DeclareLaunchArgument(
            'cap_w',
            default_value='1920',
            description='Capture width'
        ),
        DeclareLaunchArgument(
            'cap_h',
            default_value='1080',
            description='Capture height'
        ),
        DeclareLaunchArgument(
            'cap_fps',
            default_value='30',
            description='Capture framerate'
        ),
        DeclareLaunchArgument(
            'raw_format',
            default_value='UYVY',
            description='Raw video format'
        ),
        DeclareLaunchArgument(
            'raw_w',
            default_value='1280',
            description='Raw capture width'
        ),
        DeclareLaunchArgument(
            'raw_h',
            default_value='720',
            description='Raw capture height'
        ),
        DeclareLaunchArgument(
            'raw_fps',
            default_value='60',
            description='Raw capture framerate'
        ),
        
        # Unified Data Collector Node
        Node(
            package='pilot_control',
            executable='unified_data_collector',
            name='unified_data_collector',
            output='screen',
            parameters=[{
                'fastlio_odom_topic': LaunchConfiguration('fastlio_odom_topic'),
                'log_directory': LaunchConfiguration('log_directory'),
                'use_seekvision_mode': LaunchConfiguration('use_seekvision_mode'),
                'save_color_png': LaunchConfiguration('save_color_png'),
                'csv_flush_every_rows': LaunchConfiguration('csv_flush_every_rows'),
                'left_device': LaunchConfiguration('left_device'),
                'right_device': LaunchConfiguration('right_device'),
                'stream_host': LaunchConfiguration('stream_host'),
                'stream_port': LaunchConfiguration('stream_port'),
                'stream_bitrate_kbps': LaunchConfiguration('stream_bitrate_kbps'),
                'rtp_mtu': LaunchConfiguration('rtp_mtu'),
                'use_mjpeg_pipeline': LaunchConfiguration('use_mjpeg_pipeline'),
                'cap_w': LaunchConfiguration('cap_w'),
                'cap_h': LaunchConfiguration('cap_h'),
                'cap_fps': LaunchConfiguration('cap_fps'),
                'raw_format': LaunchConfiguration('raw_format'),
                'raw_w': LaunchConfiguration('raw_w'),
                'raw_h': LaunchConfiguration('raw_h'),
                'raw_fps': LaunchConfiguration('raw_fps'),
            }],
            remappings=[
                ('/Odometry', LaunchConfiguration('fastlio_odom_topic')),
            ]
        ),
    ])

