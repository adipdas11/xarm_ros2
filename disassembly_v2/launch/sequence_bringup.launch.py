#!/usr/bin/env python3
import os

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # common
    pkg_name = 'disassembly_v2'

    # 1) Part‐Graph Constructor (real mode)
    part_graph_constructor = Node(
        package=pkg_name,
        executable='part_graph_constructor.py',
        name='part_graph_constructor',
        parameters=[{'mode': 'real'}],
        output='log',
    )

    # 2) Topological Sort
    topological_sort = Node(
        package=pkg_name,
        executable='topological_sort.py',
        name='topological_sort',
        output='log',
    )

    # 3) Task Distributor
    task_distributor = Node(
        package=pkg_name,
        executable='task_distributor.py',
        name='task_distributor',
        output='log',
    )

    # 4) Part‐Graph Visualizer (GUI)
    part_graph_visualizer = Node(
        package=pkg_name,
        executable='part_graph_visualizer.py',
        name='part_graph_visualizer',
        output='log',
    )

    # return all nodes as a single LaunchDescription
    return LaunchDescription([
        part_graph_constructor,
        topological_sort,
        task_distributor,
        part_graph_visualizer,
    ])
