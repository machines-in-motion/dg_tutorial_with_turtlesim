#!/usr/bin/env python

import os
import time
import rclpy
from dg_tutorial_with_turtlesim import follow_target_graph
from dynamic_graph_manager.dynamic_graph_manager_client import DynamicGraphManagerClient


def main(args=None):

    rclpy.init(args=args)

    dgm_client = DynamicGraphManagerClient()

    graph_script = os.path.abspath(follow_target_graph.__file__)

    print("Load graph!")
    dgm_client.run_python_script(graph_script)
    dgm_client.run_python_command("print('Graph loaded.')")

    time.sleep(2)
    print("Start dynamic graph!")
    dgm_client.start_tracer()
    dgm_client.start_dynamic_graph()

    time.sleep(10)

    dgm_client.stop_tracer()

    while rclpy.ok():
        time.sleep(1)


if __name__ == "__main__":
    main()
