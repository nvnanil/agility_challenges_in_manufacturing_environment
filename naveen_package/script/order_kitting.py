#!/usr/bin/env python3

import rclpy
import threading
from copy import copy
import time
from rclpy.executors import MultiThreadedExecutor
from naveen_package.my_interface import CompetitionInterface

def main(args=None):
    rclpy.init(args=args)
    # Creating an instance of the class
    interface = CompetitionInterface()

    # Create a multithreaded executor
    executor = MultiThreadedExecutor()
    executor.add_node(interface)

    spin_thread = threading.Thread(target=executor.spin)
    spin_thread.start()
    # Complete the orders published by the ARIAC manager
    interface.complete_orders()

    # Spin the node until ctrl+c is pressed
    try:
        while rclpy.ok():
            interface.get_logger().info("Spinning")
    except KeyboardInterrupt:
        interface.end_competition()  # Clean up when the main loop exits
        rclpy.shutdown()
        spin_thread.join()

if __name__ == '__main__':
    main()