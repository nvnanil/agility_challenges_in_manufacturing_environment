#!/usr/bin/env python3

import rclpy
import threading
import time
from rclpy.executors import MultiThreadedExecutor
from naveen_package.my_interface import CompetitionInterface
from ariac_msgs.msg import ( CompetitionState, 
AGVStatus as AGVStatusMsg)

def main(args=None):
    rclpy.init(args=args)
    # creating an instance of the class
    interface = CompetitionInterface(enable_moveit=False)

    executor = MultiThreadedExecutor()
    executor.add_node(interface)

    spin_thread = threading.Thread(target=executor.spin)
    spin_thread.start()

    # printing the orders in the terminal
    interface.parse_incoming_order = True

    # calling the service client to start the competition
    interface.start_competition()
    
    while True:
        kitting_order=interface.orders[0]
        # wait time
        time.sleep(15)
        if interface.flag:
            kitting_order=interface.orders[-1]
            interface.get_logger().info("Priority order recieved")
            # wait time
            time.sleep(15)
            # calling the service clients to ship the order
            agv_number=kitting_order.order_task.agv_number
            interface.lock_agv_tray(agv_number)
            interface.move_agv(agv_number, kitting_order.order_task.destination)
            # calling the service client to submit the once the AGV has reached the warehouse
            if interface._agv_locations[agv_number] == AGVStatusMsg.WAREHOUSE :
                interface.submit_order(kitting_order.order_id)
            interface.flag = False
            continue
        
        agv_number=kitting_order.order_task.agv_number
        interface.lock_agv_tray(agv_number)
        interface.move_agv(agv_number, kitting_order.order_task.destination)
        if interface._agv_locations[agv_number] == AGVStatusMsg.WAREHOUSE :
            interface.submit_order(kitting_order.order_id)
        break
    # Waiting for order announcements to be done
    while not interface.get_competition_state == CompetitionState.ORDER_ANNOUNCEMENTS_DONE:
        pass
    
    # calling the service client to end the competition
    interface.end_competition()
    spin_thread.join()
    
    rclpy.shutdown()

if __name__ == '__main__':
    main()