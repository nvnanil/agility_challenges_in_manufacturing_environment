#!/usr/bin/env python3

import rclpy
import threading
from copy import copy
import time
from rclpy.executors import MultiThreadedExecutor
from naveen_package.my_interface_v2 import CompetitionInterface
from naveen_package.broadcaster_listener import BroadcasterDemo
from ariac_msgs.msg import ( CompetitionState, 
AGVStatus as AGVStatusMsg,
Part as PArtMsg,
Order as OrderMsg,)

def main(args=None):
    rclpy.init(args=args)
    # creating an instance of the class
    interface = CompetitionInterface()
    transform = BroadcasterDemo("broadcaster_node")

    executor = MultiThreadedExecutor()
    executor.add_node(interface)
    # executor.add_node(transform)

    spin_thread = threading.Thread(target=executor.spin)
    spin_thread.start()

    # printing the orders in the terminal
    interface.parse_incoming_order = True

    # calling the service client to start the competition
    interface.start_competition()
    interface.add_objects_to_planning_scene()
    
    for order in interface.orders:
        # interface.get_logger().info(interface._parse_order(order))
        if order.order_type == OrderMsg.KITTING:
            interface.floor_robot_pick_and_place_tray(order.order_task.tray_id, order.order_task.agv_number)
            interface.get_logger().info(f"Placed tray on the AGV:{order.order_task.agv_number}")
            for part in order.order_task.parts:
                if interface.flag:
                    priority = copy(interface.orders[-1])
                    priority : OrderMsg
                    del interface.orders[-1]
                    interface.get_logger().info("Priority order recieved")
                    interface.floor_robot_pick_and_place_tray(priority.order_task.tray_id, priority.order_task.agv_number)
                    for ppart in priority.order_task.parts:
                        # found_ = True
                        interface.floor_robot_pick_bin_part(ppart.part)
                        if not interface.found:
                            interface.get_logger().info("Picking part from conveyor")
                            interface.floor_robot_pick_conveyor_part(ppart.part)
                            interface.found = True
                        interface._floor_robot_place_part_on_kit_tray(priority.order_task.agv_number, ppart.quadrant)
                    interface.flag = False
                    interface.lock_agv_tray(priority.order_task.agv_number)
                    interface.move_agv(priority.order_task.agv_number, priority.order_task.destination)
                    if interface._agv_locations[priority.order_task.agv_number] == AGVStatusMsg.WAREHOUSE:
                        interface.submit_order(priority.order_id)
                interface.get_logger().info(f"Picking part of color: {interface._part_colors[part.part.color]} and type: {interface._part_types[part.part.type]}")
                # Test case for floor robot pickup
                # found = True
                interface.floor_robot_pick_bin_part(part.part)
                # interface.get_logger().info(found)
                if not interface.found:
                    interface.get_logger().info("Picking part from conveyor")
                    interface.floor_robot_pick_conveyor_part(part.part)
                    interface.found = True
                interface._floor_robot_place_part_on_kit_tray(order.order_task.agv_number,part.quadrant)
            # interface.lock_agv_tray(order.order_task.agv_number)
            # interface.move_agv(order.order_task.agv_number, order.order_task.destination)
            # if interface._agv_locations[order.order_task.agv_number] == AGVStatusMsg.WAREHOUSE:
            #     interface.submit_order(order.order_id)
                
    #     # wait time
        # time.sleep(15)
        # if interface.flag:
        #     kitting_order=interface.orders[-1]
        #     interface.get_logger().info("Priority order recieved")
        #     # wait time
        #     time.sleep(15)
        #     # calling the service clients to ship the order
        #     agv_number=kitting_order.order_task.agv_number
        #     interface.lock_agv_tray(agv_number)
        #     interface.move_agv(agv_number, kitting_order.order_task.destination)
        #     # calling the service client to submit the once the AGV has reached the warehouse
        #     if interface._agv_locations[agv_number] == AGVStatusMsg.WAREHOUSE :
        #         interface.submit_order(kitting_order.order_id)
        #     interface.flag = False
        #     continue
        
        # agv_number=kitting_order.order_task.agv_number
        # interface.lock_agv_tray(agv_number)
        # interface.move_agv(agv_number, kitting_order.order_task.destination)
        # if interface._agv_locations[agv_number] == AGVStatusMsg.WAREHOUSE :
        #     interface.submit_order(kitting_order.order_id)
        # break
    # Waiting for order announcements to be done
    
    while not interface.get_competition_state == CompetitionState.ORDER_ANNOUNCEMENTS_DONE:
        pass
    
    # calling the service client to end the competition
    # interface.end_competition()
     # Running the main block
    try:
        while rclpy.ok():
            interface.get_logger().info("Spinning")
    except KeyboardInterrupt:
        interface.end_competition()  # Clean up when the main loop exits
        rclpy.shutdown()
        spin_thread.join()
    
    # rclpy.shutdown()

if __name__ == '__main__':
    main()