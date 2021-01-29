#!/bin/usr/env python3

import rclpy
# from rmf_task_msgs.msg import Loop
from rmf_fleet_msgs.msg import FleetState
from rmf_fleet_msgs.msg import ModeRequest
from rmf_fleet_msgs.msg import DestinationRequest
from rmf_fleet_msgs.msg import PathRequest
from rmf_fleet_msgs.msg import RobotMode
from rmf_fleet_msgs.msg import ModeParameter
from rmf_fleet_msgs.msg import Location

import logging
from pprint import pprint
from tutorial_interfaces.msg import RnaTask 
import random
import time
from rclpy.qos import qos_profile_system_default
from rclpy.qos import QoSProfile

#RNA task reciever

from rclpy.node import Node

class Simple_Pub_Sub(Node):
    def __init__(self):
        super().__init__('simple_pub')
        qos_profile = QoSProfile(depth=20)
        self.fleet_states_dict = {}
        self.timer_period = 1.0  # seconds
        self.fleet_state_subscription = self.create_subscription(
            FleetState, 'fleet_states', self.fleet_state_cb, qos_profile=qos_profile)
        self.mode_publisher = self.create_publisher(ModeRequest, 'mode_requests', 10)
        self.mode_timer = self.create_timer(self.timer_period, self.mode_request_cb)

        self.dest_publisher = self.create_publisher(DestinationRequest, 'destination_requests', 10)
        self.dest_timer = self.create_timer(self.timer_period, self.destination_request_cb)

        self.path_publisher = self.create_publisher(PathRequest, 'path_requests', 10)
        self.path_timer = self.create_timer(self.timer_period, self.path_request_cb)

        self.rna_publisher = self.create_publisher(RnaTask, 'rna_task_reciever', 10)
        self.rnatask_timer = self.create_timer(self.timer_period, self.rna_task_reciever_cb)

    def fleet_state_cb(self, msg: FleetState):
        fleet_name = msg.name
        self.fleet_states_dict[fleet_name] = msg.robots
        pprint(msg)

    def mode_request_cb(self):
        msg = ModeRequest()
        ModeParameter =['rna_bot', str(random.randint(0, 100))]
        msg.fleet_name='rna_fleet'
        msg.robot_name='rna_bot'
        msg.mode=RobotMode(mode = 0)
        msg.task_id= str(random.randint(0, 100))
        # msg.parameters.append(['a','b'])
        self.mode_publisher.publish(msg)
    
    def destination_request_cb(self):
        msg = DestinationRequest()
        msg.fleet_name='rna_fleet'
        msg.robot_name='rna_bot'
        msg.destination.t=self.get_clock().now().to_msg()
        msg.destination.x=random.uniform(10, 100)
        msg.destination.y=random.uniform(10, 100)
        msg.destination.yaw=random.uniform(10, 100)
        msg.destination.level_name = 'B1'
        msg.task_id= str(random.randint(0, 100))
        self.dest_publisher.publish(msg)

    def path_request_cb(self):
        msg = PathRequest()
        msg.fleet_name='rna_fleet'
        msg.robot_name='rna_bot'
        path=Location(t=self.get_clock().now().to_msg(),
            x=random.uniform(10, 100),
            y=random.uniform(10, 100),
            yaw=random.uniform(10, 100),
            level_name = 'B1')
        msg.path.append(path)
        msg.task_id=str(random.randint(0, 100))
        self.path_publisher.publish(msg)

    def rna_task_reciever_cb(self):        
        msg = RnaTask()
        msg.task_id = str(random.randint(0, 100))
        msg.task_name = 'Test'
        msg.schedule_time =str(random.uniform(0,3600))
        msg.schedule_type ='Test'
        msg.priority = random.randint(0,3600)
        msg.robot_name = 'RnaBot'
        msg.patient_name='Patient_Test'
        msg.patient_id=random.randint(0,3600)
        msg.barcode=str(random.uniform(0,3600))
        msg.position.append(random.uniform(0,3600))
        msg.position_1.append(random.uniform(0,3600))
        msg.position_2.append(random.uniform(0,3600))
        msg.item_name=str(random.uniform(0,3600))
        msg.load_operation=str(random.uniform(0,3600))
        msg.escape_priority.append(random.uniform(0,3600))
        msg.nurse_station.append(random.uniform(0,3600))
        msg.home_positiion.append(random.uniform(0,3600))
        self.rna_publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)

    pubsub = Simple_Pub_Sub()

    rclpy.spin(pubsub)
    pubsub.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
