# Date Create : 2026/06/06
# Author : Ryoya SATO
# Affiliation : Public Work Research Institute
# License : Apach-2.0
# Target execution environment : Unity PhysX

import rclpy 
from rclpy.node import Node
from rclpy.parameter import Parameter
from sensor_msgs.msg import JointState
from com3_msgs.msg import JointCmd
from enum import Enum
import math


# 状態(State)の定義
class State(Enum):
    IDLE = 0
    REACH = 1
    DIG = 2
    DRAG = 3
    LIFT = 4
    SWING_TO_RELEASE = 5
    RELEASE = 6
    RETURN_1 = 7
    RETURN_2 = 8

class ZX200StateMachine(Node):
    def __init__(self):
        super().__init__('state_machine_node', parameter_overrides=[Parameter('use_sim_time', Parameter.Type.BOOL, True)])
        # 送信：Unityへの目標角度(rad)指令
        self.pub_cmd = self.create_publisher( JointCmd, '/zx200/front_cmd', 10)
        # 受信：現在の関節角度フィードバック 
        self.sub_state = self.create_subscription( JointState, '/zx200/joint_state', self.joint_state_callback, 10)

        # 変数の初期化
        self.current_state = State.IDLE
        self.joint_names = ['swing_joint','boom_joint','arm_joint','bucket_joint']
        self.current_positions = { name: 0.0 for name in self.joint_names}

        # 関節角度の許容差
        self.tolerance = 0.05

        # 各状態の目標角度
        # [ swing, boom, arm, bucket]
        self.target_angles = {
                    State.IDLE:                 [0.000, -0.595, 1.941, 1.796],
                    State.REACH:                [0.000, -0.026, 0.893, -0.132],
                    State.DIG:                  [0.000, 0.140, 0.874, 0.097],
                    State.DRAG:                 [0.000, 0.140, 1.43, 1.028],
                    State.LIFT:                 [0.000, -0.837, 1.846, 2.168],
                    State.SWING_TO_RELEASE:     [3.136, -0.837, 1.848, 2.168],
                    State.RELEASE:              [3.136, -0.687, 0.996, -0.438],
                    State.RETURN_1:             [3.136, -1.221, 2.53, 2.373],
                    State.RETURN_2:             [0.000, -1.221, 2.53, 2.373]
                }

        self.timer = self.create_timer( 0.1, self.timer_callback)

        self.wait_time = 2.5

        self.reached_time = None

        self.is_running = False

    def start_sequence( self):
            self.get_logger().info("---STARTING DIGGING SEQUENSE---")
            self.is_running = True
        
    def joint_state_callback( self, msg):
            for i, name in enumerate( msg.name):
                if name in self.current_positions:
                    self.current_positions[name] = msg.position[i]
    def transition_to( self, next_state):
            self.current_state = next_state
            self.get_logger().info(f"Transitioned to : {self.current_state.name}")
    def is_target_reached( self):
        target = self.target_angles[ self.current_state]

        for i, name in enumerate( self.joint_names):
            current_val = self.current_positions[name]
            target_val = target[i]
            if abs( current_val - target_val) > self.tolerance:
                return False
        return True

    def timer_callback( self):
        cmd_msg = JointCmd()
        cmd_msg.joint_name = self.joint_names
        cmd_msg.position = self.target_angles[ self.current_state]
        self.pub_cmd.publish( cmd_msg)
        
        if self.is_target_reached():
            if self.current_state == State.IDLE and not self.is_running:
                return
            current_time = self.get_clock().now().nanoseconds /1e9
            
            if self.reached_time is None:
                self.reached_time = current_time
                self.get_logger().info(f"{self.current_state.name} TARGET REACHED. Waiting for {self.wait_time} sec..")

            elif ( current_time - self.reached_time) >= self.wait_time:
                self.reached_time = None

                if self.current_state == State.IDLE:
                    if self.is_running:
                        self.transition_to(State.REACH)

                elif self.current_state == State.REACH:
                    self.transition_to(State.DIG)
                elif self.current_state == State.DIG:
                    self.transition_to(State.DRAG)
                elif self.current_state == State.DRAG:
                    self.transition_to(State.LIFT)
                elif self.current_state == State.LIFT:
                    self.transition_to(State.SWING_TO_RELEASE)
                elif self.current_state == State.SWING_TO_RELEASE:
                    self.transition_to(State.RELEASE)
                elif self.current_state == State.RELEASE:
                     self.transition_to( State.RETURN_1)
                elif self.current_state == State.RETURN_1:
                     self.transition_to( State.RETURN_2)
                elif self.current_state == State.RETURN_2:
                     self.transition_to( State.IDLE)
                     self.get_logger().info("--- SEQUENCE COMPLETE ---")
                     self.is_running = False
        else:
            self.reached_time = None

def main( args = None):
    rclpy.init(args=args)
    node = ZX200StateMachine()

    node.start_sequence()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
