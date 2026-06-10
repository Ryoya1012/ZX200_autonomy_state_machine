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
from std_msgs.msg import Bool
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
    RELEASE_1 = 6
    RELEASE_2 = 7
    RETURN_1 = 8
    RETURN_2 = 9
    FINISH = 10

class ZX200StateMachine(Node):
    def __init__(self):
        self.dig_count = 0
        self.max_dig_count = 3
        super().__init__('state_machine_node', parameter_overrides=[Parameter('use_sim_time', Parameter.Type.BOOL, True)])
        # 送信：Unityへの目標角度(rad)指令
        self.pub_cmd = self.create_publisher( JointCmd, '/zx200/front_cmd', 10)
        # 受信：現在の関節角度フィードバック 
        self.sub_state = self.create_subscription( JointState, '/zx200/joint_states', self.joint_state_callback, 10)
        # task_managerとtopic通信interface
        self.sub_start_dig = self.create_subscription( Bool, '/start_dig', self.start_dig_callback, 10)
        self.sub_start_release = self.create_subscription( Bool, '/start_release', self.start_release_callback, 10)
        self.pub_end_dig = self.create_publisher( Bool, '/end_dig', 10)
        self.pub_end_release = self.create_publisher( Bool, '/end_release', 10)
     
        # 変数の初期化
        self.current_state = State.IDLE
        self.joint_names = ['swing_joint','boom_joint','arm_joint','bucket_joint']
        self.current_positions = { name: 0.0 for name in self.joint_names}

        # 関節角度の許容差
        self.tolerance = 0.1

        # 各状態の目標角度
        # [ swing, boom, arm, bucket]
        self.target_angles = {
                    State.IDLE:                 [0.000, -0.595, 1.941, 1.796],
                    State.REACH:                [0.000, -0.026, 0.893, -0.132],
                    State.DIG:                  [0.000, 0.140, 0.874, 0.097],
                    State.DRAG:                 [0.000, 0.140, 1.43, 1.028],
                    State.LIFT:                 [0.000, -0.837, 1.846, 2.168],
                    State.SWING_TO_RELEASE:     [3.136, -0.837, 1.848, 2.168],
                    State.RELEASE_1:            [3.136, -0.511, 1.235, 2.366],
                    State.RELEASE_2:            [3.136, -0.511, 1.234, -0.5],
                    State.RETURN_1:             [3.136, -1.221, 2.53, 2.373],
                    State.RETURN_2:             [0.000, -1.221, 2.53, 2.373],
                    State.FINISH:               [0.000, -0.595, 1.941, 1.796]
                    }
   
        self.cmd_positions = list( self.target_angles[State.IDLE])
        self.start_positions = list( self.target_angles[State.IDLE])
        self.state_start_time = 0.0
        self.move_duration = 0.1
        self.max_speeds = [ 0.5, 0.5, 0.5, 0.5]

        self.dt = 0.1
        self.timer = self.create_timer( self.dt, self.timer_callback)
        self.wait_time = 1.0
        self.reached_time = None
        self.is_running = False
        self.current_job = None 
        self.get_logger().info("--- ZX200 AUTONOMY READY. WAITTING FOR TASKS ---")

    def start_dig_callback( self, msg):
        # 停止中, かつ初期姿勢(IDLE)にいるときのみ掘削開始を受付
        if msg.data and not self.is_running and self.current_state in [ State.IDLE, State.FINISH]:
            self.get_logger().info(">>> TASK RECEIVED: START DIGGING")
            self.current_job = 'dig'
            self.is_running = True
            self.transition_to(State.REACH)

    def start_release_callback( self, msg):
        # 土を抱え, LIFTで待機しているときのみ排土開始を受付
        if msg.data and not self.is_running and self.current_state == State.LIFT:
            self.get_logger().info(">>> TASK RECEIVED: START RELEASE")
            self.current_job = 'release'
            self.is_running = True
            self.transition_to(State.SWING_TO_RELEASE)
       
    def joint_state_callback( self, msg):
            for i, name in enumerate( msg.name):
                if name in self.current_positions:
                    self.current_positions[name] = msg.position[i]

    def transition_to( self, next_state):
            self.start_positions = list( self.cmd_positions)
            self.current_state = next_state

            if next_state == State.IDLE:
                self.cmd_positions = list( self.target_angles[ State.IDLE])
                self.start_positions = list( self.target_angles[ State.IDLE])

            target = self.target_angles[next_state]
            durations = []

            for i in range( len( self.joint_names)):
                dist = abs( target[i] - self.start_positions[i])
                speed = self.max_speeds[i] if self.max_speeds[i] > 0 else 0.01
                durations.append( dist/speed)
            self.move_duration = max( durations)

            if self.move_duration < 0.01:
                self.move_duration = 0.01

            self.state_start_time = self.get_clock().now().nanoseconds /1e9
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
        if not self.is_running:
            cmd_msg = JointCmd()
            cmd_msg.joint_name = self.joint_names
            cmd_msg.position = self.cmd_positions
            self.pub_cmd.publish( cmd_msg)
            return

        target = self.target_angles[self.current_state]
        current_time = self.get_clock().now().nanoseconds / 1e9
        elapsed = current_time - self.state_start_time
        progress = elapsed / self.move_duration

        if progress >= 1.0:
            self.cmd_positions = list(target)
        else:
            a = 12.0
            sig = 1.0 / ( 1.0 + math.exp( -a * ( progress - 0.5)))
            sig_0 = 1.0 / ( 1.0 +math.exp( -a * ( 0.0 - 0.5)))
            sig_1 = 1.0 / ( 1.0 + math.exp( -a * ( 1.0 - 0.5)))
            s = ( sig -sig_0) / ( sig_1 - sig_0)

            for i in range( len(self.joint_names)):
                self.cmd_positions[i] = self.start_positions[i] + (target[i]-self.start_positions[i])*s
        cmd_msg = JointCmd()
        cmd_msg.joint_name = self.joint_names
        cmd_msg.position = self.cmd_positions
        self.pub_cmd.publish( cmd_msg)
      
        if self.is_target_reached():
            if self.current_state == State.IDLE and not self.is_running:
                return

            if self.reached_time is None:
                self.reached_time = current_time
                self.get_logger().info(f"{self.current_state.name} TARGET REACHED. Waiting for {self.wait_time} sec..")

            elif ( current_time - self.reached_time) >= self.wait_time:
                self.reached_time = None

                if self.current_state == State.IDLE and self.is_running:
                    self.transition_to( State.REACH)

                elif self.current_state == State.REACH:
                     self.transition_to(State.DIG)
                elif self.current_state == State.DIG:
                     self.transition_to(State.DRAG)
                elif self.current_state == State.DRAG:
                     self.transition_to(State.LIFT)
                elif self.current_state == State.LIFT:
                     self.transition_to( State.SWING_TO_RELEASE)
                elif self.current_state == State.SWING_TO_RELEASE:
                     self.transition_to( State.RELEASE_1)
                elif self.current_state == State.RELEASE_1:
                     self.transition_to( State.RELEASE_2)
                elif self.current_state == State.RELEASE_2:
                     self.transition_to( State.RETURN_1)
                elif self.current_state == State.RETURN_1:
                     self.transition_to( State.RETURN_2)
                elif self.current_state == State.RETURN_2:
                     self.transition_to( State.FINISH)
                elif self.current_state == State.FINISH:
                     self.get_logger().info("--- SEQUENCE COMPLETE ---")
                     self.dig_count += 1

                     if self.dig_count < self.max_dig_count:
                        self.get_logger().info(f"--- TOTAL DIG/RELEASE COUNT: {self.dig_count} ---")
                        self.transition_to( State.IDLE)
                     else:
                         self.get_logger().info(">>> Max count reached. Triggering release for in120.")
                         self.dig_count = 0
                         self.is_running = False
                         self.current_job = None

                         msg = Bool()
                         msg.data = True
                         self.pub_end_dig.publish( msg)
                         self.transition_to( State.IDLE)

        else:
            self.reached_time = None

def main( args = None):
    rclpy.init(args=args)
    node = ZX200StateMachine()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()  
