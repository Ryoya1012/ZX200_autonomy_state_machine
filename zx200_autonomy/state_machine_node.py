# Date Create : 2026/06/06
# Author : Ryoya SATO
# Affiliation : Public Work Research Institute
# License : Apach-2.0
# Target execution environment : Unity AGX

import rclpy 
from rclpy.node import Node
from rclpy.parameter import Parameter 
from sensor_msgs.msg import JointState  # 現在の関節角度を受信用
from com3_msgs.msg import JointCmd      # 目標の関節角度を送信用
from std_msgs.msg import Bool           # True/Falseのフラグ送受信用
from enum import Enum                   # 状態を名前で管理するための機能
import math                             # シグモイド関数の計算用

# ======================================= #
# [状態定義] State
# zx200(ドラグショベル)が現在「どんなポーズ・作業をしているか」を名前で管理
# ======================================= #

class State(Enum):
    IDLE = 0                            # 待機状態(初期姿勢)
    REACH = 1                           # 掘削開始姿勢
    DIG = 2                             # バケット刃先貫入
    DRAG = 3                            # 引き寄せ
    LIFT = 4                            # 持ち上げ
    SWING_TO_RELEASE = 5                # 旋回(排土位置へ向かう)
    RELEASE_1 = 6                       # 排土1(boomを降ろしつつ, armを伸ばす)
    RELEASE_2 = 7                       # 排土2(バケットを開く)
    RETURN_1 = 8                        # 戻り1(旋回姿勢)
    RETURN_2 = 9                        # 戻り2(旋回)
    FINISH = 10                         # 待機姿勢

# ======================================== #
# [メインクラス] zx200StateMachine
# ドラグショベル(zx200)の腕の動きを順番通りに制御する「脳みそ」となるクラス
# ======================================== #

class ZX200StateMachine(Node):
    def __init__(self):
        # 掘削の回数をカウントする変数
        self.dig_count = 0
        # 掘削回数の上限
        self.max_dig_count = 1
        # nodeの名前を'state_nachine_node'として登録し, シミュレーション時間を使用する設定
        super().__init__('state_machine_node', parameter_overrides=[Parameter('use_sim_time', Parameter.Type.BOOL, True)])
        
        # ----通信の準備(送信と受信口)---

        # 送信：Unityへの目標角度(rad)指令
        # トピック名(/zx200/front_cmd)にデータの型(JointCmd)を流す
        self.pub_cmd = self.create_publisher( JointCmd, '/zx200/front_cmd', 10)

        # 受信：現在の関節角度フィードバック 
        # トピック名(/zx200/joint_states)にデータの型(JointCmd)が流れてきたら, 処理(self.joint_state_callback)を行う
        self.sub_state = self.create_subscription( JointState, '/zx200/joint_states', self.joint_state_callback, 10)

        # [タスク管理用の通信] 外部(Task Manager等)から作業の開始/終了を受け渡しする
        # トピック名(/start_dig)にデータ型(Bool)のデータが流れてきたら, 処理(self.start_dig_callback)を行う
        self.sub_start_dig = self.create_subscription( Bool, '/start_dig', self.start_dig_callback, 10)
        # トピック名(/start_release)にデータ型(Bool)のデータが流れてきたら処理(self.start_release_callback)を行う
        self.sub_start_release = self.create_subscription( Bool, '/start_release', self.start_release_callback, 10)
        # トピック名(/end_dig)にデータ型(Bool)のデータを流す
        self.pub_end_dig = self.create_publisher( Bool, '/end_dig', 10)
        # トピック名(/end_release)にデータ型(Bool)のデータを流す
        self.pub_end_release = self.create_publisher( Bool, '/end_release', 10)
     

        # ============================ 
        # 2. 変数と目標角度の初期化
        # ============================

        self.current_state = State.IDLE     # 起動時は[待機状態]からスタート

        # 動かす関数の名前リスト(※ Unity側の名前と一致させる必要がある)
        self.joint_names = ['swing_joint','boom_joint','arm_joint','bucket_joint']

        # 現在の関節角度をメモしておくための辞書(最初はすべて0.0)
        self.current_positions = { name: 0.0 for name in self.joint_names}

        # 目標角度にどれくらい近づけたら[到着した]と判定するのかの許容誤差(rad)
        self.tolerance = 0.1

        # --- 各状態(State)の目標姿勢 ---
        # 順番はself.joint_namesと同じ[ swing(旋回), boom(ブーム), arm(アーム), bucket(バケット)]の角度
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
   
        # 動作計算用のメモリ変数
        self.cmd_positions = list( self.target_angles[State.IDLE])      # 現在送信している指令値
        self.start_positions = list( self.target_angles[State.IDLE])    # 動き始めた時の角度
        self.state_start_time = 0.0                                     # 今の状態が始まった時間
        self.move_duration = 0.1                                        # 動き終わるまでの目標時間(秒)
        self.max_speeds = [ 0.5, 0.5, 0.5, 0.5]                         # 各関節の最大スピード

        # 定期実行タイマー( 0.1秒ごとに timer_callbackを呼び出して滑らかに動かす)
        self.dt = 0.1
        self.timer = self.create_timer( self.dt, self.timer_callback)

        self.wait_time = 1.0                                            # 次の動作に移る前の[タメ(待ち時間)]
        self.reached_time = None
        self.is_running = False
        self.current_job = None 
        self.get_logger().info("--- ZX200 AUTONOMY READY. WAITTING FOR TASKS ---")

    # ---- [受信処理1] 掘削開始の合図が来たとき ---
    def start_dig_callback( self, msg):
        # 条件 : 合図(True)が受信されていて, 今動いていないかつ[待機(IDLE)]か[完了(FINISH)]状態のときだけ
        if msg.data and not self.is_running and self.current_state in [ State.IDLE, State.FINISH]:
            self.get_logger().info(">>> TASK RECEIVED: START DIGGING")
            self.current_job = 'dig'
            self.is_running = True

            # REACHの姿勢に移行する指令を出す
            self.transition_to(State.REACH)
    
    # --- [受信処理2] 排土開始の合図が来たとき ---
    def start_release_callback( self, msg):
        # 条件 : 合図が来ていて, 今動いていなくて, かつ[持ち上げ(LIFT)して待っている]状態のときだけ
        if msg.data and not self.is_running and self.current_state == State.LIFT:
            self.get_logger().info(">>> TASK RECEIVED: START RELEASE")
            self.current_job = 'release'
            self.is_running = True

            # SWING_TO_RELEASE(排土場所へ旋回)状態へ移行しろ, と指令を出す
            self.transition_to(State.SWING_TO_RELEASE)

    # --- [受信処理3] Unityから現在の関節角度が送られてきたとき ---       
    def joint_state_callback( self, msg):
        # msg.name に入っている関数名(例 : 'boom_joint')を一つずつ確認
        # 最新の関節角度を保存
            for i, name in enumerate( msg.name):
                if name in self.current_positions:
                    self.current_positions[name] = msg.position[i]

    # --- 状態(State)を切り替えるときの準備処理 ---
    def transition_to( self, next_state):

        # 1. 動き始める前の[スタート地点]の角度を記憶しておく
            self.start_positions = list( self.cmd_positions)
            self.current_state = next_state     # 状態を更新

            # ※ もしIDLEに戻るなら, スタート地点を強制的にIDLEの角度にリセットする特別ルール(意味がわからない)
            if next_state == State.IDLE:
                self.cmd_positions = list( self.target_angles[ State.IDLE])
                self.start_positions = list( self.target_angles[ State.IDLE])


            # 2. 次の目標角度を取得
            target = self.target_angles[next_state]
            durations = []  # 各関節が動くのにかかる時間を計算するためのリスト

            # 3. 4つの関節それぞれについて「移動距離÷スピード=かかる時間」を計算
            for i in range( len( self.joint_names)):
                dist = abs( target[i] - self.start_positions[i])    # 距離(差の絶対値)
                speed = self.max_speeds[i] if self.max_speeds[i] > 0 else 0.01
                durations.append( dist/speed)

            # 4. 4つの関節の中で「一番時間がかかる関節」に全体の目標時間(move_duration)を合わせる
            self.move_duration = max( durations)
            if self.move_duration < 0.01:
                self.move_duration = 0.01

            # 5. [ストップウォッチをリセット]
            self.state_start_time = self.get_clock().now().nanoseconds /1e9
            self.get_logger().info(f"Transitioned to : {self.current_state.name}")

    # --- 目標の関節角度に到達したかを判定する処理 ---
    def is_target_reached( self):
        target = self.target_angles[ self.current_state]

        # 現在の関節角度と目標の角度を比較し, 許容誤差(.1rad)以内に収まったら到達と判断
        for i, name in enumerate( self.joint_names):
            current_val = self.current_positions[name]
            target_val = target[i]
            if abs( current_val - target_val) > self.tolerance:
                return False
        return True

    # --- メインループ ---
    def timer_callback( self):
        # もし動いていないときは, 今の目標角度をそのまま送信し続けて姿勢をキープ
        if not self.is_running:
            cmd_msg = JointCmd()
            cmd_msg.joint_name = self.joint_names
            cmd_msg.position = self.cmd_positions
            self.pub_cmd.publish( cmd_msg)
            return

        target = self.target_angles[self.current_state]

        # --- ストップウォッチの計算 ---
        current_time = self.get_clock().now().nanoseconds / 1e9
        elapsed = current_time - self.state_start_time      # 前回の動作終了時から経過した時間
        progress = elapsed / self.move_duration             # 進捗率 (0.0 ~ 1.0)

        # 時間が目標を超えていたら場合, 目標角度で固定する
        if progress >= 1.0:
            self.cmd_positions = list(target)
        else:
            # --- POINT TO POINT 間の状態遷移を滑らかにするためシグモイド関数を使用
            a = 12.0
            sig = 1.0 / ( 1.0 + math.exp( -a * ( progress - 0.5)))
            sig_0 = 1.0 / ( 1.0 +math.exp( -a * ( 0.0 - 0.5)))
            sig_1 = 1.0 / ( 1.0 + math.exp( -a * ( 1.0 - 0.5)))

            # S字カーブの補正をかけた[新しい進捗率(s)]
            s = ( sig -sig_0) / ( sig_1 - sig_0)

            # スタート角度 + (目標までの距離 × S字カーブの進捗率)で, 今の瞬間の理想の関節角度を指令
            for i in range( len(self.joint_names)):
                self.cmd_positions[i] = self.start_positions[i] + (target[i]-self.start_positions[i])*s
        # 計算した角度を組み立て, Unityへ送信
        cmd_msg = JointCmd()
        cmd_msg.joint_name = self.joint_names
        cmd_msg.position = self.cmd_positions
        self.pub_cmd.publish( cmd_msg)
      
        # --状態の自動進行(シーケンス)処理 ---
        # 目標に到達したかの確認

        if self.is_target_reached():
            if self.current_state == State.IDLE and not self.is_running:
                return

            # 到達した瞬間の時間を記録し,  wait_time(1s)だけ待機
            if self.reached_time is None:
                self.reached_time = current_time
                self.get_logger().info(f"{self.current_state.name} TARGET REACHED. Waiting for {self.wait_time} sec..")

            # 待機時間終了後, 次の状態へ遷移
            elif ( current_time - self.reached_time) >= self.wait_time:
                self.reached_time = None

                if self.current_state == State.IDLE and self.is_running:
                    self.transition_to( State.REACH)

                # 自動状態遷移ルール
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

                     # 指定回数区掘削していなければ, IDLE状態に戻り繰り返し掘削
                     if self.dig_count < self.max_dig_count:
                        self.get_logger().info(f"--- TOTAL DIG/RELEASE COUNT: {self.dig_count} ---")
                        self.transition_to( State.IDLE)

                     # 指定回数掘削を行ったら, task_managerに報告
                     else:
                         self.get_logger().info(">>> Max count reached. Triggering release for in120.")
                         self.dig_count = 0
                         self.is_running = False
                         self.current_job = None
                        
                         # end_dig トピックにTrueを送る
                         msg = Bool()
                         msg.data = True
                         self.pub_end_dig.publish( msg)
                         self.transition_to( State.IDLE)

        else:
            # まだ到達していない場合は, 到達判定用の時間をリセット
            self.reached_time = None

def main( args = None):
    rclpy.init(args=args)   # ROS2を開始
    node = ZX200StateMachine()  # 設計図から実体を作る
    try:
        rclpy.spin(node)    # nodeを待機状態にする(ここでloopさせる)
    except KeyboardInterrupt:
        pass                # Ctrl + Cで終了した場合はエラーを吐かない
    finally:
        node.destroy_node() # nodeを殺す
        rclpy.shutdown()    # ROS2を終了

if __name__ == '__main__':
    main()  
