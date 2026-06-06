# ZX200 Autonomy State Machine

## 概要
- opera-sim(PhysX or AGX版)に実装されている,ドラグショベル(ZX200)に対して, 掘削から放土の一連動作を行うサンプルコードである.

## ビルド手順
$ cd ~/ros_ws/src
$ git clone https://github.com/Ryoya1012/ZX200_autonomy_state_machine.git
$ colcon build --packages-select zx200_autonomy
$ source install/setup.bash
$ ros2 run zx200_autonomy state_machine_node

## ハードウェアシステム



