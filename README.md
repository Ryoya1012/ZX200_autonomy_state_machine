# ZX200 Autonomy State Machine

## 概要
- opera-sim(PhysX or AGX版)に実装されている,ドラグショベル(ZX200)に対して, 掘削から放土の一連動作を行うサンプルコードである.

https://github.com/user-attachments/assets/7415d9ad-dbb2-44c8-9321-0af6f5014437
*Environment of Opera-sim(PhysX)


*Envitonment 0f Opera-sim(AGX)

## ビルド手順
```bash
cd ~/ros_ws/src
git clone https://github.com/Ryoya1012/ZX200_autonomy_state_machine.git
colcon build --packages-select zx200_autonomy
source install/setup.bash
ros2 run zx200_autonomy state_machine_node
```

## ハードウェアシステム



