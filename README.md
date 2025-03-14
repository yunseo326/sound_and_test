모델받아서 로봇 제어하는 코드 
robot_connection-master/src/my_test_pkg_py/control.py

음원에서 각도 pub 해주는 코드
Sound-source-localization-using-TDOA-master/final/main.cpp

음원에서 나온 각도 받는 코드
robot_connection-master/src/my_test_pkg_py/receive_angle.py

움직이는 코드
```
cd robot_connection-master
colcon build --symlink-install --packages-select my_test_pkg_py
source install/setup.bash

ros2 run my_test_pkg_py control
ros2 run my_test_pkg_py receive
```

해야할꺼 
1. 음원 cmake 수정 혹은 python으로 돌리기
