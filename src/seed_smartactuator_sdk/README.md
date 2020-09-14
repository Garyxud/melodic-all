# seed_smartactuator_sdk

## サンプルコード
ROS上のパッケージから本ライブラリをインクルードし、SEED-Driverへコマンドを送信するための一連サンプルを示す。    
本サンプルの概要は下記の通りである。
1. シリアルポートを開く
2. 原点復帰（キャリブレーション）が記述されたスクリプト１番を実行する
3. スクリプトが終了するまで待機する
4. 現在値取得を送信し続ける

なお、PCからSEED-Driverへ直接指令を行う際は通信モジュール(CM4U/CMSU)が必要となる。  
通信モジュールの詳細は[HP](http://seed-solutions.net/?q=seed_jp/node/7)参照のこと。  

**多軸ロボットへ指令する際は、aero3_command.hを用いる。詳細は
[seed_r7_ros_controller](https://github.com/seed-solutions/seed_r7_ros_pkg/tree/master/seed_r7_ros_controller)を参考にされたい**

### C++
```c++
#include <ros/ros.h>
#include "seed_smartactuator_sdk/seed3_command.h"

int main(int argc, char **argv)
{
  ros::init(argc,argv,"cpp_sample_node");
  ros::NodeHandle nh;

  seed::controller::SeedCommand seed;

  seed.openPort("/dev/ttyACM0",115200);
  seed.openCom();

  int id = 1;
  std::array<int,3> motor_state;

  seed.runScript(id,1);
  seed.waitForScriptEnd(1);

  while (ros::ok())
  {
    motor_state = seed.getPosition(id);
    if(motor_state[0]) std::cout << motor_state[2] << std::endl;
  }
  return 0;
}
```

### Python

```python
#!/usr/bin/env python
import rospy
from seed_smartactuator_sdk.seed3_command import SeedCommand

#----------- main -----------------#
if __name__ == "__main__":
  rospy.init_node('python_sample_node')

  seed=SeedCommand()

  seed.open_port("/dev/ttyACM0",115200)
  seed.open_com()

  id = 1
  motor_state = []

  seed.run_script(id,1)
  seed.wait_for_script_end(1)
  
  while not rospy.is_shutdown():
    motor_state = seed.get_position(id)
    if(motor_state[0]): print motor_state[2]
```
