# سبق 13.2: سیمولیشن کی تعمیر

یہ سبق ہماری تفصیلات کی بنیاد پر سیمولیشن کے ٹکڑوں کو جمع کرنے پر مشتمل ہے۔ آپ دنیا بنائیں گے، اپنی URDF کی مکمل تصدیق کریں گے، اور حتمی لانچ فائل لکھیں گے۔

## 1. دنیا بنائیں

ایک نئی ورلڈ فائل بنائیں، `worlds/wall_follower_world.sdf`۔ اس دنیا میں روبوٹ کے لیے ایک دیوار ہونی چاہیے۔ ایک سادہ باکس کافی ہے۔

```xml
<?xml version="1.0" ?>
<sdf version="1.7">
  <world name="wall_follower_world">
    <include>
      <uri>model://sun</uri>
    </include>
    <include>
      <uri>model://ground_plane</uri>
    </include>
    
    <model name="wall">
      <pose>0 5 0.5 0 0 0</pose>
      <link name="link">
        <collision name="collision">
          <geometry>
            <box>
              <size>10 0.2 1</size>
            </box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box>
              <size>10 0.2 1</size>
            </box>
          </geometry>
        </visual>
      </link>
      <static>true</static>
    </model>

  </world>
</sdf>
```
یہ `y=5` پر ایک دیوار کے طور پر کام کرنے کے لیے ایک لمبا، پتلا باکس بناتا ہے۔

## 2. روبوٹ URDF کو اپ ڈیٹ کریں

یقینی بنائیں کہ پچھلے ابواب سے آپ کی `two_wheeled_robot.urdf` مکمل ہے اور تفصیلات سے ملتی ہے۔ اس میں شامل ہونا چاہیے:
*   `chassis`، `left_wheel`، اور `right_wheel` لنکس جن میں `<visual>`، `<collision>`، اور `<inertial>` ٹیگز ہوں۔
*   `DiffDrive` پلگ ان `/cmd_vel` ٹاپک پر سننے کے لیے ترتیب دیا گیا ہو۔
*   ایک `lidar_link` اور `lidar_joint`۔
*   `lidar_link` سے منسلک `gpu_lidar` سینسر پلگ ان اور `/scan` پر شائع کرنے کے لیے ترتیب دیا گیا ہو۔

## 3. وال فالور نوڈ کو نافذ کریں

یہ `wall_follower.py` نوڈ ہے جسے آپ نے پچھلے باب میں لکھا تھا۔ یقینی بنائیں کہ یہ مکمل ہے اور تفصیلات سے منطق کو صحیح طریقے سے نافذ کرتا ہے۔ یقینی بنائیں کہ یہ مطلوبہ پیرامیٹرز کا اعلان اور استعمال کرتا ہے۔

## 4. کیپسٹون لانچ فائل بنائیں

`launch/module2_capstone.launch.py` بنائیں۔ یہ فائل تمام اجزاء کو مربوط کرے گی۔

```python
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node

def generate_launch_description():
    pkg_share = get_package_share_directory('urdf_tutorial')
    world_file = os.path.join(pkg_share, 'worlds', 'wall_follower_world.sdf')
    urdf_file = os.path.join(pkg_share, 'urdf', 'two_wheeled_robot.urdf')

    # 1. Gazebo لانچ کریں
    gz_sim = ExecuteProcess(
        cmd=['gz', 'sim', '-r', world_file],
        output='screen'
    )

    # 2. روبوٹ کو اسپان کریں
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-file', urdf_file,
            '-name', 'my_robot',
            '-x', '0.0',
            '-y', '0.0',
            '-z', '0.1'
        ],
        output='screen',
    )
    
    # 3. وال فالور نوڈ شروع کریں
    wall_follower = Node(
        package='my_first_package', # فرض کرتے ہوئے کہ نوڈ اس پیکیج میں ہے
        executable='wall_follower', # setup.py سے نام
        name='wall_follower_node',
        output='screen',
        parameters=[
            {'desired_distance': 1.5},
            {'forward_velocity': 0.3},
            {'proportional_gain': 0.8}
        ]
    )

    return LaunchDescription([
        gz_sim,
        spawn_robot,
        wall_follower,
    ])
```

## 5. بنائیں اور چلائیں

1.  یقینی بنائیں کہ آپ کے تمام نوڈس (`wall_follower`) ان کی متعلقہ `setup.py` فائلوں میں صحیح طریقے سے رجسٹرڈ ہیں اور تمام انحصار `package.xml` میں ہیں۔
2.  `colcon build`
3.  `source install/setup.bash`
4.  `ros2 launch urdf_tutorial module2_capstone.launch.py`

جب آپ سسٹم لانچ کریں گے، تو آپ کو Gazebo کھلتا ہوا نظر آئے گا، دیوار والی دنیا ظاہر ہوگی، آپ کا روبوٹ اسپان ہوگا، اور روبوٹ کو فوری طور پر حرکت کرنا اور دیوار کے ساتھ چلنے کی کوشش کرنا شروع کر دینا چاہیے۔

اب آپ نے ایک مربوط، کلوزڈ-لوپ روبوٹکس سیمولیشن بنائی ہے۔ آخری سبق میں، آپ اسے باضابطہ طور پر جانچیں گے۔
