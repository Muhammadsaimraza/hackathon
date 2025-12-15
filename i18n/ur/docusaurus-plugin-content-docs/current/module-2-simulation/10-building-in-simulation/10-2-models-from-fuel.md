# سبق 10.2: فیول سے ماڈل

اپنی دنیا میں ہر چیز کو شروع سے بنانا وقت طلب ہوگا۔ خوش قسمتی سے، Gazebo کو پہلے سے بنے ہوئے ماڈلز کا ایک بڑا آن لائن ڈیٹا بیس دستیاب ہے جسے **Ignition Fuel** مارکیٹ پلیس کہا جاتا ہے۔ آپ اسے [app.ignitionrobotics.org](https://app.ignitionrobotics.org/fuel/models) پر براؤز کر سکتے ہیں۔

آپ اپنے SDF ورلڈ فائل میں اس مارکیٹ پلیس سے کسی بھی ماڈل کو `<include>` ٹیگ کا استعمال کرتے ہوئے شامل کر سکتے ہیں، جیسا کہ ہم نے سورج اور گراؤنڈ پلین کے لیے کیا تھا۔

## فیول ماڈل کو شامل کرنا

آئیے اپنی دنیا میں ایک کنسٹرکشن کون شامل کرتے ہیں۔

پہلے، فیول ویب سائٹ پر ماڈل تلاش کریں۔ اگر آپ "construction cone" تلاش کرتے ہیں، تو آپ کو یہ مل جائے گا۔ ماڈل کا "مالک" `OpenRobotics` ہے۔

اب، ہم اسے اپنی `empty_world.sdf` فائل میں شامل کر سکتے ہیں۔
```xml
<?xml version="1.0" ?>
<sdf version="1.7">
  <world name="empty_world">

    <!-- ایک عالمی روشنی کا ذریعہ -->
    <include>
      <uri>model://sun</uri>
    </include>

    <!-- ایک گراؤنڈ پلین -->
    <include>
      <uri>model://ground_plane</uri>
    </include>
    
    <!-- فیول سے ایک کنسٹرکشن کون -->
    <include>
      <uri>https://fuel.ignitionrobotics.org/1.0/OpenRobotics/models/Construction Cone</uri>
      <name>cone_1</name>
      <pose>2.0 1.0 0.0 0 0 0</pose>
    </include>

  </world>
</sdf>
```

### وضاحت
*   **`<uri>`:** `model://` کے بجائے، ہم فیول سرور پر ماڈل کا مکمل URL فراہم کرتے ہیں۔ فارمیٹ `https://fuel.ignitionrobotics.org/1.0/<OWNER>/models/<MODEL_NAME>` ہے۔
*   **`<name>`:** ہمیں اپنی دنیا میں ماڈل کے اس انسٹنس کو ایک منفرد نام دینا ہوگا، جیسے `cone_1`۔
*   **`<pose>`:** یہ ٹیگ ہمیں دنیا میں ماڈل کی ابتدائی پوزیشن اور اوریئنٹیشن سیٹ کرنے دیتا ہے۔ فارمیٹ `x y z roll pitch yaw` ہے۔ یہاں، ہم کون کو `(x=2.0, y=1.0, z=0.0)` پر رکھ رہے ہیں۔

## کیشنگ

پہلی بار جب آپ ایک ایسی دنیا لانچ کرتے ہیں جس میں ایک فیول ماڈل شامل ہوتا ہے، تو Gazebo اسے انٹرنیٹ سے ڈاؤن لوڈ کرے گا اور اسے آپ کی مقامی مشین پر کیش کرے گا (عام طور پر `~/.ignition/fuel` میں)۔ اگلی بار جب آپ دنیا لانچ کریں گے، تو یہ ماڈل کو مقامی کیش سے لوڈ کرے گا، جو بہت تیز ہے۔

## اپنا روبوٹ شامل کرنا

ہم اپنا روبوٹ کیسے شامل کریں، جسے ہم نے ایک URDF فائل میں بیان کیا تھا؟ ہم ایک URDF فائل کو براہ راست ورلڈ SDF فائل میں شامل نہیں کر سکتے۔

اس کے بجائے، ہم `ros_gz_sim` پیکیج سے ایک خاص "اسپانر" نوڈ استعمال کریں گے تاکہ روبوٹ کو شروع ہونے کے بعد سیمولیشن میں شامل کیا جا سکے۔

ہم اس کے لیے اپنی `gazebo.launch.py` میں ترمیم کریں گے۔
```python
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node

def generate_launch_description():
    pkg_share = get_package_share_directory('urdf_tutorial')
    world_file_path = os.path.join(pkg_share, 'worlds', 'empty_world.sdf')
    urdf_file_path = os.path.join(pkg_share, 'urdf', 'two_wheeled_robot.urdf')

    return LaunchDescription([
        # Gazebo لانچ کریں
        ExecuteProcess(
            cmd=['gz', 'sim', '-r', world_file_path],
            output='screen'
        ),
        
        # روبوٹ کو اسپان کریں
        Node(
            package='ros_gz_sim',
            executable='create',
            arguments=[
                '-file', urdf_file_path,
                '-name', 'my_robot',
                '-x', '0.0',
                '-y', '0.0',
                '-z', '0.1'
            ],
            output='screen',
        ),
    ])
```
### وضاحت
1.  ہم ایک نیا `Node` ایکشن شامل کرتے ہیں۔
2.  **`package='ros_gz_sim'`, `executable='create'`**: یہ اسپانر نوڈ کو چلاتا ہے۔
3.  **`arguments=[...]`**: ہم اسپانر کو کمانڈ لائن دلائل پاس کرتے ہیں۔
    *   `-file`: ہماری URDF فائل کا پاتھ۔
    *   `-name`: سیمولیشن میں ہمارے روبوٹ کے لیے ایک منفرد نام۔
    *   `-x`, `-y`, `-z`: روبوٹ کو اسپان کرنے کے لیے ابتدائی کوآرڈینیٹس۔

اب، بنائیں اور لانچ کریں۔ آپ کو کنسٹرکشن کون کے ساتھ اپنی خالی دنیا نظر آئے گی، اور ایک لمحے کے بعد، آپ کا دو پہیوں والا روبوٹ اصل میں ظاہر ہو جائے گا۔

یہ شاید گر جائے گا اور عجیب لگے گا۔ کیوں؟ کیونکہ جب کہ ہماری URDF میں `<inertial>` ٹیگز ہیں، اس میں مکمل طبعی سیمولیشن کے لیے درکار Gazebo کے مخصوص ٹیگز نہیں ہیں۔ ہم اسے اگلے سبق میں ٹھیک کریں گے۔
