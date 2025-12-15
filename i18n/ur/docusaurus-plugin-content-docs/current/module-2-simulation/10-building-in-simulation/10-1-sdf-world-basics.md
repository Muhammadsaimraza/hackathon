# سبق 10.1: SDF ورلڈ کی بنیادی باتیں

ایک SDF فائل ایک XML فائل ہے جو ایک سیمولیشن ورلڈ کو بیان کرتی ہے۔ آئیے صرف ایک لائٹ سورس اور ایک گراؤنڈ پلین کے ساتھ ایک سادہ، خالی دنیا بناتے ہیں۔

اپنے `urdf_tutorial` پیکیج میں ایک `worlds` ڈائرکٹری بنائیں، اور اس کے اندر `empty_world.sdf` نامی ایک فائل بنائیں۔

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

  </world>
</sdf>
```

### وضاحت

*   **`<sdf version="1.7">`**: روٹ ٹیگ، جو SDF تفصیلات کا استعمال شدہ ورژن بتاتا ہے۔
*   **`<world name="empty_world">`**: اس ٹیگ میں ہماری دنیا میں سب کچھ شامل ہے۔
*   **`<include>`**: یہ ایک طاقتور ٹیگ ہے جو ہمیں پہلے سے موجود ماڈلز کو شامل کرنے دیتا ہے۔ Gazebo کئی بلٹ ان ماڈلز کے ساتھ آتا ہے، بشمول `sun` اور `ground_plane`۔ `model://` پریفکس Gazebo کو بتاتا ہے کہ ان ماڈلز کو اس کی معیاری لائبریری میں تلاش کرے۔

## Gazebo میں ایک دنیا لانچ کرنا

اس سیمولیشن کو چلانے کے لیے، ہمیں ایک لانچ فائل کی ضرورت ہے جو Gazebo سمیلیٹر کو شروع کرے اور اسے بتائے کہ کون سی ورلڈ فائل لوڈ کرنی ہے۔

ہم `ros_gz_sim` پیکیج استعمال کریں گے، جو ROS 2 اور Gazebo کے درمیان پل فراہم کرتا ہے۔

ایک نئی لانچ فائل بنائیں، `launch/gazebo.launch.py`۔
```python
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess

def generate_launch_description():
    pkg_share = get_package_share_directory('urdf_tutorial')
    world_file_path = os.path.join(pkg_share, 'worlds', 'empty_world.sdf')

    return LaunchDescription([
        ExecuteProcess(
            cmd=['gz', 'sim', '-r', world_file_path],
            output='screen'
        ),
    ])
```

### وضاحت
1.  ہماری `empty_world.sdf` فائل کا مکمل پاتھ تلاش کرتے ہیں۔
2.  ہم ایک کمانڈ لائن پروسیس کو چلانے کے لیے `ExecuteProcess` ایکشن کا استعمال کرتے ہیں۔
3.  کمانڈ `gz sim -r empty_world.sdf` ہے۔
    *   `gz sim`: Gazebo سمیلیٹر کو چلانے کی کمانڈ۔
    *   `-r`: یہ فلیگ Gazebo کو بتاتا ہے کہ سیمولیشن کو روک کر شروع کرے۔ یہ اچھی عادت ہے، کیونکہ یہ آپ کے تمام نوڈس کو فزکس انجن کے چلنے سے پہلے شروع ہونے کا وقت دیتا ہے۔
    *   `world_file_path`: اس دنیا کا پاتھ جسے ہم لوڈ کرنا چاہتے ہیں۔

## بنائیں اور لانچ کریں

1.  **انحصار شامل کریں:** اپنے `urdf_tutorial/package.xml` میں `ros_gz_sim` پر ایک انحصار شامل کریں۔
    ```xml
    <exec_depend>ros_gz_sim</exec_depend>
    ```
2.  **بنائیں:** `colcon build`۔
3.  **سورس:** `source install/setup.bash`۔
4.  **لانچ کریں:**
    ```bash
    ros2 launch urdf_tutorial gazebo.launch.py
    ```
یہ Gazebo GUI کھولے گا۔ آپ کو ایک روشن روشنی کے نیچے ایک چپٹا، سرمئی گراؤنڈ پلین نظر آئے گا۔ نیچے، آپ دیکھیں گے کہ سیمولیشن کا وقت `0.000` ہے اور یہ رکا ہوا ہے۔ آپ سیمولیشن شروع کرنے کے لیے GUI میں "Play" بٹن دبا سکتے ہیں۔

اب آپ نے اپنی پہلی Gazebo دنیا بنائی اور لانچ کی ہے۔ اگلے اسباق میں، ہم اس میں مزید دلچسپ اشیاء شامل کرنا سیکھیں گے۔
