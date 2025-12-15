# سبق 11.1: کیمرہ سیمولیشن

کیمرہ روبوٹکس میں سب سے امیر سینسر ہے، جو آبجیکٹ ڈیٹیکشن، نیویگیشن، اور تعامل کے لیے درکار ڈیٹا فراہم کرتا ہے۔ آئیے اپنے دو پہیوں والے روبوٹ میں ایک کیمرہ شامل کرتے ہیں۔

Gazebo میں ایک سینسر کو دو چیزوں کی ضرورت ہوتی ہے:
1.  اس سے منسلک کرنے کے لیے ایک `<link>`۔
2.  پلگ ان اور اس کی خصوصیات کی وضاحت کے لیے ایک `<gazebo>` بلاک کے اندر ایک `<sensor>` ٹیگ۔

## 1. ایک کیمرہ لنک شامل کریں

سب سے پہلے، ہمیں کیمرے کے لیے ایک طبعی لنک کی ضرورت ہے۔ آئیے اپنے چیسس کے سامنے ایک چھوٹا باکس شامل کرتے ہیں۔

```xml
<!-- اپنی URDF فائل میں شامل کریں -->

<!-- کیمرہ لنک -->
<link name="camera_link">
  <visual>
    <geometry>
      <box size="0.05 0.05 0.05"/>
    </geometry>
    <material name="red">
      <color rgba="1.0 0.0 0.0 1.0"/>
    </material>
  </visual>
  <collision>
      <geometry>
          <box size="0.05 0.05 0.05"/>
      </geometry>
  </collision>
  <inertial>
      <mass value="0.01"/>
      <inertia ixx="1e-6" ixy="0.0" ixz="0.0" iyy="1e-6" iyyy="0.0" izz="1e-6"/>
  </inertial>
</link>

<!-- کیمرہ جوائنٹ -->
<joint name="camera_joint" type="fixed">
  <parent link="chassis"/>
  <child link="camera_link"/>
  <origin xyz="0.225 0 0.075" rpy="0 0 0"/>
</joint>
```
ہم نے ایک چھوٹا سرخ باکس بنایا ہے اور اسے چیسس کے سامنے منسلک کیا ہے (`x=0.225` چیسس کی لمبائی کا نصف ہے، `z=0.075` چیسس کی اونچائی کا نصف پلس کیمرہ کی اونچائی کا نصف ہے)۔

## 2. Gazebo کیمرہ پلگ ان شامل کریں

اب، ہم کیمرہ سینسر کو `camera_link` سے منسلک کرنے کے لیے `<gazebo>` ٹیگ شامل کرتے ہیں۔

```xml
<!-- اپنی URDF فائل میں شامل کریں -->

<gazebo reference="camera_link">
  <sensor name="camera_sensor" type="camera">
    <update_rate>30.0</update_rate>
    <camera name="head">
      <horizontal_fov>1.3962634</horizontal_fov>
      <image>
        <width>800</width>
        <height>800</height>
        <format>R8G8B8</format>
      </image>
      <clip>
        <near>0.02</near>
        <far>300</far>
      </clip>
    </camera>
    <plugin name="camera_controller" filename="libgz-sim-ros2-camera-system.so">
      <topic_name>image_raw</topic_name>
      <camera_info_topic_name>camera_info</camera_info_topic_name>
      <frame_name>camera_link</frame_name>
    </plugin>
  </sensor>
</gazebo>
```
### وضاحت
*   **`<gazebo reference="camera_link">`**: اس بلاک کو `camera_link` سے جوڑتا ہے۔
*   **`<sensor type="camera">`**: سینسر کی وضاحت کرتا ہے۔
*   **`<update_rate>`**: 30 فریم فی سیکنڈ۔
*   **`<camera>` بلاک**: کیمرہ کی اندرونی خصوصیات کی وضاحت کرتا ہے۔
    *   `<horizontal_fov>`: ریڈین میں افقی فیلڈ آف ویو (1.396 ریڈین = 80 ڈگری)۔
    *   `<image>`: ریزولوشن اور پکسل فارمیٹ۔
    *   `<clip>`: قریب اور دور کلپنگ پلین۔
*   **`<plugin>`**: یہ سب سے اہم حصہ ہے۔
    *   `filename="libgz-sim-ros2-camera-system.so"`: یہ Gazebo پلگ ان کی وضاحت کرتا ہے جو ایک کیمرے کی نقالی کرتا ہے اور اس کی تصاویر کو ROS 2 پر شائع کرتا ہے۔
    *   `<topic_name>`: تصاویر شائع کرنے کے لیے ROS 2 ٹاپک۔ ہم `image_raw` استعمال کریں گے۔
    *   `<frame_name>`: TF فریم جو تصویری ڈیٹا سے منسلک ہوگا، جو اس لنک کا نام ہونا چاہیے جس سے سینسر منسلک ہے۔

## 3. کیمرہ فیڈ کو دیکھنا

اب، اپنی `gazebo.launch.py` کو دوبارہ بنائیں اور لانچ کریں۔ آپ کا روبوٹ اوپر سرخ کیمرہ باکس کے ساتھ اسپان ہوگا۔

کیمرہ ڈیٹا دیکھنے کے لیے، آپ کو ایک اور ROS 2 ٹول استعمال کرنے کی ضرورت ہے: **RViz** یا **`rqt_image_view`**۔

ایک نئے ٹرمینل میں، چلائیں:
```bash
ros2 run rqt_image_view rqt_image_view
```
یہ ایک GUI ونڈو کھولے گا۔ اوپر والے ٹاپک ڈراپ ڈاؤن میں، `/image_raw` کو منتخب کریں۔ اب آپ کو اپنے سمیولیٹڈ روبوٹ کے نقطہ نظر سے ایک ریئل ٹائم ویڈیو فیڈ نظر آنا چاہیے!

`/cmd_vel` ٹاپک پر `ros2 topic pub` کمانڈز کا استعمال کرتے ہوئے روبوٹ کو ادھر ادھر چلائیں، اور آپ کو `rqt_image_view` میں تصویر اپ ڈیٹ ہوتی نظر آئے گی۔

آپ نے اپنے روبوٹ کو کامیابی سے بصارت دی ہے۔ اگلے سبق میں، آپ اسے ایک اور طاقتور احساس دیں گے: LiDAR۔
