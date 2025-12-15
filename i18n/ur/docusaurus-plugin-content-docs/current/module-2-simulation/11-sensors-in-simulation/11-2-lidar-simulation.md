# سبق 11.2: LiDAR سیمولیشن

LiDAR (Light Detection and Ranging) نیویگیشن اور رکاوٹوں سے بچنے کے لیے ایک اہم سینسر ہے۔ یہ لیزر بیم کے ساتھ اشیاء تک فاصلے کی پیمائش کرکے ماحول کا ایک درست 2D یا 3D سکین فراہم کرتا ہے۔

آئیے اپنے روبوٹ میں ایک LiDAR سینسر شامل کرتے ہیں۔ ہم اسے `camera_link` سے منسلک کریں گے جسے ہم نے پچھلے سبق میں بنایا تھا۔

## Gazebo LiDAR پلگ ان شامل کریں

کیمرے کی طرح، LiDAR سینسر ایک پلگ ان ہے جسے ہم ایک لنک سے منسلک کرتے ہیں۔ ہم `<gazebo reference="camera_link">` ٹیگ کے اندر ایک دوسرا `<sensor>` بلاک شامل کریں گے۔ ایک ہی لنک سے متعدد سینسر منسلک کرنا بالکل ٹھیک ہے۔

```xml
<!-- اپنی URDF میں <gazebo reference="camera_link"> ٹیگ کے اندر یہ شامل کریں -->

<sensor name="lidar_sensor" type="gpu_lidar">
  <topic>scan</topic>
  <update_rate>10</update_rate>
  <ray>
    <scan>
      <horizontal>
        <samples>360</samples>
        <resolution>1.0</resolution>
        <min_angle>-3.14159</min_angle>
        <max_angle>3.14159</max_angle>
      </horizontal>
    </scan>
    <range>
      <min>0.1</min>
      <max>20.0</max>
    </range>
  </ray>
  <plugin name="lidar_controller" filename="libgz-sim-ros2-lidar-system.so">
      <topic_name>scan</topic_name>
      <frame_name>camera_link</frame_name>
  </plugin>
</sensor>
```
### وضاحت

*   **`<sensor type="gpu_lidar">`**: سینسر کی قسم کی وضاحت کرتا ہے۔ `gpu_lidar` ایک ہارڈ ویئر-ایکسلریٹڈ ورژن ہے جو معیاری `lidar` قسم سے کہیں زیادہ کارکردگی کا حامل ہے۔
*   **`<topic>` (اختیاری):** یہ ایک اندرونی Gazebo ٹاپک ہے۔ ہم اسے نظر انداز کر سکتے ہیں کیونکہ ROS 2 پلگ ان پبلشنگ کو سنبھال لے گا۔
*   **`<update_rate>`**: 10 سکین فی سیکنڈ۔
*   **`<ray>` بلاک**: لیزر کی خصوصیات کی وضاحت کرتا ہے۔
    *   `<scan><horizontal>`: ہم ایک 2D افقی سکین کی وضاحت کر رہے ہیں۔
        *   `<samples>`: سکین میں 360 پوائنٹس۔
        *   `<min_angle>` اور `<max_angle>`: ایک مکمل 360 ڈگری (-π سے +π) سکین۔
    *   `<range>`: سینسر 0.1m اور 20.0m کے درمیان اشیاء کا پتہ لگا سکتا ہے۔
*   **`<plugin>`**:
    *   `filename="libgz-sim-ros2-lidar-system.so"`: وہ پلگ ان جو LiDAR کی نقالی کرتا ہے اور `sensor_msgs/msg/LaserScan` پیغامات ROS 2 پر شائع کرتا ہے۔
    *   `<topic_name>`: شائع کرنے کے لیے ROS 2 ٹاپک۔ ہم `/scan` استعمال کریں گے۔
    *   `<frame_name>`: سکین کا TF فریم، جو اس لنک کا نام ہے جس سے سینسر منسلک ہے۔

## 2. LiDAR ڈیٹا کو دیکھنا

اب، اپنی `gazebo.launch.py` کو دوبارہ بنائیں اور لانچ کریں۔ آپ کا روبوٹ اب اپنے نئے لیزر سکینر کے ساتھ دنیا کو "دیکھ" رہا ہے۔

`LaserScan` پیغام کو دیکھنے کے لیے بہترین ٹول **RViz** ہے۔

1.  **RViz لانچ کریں:** ایک نئے ٹرمینل میں، `rviz2` چلائیں۔
2.  **فکسڈ فریم سیٹ کریں:** "Displays" پینل میں، "Fixed Frame" کو `chassis` پر سیٹ کریں۔
3.  **TF ڈسپلے شامل کریں:** "Add" پر کلک کریں اور ایک "TF" ڈسپلے شامل کریں۔ یہ آپ کے روبوٹ ماڈل کے کوآرڈینیٹ فریم دکھائے گا۔ آپ کو `chassis`، `left_wheel`، `right_wheel`، اور `camera_link` نظر آنا چاہیے۔
4.  **LaserScan ڈسپلے شامل کریں:** "Add" پر کلک کریں اور ایک "LaserScan" ڈسپلے شامل کریں۔
5.  **LaserScan کو ترتیب دیں:** LaserScan ڈسپلے کی سیٹنگز میں، "Topic" کو `/scan` پر تبدیل کریں۔

اب آپ کو RViz میں سرخ نقطے نظر آنے چاہئیں، جو ان پوائنٹس کی نمائندگی کرتے ہیں جہاں لیزر بیم Gazebo دنیا میں اشیاء (جیسے کنسٹرکشن کون یا دیواریں) سے ٹکرا رہے ہیں۔ جب آپ `/cmd_vel` کا استعمال کرتے ہوئے روبوٹ کو ادھر ادھر چلاتے ہیں، تو آپ کو سکین ریئل ٹائم میں اپ ڈیٹ ہوتا نظر آئے گا۔

یہ وہ ڈیٹا ہے جو SLAM (Simultaneous Localization and Mapping) جیسا نیویگیشن الگورتھم ماحول کا نقشہ بنانے کے لیے استعمال کرے گا۔ اب آپ نے اپنے روبوٹ کو خود مختار نیویگیشن کے لیے دو سب سے اہم احساسات سے لیس کر دیا ہے۔
