# سبق 11.3: IMU اور کانٹیکٹ سینسرز

آئیے اپنے سینسر سوٹ کو دو مزید اہم اقسام کے ساتھ مکمل کرتے ہیں: توازن اور واقفیت کے لیے ایک IMU، اور تصادم کا پتہ لگانے کے لیے ایک کانٹیکٹ سینسر۔

## 1. IMU سینسر

ایک Inertial Measurement Unit (IMU) واقفیت، کونیی رفتار، اور لکیری سرعت کی پیمائش کرتا ہے۔ یہ روبوٹ کا اندرونی کان ہے، جو اسے توازن کا احساس فراہم کرتا ہے۔

ہم IMU کو `chassis` لنک سے منسلک کریں گے۔
```xml
<!-- اپنی URDF میں <gazebo reference="chassis"> ٹیگ کے اندر یہ شامل کریں -->

<sensor name="imu_sensor" type="imu">
  <update_rate>50</update_rate>
  <plugin name="imu_controller" filename="libgz-sim-ros2-imu-system.so">
    <topic_name>imu</topic_name>
    <frame_id>chassis</frame_id>
  </plugin>
</sensor>
```
### وضاحت
*   **`<sensor type="imu">`**: سینسر کی قسم کی وضاحت کرتا ہے۔
*   **`<plugin>`**:
    *   `filename="libgz-sim-ros2-imu-system.so"`: وہ پلگ ان جو IMU کی نقالی کرتا ہے اور `sensor_msgs/msg/Imu` پیغامات شائع کرتا ہے۔
    *   `<topic_name>`: شائع کرنے کے لیے ROS 2 ٹاپک، `/imu`۔
    *   `<frame_id>`: ڈیٹا کا TF فریم۔

دوبارہ بنانے اور لانچ کرنے کے بعد، آپ خام ڈیٹا دیکھنے کے لیے ٹاپک کو echo کر سکتے ہیں:
```bash
ros2 topic echo /imu
```
آپ کو واقفیت (ایک کوارٹرنیون کے طور پر)، کونیی رفتار، اور لکیری سرعت پر مشتمل پیغامات کا ایک سلسلہ نظر آئے گا۔

## 2. کانٹیکٹ/بمپر سینسر

کبھی کبھی آپ کو یہ جاننے کی ضرورت ہوتی ہے کہ آیا آپ کا روبوٹ کسی چیز سے طبعی طور پر ٹکرا گیا ہے۔ ایک کانٹیکٹ یا بمپر سینسر تصادم کا پتہ لگاتا ہے۔

آئیے اپنے چیسس کے سامنے ایک "بمپر" شامل کرتے ہیں۔ ہم اسے سامنے والے `camera_link` سے منسلک کریں گے۔

```xml
<!-- اپنی URDF میں <gazebo reference="camera_link"> ٹیگ کے اندر یہ شامل کریں -->

<sensor name="bumper_sensor" type="contact">
  <contact>
    <collision>camera_link_collision</collision>
  </contact>
  <update_rate>10</update_rate>
  <plugin name="bumper_controller" filename="libgz-sim-ros2-bumper-system.so">
    <topic_name>bumper_state</topic_name>
    <frame_id>camera_link</frame_id>
  </plugin>
</sensor>
```
### وضاحت
*   **`<sensor type="contact">`**: سینسر کی قسم کی وضاحت کرتا ہے۔
*   **`<contact><collision>`**: یہ اہم ہے۔ یہ بتاتا ہے کہ یہ سینسر کس **تصادم جیومیٹری** سے منسلک ہے۔ `camera_link_collision` نام `camera_link` کے `<collision>` عنصر سے مراد ہے۔ اس کا مطلب ہے کہ سینسر صرف اس وقت ٹرگر ہوگا جب وہ مخصوص تصادم شکل کسی چیز سے ٹکرائے گی۔
*   **`<plugin>`**:
    *   `filename="libgz-sim-ros2-bumper-system.so"`: وہ پلگ ان جو تصادم کا پتہ چلنے پر `gz_ros2_interfaces/msg/Contact` پیغامات شائع کرتا ہے۔
    *   `<topic_name>`: شائع کرنے کے لیے ٹاپک، `/bumper_state`۔

دوبارہ بنانے اور لانچ کرنے کے بعد، آپ ٹاپک کو echo کر سکتے ہیں:
```bash
ros2 topic echo /bumper_state
```
ابتدائی طور پر، یہ خالی ہوگا۔ اب، اپنے روبوٹ کو آگے بڑھائیں جب تک کہ وہ کنسٹرکشن کون یا دیوار سے نہ ٹکرائے۔ جیسے ہی `camera_link` کی تصادم جیومیٹری رابطہ کرتی ہے، آپ کو ٹاپک پر پیغامات نظر آئیں گے، جو تصادم کو بیان کریں گے۔

آپ کا روبوٹ اب سینسرز کے ایک جامع سوٹ سے لیس ہے، جو اسے بصارت، لیزرز، اور ٹچ کے ذریعے دنیا کو سمجھنے، اور خلا میں اپنی واقفیت کو سمجھنے کی اجازت دیتا ہے۔ اس باب کے آخری سبق میں، ہم ان ٹولز کا جائزہ لیں گے جو ہمیں یہ یقینی بنانے کے لیے استعمال کرتے ہیں کہ یہ تمام ڈیٹا درست ہے۔
