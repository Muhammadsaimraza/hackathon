# سبق 4.2: ایک پبلشر لکھنا

آئیے اپنا پہلا حقیقی ROS 2 نوڈ لکھیں۔ ہم ایک "ٹاکر" نوڈ بنائیں گے جو ایک سادہ سٹرنگ پیغام کو ایک ٹاپک پر باقاعدہ وقفے سے شائع کرتا ہے۔

یہ نوڈ ایک پائتھن کلاس ہوگی جو clpy.node.Node کلاس سے وراثت میں ملتی ہے۔

## کوڈ

اپنی my_first_package ڈائرکٹری کے اندر، ایک نئی فائل بنائیں: my_first_package/talker.py۔
```python
# my_first_package/my_first_package/talker.py

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class TalkerNode(Node):
    """
    A ROS 2 node that publishes a string message every second.
    """
    def __init__(self):
        super().__init__('talker_node')
        self.publisher_ = self.create_publisher(String, 'chatter', 10)
        timer_period = 1.0  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0
        self.get_logger().info('Talker node has been started.')

    def timer_callback(self):
        msg = String()
        msg.data = f'Hello World: {self.i}'
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: "{msg.data}"')
        self.i += 1

def main(args=None):
    rclpy.init(args=args)
    
    talker_node = TalkerNode()
    
    try:
        rclpy.spin(talker_node)
    except KeyboardInterrupt:
        pass
    
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    talker_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### کوڈ کی تفصیل

1.  **امپورٹس:** ہم clpy (پائتھن کے لیے ROS 2 کلائنٹ لائبریری)، Node، اور std_msgs.msg سے String پیغام کی قسم امپورٹ کرتے ہیں۔
2.  **TalkerNode کلاس:**
    *   ہماری کلاس Node سے وراثت میں ملتی ہے۔
    *   super().__init__('talker_node'): ہم پیرنٹ کنسٹرکٹر کو کال کرتے ہیں اور اپنے نوڈ کو ایک نام دیتے ہیں، talker_node۔
    *   self.create_publisher(String, 'chatter', 10): یہ پبلشر کا مرکز ہے۔ ہم ایک پبلشر بناتے ہیں جو chatter نامی ٹاپک پر String پیغامات بھیجتا ہے۔ 10 "قطار کا سائز" ہے، جو سروس کے معیار کی ترتیب ہے جسے ہم بعد میں دریافت کریں گے۔
    *   self.create_timer(1.0, self.timer_callback): ہم ایک ٹائمر بناتے ہیں جو ہمارے timer_callback میتھڈ کو ہر سیکنڈ میں ایک بار کال کرے گا۔
3.  **timer_callback میتھڈ:**
    *   یہ فنکشن ہر بار ٹائمر فائر ہونے پر عمل میں آتا ہے۔
    *   msg = String(): ہم ایک نیا String پیغام آبجیکٹ بناتے ہیں۔
    *   msg.data = ...: ہم پیغام کے data فیلڈ کو سیٹ کرتے ہیں۔
    *   self.publisher_.publish(msg): ہم پیغام کو ٹاپک پر شائع کرتے ہیں۔
    *   self.get_logger().info(...): ہم کنسول پر ایک لاگ پیغام لکھتے ہیں۔
4.  **main فنکشن:**
    *   clpy.init(): ROS 2 کلائنٹ لائبریری کو شروع کرتا ہے۔
    *   talker_node = TalkerNode(): ہم اپنے نوڈ کا ایک انسٹنس بناتے ہیں۔
    *   clpy.spin(talker_node): یہ ایک اہم لائن ہے۔ یہ نوڈ کو "گھماتی" ہے، جس کا مطلب ہے کہ یہ ایک لوپ میں داخل ہوتی ہے جو نوڈ کو زندہ رکھتی ہے اور اس کے کال بیکس (جیسے ہمارا ٹائمر کال بیک) پر کارروائی کرنے کی اجازت دیتی ہے۔ پروگرام اس لوپ میں اس وقت تک رہے گا جب تک کہ آپ ٹرمینل میں Ctrl+C نہ دبائیں۔
    *   destroy_node() اور shutdown(): یہ پروگرام سے باہر نکلتے وقت نوڈ اور ROS 2 کلائنٹ لائبریری کو صاف کرتے ہیں۔

## نوڈ کو رجسٹر کرنا

اب ہمیں ROS 2 کو بتانے کی ضرورت ہے کہ اس فائل میں ایک ایگزیکیوٹیبل نوڈ ہے۔ ہم یہ setup.py میں کرتے ہیں۔ اپنے پیکیج میں setup.py فائل کھولیں اور setup() فنکشن کے اندر entry_points سیکشن شامل کریں:
```python
# setup.py

...
setup(
    ...
    entry_points={
        'console_scripts': [
            'talker = my_first_package.talker:main',
        ],
    },
)
```
یہ colcon کو بتاتا ہے کہ talker نامی ایک ایگزیکیوٹیبل بنائے جو آپ کی my_first_package لائبریری میں talker.py فائل سے main فنکشن چلاتا ہے۔

## بنائیں اور چلائیں

1.  **بنائیں:** اپنی ورک اسپیس کی جڑ (os2_ws) پر جائیں اور colcon build چلائیں۔
2.  **سورس:** ایک نئے ٹرمینل میں، اپنی ورک اسپیس کو سورس کریں: source install/setup.bash۔
3.  **چلائیں:** اب آپ اپنا نیا نوڈ چلا سکتے ہیں!
    `bash
    ros2 run my_first_package talker
    `
آپ کو اپنے لاگ پیغامات ہر سیکنڈ میں ایک بار اسکرین پر پرنٹ ہوتے نظر آئیں گے۔

آپ اپنے نوڈ کا معائنہ کرنے کے لیے پچھلے باب میں سیکھے گئے کمانڈ لائن ٹولز بھی استعمال کر سکتے ہیں۔ دوسرے ٹرمینل میں (سورس کرنا نہ بھولیں!)، کوشش کریں:
`bash
ros2 node list
ros2 topic list
ros2 topic echo /chatter
`
آپ نے کامیابی سے اپنا پہلا ROS 2 پبلشر نوڈ بنایا اور چلایا ہے۔ اگلے سبق میں، آپ اسے سننے کے لیے ایک سبسکرائبر بنائیں گے۔