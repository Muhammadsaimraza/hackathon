# سبق 12.3: کلوزڈ-لوپ کنٹرول

اب تک، ہمارا کنٹرول **اوپن-لوپ** رہا ہے۔ ہم ایک رفتار کمانڈ شائع کرتے ہیں اور صرف امید کرتے ہیں کہ روبوٹ وہی کرے گا جو ہم چاہتے ہیں۔ ہم اپنے کمانڈز کو ایڈجسٹ کرنے کے لیے کسی سینسر فیڈ بیک کا استعمال نہیں کر رہے ہیں۔

**کلوزڈ-لوپ کنٹرول** روبوٹ کے رویے کو مسلسل درست کرنے کے لیے سینسر فیڈ بیک استعمال کرنے کا عمل ہے۔ یہ ذہین اور مضبوط روبوٹس بنانے کی کلید ہے۔

آئیے ایک کلاسیکی کلوزڈ-لوپ رویہ بناتے ہیں: ایک دیوار کا فالور۔ مقصد یہ ہے کہ ہمارا روبوٹ آگے بڑھے جبکہ اپنی دائیں جانب کی دیوار سے ایک مستقل فاصلہ برقرار رکھے۔

## منطق

1.  **محسوس کریں:** دائیں جانب کی دیوار سے فاصلہ کی پیمائش کے لیے LiDAR سکینر استعمال کریں۔
2.  **موازنہ کریں:** اس پیمائش شدہ فاصلہ کا ہمارے مطلوبہ فاصلہ (ہمارا "سیٹ پوائنٹ") سے موازنہ کریں۔
3.  **عمل کریں:**
    *   اگر روبوٹ دیوار سے بہت دور ہے، تو تھوڑا سا دائیں طرف مڑیں۔
    *   اگر روبوٹ دیوار سے بہت قریب ہے، تو تھوڑا سا بائیں طرف مڑیں۔
    *   اگر روبوٹ صحیح فاصلے پر ہے، تو سیدھا چلیں۔

یہ **تناسب کنٹرولر** کی ایک سادہ شکل ہے۔ موڑنے کی کمانڈ مطلوبہ فاصلہ اور پیمائش شدہ فاصلہ کے درمیان غلطی کے متناسب ہے۔

## کوڈ

ہم ایک نیا نوڈ بنائیں گے جو `/scan` ٹاپک کو سبسکرائب کرے گا اور `/cmd_vel` ٹاپک پر شائع کرے گا۔

ایک نئی فائل بنائیں، `my_first_package/wall_follower.py`۔

```python
# my_first_package/my_first_package/wall_follower.py
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

class WallFollowerNode(Node):
    def __init__(self):
        super().__init__('wall_follower_node')
        
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10)
        
        self.get_logger().info('Wall Follower Node has been started.')

    def scan_callback(self, msg):
        # ہم اپنی دائیں جانب براہ راست لیزر ریڈنگ کو دیکھنا چاہتے ہیں۔
        # ہمارا سکین 360 پوائنٹس کا ہے، -pi سے +pi تک۔
        # دائیں جانب کا پوائنٹ -pi/2 ریڈینز پر ہے، جو 90 واں پوائنٹ ہے۔
        # (انڈیکس 0 سے 359 تک جاتے ہیں، زاویوں سے نقشہ سازی کرتے ہیں)
        
        distance_to_right = msg.ranges[90]
        
        # سادہ تناسب کنٹرولر
        desired_distance = 1.0
        error = desired_distance - distance_to_right
        
        # 'k' ہمارا تناسب گیین ہے۔ یہ ایک ٹیوننگ پیرامیٹر ہے۔
        k_proportional = 0.5
        
        # ایک Twist پیغام بنائیں
        twist_msg = Twist()
        twist_msg.linear.x = 0.5  # ہمیشہ آگے بڑھیں
        twist_msg.angular.z = k_proportional * error # غلطی کی بنیاد پر مڑیں
        
        self.get_logger().info(f'Dist: {distance_to_right:.2f}, Err: {error:.2f}, Ang. Vel: {twist_msg.angular.z:.2f}')
        self.publisher_.publish(twist_msg)

def main(args=None):
    rclpy.init(args=args)
    wall_follower_node = WallFollowerNode()
    rclpy.spin(wall_follower_node)
    wall_follower_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## سسٹم چلانا

1.  **رجسٹر کریں اور بنائیں:** اپنے `setup.py` میں نیا نوڈ شامل کریں اور `colcon build` چلائیں۔
2.  **ایک لانچ فائل بنائیں:** ایک نئی لانچ فائل بنائیں، `launch/wall_follower.launch.py`۔ اس میں یہ ہونا چاہیے:
    *   Gazebo کو ایک ایسی دنیا کے ساتھ لانچ کریں جس میں کچھ دیواریں ہوں (مثلاً، آپ کا `obstacle_course.sdf`)۔
    *   اپنے دو پہیوں والے روبوٹ URDF کو اسپان کریں۔
    *   اپنا نیا `wall_follower` نوڈ لانچ کریں۔
3.  **لانچ کریں اور مشاہدہ کریں:** اپنی نئی لانچ فائل چلائیں۔ اپنے روبوٹ کو سیمولیشن میں ایک دیوار کے قریب رکھیں۔ اسے آگے بڑھنا شروع کر دینا چاہیے، اپنے زاویہ کو ایڈجسٹ کرتے ہوئے اپنی دائیں جانب کی دیوار سے 1.0 میٹر فاصلہ برقرار رکھنے کی کوشش کریں۔

اب آپ نے ایک حقیقی روبوٹک "مہارت" بنائی ہے۔ آپ کا نظام اب صرف پہلے سے پروگرام شدہ حرکات کو انجام نہیں دے رہا ہے؛ یہ اپنے ماحول کو محسوس کر رہا ہے اور ریئل ٹائم میں اس پر رد عمل ظاہر کر رہا ہے۔ یہ پرسیپشن-ایکشن لوپ ذہین روبوٹکس کا جوہر ہے۔
