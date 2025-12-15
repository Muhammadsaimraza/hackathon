# سبق 6.1: پیرامیٹرز

اپنے کوڈ کے اندر اقدار کو ہارڈ کوڈ کرنا عام طور پر ایک بری مشق ہے۔ کیا ہوگا اگر آپ اپنے ٹاکر نوڈ کی ٹائمر کی شرح کو تبدیل کرنا چاہتے ہیں؟ یا chatter ٹاپک کو whisper میں تبدیل کرنا چاہتے ہیں؟ آپ کوڈ میں ترمیم کرکے دوبارہ تعمیر کرسکتے ہیں، لیکن ایک بہت بہتر طریقہ ہے: **پیرامیٹرز**۔

پیرامیٹرز ایک نوڈ کو شروع ہونے پر کنفیگریشن اقدار فراہم کرنے کا ایک طریقہ ہیں۔ ہر نوڈ میں ایک بلٹ ان پیرامیٹر سرور ہوتا ہے جس تک دوسرے نوڈس رسائی حاصل کرسکتے ہیں۔

## پیرامیٹرز کا اعلان

آئیے اپنے ٹاکر نوڈ میں ترمیم کریں تاکہ پیغام کے مواد کے لیے ایک پیرامیٹر استعمال کریں۔

```python
# my_first_package/talker_with_params.py

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class TalkerWithParamsNode(Node):
    def __init__(self):
        super().__init__('talker_with_params_node')
        
        # پیرامیٹر کا اعلان کریں
        self.declare_parameter('greeting', 'Hello')
        
        self.publisher_ = self.create_publisher(String, 'chatter', 10)
        timer_period = 1.0
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0
        self.get_logger().info('Talker with params node has been started.')

    def timer_callback(self):
        # پیرامیٹر کی قدر حاصل کریں
        greeting = self.get_parameter('greeting').get_parameter_value().string_value
        
        msg = String()
        msg.data = f'{greeting} World: {self.i}'
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: "{msg.data}"')
        self.i += 1

# ... main فنکشن وہی رہتا ہے ...
def main(args=None):
    rclpy.init(args=args)
    node = TalkerWithParamsNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### کوڈ کی تفصیل
1.  **self.declare_parameter('greeting', 'Hello')**:
    *   یہ فنکشن دو کام کرتا ہے: یہ نوڈ کو بتاتا ہے کہ greeting نامی ایک پیرامیٹر موجود ہے، اور یہ ایک **ڈیفالٹ قدر** ('Hello') فراہم کرتا ہے۔
    *   اگر نوڈ شروع ہونے پر اس پیرامیٹر کے لیے کوئی قدر فراہم نہیں کی جاتی ہے، تو یہ ڈیفالٹ استعمال کرے گا۔
2.  **self.get_parameter('greeting').get_parameter_value().string_value**:
    *   اس طرح آپ اپنے نوڈ کے اندر پیرامیٹر کی موجودہ قدر حاصل کرتے ہیں۔
    *   یہ تھوڑا سا لمبا ہے، لیکن یہ واضح ہے: آپ پیرامیٹر آبجیکٹ حاصل کرتے ہیں، پھر اس کا ویلیو آبجیکٹ حاصل کرتے ہیں، پھر اصل قدر کو صحیح قسم میں کاسٹ کرتے ہیں (string_value, integer_value, وغیرہ)۔

## رجسٹر کریں، بنائیں، اور چلائیں

1.  **نوڈ رجسٹر کریں:** اپنے setup.py میں talker_with_params = my_first_package.talker_with_params:main شامل کریں۔
2.  **بنائیں:** colcon build۔
3.  **سورس:** source install/setup.bash۔
4.  **چلائیں:**
    *   **پیرامیٹر سیٹ کیے بغیر:**
        `bash
        ros2 run my_first_package talker_with_params
        `
        نوڈ "Hello World: ..." شائع کرے گا۔

    *   **کمانڈ لائن سے پیرامیٹر سیٹ کرنا:**
        `bash
        ros2 run my_first_package talker_with_params --ros-args -p greeting:="Hola"
        `
        اب نوڈ "Hola World: ..." شائع کرے گا۔

`--ros-args -p <param_name>:=<param_value>` نحو ایک نوڈ کے لیے اسٹارٹ اپ پر پیرامیٹر سیٹ کرنے کا معیاری طریقہ ہے۔

## رن ٹائم پر پیرامیٹرز کو تبدیل کرنا

آپ نوڈ چلتے وقت بھی پیرامیٹر کو تبدیل کرسکتے ہیں، جیسا کہ آپ نے باب 3 میں Turtlesim کے ساتھ کیا تھا۔

1.  پیرامیٹر سیٹ کیے بغیر نوڈ شروع کریں۔
2.  ایک دوسرے ٹرمینل میں، نوڈ کے لیے پیرامیٹرز کی فہرست بنائیں:
    `bash
    ros2 param list
    `
    آپ کو /talker_with_params_node اور اس کا greeting پیرامیٹر نظر آئے گا۔
3.  اب، پیرامیٹر کو تبدیل کریں:
    `bash
    ros2 param set /talker_with_params_node greeting "Bonjour"
    `
آپ کو فوری طور پر اپنے چلنے والے نوڈ کا آؤٹ پٹ "Hello World" سے "Bonjour World" میں تبدیل ہوتا نظر آئے گا۔

پیرامیٹرز دوبارہ قابل استعمال اور لچکدار نوڈس بنانے کی کلید ہیں۔ کنفیگریشن کو بیرونی بنا کر، آپ اپنے نوڈ کو مختلف روبوٹس اور مختلف حالات کے مطابق ڈھال سکتے ہیں بغیر سورس کوڈ کو چھوئے۔ اگلے سبق میں، ہم دیکھیں گے کہ لانچ فائل سے پیرامیٹرز سیٹ کرکے اسے مزید آسان کیسے بنایا جائے۔
