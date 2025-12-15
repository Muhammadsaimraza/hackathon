# سبق 4.3: ایک سبسکرائبر لکھنا

ایک پبلشر اپنے طور پر بہت مفید نہیں ہے۔ ہمیں پیغامات وصول کرنے کے لیے ایک اور نوڈ کی ضرورت ہے۔ اس سبق میں، آپ ایک "لسنر" نوڈ بنائیں گے جو chatter ٹاپک کو سبسکرائب کرتا ہے اور موصول ہونے والے پیغامات کو پرنٹ کرتا ہے۔

## کوڈ

اپنے پیکیج میں ایک نئی فائل بنائیں: my_first_package/listener.py۔
```python
# my_first_package/my_first_package/listener.py

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class ListenerNode(Node):
    """
    A ROS 2 node that subscribes to a string message and prints it.
    """
    def __init__(self):
        super().__init__('listener_node')
        self.subscription = self.create_subscription(
            String,
            'chatter',
            self.listener_callback,
            10)
        self.get_logger().info('Listener node has been started.')

    def listener_callback(self, msg):
        self.get_logger().info(f'I heard: "{msg.data}"')

def main(args=None):
    rclpy.init(args=args)
    
    listener_node = ListenerNode()
    
    try:
        rclpy.spin(listener_node)
    except KeyboardInterrupt:
        pass
        
    listener_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### کوڈ کی تفصیل

ساخت پبلشر سے بہت ملتی جلتی ہے۔

1.  **ListenerNode کلاس:**
    *   super().__init__('listener_node'): ہم نوڈ کو ایک منفرد نام دیتے ہیں۔
    *   self.create_subscription(...): یہ سبسکرائبر کا مرکز ہے۔
        *   String: سننے کے لیے پیغام کی قسم۔
        *   'chatter': سبسکرائب کرنے کے لیے ٹاپک کا نام۔ یہ پبلشر کے استعمال کردہ ٹاپک کے نام سے **ملنا چاہیے**۔
        *   self.listener_callback: جب بھی کوئی نیا پیغام موصول ہوتا ہے تو کال کرنے کے لیے فنکشن کا نام۔
        *   10: قطار کا سائز۔
2.  **listener_callback میتھڈ:**
    *   یہ فنکشن **کال بیک** ہے۔ یہ clpy کے ذریعہ خود بخود عمل میں لایا جاتا ہے جب بھی chatter ٹاپک پر کوئی پیغام آتا ہے۔
    *   پیغام خود پہلے دلیل کے طور پر منتقل کیا جاتا ہے، msg۔
    *   self.get_logger().info(...): ہم صرف موصولہ پیغام کے ڈیٹا فیلڈ کو لاگ کرتے ہیں۔
3.  **main فنکشن:**
    *   main فنکشن تقریباً پبلشر جیسا ہی ہے۔ ہم clpy کو شروع کرتے ہیں، اپنے نوڈ کا ایک انسٹنس بناتے ہیں، اور پھر اسے زندہ رکھنے اور آنے والے پیغامات پر کارروائی کرنے کے لیے clpy.spin() کو کال کرتے ہیں۔

## نوڈ کو رجسٹر کرنا

بالکل پبلشر کی طرح، ہمیں اپنے نئے نوڈ کے لیے setup.py میں ایک انٹری پوائنٹ شامل کرنے کی ضرورت ہے۔
```python
# setup.py

...
setup(
    ...
    entry_points={
        'console_scripts': [
            'talker = my_first_package.talker:main',
            'listener = my_first_package.listener:main', # Add this line
        ],
    },
)
```

## بنائیں اور چلائیں

1.  **بنائیں:** اپنی ورک اسپیس کی جڑ (os2_ws) پر جائیں اور دوبارہ colcon build چلائیں۔ Colcon اتنا ہوشیار ہے کہ صرف وہی تعمیر کرے جو تبدیل ہوا ہے۔
2.  **سورس:** آپ کو کسی بھی نئے ٹرمینل میں اپنی ورک اسپیس کو دوبارہ سورس کرنے کی ضرورت ہے۔ source install/setup.bash۔
3.  **چلائیں:** اب آپ دونوں نوڈز چلا سکتے ہیں۔
    *   ایک ٹرمینل میں، ٹاکر چلائیں:
        `bash
        ros2 run my_first_package talker
        `
    *   ایک **دوسرے** ٹرمینل میں، لسنر چلائیں:
        `bash
        ros2 run my_first_package listener
        `

آپ کو ٹاکر اپنے "ہیلو ورلڈ" پیغامات شائع کرتے ہوئے نظر آئے گا، اور آپ کو لسنر ہر موصول ہونے والے پیغام کے لیے "میں نے سنا: ..." پرنٹ کرتے ہوئے نظر آئے گا۔

اب آپ نے ROS 2 میں ایک مکمل، ملٹی نوڈ مواصلاتی نظام بنایا ہے۔ آپ کے پاس دو آزاد پائتھن پروگرام ہیں جو ROS 2 مڈل ویئر کے ذریعے ترتیب دیے گئے پبلش/سبسکرائب پیٹرن کا استعمال کرتے ہوئے ایک دوسرے سے بات چیت کر رہے ہیں۔

اگلے سبق میں، ہم دریافت کریں گے کہ آپ اس ترقیاتی عمل کو تیز کرنے کے لیے اے آئی کا استعمال کیسے کر سکتے ہیں۔