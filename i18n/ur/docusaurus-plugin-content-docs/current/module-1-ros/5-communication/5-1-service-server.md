# سبق 5.1: سروس سرور

ایک ROS 2 سروس ایک درخواست/جواب مواصلاتی نمونہ ہے۔ ایک **سرور** نوڈ ایک خدمت کی تشہیر کرتا ہے، اور ایک **کلائنٹ** نوڈ اس خدمت کو کال کر سکتا ہے۔ کلائنٹ پھر سرور کے جواب کا انتظار کرتا ہے۔

اس سبق میں، ہم ایک سرور نوڈ بنائیں گے جو دو عددیوں کو جوڑنے کی ایک سادہ خدمت فراہم کرتا ہے۔

## کوڈ

ہم بلٹ ان سروس کی قسم example_interfaces/srv/AddTwoInts استعمال کریں گے۔ آپ اس کی ساخت کا معائنہ کرکے چلا سکتے ہیں:
`bash
ros2 interface show example_interfaces/srv/AddTwoInts
`
آپ دیکھیں گے کہ درخواست میں دو 64-بٹ عددی، a اور b ہیں، اور جواب میں ایک 64-بٹ عددی، sum ہے۔

اب، آئیے سرور بناتے ہیں۔ اپنے پیکیج میں ایک نئی فائل بنائیں: my_first_package/add_two_ints_server.py۔
```python
# my_first_package/my_first_package/add_two_ints_server.py

import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts

class AddTwoIntsServerNode(Node):
    """
    A ROS 2 node that provides a service to add two integers.
    """
    def __init__(self):
        super().__init__('add_two_ints_server_node')
        self.srv = self.create_service(
            AddTwoInts,
            'add_two_ints',
            self.add_two_ints_callback)
        self.get_logger().info('Add Two Ints server has been started.')

    def add_two_ints_callback(self, request, response):
        response.sum = request.a + request.b
        self.get_logger().info(f'Incoming request: a={request.a}, b={request.b}. Returning sum={response.sum}')
        return response

def main(args=None):
    rclpy.init(args=args)
    add_two_ints_server_node = AddTwoIntsServerNode()
    rclpy.spin(add_two_ints_server_node)
    add_two_ints_server_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### کوڈ کی تفصیل

1.  **امپورٹس:** ہم example_interfaces.srv سے سروس کی قسم AddTwoInts امپورٹ کرتے ہیں۔
2.  **AddTwoIntsServerNode کلاس:**
    *   self.create_service(...): یہ سرور کا مرکز ہے۔
        *   AddTwoInts: خدمت کی قسم۔
        *   'add_two_ints': خدمت کا نام۔
        *   self.add_two_ints_callback: جب کوئی درخواست موصول ہوتی ہے تو کال کرنے کا فنکشن۔
3.  **add_two_ints_callback میتھڈ:**
    *   یہ فنکشن دو دلائل لیتا ہے: request اور response۔ request آنے والے ڈیٹا (a اور b) کو رکھتا ہے، اور response ایک خالی آبجیکٹ ہے جسے ہمیں بھرنا ہے۔
    *   response.sum = request.a + request.b: ہم اضافہ کرتے ہیں اور جواب آبجیکٹ کے sum فیلڈ کو سیٹ کرتے ہیں۔
    *   return response: ہم آباد شدہ جواب آبجیکٹ واپس کرتے ہیں۔ یہ جواب کلائنٹ کو واپس بھیجتا ہے۔

## رجسٹرنگ اور بلڈنگ

1.  **انحصار شامل کریں:** آپ کو اپنے package.xml میں example_interfaces پر ایک انحصار شامل کرنا ہوگا:
    ```xml
    <depend>example_interfaces</depend>
    ```
2.  **نوڈ رجسٹر کریں:** اپنے setup.py میں نیا نوڈ شامل کریں:
    ```python
    'console_scripts': [
        'add_two_ints_server = my_first_package.add_two_ints_server:main',
        ...
    ],
    ```
3.  **بنائیں:** اپنی ورک اسپیس کی جڑ سے colcon build چلائیں۔
4.  **سورس:** ایک نئے ٹرمینل میں source install/setup.bash کے ساتھ اپنی ورک اسپیس کو سورس کریں۔
5.  **چلائیں:** اپنا سرور نوڈ چلائیں:
    `bash
    ros2 run my_first_package add_two_ints_server
    `
    آپ کو "سرور شروع ہو گیا ہے" کا لاگ پیغام نظر آنا چاہیے۔

## CLI سے جانچ

آپ کا سرور اب چل رہا ہے اور درخواستوں کا انتظار کر رہا ہے۔ ہم اسے کمانڈ لائن سے os2 service call کا استعمال کرکے جانچ سکتے ہیں۔

ایک دوسرا ٹرمینل کھولیں اور چلائیں:
`bash
ros2 service call /add_two_ints example_interfaces/srv/AddTwoInts "{a: 5, b: 10}"
`
سرور ٹرمینل میں، آپ کو آنے والی درخواست اور اس کے حساب کردہ مجموعے کو ظاہر کرنے والا لاگ پیغام نظر آئے گا۔ کلائنٹ ٹرمینل میں، آپ کو موصول ہونے والا جواب نظر آئے گا:
`
requester: making request: example_interfaces.srv.AddTwoInts_Request(a=5, b=10)

response:
example_interfaces.srv.AddTwoInts_Response(sum=15)
`
آپ نے کامیابی سے ایک ROS 2 سروس بنائی ہے۔ اگلے سبق میں، آپ اسے کال کرنے کے لیے ایک وقف شدہ کلائنٹ نوڈ بنائیں گے۔