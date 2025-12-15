# سبق 5.2: سروس کلائنٹ

کمانڈ لائن سے سروس کو کال کرنا جانچ کے لیے مفید ہے، لیکن ایک حقیقی روبوٹ میں، آپ کے پاس ایک اور نوڈ ہوگا جو **کلائنٹ** کے طور پر کام کرتا ہے۔ کلائنٹ درخواست بھیجنے اور جواب کا انتظار کرنے کا ذمہ دار ہے۔

سبسکرائبرز سے ایک اہم فرق یہ ہے کہ سروس کال عام طور پر **ہم وقت ساز** ہوتی ہے۔ کلائنٹ کا کوڈ رک جائے گا اور اس وقت تک انتظار کرے گا جب تک کہ سرور جواب واپس نہ بھیج دے۔

## کوڈ

آئیے اپنی add_two_ints سروس کو کال کرنے کے لیے ایک کلائنٹ نوڈ بنائیں۔ ایک نئی فائل بنائیں: my_first_package/add_two_ints_client.py۔
```python
# my_first_package/my_first_package/add_two_ints_client.py

import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts

class AddTwoIntsClientNode(Node):
    """
    A ROS 2 node that calls the add_two_ints service.
    """
    def __init__(self):
        super().__init__('add_two_ints_client_node')
        self.client = self.create_client(AddTwoInts, 'add_two_ints')
        
        # Wait for the service to be available
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')
        
        self.req = AddTwoInts.Request()

    def send_request(self, a, b):
        self.req.a = a
        self.req.b = b
        self.future = self.client.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()

def main(args=None):
    rclpy.init(args=args)
    
    client_node = AddTwoIntsClientNode()
    response = client_node.send_request(5, 10)
    
    if response:
        client_node.get_logger().info(
            f'Result of add_two_ints: for {5} + {10} = {response.sum}')
    else:
        client_node.get_logger().error('Service call failed')
        
    client_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### کوڈ کی تفصیل

1.  **AddTwoIntsClientNode کلاس:**
    *   self.create_client(AddTwoInts, 'add_two_ints'): ہم AddTwoInts قسم کی سروس کے لیے 'add_two_ints' ٹاپک پر ایک کلائنٹ بناتے ہیں۔
    *   self.client.wait_for_service(...): یہ ایک اہم قدم ہے۔ کلائنٹ یہاں اس وقت تک انتظار کرے گا جب تک کہ وہ سرور سے منسلک نہ ہو جائے۔ یہ کلائنٹ کو سرور کے تیار ہونے سے پہلے درخواست بھیجنے کی کوشش کرنے سے روکتا ہے۔
2.  **send_request میتھڈ:**
    *   self.req.a = a; self.req.b = b: ہم درخواست آبجیکٹ کو آباد کرتے ہیں۔
    *   self.client.call_async(self.req): ہم درخواست بھیجتے ہیں۔ یہ کال **غیر ہم وقت ساز** ہے — یہ فوری طور پر ایک "فیوچر" آبجیکٹ کے ساتھ واپس آتی ہے جو ایک ایسے کام کی نمائندگی کرتا ہے جو مستقبل میں مکمل ہوگا۔
    *   clpy.spin_until_future_complete(self, self.future): یہ ہم وقت ساز حصہ ہے۔ ہم نوڈ کو گھماتے ہیں، اپنے پروگرام کو روکتے ہیں، جب تک کہ فیوچر آبجیکٹ سرور سے نتیجے کے ساتھ آباد نہ ہو جائے۔
    *   return self.future.result(): ہم فیوچر میں موجود نتیجہ واپس کرتے ہیں۔
3.  **main فنکشن:**
    *   ہم کلائنٹ نوڈ بناتے ہیں۔
    *   ہم کچھ نمبروں کے ساتھ send_request کو کال کرتے ہیں۔
    *   ہم واپس ملنے والے جواب کو لاگ کرتے ہیں۔
    *   **نوٹ:** ہمیں یہاں clpy.spin کے ساتھ try/except بلاک کی ضرورت نہیں ہے کیونکہ ہمارے نوڈ کو ہمیشہ کے لیے چلنے کی ضرورت نہیں ہے۔ یہ ایک درخواست بھیجتا ہے، ایک جواب حاصل کرتا ہے، اور پھر باہر نکل جاتا ہے۔

## رجسٹرنگ اور بلڈنگ

1.  **نوڈ رجسٹر کریں:** اپنے setup.py میں نیا کلائنٹ نوڈ شامل کریں:
    ```python
    'console_scripts': [
        'add_two_ints_server = my_first_package.add_two_ints_server:main',
        'add_two_ints_client = my_first_package.add_two_ints_client:main',
    ],
    ```
2.  **بنائیں:** colcon build چلائیں۔
3.  **سورس:** source install/setup.bash۔
4.  **چلائیں:**
    *   ایک ٹرمینل میں، یقینی بنائیں کہ آپ کا سرور چل رہا ہے:
        `bash
        ros2 run my_first_package add_two_ints_server
        `
    *   ایک دوسرے ٹرمینل میں، اپنا نیا کلائنٹ چلائیں:
        `bash
        ros2 run my_first_package add_two_ints_client
        `

کلائنٹ شروع ہوگا، اپنی درخواست بھیجے گا، جواب حاصل کرے گا، نتیجہ پرنٹ کرے گا، اور پھر باہر نکل جائے گا۔ اب آپ نے ایک مکمل درخواست/جواب کا نظام بنایا ہے۔

اگلے سبق میں، آپ بلٹ ان پیغام کی اقسام سے آزاد ہونا اور اپنی خود کی تخلیق کرنا سیکھیں گے۔