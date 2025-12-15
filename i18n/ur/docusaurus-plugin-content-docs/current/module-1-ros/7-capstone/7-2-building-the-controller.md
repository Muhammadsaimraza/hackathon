# سبق 7.2: کنٹرولر کی تعمیر

ایک واضح تفصیلات کے ساتھ، کوڈنگ بہت زیادہ سیدھی ہو جاتی ہے۔ ہم اب پرواز پر ڈیزائن نہیں کر رہے ہیں؛ ہم صرف اس منصوبے کو نافذ کر رہے ہیں جو ہم نے پہلے ہی بنایا ہے۔

یہ سبق آپ کی سیکھی ہوئی ہر چیز کا نتیجہ ہے۔ آپ کو ایک نیا سروس انٹرفیس بنانا ہوگا، ایک پیچیدہ نوڈ لکھنا ہوگا جو سروس سرور اور پبلشر دونوں کے طور پر کام کرتا ہے، اور ان سب کو ایک ساتھ جوڑنے کے لیے ایک لانچ فائل بنانی ہوگی۔

## 1. کسٹم سروس بنائیں

سب سے پہلے، آئیے اپنے `my_custom_interfaces` پیکیج میں `DrawShape.srv` فائل بنائیں۔
1.  **فائل بنائیں:** `my_custom_interfaces/srv/DrawShape.srv`
    ```
    float32 edge_length
    int32 sides
    ---
    bool success
    ```
2.  **`CMakeLists.txt` میں ترمیم کریں:** نئی سروس فائل کو `rosidl_generate_interfaces` کال میں شامل کریں۔
    ```cmake
    rosidl_generate_interfaces(${PROJECT_NAME}
      "msg/Person.msg"
      "srv/SendPerson.srv"
      "srv/DrawShape.srv"  # اسے شامل کریں
    )
    ```

## 2. `shape_drawer_node` کو نافذ کریں

ایک نئی فائل بنائیں، `my_first_package/shape_drawer.py`۔ اس نوڈ کا کوڈ زیادہ پیچیدہ ہے کیونکہ یہ کئی تصورات کو یکجا کرتا ہے۔

```python
# my_first_package/my_first_package/shape_drawer.py

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from my_custom_interfaces.srv import DrawShape
import time
import math

class ShapeDrawerNode(Node):
    def __init__(self):
        super().__init__('shape_drawer_node')
        
        # پیرامیٹرز کا اعلان کریں
        self.declare_parameter('linear_velocity', 1.0)
        self.declare_parameter('angular_velocity', 1.0)
        
        # رفتار کے کمانڈز کے لیے پبلشر بنائیں
        self.publisher_ = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        
        # سروس سرور بنائیں
        self.srv = self.create_service(DrawShape, 'draw_shape', self.draw_shape_callback)
        
        self.get_logger().info('شکل بنانے والا نوڈ شروع ہو گیا ہے۔')

    def draw_shape_callback(self, request, response):
        linear_vel = self.get_parameter('linear_velocity').get_parameter_value().double_value
        angular_vel = self.get_parameter('angular_velocity').get_parameter_value().double_value
        
        edge_length = request.edge_length
        sides = request.sides

        self.get_logger().info(f'{sides} اطراف اور {edge_length} لمبائی کی شکل بنا رہا ہے۔')

        # حرکت اور موڑ کے لیے درکار اوقات کا حساب لگائیں
        move_duration = edge_length / linear_vel
        turn_angle = 2 * math.pi / sides
        turn_duration = turn_angle / angular_vel

        # شکل بنانے کے لیے لوپ
        for i in range(sides):
            # آگے بڑھیں
            self.publish_velocity(linear_vel, 0.0)
            time.sleep(move_duration)
            
            # رکیں
            self.publish_velocity(0.0, 0.0)
            time.sleep(0.1)

            # مڑیں
            self.publish_velocity(0.0, angular_vel)
            time.sleep(turn_duration)

            # رکیں
            self.publish_velocity(0.0, 0.0)
            time.sleep(0.1)

        self.get_logger().info('شکل کی ڈرائنگ مکمل ہوئی۔')
        response.success = True
        return response

    def publish_velocity(self, linear, angular):
        msg = Twist()
        msg.linear.x = linear
        msg.angular.z = angular
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    shape_drawer_node = ShapeDrawerNode()
    rclpy.spin(shape_drawer_node)
    shape_drawer_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```
**نوٹ:** یہاں `time.sleep()` کا استعمال اس مشق کے لیے ایک سادہ طریقہ ہے۔ ایک حقیقی، پروڈکشن-کوالٹی روبوٹ میں، آپ زیادہ نفیس فیڈ بیک پر مبنی کنٹرول استعمال کریں گے (مثلاً، `/turtle1/pose` ٹاپک سے کچھوے کی پوز چیک کرنا) بجائے اس کے کہ مقررہ وقت کی حرکات پر انحصار کریں۔

## 3. لانچ فائل بنائیں

اپنے `my_first_package` میں `launch/capstone.launch.py` بنائیں۔

```python
# my_first_package/launch/capstone.launch.py

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='turtlesim',
            executable='turtlesim_node',
            name='turtlesim'
        ),
        Node(
            package='my_first_package',
            executable='shape_drawer',
            name='shape_drawer_node',
            parameters=[
                {'linear_velocity': 0.5},
                {'angular_velocity': 0.5}
            ]
        )
    ])
```

## 4. رجسٹر، بنائیں، اور چلائیں

1.  **نوڈ رجسٹر کریں:** اپنے `setup.py` میں `shape_drawer = my_first_package.shape_drawer:main` شامل کریں۔
2.  **انحصار شامل کریں:** `my_first_package` کی `package.xml` میں `<depend>my_custom_interfaces</depend>` شامل کریں۔
3.  **بنائیں:** اپنی ورک اسپیس کی جڑ سے `colcon build` چلائیں۔ اسے `my_custom_interfaces` اور `my_first_package` دونوں کو بنانا چاہیے۔
4.  **سورس:** `source install/setup.bash`۔
5.  **سسٹم لانچ کریں:**
    ```bash
    ros2 launch my_first_package capstone.launch.py
    ```
    یہ ٹرٹل سم سمیلیٹر اور آپ کا کنٹرولر نوڈ دونوں شروع کر دے گا۔ آپ کا سسٹم اب چل رہا ہے اور کمانڈ کا انتظار کر رہا ہے۔

آخری سبق میں، ہم سسٹم کو اس کی تفصیلات کے خلاف جانچیں گے۔