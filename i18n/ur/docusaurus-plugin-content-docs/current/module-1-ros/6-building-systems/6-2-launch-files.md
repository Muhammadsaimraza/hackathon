# سبق 6.2: لانچ فائلیں

اب تک، جب بھی آپ اپنا ٹاکر/لسنر سسٹم چلانا چاہتے تھے، آپ کو دو ٹرمینلز کھولنے، دونوں میں ماحول کو سورس کرنے، اور ہر ایک میں os2 run کمانڈ چلانے کی ضرورت پڑتی تھی۔ یہ تھکا دینے والا ہے اور اسکیل نہیں ہوتا۔

ایک **لانچ فائل** ایک اسکرپٹ ہے جو اس عمل کو خودکار بناتی ہے۔ یہ آپ کو نوڈس کے پورے نظام کی وضاحت کرنے، انہیں ترتیب دینے، اور ان سب کو ایک ہی کمانڈ کے ساتھ شروع کرنے کی اجازت دیتی ہے۔ ROS 2 میں، لانچ فائلیں پائتھن میں لکھی جاتی ہیں۔

## ایک لانچ فائل بنانا

1.  **ایک launch ڈائرکٹری بنائیں:** اپنی my_first_package ڈائرکٹری کے اندر، launch نامی ایک نئی ڈائرکٹری بنائیں۔
2.  **لانچ فائل بنائیں:** launch ڈائرکٹری کے اندر، ایک نئی پائتھن فائل بنائیں، مثال کے طور پر talker_listener.launch.py۔

```python
# my_first_package/launch/talker_listener.launch.py

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    """
    Generates the launch description for the talker and listener system.
    """
    return LaunchDescription([
        Node(
            package='my_first_package',
            executable='talker',
            name='my_talker'
        ),
        Node(
            package='my_first_package',
            executable='listener',
            name='my_listener'
        )
    ])
```

### کوڈ کی تفصیل

1.  **امپورٹس:** ہم LaunchDescription اور ROS-مخصوص Node ایکشن امپورٹ کرتے ہیں۔
2.  **generate_launch_description():** یہ مرکزی فنکشن ہے جسے ROS 2 لانچ فائل چلانے کے لیے تلاش کرے گا۔ اسے ایک LaunchDescription آبجیکٹ واپس کرنا ہوگا۔
3.  **LaunchDescription([...])**: یہ آبجیکٹ ان تمام **اعمال** کی فہرست رکھتا ہے جو آپ انجام دینا چاہتے ہیں۔ سب سے عام عمل Node ہے۔
4.  **Node(...)**: یہ عمل لانچ سسٹم کو ایک نوڈ شروع کرنے کے لیے کہتا ہے۔ آپ کو package اور executable (وہ نام جو آپ نے setup.py میں بیان کیا ہے) کی وضاحت کرنی ہوگی۔ آپ اختیاری طور پر نوڈ کو ایک منفرد name بھی دے سکتے ہیں۔

## لانچ فائل کو انسٹال کرنا

آپ کو colcon کو بتانے کی ضرورت ہے کہ آپ کی launch ڈائرکٹری کو انسٹال کرے تاکہ os2 launch اسے تلاش کر سکے۔ اپنی setup.py کھولیں اور درج ذیل data_files اندراج شامل کریں:

```python
# setup.py
import os
from glob import glob
...
setup(
    ...
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # لانچ ڈائرکٹری سے تمام فائلیں انسٹال کرنے کے لیے یہ لائن شامل کریں
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    ...
)
```

## بنائیں اور لانچ کریں

1.  **بنائیں:** colcon build چلائیں۔
2.  **سورس:** source install/setup.bash۔
3.  **لانچ:** اب، دو os2 run کمانڈز کے بجائے، آپ ایک os2 launch کمانڈ استعمال کرسکتے ہیں:
    `bash
    ros2 launch my_first_package talker_listener.launch.py
    `
آپ کو ایک ہی ٹرمینل میں ٹاکر اور لسنر دونوں سے آؤٹ پٹ نظر آئے گا۔ دونوں نوڈس ایک ہی کمانڈ سے شروع ہوگئے ہیں۔

## لانچ فائل میں پیرامیٹرز سیٹ کرنا

لانچ فائلیں پیرامیٹرز سیٹ کرنے کے لیے بھی بہترین جگہ ہیں۔ آئیے اپنے talker_with_params نوڈ کے لیے ایک نئی لانچ فائل بنائیں۔

```python
# my_first_package/launch/params_talker.launch.py

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='my_first_package',
            executable='talker_with_params',
            name='my_params_talker',
            parameters=[
                {'greeting': 'Bonjour'}
            ]
        )
    ])
```
parameters دلیل لغات کی ایک فہرست لیتی ہے۔

اب، جب آپ اس لانچ فائل کو بناتے اور چلاتے ہیں، تو نوڈ شروع ہوگا اور فوری طور پر "Bonjour World: ..." شائع کرنا شروع کردے گا بغیر کسی کمانڈ لائن دلائل کی ضرورت کے۔
`bash
ros2 launch my_first_package params_talker.launch.py
`

لانچ فائلیں کسی بھی غیر معمولی ROS 2 سسٹم کو چلانے کا معیاری طریقہ ہیں۔ وہ ایک حقیقی روبوٹ کی پیچیدگی کو منظم کرنے کے لیے ضروری ہیں۔ اس باب کے آخری سبق میں، ہم ان ٹولز کو دیکھیں گے جو آپ کو ان پیچیدہ نظاموں کو ڈیبگ کرنے میں مدد کرتے ہیں۔