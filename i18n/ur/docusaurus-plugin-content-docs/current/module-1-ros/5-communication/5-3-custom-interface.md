# سبق 5.3: کسٹم انٹرفیسز

اب تک، ہم نے بلٹ ان پیغام اور سروس کی اقسام جیسے std_msgs/msg/String اور example_interfaces/srv/AddTwoInts استعمال کی ہیں۔ لیکن کسی بھی حقیقی روبوٹ کے لیے، آپ کو اپنی ڈیٹا کی ساخت کی وضاحت کرنے کی ضرورت ہوگی۔ ROS 2 میں، انہیں **انٹرفیسز** کہا جاتا ہے۔

انٹرفیسز کی تین اقسام ہیں:
*   .msg: پیغام فائلیں، ٹاپکس کے لیے۔
*   .srv: سروس فائلیں، خدمات کے لیے۔
*   .action: ایکشن فائلیں، طویل عرصے تک چلنے والے کاموں کے لیے (جن کا ہم بعد میں احاطہ کریں گے)۔

آئیے اپنے کسٹم انٹرفیسز کو رکھنے کے لیے ایک پیکیج بنائیں اور پھر انہیں اپنے پائتھن نوڈس میں استعمال کریں۔

## ایک انٹرفیس پیکیج بنانا

یہ ایک بہترین عمل ہے کہ اپنے انٹرفیس کی تعریفوں کو ان نوڈس سے الگ پیکیج میں رکھیں جو انہیں استعمال کرتے ہیں۔

1.  **پیکیج بنائیں:** اپنی ورک اسپیس کی src ڈائرکٹری میں، ایک نیا پیکیج بنائیں۔
    `ash
    ros2 pkg create --build-type ament_cmake my_custom_interfaces
    `
    **اہم:** انٹرفیس پیکیجز لازمی طور پر ment_cmake پیکیجز ہونے چاہئیں، نہ کہ ment_python۔

2.  **ڈائرکٹریاں بنائیں:** نئے پیکیج کے اندر، msg اور srv نامی ڈائرکٹریاں بنائیں۔
    `ash
    cd my_custom_interfaces
    mkdir msg
    mkdir srv
    `

3.  **ایک .msg فائل کی وضاحت کریں:** ایک نئی فائل msg/Person.msg بنائیں۔
    `
    # msg/Person.msg
    string first_name
    string last_name
    uint8 age
    `
    فارمیٹ سادہ ہے: قسم نام۔

4.  **ایک .srv فائل کی وضاحت کریں:** srv/SendPerson.srv بنائیں۔
    `
    # srv/SendPerson.srv
    my_custom_interfaces/msg/Person person_to_send
    ---
    bool success
    `
    --- درخواست کو جواب سے الگ کرتا ہے۔ یہاں، درخواست ہمارا کسٹم Person پیغام ہے، اور جواب ایک سادہ بولین ہے۔

## بلڈ کو ترتیب دینا

چونکہ یہ ایک ment_cmake پیکیج ہے، بلڈ کی ترتیب ہمارے پائتھن پیکیج سے مختلف ہے۔

1.  **CMakeLists.txt میں ترمیم کریں:** آپ کو CMake کو بتانا ہوگا کہ ROS 2 انٹرفیس جنریشن ٹولز کو تلاش کرے اور پھر بتائیں کہ کون سی فائلیں بنانی ہیں۔ یہ لائنیں شامل کریں:
    `cmake
    find_package(rosidl_default_generators REQUIRED)

    rosidl_generate_interfaces(
      "msg/Person.msg"
      "srv/SendPerson.srv"
    )
    `

2.  **package.xml میں ترمیم کریں:** آپ کو بلڈ انحصار شامل کرنے کی ضرورت ہے:
    `xml
    <build_depend>rosidl_default_generators</build_depend>
    <exec_depend>rosidl_default_runtime</exec_depend>
    <member_of_group>rosidl_interface_packages</member_of_group>
    `

## کسٹم انٹرفیسز کا استعمال

اب آپ ان انٹرفیسز کو اپنے پائتھن پیکیج میں استعمال کر سکتے ہیں۔

1.  **انحصار شامل کریں:** اپنے my_first_package/package.xml میں، اپنے نئے انٹرفیس پیکیج پر ایک انحصار شامل کریں۔
    ```xml
    <depend>my_custom_interfaces</depend>
    ```

2.  **امپورٹ اور استعمال:** اب آپ Person اور SendPerson کو کسی بھی دوسرے پیغام یا سروس کی قسم کی طرح امپورٹ کر سکتے ہیں۔
    ```python
    # اپنے پائتھن نوڈ میں
    from my_custom_interfaces.msg import Person
    from my_custom_interfaces.srv import SendPerson
    
    # ...
    
    # ایک کسٹم پیغام شائع کرنا
    person_msg = Person()
    person_msg.first_name = "John"
    person_msg.last_name = "Doe"
    person_msg.age = 30
    self.publisher_.publish(person_msg)
    
    # ...
    
    # ایک سروس کال بیک کے اندر
    # request.person_to_send ایک Person آبجیکٹ ہوگا
    self.get_logger().info(f'Received person: {request.person_to_send.first_name}')
    response.success = True
    return response
    ```

## بنائیں اور چلائیں

1.  **بنائیں:** اپنی ورک اسپیس کی جڑ (os2_ws) پر جائیں اور colcon build چلائیں۔ Colcon اتنا ہوشیار ہوگا کہ my_first_package سے *پہلے* my_custom_interfaces بنائے کیونکہ آپ نے انحصار شامل کیا تھا۔
2.  **سورس:** source install/setup.bash۔
3.  **توثیق کریں:** اب آپ اپنی نئی اقسام کو os2 interface کمانڈ کے ساتھ دیکھ سکتے ہیں:
    `ash
    ros2 interface show my_custom_interfaces/msg/Person
    ros2 interface show my_custom_interfaces/srv/SendPerson
    `

کسٹم انٹرفیسز بنانا ایک بنیادی مہارت ہے۔ یہ آپ کو اپنے سسٹم میں تمام نوڈس کے لیے صاف، خود دستاویزی APIs بنانے کی اجازت دیتا ہے۔ اس باب کے آخری سبق میں، ہم بحث کریں گے کہ آپ کے API کے لیے ٹاپک بمقابلہ سروس کا انتخاب کب کرنا ہے۔
