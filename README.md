# ros2_mks
## Описание работы и разработки узла ROS2 для управления серводвигателем MKS Servo

- install MKS Servo 57D python lib
```
   git clone https://github.com/SergeyPopovGit/mks-servo-can.git     
   cd mks-servo-can
   pip install .
```

## Загрузка переменных среды ROS2
    включение автозагрузки переменных среды ROS2
    ```
       echo "source /opt/ros/iron/setup.bash" >> ~/.bashrc
       source ~/.bashrc
    ```

## Cоздание  узла
Запрос Gemini
```
Ты инженер и у тебя задача создать на базе Ubuntu робота на базе ROS2.
У тебя есть сервер с Ubuntu Desktop 22.04 на котором установленн ROS2 Humble c GUI инструментами назовем его ROS2GUI .

Вторая часть робота это Raspberry Pi 4 с установленной  Ubuntu server 22.04 и установленным ROS2 Humble, назовем его ROS2PI.

К ROS2PI подключен MKS CANable V2.0 адаптер который по CAN интерфейсу подключен к двигателю MKS SERVO57D.
Для управления MKS SERVO57D используем библиотеку mks-servo-can которая находиться по адресу https://github.com/SergeyPopovGit/mks-servo-can.git

Цель: через GUI управлять двигателем  MKS SERVO57D.

    Создай файлы для ROS2PI для узла ROS2 в рабочей директории ros2_mks с применение mks-servo-can
```
## Установите ROS2

[[ROS2_Install]]
## Инструкция по развертыванию локального узла драйвера двигателя MKS57.

(**После редактирования заменить на  GitHub)**
[Репозиторий с проектом узла ros2_mks c инструкцией на GitHub ](https://github.com/SergeyPopovGit/ros2_mks)

**Шаг 3: Установка библиотеки `mks-servo-can`**

1. **Создание рабочего пространства ROS2 (если его еще нет):**
    
    Bash
    
    ```
    mkdir -p ~/ros2_mks/src
    cd ~/ros2_mks/src
    ```
    
2. **Клонирование репозитория `mks-servo-can`:**
    
    Bash
    
    ```
    git clone https://github.com/SergeyPopovGit/mks-servo-can.git
    ```
    
3. **Установка зависимостей Python:** Перейдите в каталог `mks-servo-can` и установите зависимости.
    
    Bash
    
    ```
    cd mks-servo-can
    sudo pip install -e . # Установка в "редактируемом" режиме
    ```
    
    - **Важно:** Убедитесь, что у вас установлен `pip` для вашей версии Python, используемой ROS2. Обычно это `python3 -m pip`. Если `pip` не найден: `sudo apt install python3-pip`.

**Шаг 4: Создание ROS2-узла для управления MKS SERVO**

Теперь нам нужно создать Python-скрипт, который будет использовать библиотеку `mks-servo-can` и взаимодействовать с ROS2.

1. **Создание нового пакета ROS2:** Перейдите в каталог `~/ros2_mks/src`.
    
    Bash
    
    ```
    cd ~/ros2_mks/src
    ros2 pkg create --build-type ament_python mks_servo_driver --dependencies rclpy
    ```
    
2. **Создание файла узла Python:** Создайте файл `mks_servo_driver_node.py` внутри `~/ros2_mks/src/mks_servo_driver/mks_servo_driver/` (дважды `mks_servo_driver`).
    
    Bash
    
    ```
    nano ~/ros2_mks/src/mks_servo_driver/mks_servo_driver/mks_servo_driver_node.py
    ```
    
    Вставьте следующий код:
    
``` Python
import rclpy
import can
from rclpy.node import Node
from std_msgs.msg import Int32  # Пример: для получения команды угла
    # Или можете создать свое собственное сообщение, если нужно больше параметров
from mks_servo_can import MksServo
    
class MKSServoDriverNode(Node):
        def __init__(self):
            super().__init__('mks_servo_driver_node')
            self.get_logger().info('Initializing MKS Servo Driver Node...')
    
            # Параметры двигателя
            self.servo_id = 1  # ID вашего двигателя MKS SERVO57D (по умолчанию 1)
            self.can_interface = can.Bus(interface='socketcan', channel='mkscan') # Имя CAN-интерфейса
            self.notifier = can.Notifier(self.can_interface, [])

            # Инициализация MKS SERVO CAN
            try:
                self.servo = MksServo(self.can_interface, self.notifier ,self.servo_id)
                self.get_logger().info(f'Connected to MKS SERVO with ID: {self.servo_id} on {self.can_interface}')
                # Пример: Включение двигателя (зависит от вашей логики, может быть вынесено в команду ROS)
                # self.servo.enable_motor() 
            except Exception as e:
                self.get_logger().error(f'Failed to initialize MKS SERVO: {e}')
                # Вы можете решить, что делать в случае ошибки (например, остановить узел)
                exit() # Для простоты, выходим
    
            # Создание подписчика на топик для команд
            # Предположим, мы получаем команду угла в градусах
            self.subscription = self.create_subscription(
                Int32,
                'servo_angle_command', # Топик, на который мы будем публиковать команды из ROS2GUI
                self.angle_command_callback,
                10)
            self.subscription  # Предотвратить неиспользуемую переменную предупреждение
    
            self.get_logger().info('MKS Servo Driver Node ready to receive commands.')
    
        def angle_command_callback(self, msg):
            target_angle = msg.data
            self.get_logger().info(f'Received angle command: {target_angle} degrees')
            try:
                # Пример: Отправка команды установки абсолютного угла
                self.servo.set_absolute_position(target_angle)
                self.get_logger().info(f'Sent command to set angle to {target_angle} degrees.')
            except Exception as e:
                self.get_logger().error(f'Failed to send angle command: {e}')
    
        def destroy_node(self):
            # Пример: Отключение двигателя при завершении работы узла
            try:
                # self.servo.disable_motor()
                self.get_logger().info('MKS SERVO motor disabled.')
            except Exception as e:
                self.get_logger().error(f'Error disabling motor: {e}')
            super().destroy_node()
    
def main(args=None):
        rclpy.init(args=args)
        node = MKSServoDriverNode()
        try:
            rclpy.spin(node)
        except KeyboardInterrupt:
            node.get_logger().info('Node stopped by user.')
        finally:
            node.destroy_node()
            rclpy.shutdown()
    
if __name__ == '__main__':
        main()
    ```
    
3. **Редактирование `setup.py`:** Откройте файл `~/ros2_mks/src/mks_servo_driver/setup.py` и добавьте информацию о вашей точке входа для узла.
    
    Bash
    
    ```
    nano ~/ros2_mks/src/mks_servo_driver/setup.py
    ```
    
    Найдите раздел `entry_points` и добавьте строку для вашего узла:
    
    Python
    
    ```
    entry_points={
        'console_scripts': [
            'mks_servo_driver_node = mks_servo_driver.mks_servo_driver_node:main',
        ],
    },
    ```
    
4. **Сборка рабочего пространства:** Перейдите в корневой каталог вашего рабочего пространства ROS2.
    
    Bash
    
    ```
    cd ~/ros2_mks
    colcon build
    ```
    
    - **Возможные ошибки:** Если `colcon build` выдает ошибки, проверьте правильность написания имени пакета, пути к файлам и зависимости. Убедитесь, что все зависимости Python установлены.
5. **Исходный код рабочего пространства:** После успешной сборки, вам нужно будет _источник_ рабочего пространства, чтобы ROS2 мог найти ваш новый узел.
    
    Bash
    
    ```
    source install/setup.bash
    ```
    
    - **Важно:** Добавьте эту строку в ваш `~/.bashrc` на ROS2PI, чтобы она автоматически выполнялась при каждом входе в систему:
        
        Bash
        
        ```
        echo "source ~/ros2_mks/install/setup.bash" >> ~/.bashrc
        ```
        
        И примените изменения:
        
        Bash
        
        ```
        source ~/.bashrc
        ```
        

**Шаг 5: Запуск узла на ROS2PI**

Теперь вы можете запустить ваш узел.

Bash

```
ros2 run mks_servo_driver mks_servo_driver_node
```

Если все настроено правильно, вы должны увидеть сообщения об инициализации узла и подключении к двигателю.

**Шаг 6: Тестирование связи ROS2 между ROS2GUI и ROS2PI**

Прежде чем переходить к GUI, убедитесь, что ROS2-узлы на разных машинах могут "видеть" друг друга.

1. **Настройка сетевого окружения ROS2:**
    
    - Убедитесь, что ROS2GUI и ROS2PI находятся в одной локальной сети и могут пинговать друг друга.
    - **Важно:** ROS2 использует DDS для обнаружения узлов. Убедитесь, что брандмауэры (если они включены) разрешают трафик по портам DDS (по умолчанию UDP-порты 7400-7500).
    - Также убедитесь, что переменная `ROS_DOMAIN_ID` одинакова на обеих машинах. По умолчанию она 0. Если вы ее меняли, то установите на обеих машинах одно и то же значение:
        
        Bash
        
        ```
        echo "export ROS_DOMAIN_ID=0" >> ~/.bashrc # Или любое другое число
        source ~/.bashrc
        ```
        
2. **Проверка списка узлов:** На ROS2GUI:
    
    Bash
    
    ```
    ros2 node list
    ```
    
    Вы должны увидеть `mks_servo_driver_node` в списке. Если нет, проверьте сетевые настройки и брандмауэры.
    
3. **Публикация команды из ROS2GUI:** На ROS2GUI, в новом терминале, опубликуйте тестовую команду в топик `servo_angle_command`:
    
    Bash
    
    ```
    ros2 topic pub /servo_angle_command std_msgs/msg/Int32 "data: 90" --once
    ```
    
    - Если ваш `mks_servo_driver_node` запущен на ROS2PI, вы должны увидеть в его консоли сообщение о получении команды: `Received angle command: 90 degrees`.
    - И, конечно же, сам двигатель MKS SERVO57D должен переместиться на 90 градусов.

**Дальнейшие шаги (для ROS2GUI):**

После успешного развертывания узла и проверки связи, на ROS2GUI вы сможете:

- **Создать узел GUI:** Используйте `rviz2` или напишите свое собственное приложение на Python с `rclpy` и, например, `PyQt` или `Tkinter`, которое будет публиковать сообщения в топик `/servo_angle_command`.
- **Использовать `rqt_gui`:** Для быстрого создания простого интерфейса можно использовать `rqt_gui`. Например, плагин `rqt_publisher` позволит вам публиковать данные в топик вручную или с определенной частотой.

Это полный набор инструкций для развертывания узла управления двигателем MKS SERVO на вашем ROS2PI. Удачи в вашей робототехнической работе!
