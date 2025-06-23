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
## Установка библиотеки mks-servo-can

Создание рабочего пространства ROS2 (если его еще нет):

```Bash

mkdir -p ~/ros2_mks/src
cd ~/ros2_mks/src
```
Клонирование репозитория mks-servo-can:

```Bash 
git clone https://github.com/SergeyPopovGit/mks-servo-can.git
Установка зависимостей Python:
```
Перейдите в каталог mks-servo-can и установите зависимости.

```Bash

cd mks-servo-can
pip install -e . # Установка в "редактируемом" режиме
```
Важно: Убедитесь, что у вас установлен pip для вашей версии Python, используемой ROS2. Обычно это python3 -m pip. Если pip не найден: sudo apt install python3-pip.
Создание ROS2-узла для управления MKS SERVO
## Включение ROS2
The setup script is usually located at /opt/ros/{ROS_DISTRO}/setup.bash

## Создание узла
Теперь нам нужно создать Python-скрипт, который будет использовать библиотеку mks-servo-can и взаимодействовать с ROS2.

- Создание нового пакета ROS2:
Перейдите в каталог ~/ros2_mks/src.

```Bash

cd ~/ros2_mks/src
ros2 pkg create --build-type ament_python mks_servo_driver --dependencies rclpy
```
- Создание файла узла Python:
Создайте файл mks_servo_driver_node.py внутри ~/ros2_mks/src/mks_servo_driver/mks_servo_driver/ (дважды mks_servo_driver).

```Bash

nano ~/ros2_mks/src/mks_servo_driver/mks_servo_driver/mks_servo_driver_node.py
```
Скопировать содержимое файла из данного репозитория

## Редактирование setup.py:
Откройте файл ~/ros2_ws/src/mks_servo_driver/setup.py и добавьте информацию о вашей точке входа для узла.

```Bash
nano ~/ros2_ws/src/mks_servo_driver/setup.py
```
 Найдите раздел entry_points и добавьте строку для вашего узла:

```Python
entry_points={
    'console_scripts': [
        'mks_servo_driver_node = mks_servo_driver.mks_servo_driver_node:main',
    ],
},
```
## Сборка рабочего пространства:

Перейдите в корневой каталог вашего рабочего пространства ROS2.

```Bash
cd ~/ros2_ws
colcon build
```
## Исходный код рабочего пространства:
После успешной сборки, вам нужно будет источник рабочего пространства, чтобы ROS2 мог найти ваш новый узел.

Добавьте эту строку в ваш ~/.bashrc на ROS2PI, чтобы она автоматически выполнялась при каждом входе в систему:

```Bash
echo "source ~/ros2_ws/install/setup.bash" >> ~/.bashrc
source ~/.bashrc
```
## ROS2 инженер
Власний Gem-бот
Прекрасно, коллега! Давайте поэтапно развернем узел управления двигателем MKS SERVO на вашем Raspberry Pi (ROS2PI). Учитывая, что у вас уже установлен ROS2 Humble и Ubuntu Server 22.04, нам потребуется установить драйвер CAN, настроить его, склонировать библиотеку mks-servo-can и создать ROS2-узел.

Шаг 1: Подготовка ROS2PI (Raspberry Pi 4)

Подключение к ROS2PI:
Убедитесь, что вы можете подключиться к вашему Raspberry Pi по SSH.

Bash

ssh user@ip_address_of_ros2pi
(Замените user и ip_address_of_ros2pi на свои данные.)

Обновление системы:
Перед началом любых установок всегда рекомендуется обновить систему:

Bash

sudo apt update && sudo apt upgrade -y
Шаг 2: Настройка CAN-интерфейса на ROS2PI

Для работы с CANable вам нужно активировать CAN-интерфейс.

Установка необходимых утилит:

Bash

sudo apt install -y can-utils
Редактирование /boot/firmware/config.txt (для Raspberry Pi, если CANable подключен напрямую к GPIO):
Если MKS CANable V2.0 подключен к Raspberry Pi через USB, то этот шаг не нужен. Если же вы используете какой-либо CAN HAT, который подключается к GPIO, то вам нужно активировать наложение device tree. Откройте файл:

Bash

sudo nano /boot/firmware/config.txt
Добавьте в конец файла следующие строки:

dtparam=spi=on
dtoverlay=mcp2515-can0,oscillator=16000000,interrupt=25
dtparam=interrupt=25
Важное примечание: Параметры oscillator и interrupt могут отличаться в зависимости от вашего CAN HAT. Уточните их в документации к вашему HAT. Если CANable подключается как USB-устройство, пропустите этот шаг.
Перезагрузка Raspberry Pi (если редактировали config.txt):

Bash

sudo reboot
После перезагрузки снова подключитесь по SSH.

Активация CAN-интерфейса (для USB CANable):
Если MKS CANable V2.0 подключен через USB, он должен появиться как устройство can0 (или can1 и т.д.). Вам нужно его поднять и настроить скорость.
Обычно CANable эмулирует стандартный интерфейс socketcan.

Проверка наличия устройства:

Bash

ip a
Вы должны увидеть интерфейс can0 (или подобный).

Подъем интерфейса и установка скорости:

Bash

sudo ip link set can0 up type can bitrate 1000000 # Стандартная скорость для MKS SERVO57D
Важное примечание: Скорость 1000000 (1 Мбит/с) является стандартной для MKS SERVO57D. Убедитесь, что ваш двигатель настроен на эту скорость. Если нет, измените bitrate.
Проверка работы CAN (опционально):
Вы можете отправить тестовое сообщение, чтобы убедиться, что CAN работает:

Bash

cansend can0 123#1122334455667788
Если нет ошибок, значит CAN-интерфейс настроен.

Шаг 3: Установка библиотеки mks-servo-can

Создание рабочего пространства ROS2 (если его еще нет):

Bash

mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
Клонирование репозитория mks-servo-can:

Bash

git clone https://github.com/SergeyPopovGit/mks-servo-can.git
Установка зависимостей Python:
Перейдите в каталог mks-servo-can и установите зависимости.

Bash

cd mks-servo-can
pip install -e . # Установка в "редактируемом" режиме
Важно: Убедитесь, что у вас установлен pip для вашей версии Python, используемой ROS2. Обычно это python3 -m pip. Если pip не найден: sudo apt install python3-pip.
Шаг 4: Создание ROS2-узла для управления MKS SERVO

Теперь нам нужно создать Python-скрипт, который будет использовать библиотеку mks-servo-can и взаимодействовать с ROS2.

Создание нового пакета ROS2:
Перейдите в каталог ~/ros2_ws/src.

Bash

cd ~/ros2_ws/src
ros2 pkg create --build-type ament_python mks_servo_driver --dependencies rclpy
Создание файла узла Python:
Создайте файл mks_servo_driver_node.py внутри ~/ros2_ws/src/mks_servo_driver/mks_servo_driver/ (дважды mks_servo_driver).

Bash

nano ~/ros2_ws/src/mks_servo_driver/mks_servo_driver/mks_servo_driver_node.py
Вставьте следующий код:

Python

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32  # Пример: для получения команды угла
# Или можете создать свое собственное сообщение, если нужно больше параметров
from mks_servo_can import MKSServoCAN

class MKSServoDriverNode(Node):
    def __init__(self):
        super().__init__('mks_servo_driver_node')
        self.get_logger().info('Initializing MKS Servo Driver Node...')

        # Параметры двигателя
        self.servo_id = 1  # ID вашего двигателя MKS SERVO57D (по умолчанию 1)
        self.can_interface = 'can0' # Имя CAN-интерфейса

        # Инициализация MKS SERVO CAN
        try:
            self.servo = MKSServoCAN(self.can_interface, self.servo_id)
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
Редактирование setup.py:
Откройте файл ~/ros2_ws/src/mks_servo_driver/setup.py и добавьте информацию о вашей точке входа для узла.

```Bash

nano ~/ros2_ws/src/mks_servo_driver/setup.py
```
Найдите раздел entry_points и добавьте строку для вашего узла:

```Python

entry_points={
    'console_scripts': [
        'mks_servo_driver_node = mks_servo_driver.mks_servo_driver_node:main',
    ],
},
```
Сборка рабочего пространства:
Перейдите в корневой каталог вашего рабочего пространства ROS2.

``Bash

cd ~/ros2_ws
colcon build
```
Возможные ошибки: Если colcon build выдает ошибки, проверьте правильность написания имени пакета, пути к файлам и зависимости. Убедитесь, что все зависимости Python установлены.
Исходный код рабочего пространства:
После успешной сборки, вам нужно будет источник рабочего пространства, чтобы ROS2 мог найти ваш новый узел.

``` Bash

source install/setup.bash
```

Важно: Добавьте эту строку в ваш ~/.bashrc на ROS2PI, чтобы она автоматически выполнялась при каждом входе в систему:
``` Bash

echo "source ~/ros2_ws/install/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

Шаг 5: Запуск узла на ROS2PI

Теперь вы можете запустить ваш узел.

``` Bash

ros2 run mks_servo_driver mks_servo_driver_node
```
Если все настроено правильно, вы должны увидеть сообщения об инициализации узла и подключении к двигателю.


