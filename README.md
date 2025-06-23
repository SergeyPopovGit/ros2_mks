# ros2_mks
## Описание работы и разработки узла ROS2 для управления серводвигателем MKS Servo

- install MKS Servo 57D python lib
```
   git clone https://github.com/SergeyPopovGit/mks-servo-can.git     
   cd mks-servo-can
   pip install .
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
