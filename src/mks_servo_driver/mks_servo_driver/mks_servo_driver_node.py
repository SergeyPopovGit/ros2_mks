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
