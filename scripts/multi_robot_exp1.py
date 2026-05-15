#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import threading
import math
import time

class MultiRobotPreciseSequence(Node):
    def __init__(self, executor):
        super().__init__('multi_robot_exp1')
        self.executor = executor
        
        # Ativa sincronização rigorosa com o relógio do Gazebo
        self.set_parameters([rclpy.parameter.Parameter('use_sim_time', value=True)])
        
        self.group = ReentrantCallbackGroup()
        
        # Publicadores
        self.pub1 = self.create_publisher(Twist, '/artbot1/cmd_vel', 10, callback_group=self.group)
        self.pub2 = self.create_publisher(Twist, '/artbot2/cmd_vel', 10, callback_group=self.group)
        
        # Variáveis para armazenar as posições atuais vindas da odometria
        self.pose1 = {'x': 0.0, 'y': 0.0, 'yaw': 0.0}
        self.pose2 = {'x': 0.0, 'y': 0.0, 'yaw': 0.0}
        
        # Assinaturas dos tópicos de Odometria (certifique-se de que correspondem aos seus namespaces)
        self.sub1 = self.create_subscription(Odometry, '/artbot1/odom', self.odom1_callback, 10, callback_group=self.group)
        self.sub2 = self.create_subscription(Odometry, '/artbot2/odom', self.odom2_callback, 10, callback_group=self.group)
        
        self.robot1_done = False
        self.robot2_done = False
        
        # Aguarda preenchimento da primeira leitura odométrica antes de iniciar
        time.sleep(2.0)
        
        self.get_logger().info('Iniciando rotinas do Experimento 1 ...')
        threading.Thread(target=self.routine_robot1).start()
        threading.Thread(target=self.routine_robot2).start()

    def odom1_callback(self, msg):
        self.pose1['x'] = msg.pose.pose.position.x
        self.pose1['y'] = msg.pose.pose.position.y
        self.pose1['yaw'] = self.quaternion_to_yaw(msg.pose.pose.orientation)

    def odom2_callback(self, msg):
        self.pose2['x'] = msg.pose.pose.position.x
        self.pose2['y'] = msg.pose.pose.position.y
        self.pose2['yaw'] = self.quaternion_to_yaw(msg.pose.pose.orientation)

    def quaternion_to_yaw(self, q):
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    def move_distance(self, pub, pose_ref, distance, speed=0.3):
        """Move o robô para frente por uma distância métrica exata."""
        start_x = pose_ref['x']
        start_y = pose_ref['y']
        msg = Twist()
        msg.linear.x = float(speed)
        
        rate = self.create_rate(20) # 20Hz sincronizado via Sim Time
        while rclpy.ok():
            current_dist = math.sqrt((pose_ref['x'] - start_x)**2 + (pose_ref['y'] - start_y)**2)
            if current_dist >= distance:
                break
            pub.publish(msg)
            rate.sleep()
            
        pub.publish(Twist()) # Parada imediata

    def rotate_angle(self, pub, pose_ref, angle_rad, speed=0.4):
        """Gira o robô por um ângulo em radianos exato."""
        start_yaw = pose_ref['yaw']
        target_yaw = start_yaw + angle_rad
        
        # Normaliza o ângulo alvo entre -pi e pi
        target_yaw = math.atan2(math.sin(target_yaw), math.cos(target_yaw))
        
        msg = Twist()
        # Define a direção do giro baseado no sinal do ângulo desejado
        msg.angular.z = float(speed) if angle_rad > 0 else float(-speed)
        
        rate = self.create_rate(20)
        while rclpy.ok():
            error = target_yaw - pose_ref['yaw']
            error = math.atan2(math.sin(error), math.cos(error)) # Normaliza erro
            
            if abs(error) < 0.03: # Tolerância de parada em radianos (~1.7 graus)
                break
            pub.publish(msg)
            rate.sleep()
            
        pub.publish(Twist()) # Parada imediata

    def routine_robot1(self):
        linear_speed_robot_1 = 3.0
        angular_speed_robot_1 = 0.5

        self.move_distance(self.pub1, self.pose1, distance=1.85, speed=linear_speed_robot_1)
        self.rotate_angle(self.pub1, self.pose1, angle_rad=-1.52, speed=angular_speed_robot_1)
        self.move_distance(self.pub1, self.pose1, distance=3.85, speed=linear_speed_robot_1)
        self.rotate_angle(self.pub1, self.pose1, angle_rad=-1.52, speed=angular_speed_robot_1)
        self.move_distance(self.pub1, self.pose1, distance=2.75, speed=linear_speed_robot_1)
        self.rotate_angle(self.pub1, self.pose1, angle_rad=-1.52, speed=angular_speed_robot_1)
        self.rotate_angle(self.pub1, self.pose1, angle_rad=-1.57, speed=angular_speed_robot_1)
        self.move_distance(self.pub1, self.pose1, distance=2.75, speed=linear_speed_robot_1)
        self.rotate_angle(self.pub1, self.pose1, angle_rad=-1.52, speed=angular_speed_robot_1)
        self.move_distance(self.pub1, self.pose1, distance=3.65, speed=linear_speed_robot_1)
        self.rotate_angle(self.pub1, self.pose1, angle_rad=-0.72, speed=angular_speed_robot_1)
        self.move_distance(self.pub1, self.pose1, distance=2.3, speed=linear_speed_robot_1)
        self.rotate_angle(self.pub1, self.pose1, angle_rad=-0.74, speed=angular_speed_robot_1)
        self.move_distance(self.pub1, self.pose1, distance=3.0, speed=linear_speed_robot_1)
        self.rotate_angle(self.pub1, self.pose1, angle_rad=-0.73, speed=angular_speed_robot_1)
        self.move_distance(self.pub1, self.pose1, distance=2.3, speed=linear_speed_robot_1)
        self.rotate_angle(self.pub1, self.pose1, angle_rad=-0.73, speed=angular_speed_robot_1)
        self.move_distance(self.pub1, self.pose1, distance=2.5, speed=linear_speed_robot_1)
        
        self.get_logger().info('Robot 1 finalizou o trajeto.')
        self.robot1_done = True
        self.check_finished()

    def routine_robot2(self):
        linear_speed_robot_2 = 3.0
        angular_speed_robot_2 = 0.5

        self.move_distance(self.pub2, self.pose2, distance=0.25, speed=linear_speed_robot_2)
        self.rotate_angle(self.pub2, self.pose2, angle_rad=1.50, speed=angular_speed_robot_2)
        self.move_distance(self.pub2, self.pose2, distance=3.0, speed=linear_speed_robot_2)
        self.rotate_angle(self.pub2, self.pose2, angle_rad=-1.50, speed=angular_speed_robot_2)
        self.move_distance(self.pub2, self.pose2, distance=5.0, speed=linear_speed_robot_2)
        self.rotate_angle(self.pub2, self.pose2, angle_rad=-1.52, speed=angular_speed_robot_2)
        self.move_distance(self.pub2, self.pose2, distance=2.5, speed=linear_speed_robot_2)
        self.rotate_angle(self.pub2, self.pose2, angle_rad=1.52, speed=angular_speed_robot_2)
        self.move_distance(self.pub2, self.pose2, distance=3.75, speed=linear_speed_robot_2)
        self.rotate_angle(self.pub2, self.pose2, angle_rad=1.52, speed=angular_speed_robot_2)
        self.move_distance(self.pub2, self.pose2, distance=2.75, speed=linear_speed_robot_2)
        self.rotate_angle(self.pub2, self.pose2, angle_rad=-1.52, speed=angular_speed_robot_2)
        self.move_distance(self.pub2, self.pose2, distance=3.5, speed=linear_speed_robot_2)
        self.rotate_angle(self.pub2, self.pose2, angle_rad=-1.52, speed=angular_speed_robot_2)
        self.move_distance(self.pub2, self.pose2, distance=3.5, speed=linear_speed_robot_2)
        
        self.get_logger().info('Robot 2 finalizou o trajeto.')
        self.robot2_done = True
        self.check_finished()

    def check_finished(self):
        if self.robot1_done and self.robot2_done:
            self.get_logger().info('Todos os robôs executaram suas rotinas. Encerrando...')
            self.executor.shutdown()

def main(args=None):
    rclpy.init(args=args)
    executor = MultiThreadedExecutor()
    node = MultiRobotPreciseSequence(executor)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
