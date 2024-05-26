import math
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import ReliabilityPolicy, QoSProfile
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import cv2
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge
from my_package.odom import Odom
from my_package.laser import Laser
from time import time
from random import randint


class Circuito(Node,Odom,Laser): # Mude o nome da classe

    def __init__(self, cor):
        Node.__init__(self, 'circuito_node')
        Odom.__init__(self)
        Laser.__init__(self)
        self.timer = self.create_timer(0.25, self.control)
        self.cor_desejada = cor
        self.time = None
        self.initialPosition = None
        self.doesAlreadyLeft = False
        self.mask = None
        self.robot_state = 'beyblade'
        self.robo_x = 0.0
        self.robo_y = 0.0
        self.color = None
        self.state_machine = {
            'stop': self.stop,
            'age': self.age,
            'beyblade': self.beyblade,
            'retorna': self.retorna,
        }

        self.cyellow = {
            'lower': (20, 50, 50),
            'upper': (30, 255, 255)
        }

        self.masks = {
            'azul':{
                'lower': (100,150,0),
                'upper': (140,255,255),
            },
            'verde':{
                'lower': (40,150,20),
                'upper': (70,255,255),
            },
            'vermelho':{
                'lower': (0,50,50),
                'upper': (10,255,255),
            },
            'magenta': {
                'lower': (145,50,50),
                'upper': (160,255,255),
            },
            'amarelo': {
                'lower': (20, 50, 50),
                'upper': (30, 255, 255),
            },
            'ciano': {
                'lower': (80, 50, 50),
                'upper': (100, 255, 255),
            },
            'rosa': {
                'lower': (150, 50, 50),
                'upper': (165, 255, 255),
            }
        }
        # Inicialização de variáveis
        self.twist = Twist()
        self.bridge = CvBridge()
        self.kernel = np.ones((5, 5), np.uint8)
        
        # Subscribers
        ## Coloque aqui os subscribers

        self.subcomp = self.create_subscription(
            CompressedImage,
            '/camera/image_raw/compressed',
            self.image_callback,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        )
        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE))

        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        ## Coloque aqui os publishers
        self.cx = np.inf
        self.kp = 0.005

    def stop(self):
        self.twist = Twist()
    

    def calc_erro(self):
        self.erro = self.w - self.cx
        self.rot = self.erro * self.kp
        
    def age(self):
        self.twist = Twist()
        if self.color is not None:
            self.twist.linear.x = 0.2
            if self.color == 'amarelo':
                self.twist.linear.x = -0.2
            self.calc_erro()
            self.twist.angular.z = self.rot
            if self.color == 'amarelo':
                if min(self.back)<0.5:
                    self.robot_state = 'retorna'
            else:
                if min(self.front)<0.5:
                    self.robot_state = 'retorna'
    def get_angular_erro(self , point):
        self.err_x = point[0] - self.robo_x
        self.err_y = point[1] - self.robo_y
        self.dist = math.sqrt(self.err_x ** 2 + self.err_y ** 2)
        self.theta = np.arctan2(point[1]-self.robo_y , point[0]-self.robo_x)
        self.erro_ang = self.theta - self.yaw
        self.erro_ang = np.arctan2(np.sin(self.erro_ang), np.cos(self.erro_ang))
    def retorna(self):
        self.twist = Twist()
        self.get_angular_erro(self.initialPosition)
        self.twist.angular.z = 0.04 * self.erro_ang
        if self.erro_ang<0.2:
            self.twist.angular.z = 0.005 * self.erro_ang
            self.twist.linear.x = 0.4 * self.dist
        print(self.dist)
        if self.dist < 0.005:
            self.robot_state = 'stop'
    def beyblade(self):
        self.twist = Twist()
        if self.initialPosition is None:
            self.initialPosition = (self.robo_x,self.robo_y)
        if self.time is None:
            self.time = time()
            self.duration = randint(5,10)
        self.twist.angular.z = 0.5
        if (time() - self.time)>self.duration:
            self.robot_state = 'age'
    def control(self):
        self.twist = Twist()
        print(f'Estado Atual: {self.robot_state}')
        self.state_machine[self.robot_state]()

        self.cmd_vel_pub.publish(self.twist)
    
    def odom_callback(self, msg):
        self.robo_x = msg.pose.pose.position.x
        self.robo_y = msg.pose.pose.position.y
        quaternion = (msg.pose.pose.orientation.x,
                msg.pose.pose.orientation.y,
                msg.pose.pose.orientation.z,
                msg.pose.pose.orientation.w)
        _, _, self.yaw = self.euler_from_quaternion(quaternion)
    def image_callback(self, msg):
        cv_image = self.bridge.compressed_imgmsg_to_cv2(msg, "bgr8") # if CompressedImage
        h,w,_ = cv_image.shape
        self.w = w/2
        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

        mask_magenta =  cv2.inRange(hsv,self.masks['magenta']['lower'],self.masks['magenta']['upper'])
        mask_magenta = cv2.morphologyEx(mask_magenta, cv2.MORPH_OPEN, self.kernel)
        mask_magenta = cv2.morphologyEx(mask_magenta, cv2.MORPH_CLOSE, self.kernel)

        contours_magenta, _ = cv2.findContours(mask_magenta, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        mask_amarela =  cv2.inRange(hsv,self.masks['amarelo']['lower'],self.masks['amarelo']['upper'])
        mask_amarela = cv2.morphologyEx(mask_amarela, cv2.MORPH_OPEN, self.kernel)
        mask_amarela = cv2.morphologyEx(mask_amarela, cv2.MORPH_CLOSE, self.kernel)

        contours_amarelo, _ = cv2.findContours(mask_amarela, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        if len(contours_amarelo)>0 or len(contours_magenta) > 0:
            if np.sum(mask_amarela)>np.sum(mask_magenta):
                contours = contours_amarelo
                mask = mask_amarela
                self.color = 'amarelo'
            else:
                contours = contours_magenta
                mask = mask_magenta
                self.color = 'magenta'
            contour = max(contours, key=cv2.contourArea)
            cv2.drawContours(cv_image, contour, -1, [255, 0, 0], 3)

            M = cv2.moments(contour)
            self.cx = int(M["m10"] / M["m00"])
            self.cy = int(M["m01"] / M["m00"])

            cv2.circle(cv_image, (self.cx, self.cy), 5, (0, 0, 255), -1)
            cv2.imshow("cv_image", mask)
            cv2.waitKey(1)
        else:
            return -1
            
def main(args=None):
    rclpy.init(args=args)
    ros_node = Circuito(cor="azul") # Mude o nome da classe

    rclpy.spin(ros_node)

    ros_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()