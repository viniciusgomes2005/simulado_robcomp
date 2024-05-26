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


class Circulos(Node,Odom,Laser): # Mude o nome da classe

    def __init__(self):
        Node.__init__(self, 'circulo_node')
        Odom.__init__(self)
        Laser.__init__(self)
        self.timer = self.create_timer(0.25, self.control)
        self.time = None
        self.redTime = None
        self.initialPosition = None
        self.red_point = None
        self.green_point = None
        self.blue_point = None
        self.passedBluePointTwice = False
        self.podeIrProAzul = False
        self.girarNoVerde = False
        self.passedRedPoint = False
        self.passedGreenPointOnce = False
        self.passedGreenPointTwice = False
        self.passedGreenPointThird = False
        self.passedGreenPoint = False
        self.isToTurn = False
        self.reverteu = False
        self.mask = None
        self.goalYaw = None
        self.passedBluePoint = False
        self.tempo = None
        self.yaw = None
        self.rodou1x = False
        self.rodou2x = False
        self.cx_intersection_va = -0.5442430106697671 
        self.cx_intersection_vv = 0.44770658577298716
        self.cy_intersection_vv = 0.596433289466945
        self.cy_intersection_va = -0.8291364352288121
        self.robot_state = 'rotaciona_primeira_vez'
        self.robo_x = 0.0
        self.robo_y = 0.0
        self.color = None
        self.state_machine = {
            'stop': self.stop,
            'segue': self.segue,
            'rotaciona':self.rotaciona,
            'rotaciona_primeira_vez':self.rotaciona_primeira_vez,
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
        self.cx_verde = np.inf
        self.cx_azul = np.inf
        self.cx_vermelho = np.inf
        self.cy_verde = np.inf
        self.cy_azul = np.inf
        self.time = None
        self.cy_vermelho = np.inf
        self.kp = 0.005

    def stop(self):
        self.twist = Twist()
    def rotaciona(self):
        self.twist = Twist()
        if self.goalYaw is None and self.yaw is not None:
            self.goalYaw = self.yaw + math.pi*0.2
        self.twist.angular.z = 0.1
        if abs(self.goalYaw-self.yaw)<0.1:
            self.goalYaw = None
            if self.passedGreenPointTwice:
                self.girarNoVerde = True
            self.robot_state = 'segue'
    
    def rotaciona_primeira_vez(self):
        self.twist = Twist()
        if self.goalYaw is None and self.yaw is not None:
            self.goalYaw = self.yaw + math.pi*0.5
        self.twist.angular.z = 0.1
        if abs(self.goalYaw-self.yaw)<0.1:
            self.goalYaw = None
            self.robot_state = 'segue'
    def seleciona_cx(self):
        dist_verm = abs(((self.cx_vermelho - self.robo_x)**2 + (self.robo_y - self.cy_vermelho)**2)**0.5)
        print(self.passedGreenPoint,self.passedGreenPointOnce,self.passedGreenPointTwice,self.passedBluePoint)
        if self.podeIrProAzul:
            if self.cx_azul == np.inf:
                self.cx = self.cx_verde
            else:
                self.cx = self.cx_azul
        elif self.passedGreenPointTwice:
            print(self.cx_verde)
            if self.cx_verde == np.inf:
                self.cx = self.cx_vermelho
            else:
                self.cx =  self.cx_verde
        else:
            if self.red_point == None and dist_verm<400:
                self.redTime = time()
                self.red_point = (self.robo_x,self.robo_y)
            self.cx =  self.cx_vermelho

    def calc_erro(self):
        self.erro = self.w - self.cx
        self.rot = self.erro * self.kp
    def control(self):
        self.twist = Twist()
        print(f'Estado Atual: {self.robot_state}')
        self.state_machine[self.robot_state]()

        self.cmd_vel_pub.publish(self.twist)
    def segue(self):
        self.total_rotation = 0
        print(self.robo_x,self.robo_y)
        self.seleciona_cx()
        if self.green_point is None:
            if abs(self.robo_x - self.cx_intersection_vv)<0.3 and abs(self.robo_y - self.cy_intersection_vv)<0.3:
                self.green_point = (self.robo_x,self.robo_y)
        if self.blue_point is None:
            print(self.robo_x,self.robo_y)
            print(self.cx_intersection_va,self.cy_intersection_va)
            if abs(self.robo_x - self.cx_intersection_va)<0.3 and abs(self.robo_y - self.cy_intersection_va)<0.3:
                self.blue_point = (self.robo_x,self.robo_y)
        if self.cx == np.inf:
            self.twist.angular.z = 0.8
        else:
            self.calc_erro()
            self.twist.linear.x = 0.3
            self.twist.angular.z = self.rot
        if self.red_point is not None and self.passedGreenPointOnce:
            if abs(self.robo_x - self.red_point[0])<1 and abs(self.robo_y-self.red_point[1])<1 and (time()-self.redTime)>3:
                self.passedRedPoint = True
        if not self.passedGreenPointTwice and self.passedGreenPointOnce:
            if abs(self.robo_x - self.green_point[0])<0.3 and abs(self.robo_y-self.green_point[1])<0.3 and (time()-self.time)>2:
                if not self.rodou1x:
                    self.robot_state = 'rotaciona'
                    self.rodou1x = True
                self.passedGreenPointTwice = True
        #if self.green_point is not None and self.passedGreenPointTwice:
        #    if abs(self.robo_x - self.green_point[0])<0.3 and abs(self.robo_y-self.green_point[1])<0.3 and (time()-self.time)>2:
        #        if not self.rodou2x:
        #            self.robot_state = 'rotaciona'
        #            self.rodou2x = True
        #        self.passedGreenPointThird = True
        #if self.green_point is not None and self.passedGreenPointThird:
        #    if abs(self.robo_x - self.green_point[0])<0.3 and abs(self.robo_y-self.green_point[1])<0.3 and (time()-self.time)>2:
        #        self.passedGreenPoint = True
        if self.girarNoVerde:
            print(self.blue_point)
            if self.blue_point is not None and not self.passedBluePoint:
                if abs(self.robo_x - self.blue_point[0])<0.3 and abs(self.robo_y-self.blue_point[1])<0.3:
                    self.tempo = time()
                    self.passedBluePoint = True
            if self.tempo is None:
                self.tempo = time()
            if time()-self.tempo>3:
                self.passedGreenPoint = True
            if self.passedBluePoint and abs(self.robo_x - self.green_point[0])<0.3 and abs(self.robo_y-self.green_point[1])<0.3:
                self.passedBluePointTwice = True
        if self.passedBluePointTwice and abs(self.robo_x - self.blue_point[0])<0.3 and abs(self.robo_y-self.blue_point[1])<0.3:
            if not self.podeIrProAzul:
                self.podeIrProAzul = True
                self.robot_state = 'rotaciona'
        if self.green_point is not None:
            if not self.reverteu:
                self.reverteu = True
            elif abs(self.robo_x - self.green_point[0])<0.3 and abs(self.robo_y-self.green_point[1])<0.3:
                self.time = time()
                self.passedGreenPointOnce = True
    def odom_callback(self, msg):
        self.robo_x = msg.pose.pose.position.x
        self.robo_y = msg.pose.pose.position.y
        quaternion = (msg.pose.pose.orientation.x,
                msg.pose.pose.orientation.y,
                msg.pose.pose.orientation.z,
                msg.pose.pose.orientation.w)
        _, _, self.yaw = self.euler_from_quaternion(quaternion)
    def image_callback(self, msg):
        cv_image = self.bridge.compressed_imgmsg_to_cv2(msg, "bgr8") 
        h, w, _ = cv_image.shape
        self.w = w / 2
        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

        mask_vermelho = cv2.inRange(hsv, self.masks['vermelho']['lower'], self.masks['vermelho']['upper'])
        mask_vermelho = cv2.morphologyEx(mask_vermelho, cv2.MORPH_OPEN, self.kernel)
        mask_vermelho = cv2.morphologyEx(mask_vermelho, cv2.MORPH_CLOSE, self.kernel)
        contours_vermelho, _ = cv2.findContours(mask_vermelho, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        mask_azul = cv2.inRange(hsv, self.masks['azul']['lower'], self.masks['azul']['upper'])
        mask_azul = cv2.morphologyEx(mask_azul, cv2.MORPH_OPEN, self.kernel)
        mask_azul = cv2.morphologyEx(mask_azul, cv2.MORPH_CLOSE, self.kernel)
        contours_azul, _ = cv2.findContours(mask_azul, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        mask_verde = cv2.inRange(hsv, self.masks['verde']['lower'], self.masks['verde']['upper'])
        mask_verde = cv2.morphologyEx(mask_verde, cv2.MORPH_OPEN, self.kernel)
        mask_verde = cv2.morphologyEx(mask_verde, cv2.MORPH_CLOSE, self.kernel)
        contours_verde, _ = cv2.findContours(mask_verde, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        def get_largest_contour(contours):
            if len(contours) == 0:
                return None
            largest_contour = max(contours, key=cv2.contourArea)
            return largest_contour

        if contours_azul:
            largest_contour_azul = get_largest_contour(contours_azul)
            if largest_contour_azul is not None:
                M = cv2.moments(largest_contour_azul)
                if M["m00"] != 0:
                    self.cx_azul = int(M["m10"] / M["m00"])
                    self.cy_azul = int(M["m01"] / M["m00"])
                    cv_image = cv2.circle(cv_image, (self.cx_azul, self.cy_azul), 5, (0, 0, 255), -1)

        if contours_verde:
            largest_contour_verde = get_largest_contour(contours_verde)
            if largest_contour_verde is not None:
                M = cv2.moments(largest_contour_verde)
                if M["m00"] != 0:
                    self.cx_verde = int(M["m10"] / M["m00"])
                    self.cy_verde = int(M["m01"] / M["m00"])
                    cv_image = cv2.circle(cv_image, (self.cx_verde, self.cy_verde), 5, (0, 0, 255), -1)

        if contours_vermelho:
            largest_contour_vermelho = get_largest_contour(contours_vermelho)
            if largest_contour_vermelho is not None:
                M = cv2.moments(largest_contour_vermelho)
                if M["m00"] != 0:
                    self.cx_vermelho = int(M["m10"] / M["m00"])
                    self.cy_vermelho = int(M["m01"] / M["m00"])
                    cv_image = cv2.circle(cv_image, (self.cx_vermelho, self.cy_vermelho), 5, (0, 0, 255), -1)


        intersection_mask_va = cv2.bitwise_and(mask_vermelho, mask_verde)
        intersection_mask_vv = cv2.bitwise_and(mask_azul, mask_verde)

        contours_intersection, _ = cv2.findContours(intersection_mask_va, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        if contours_intersection:
            largest_contour_intersection = max(contours_intersection, key=cv2.contourArea)
            M = cv2.moments(largest_contour_intersection)
            if M["m00"] != 0:
                self.cx_intersection_va = int(M["m10"] / M["m00"])
                self.cy_intersection_va = int(M["m01"] / M["m00"])
                cv2.circle(cv_image, (self.cx_intersection_va, self.cy_intersection_va), 5, (255, 0, 0), -1)
        contours_intersection, _ = cv2.findContours(intersection_mask_vv, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        if contours_intersection:
            largest_contour_intersection = max(contours_intersection, key=cv2.contourArea)
            M = cv2.moments(largest_contour_intersection)
            if M["m00"] != 0:
                self.cx_intersection_vv = int(M["m10"] / M["m00"])
                self.cy_intersection_vv = int(M["m01"] / M["m00"])
                cv2.circle(cv_image, (self.cx_intersection_vv, self.cy_intersection_vv), 5, (255, 0, 0), -1)

        #cv2.imshow("Mask Azul", mask_azul)
        #cv2.imshow("Mask Verde", mask_verde)
        #cv2.imshow("Mask Vermelho", mask_vermelho)
        #cv2.imshow("Intersection Mask", intersection_mask_va)
        #cv2.imshow("Intersection Mask 2", intersection_mask_vv)
        #cv2.imshow("Image with Intersection", cv_image)
        #cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    ros_node = Circulos() # Mude o nome da classe

    rclpy.spin(ros_node)

    ros_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()