import rclpy
from rclpy.node import Node
from rclpy.logging import get_logger
from rclpy.parameter import Parameter
from time import sleep
import time
import copy
import math
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool, Float64MultiArray, Int64MultiArray, Int16MultiArray
from .doro_packet_handler import PacketHandler

class DOROMobileNode(Node):
  def __init__(self):
    super().__init__('doro_mobile_robot_setting')
    # self.declare_parameter('port_name', '/dev/ttyUSB0')
    # self.declare_parameter('port_baudrate', 115200)
    # self.declare_parameter('wheel_seperation', 0.185)
    # self.declare_parameter('wheel_radius', 0.082)
    # _port_name = self.get_parameter('port_name').value
    # _port_baudrate = self.get_parameter('port_baudrate').value
    # self.wheel_separation = self.get_parameter('wheel_seperation').value
    # self.wheel_radius = self.get_parameter('wheel_radius').value
    _port_name = '/dev/ttyUSB0'
    _port_baudrate = 115200
    self.wheel_separation = 0.185
    self.wheel_radius = 0.082
    print('WHEEL SEPARATION:\t%s'%(self.wheel_separation))
    print('WHEEL RADIUS:\t\t%s'%(self.wheel_radius))

    self.ph = PacketHandler(_port_name, _port_baudrate)

    self.subCmdVelMsg = self.create_subscription(Twist, 'cmd_vel', self.cbCmdVelMsg, 10)
    self.subLiftMsg = self.create_subscription(Bool, 'lift', self.cbLiftMsg, 10)

    self.pub_WheelPos = self.create_publisher(Float64MultiArray, 'WheelPos', 10)
    self.pub_RPM = self.create_publisher(Int64MultiArray, 'rpm', 10)
    self.pub_LiftState = self.create_publisher(Int16MultiArray, 'lift_state', 10)
    self.pub_errorState = self.create_publisher(Int16MultiArray, 'error_state', 10)

    self.sendState = self.create_timer(0.01, self.update_robot)

    self.wheel_pos = [0.0, 0.0]
    self.wheel_rpm = [0,0]
    self.lift_wheel_err = [False, False]
    self.wheel_err = [False, False]
    self.is_lift_wheel = [False, False] # lift1, lift0

    self.max_linear_x = 10.0 # test
    self.max_angular_z = 10.0 # test
    self.is_lift = False

  def cbLiftMsg(self, msg):
    if msg.data: # up
      self.is_lift = msg.data
    else:
      self.is_lift = msg.data

  def cbCmdVelMsg(self, msg):
    linear_x = msg.linear.x
    angular_z = msg.angular.z

    linear_x = max(-self.max_linear_x, min(self.max_linear_x, linear_x))
    angular_z = max(-self.max_angular_z, min(self.max_angular_z, angular_z))
    rmp_r, rmp_l = self.cal_RPM(linear_x, angular_z)
    self.ph.write_motor(rmp_r, rmp_l, self.is_lift)
    
  def cal_RPM(self, linear_x, angular_z):
    rmp_r = (60.)/(math.pi*2*self.wheel_radius)*(linear_x + (angular_z*self.wheel_separation)/2.)
    rmp_l = (60.)/(math.pi*2*self.wheel_radius)*(linear_x - (angular_z*self.wheel_separation)/2.)
    # print((angular_z*self.wheel_separation)/2.)
    return int(rmp_r),int(rmp_l)

  def updateWheelPos(self, r_pos, l_pos):
    wheelPos = Float64MultiArray()
    wheelPos.data.append(r_pos)
    wheelPos.data.append(l_pos)
    self.pub_WheelPos.publish(wheelPos)

  def updateRPM(self, r_rpm, l_rpm):
    wheelRPM = Int64MultiArray()
    wheelRPM.data.append(r_rpm)
    wheelRPM.data.append(l_rpm)
    self.pub_RPM.publish(wheelRPM)

  def updateLiftState(self, lift_0, lift_1):
    liftState = Int16MultiArray()
    if lift_0: liftState.data.append(1)
    else: liftState.data.append(0)
    if lift_1: liftState.data.append(1)
    else: liftState.data.append(0)
    self.pub_LiftState.publish(liftState)

  def updateErrorState(self, lift_1_err, lift_0_err, wheel_r_err, wheel_l_err):
    error_state = Int16MultiArray()
    if lift_0_err: error_state.data.append(1)
    else: error_state.data.append(0)
    if lift_1_err: error_state.data.append(1)
    else: error_state.data.append(0)

    if wheel_r_err: error_state.data.append(1)
    else: error_state.data.append(0)
    if wheel_l_err: error_state.data.append(1)
    else: error_state.data.append(0)

    self.pub_errorState.publish(error_state)

  def update_robot(self):
    self.ph.read_packet()
    self.wheel_pos = self.ph.get_Wheel_pos()
    self.wheel_rpm = self.ph.get_Wheel_RPM()
    self.is_lift_wheel = self.ph.get_is_Lift()
    self.wheel_err = self.ph.get_Wheel_err()
    self.lift_wheel_err = self.ph.get_Lift_err() # lift1 err, lift0 err

    self.updateWheelPos(self.wheel_pos[0], self.wheel_pos[1])
    self.updateRPM(self.wheel_rpm[0], self.wheel_rpm[1])
    self.updateLiftState(self.is_lift_wheel[1], self.is_lift_wheel[0])
    self.updateErrorState(self.lift_wheel_err[0], self.lift_wheel_err[1],
                          self.wheel_err[0], self.wheel_err[1])



def main(args=None):
  rclpy.init(args=args)
  DoroMobileRobot = DOROMobileNode()
  rclpy.spin(DoroMobileRobot)

  DoroMobileRobot.destroy_node()
  rclpy.shutdown()

if __name__ == '__main__':
  main()
