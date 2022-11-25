#!/usr/bin/env python3
import rospy
import can
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray

def map(Input, min_input, max_input, min_output, max_output):
    value = ((Input - min_input)*(max_output-min_output)/(max_input - min_input) + min_output)
    return value
class can_testing:
    def __init__(self):
        self.bus = can.interface.Bus(channel="can0", bustype="socketcan")
        # ros
        self.sub_cmd  = rospy.Subscriber('cmd_vel', Twist, self.subCmdCB, queue_size=10)
        self.timer_can = rospy.Timer(rospy.Duration(1/500), self.timerCanCB)    # timer 100Hz
        self.msg_pub = rospy.Publisher('feedback',Float32MultiArray)
    def subCmdCB(self, msg):
        vx = msg.linear.y
        Vx = int(map(vx,-100 ,100 ,0 ,65535))
################################################
        self.TxData[0] = ((Vx & 0xFF00) >> 8)
        self.TxData[1] = (Vx & 0x00FF)
        self.TxData[2] = ((Vx & 0xFF00) >> 8)
        self.TxData[3] = (Vx & 0x00FF)
        self.TxData[4] = ((Vx & 0xFF00) >> 8)
        self.TxData[5] = (Vx & 0x00FF)
        self.TxData[6] = ((Vx & 0xFF00) >> 8)
        self.TxData[7] = (Vx & 0x00FF)
        
    def timerCanCB(self, event):
        message = can.Message(arbitration_id=0X111, is_extended_id=False,
                      data=self.TxData)
        self.bus.send(message,0.1)
        for i in range(3):
            can_msg = self.bus.recv(0.1)
            if can_msg.arbitration_id == 0x155:
                V1_back = can_msg.data[0] << 8 | can_msg.data[1]
                V2_back = can_msg.data[2] << 8 | can_msg.data[3]
            elif can_msg.arbitration_id == 0x140:
                V3_back = can_msg.data[0] << 8 | can_msg.data[1]
                V4_back = can_msg.data[2] << 8 | can_msg.data[3]
            elif can_msg.arbitration_id == 0x101:
                gyro_yaw = can_msg.data[0] << 8 | can_msg.data[1]
        pub_msg = Float32MultiArray()
        pub_msg.data = [V1_back, V2_back, V3_back, V4_back, gyro_yaw]
        self.msg_pub.publish(pub_msg)
        
if __name__ == '__main__':
    try:
        rospy.init_node('can_node', anonymous=True)
        can_testing()
        rospy.spin()
    except KeyboardInterrupt:
        pass