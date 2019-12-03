// The purpose of this node is to subscriber to the topics publishing wheel ticks 
// from left and right wheel and convert them to distance travelled in cm


import rospy
from std_msgs.msg import Int32


wheel_diameter_cm = 102 // wheel diameter in cm
wheel_circumference_cm = wheel_diameter * math.pi
motor_rpm = 251 //rpm of the wheel
total_encoder_ticks = 700 //total number of encoder ticks per one revolution of the motor
one_encoder_tick = wheel_circumference_cm / total_encoder_ticks = 700


def callback_left(data):
    left_tick_amount = data.msg
    left_distance_moved = left_tick_amount * one_encoder_tick
    //publish left_distance_moved
    rospy.loginfo("This is the left tick amount: "+left_tick_amount)

def callback_right(data):
    right_tick_amount = data.msg
    right_distance_moved = left_tick_amount * one_encoder_tick
    //publish right_distance_moved
    rospy.loginfo("This is the right tick amount: "+right_tick_amount)

    
    
def listener():
    rospy.init_node('ticks_to_distance_converter')
    rospy.Subscriber("/left_encoder_ticks", Int32, callback_left)
    rospy.Subscriber("/right_encoder_ticks", Int32, callback_right)
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()
