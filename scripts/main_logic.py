from baby_walker.msg import HwFloatValue
from std_msgs.msg import Float32, String
from baby_walker.msg import HwBoolValue
import rospy


def callback_hall(data: HwBoolValue):
    """
    If Hall Effect sensor tripped to True, break corresponding motor and turn on buzzer. If tripped to false,
    remove break on corresponding motor and turn off buzzer.

    @param data: Message from the hall effect topic.
    """
    if data.val:
        msg_servo = HwFloatValue(data.hw, brake_power)
        pub_servo.publish(msg_servo)
        pub_buzz.publish(1)
    elif not data.val:
        msg_servo = HwFloatValue(data.hw, 0)
        pub_servo.publish(msg_servo)
        pub_buzz.publish(0)


def callback_brake_power(data: String):
    brake_power = int(data.data)

def callback_new_environment(data: String):
    pass

if __name__ == '__main__':
    brake_power = 5
    rospy.init_node('main_node', anonymous=True)

    rospy.loginfo("Initialization complete")

    rospy.Subscriber('sensors/hall_effect', HwBoolValue, callback_hall)
    rospy.Subscriber('settings/brake_power', String, callback_brake_power)
    rospy.Subscriber('settings/new_environment', String, callback_new_environment)
    pub_servo = rospy.Publisher('commands/servo', HwFloatValue)
    pub_buzz = rospy.Publisher('commands/buzzer', Float32)

    rospy.spin()