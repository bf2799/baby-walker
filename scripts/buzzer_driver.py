#!/usr/bin/env python3

import traceback
import rospy
import RPi.GPIO as GPIO

from std_msgs.msg import Float32

PWM_FREQUENCY = 911

class Buzzer:
    def __init__(self):
        """
        Initialize Buzzer Delay
        """
        self.delay = 1

    def set_delay(self, delay_dur):
        self.delay = delay_dur


def set_buzzer_delay(data: Float32) -> None:
    """
    Set given servo to particular duty cycle.

    @param data: Time to sleep buzzer
    """
    buzz.set_delay(data.data)



def setup_gpio(pin_num: int) -> GPIO.PWM:
    """
    Set up GPIO as PWM for provided pin

    @param pin_num: Pin number to set up as PWM output
    @return PWM instance
    """
    GPIO.setup(pin_num, GPIO.OUT)
    pwm = GPIO.PWM(pin_num, PWM_FREQUENCY)
    pwm.start(0)
    return pwm


if __name__ == "__main__":
    try:
        rospy.init_node("buzzer_driver")
        pin_num_buzz = int(rospy.get_param("~pin_num_buzz"))

        # Set up all servos as PWM outputs
        GPIO.setwarnings(False)
        GPIO.setmode(GPIO.BOARD)
        pwm_buzz = setup_gpio(pin_num_buzz)

        buzz = Buzzer()
        subscriber = rospy.Subscriber("commands/buzzer", Float32, set_buzzer_delay)

        while not rospy.is_shutdown():
            if buzz.delay == 0:
                pwm_buzz.ChangeDutyCycle(0)
            else:
                pwm_buzz.ChangeDutyCycle(50)
                rospy.sleep(buzz.delay)
                pwm_buzz.ChangeDutyCycle(0)
                rospy.sleep(buzz.delay)


    except Exception:
        rospy.logerr(f"Failed to initialize buzzer_driver node: {traceback.format_exc()}")

    GPIO.cleanup()

