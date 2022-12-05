#!/usr/bin/env python3
from typing import Dict

from baby_walker.msg import HwFloatValue

import traceback
import rospy
import RPi.GPIO as GPIO

PWM_FREQUENCY = 1


def set_servo_cb(data: HwFloatValue) -> None:
    """
    Set given servo to particular duty cycle.

    @param data: Servo and duty cycle % to set to
    """
    if not (0 <= data.val <= 100):
        rospy.logwarn(
            f"Servo value for servo {data.hw} out of bounds 0-1: {data.val}"
        )
        return
    if data.hw not in servo_hw_dict:
        rospy.logwarn(f"Invalid servo {data.hw}")
        return
    servo_hw_dict[data.hw].ChangeDutyCycle(data.val)


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
        rospy.init_node("servo_driver")
        pin_num_fl = int(rospy.get_param("~pin_num_fl"))
        pin_num_fr = int(rospy.get_param("~pin_num_fr"))
        pin_num_bl = int(rospy.get_param("~pin_num_bl"))
        pin_num_br = int(rospy.get_param("~pin_num_br"))

        # Set up all servos as PWM outputs
        GPIO.setwarnings(False)
        GPIO.setmode(GPIO.BOARD)
        pwm_fl = setup_gpio(pin_num_fl)
        pwm_fr = setup_gpio(pin_num_fr)
        pwm_bl = setup_gpio(pin_num_bl)
        pwm_br = setup_gpio(pin_num_br)

        servo_hw_dict: Dict[str, GPIO.PWM] = {
            "fl": pwm_fl,
            "fr": pwm_fr,
            "bl": pwm_bl,
            "br": pwm_br,
        }

        subscriber = rospy.Subscriber("commands/servo", HwFloatValue, set_servo_cb)

        rospy.spin()

    except Exception:
        rospy.logerr(f"Failed to initialize menu_driver node: {traceback.format_exc()}")

    GPIO.cleanup()
