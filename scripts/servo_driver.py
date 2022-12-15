#!/usr/bin/env python3
from typing import Dict

from baby_walker.msg import HwFloatValue

import dataclasses
import math
import traceback
import rospy
import RPi.GPIO as GPIO

from typing import List


SPEED_MMPS = 10.16

@dataclasses.dataclass
class ServoProperties:
    pin_num_fwd: int
    pin_num_bkwd: int
    cur_pos: float


def set_servo_cb(data: HwFloatValue) -> None:
    """
    Set given servo to particular position (in m).

    @param data: Servo and position (in m) to set to
    """
    if data.hw not in servo_hw_dict:
        rospy.logwarn(f"Invalid servo {data.hw}")
        return
    cur_servo = servo_hw_dict[data.hw]
    pos_change = data.val - cur_servo.cur_pos
    if math.isclose(pos_change, 0):
        return
    target_pin = cur_servo.pin_num_fwd if pos_change > 0 else cur_servo.pin_num_bkwd
    GPIO.output(target_pin, GPIO.HIGH)
    rospy.Timer(rospy.Duration(abs(data.val - cur_servo.cur_pos) / SPEED_MMPS), lambda _: GPIO.output(target_pin, GPIO.LOW), oneshot=True)
    servo_hw_dict[data.hw].cur_pos = data.val


def setup_gpio(pin_num: int) -> None:
    """
    Set up GPIO as PWM for provided pin

    @param pin_num: Pin number to set up as GPIO output
    """
    GPIO.setup(pin_num, GPIO.OUT)
    GPIO.output(pin_num, GPIO.LOW)


if __name__ == "__main__":
    try:
        rospy.init_node("servo_driver")
        pin_num_flf = int(rospy.get_param("~pin_num_flf"))
        pin_num_flb = int(rospy.get_param("~pin_num_flb"))
        pin_num_frf = int(rospy.get_param("~pin_num_frf"))
        pin_num_frb = int(rospy.get_param("~pin_num_frb"))
        pin_num_blf = int(rospy.get_param("~pin_num_blf"))
        pin_num_blb = int(rospy.get_param("~pin_num_blb"))
        pin_num_brf = int(rospy.get_param("~pin_num_brf"))
        pin_num_brb = int(rospy.get_param("~pin_num_brb"))

        # Set up all servos as PWM outputs
        GPIO.setwarnings(False)
        GPIO.setmode(GPIO.BOARD)
        for pin_num in (pin_num_flf, pin_num_frf, pin_num_blf, pin_num_brf, pin_num_flb, pin_num_frb, pin_num_blb, pin_num_brb):
            setup_gpio(pin_num)

        servo_hw_dict: Dict[str, ServoProperties] = {
            "fl": ServoProperties(pin_num_flf, pin_num_flb, 0),
            "fr": ServoProperties(pin_num_frf, pin_num_frb, 0),
            "bl": ServoProperties(pin_num_blf, pin_num_blb, 0),
            "br": ServoProperties(pin_num_brf, pin_num_brb, 0),
        }

        subscriber = rospy.Subscriber("commands/servo", HwFloatValue, set_servo_cb)

        rospy.spin()

    except Exception:
        rospy.logerr(f"Failed to initialize menu_driver node: {traceback.format_exc()}")

    GPIO.cleanup()
