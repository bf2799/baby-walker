#!/usr/bin/env python3

import enum
import traceback
from typing import Any, Dict, List

from baby_walker.msg import HwBoolValue

import rospy
import RPi.GPIO as GPIO
from std_msgs.msg import String


class HallEffect:
    def __init__(self):
        """
        Create Publisher for Hall Effect Sensors
        """
        self._publisher = rospy.Publisher(
            f"baby_walker/sensors/hall_effect", HwBoolValue, queue_size=3
        )

    def hall_effect_high(self, index):
        msg = HwBoolValue(index, False)
        self._publisher.publish(msg)

    def hall_effect_low(self, index):
        msg = HwBoolValue(index, True)
        self._publisher.publish(msg)


def hall_callback(pin_num: int) -> None:
    """
    Notify publisher which hall effect went high
    @param pin_num: Pin number of hall effect
    """
    if GPIO.input(pin_num):
        sensors.hall_effect_high(pin_to_hall_type[pin_num])
    else:
        sensors.hall_effect_low(pin_to_hall_type[pin_num])


def setup_hall_effect_gpio(pin_num: int) -> None:
    """
    Set up GPIO and callback for a button on a given pin on the board
    @param pin_num: Pin number of digital button
    """
    GPIO.setup(pin_num, GPIO.IN)
    GPIO.add_event_detect(
        pin_num, GPIO.BOTH, callback=hall_callback, bouncetime=10
    )


if __name__ == "__main__":
    try:
        rospy.init_node("hall_effect_driver")
        pin_num_hall_fl = int(rospy.get_param("~pin_num_hall_fl"))
        pin_num_hall_fr = int(rospy.get_param("~pin_num_hall_fr"))
        pin_num_hall_bl = int(rospy.get_param("~pin_num_hall_bl"))
        pin_num_hall_br = int(rospy.get_param("~pin_num_hall_br"))

        sensors = HallEffect()
        # Set up all buttons so pressed = HIGH and not pressed = LOW
        GPIO.setwarnings(False)  # Documentation says this is OK due to bugs in GPIO
        GPIO.setmode(GPIO.BOARD)
        setup_hall_effect_gpio(pin_num_hall_fl)
        setup_hall_effect_gpio(pin_num_hall_fr)
        setup_hall_effect_gpio(pin_num_hall_bl)
        setup_hall_effect_gpio(pin_num_hall_br)

        pin_to_hall_type: Dict[int, str] = {
            pin_num_hall_fl : "front-left",
            pin_num_hall_fr: "front-right",
            pin_num_hall_bl: "back-left",
            pin_num_hall_br: "back-right",
        }


        rospy.loginfo("Successfully initialized menu_driver node")

        #while not rospy.is_shutdown():
            #rospy.logwarn(GPIO.input(pin_num_hall_0))
            #rospy.sleep(0.1)
        rospy.spin()
    except Exception as e:
        rospy.logerr(f"Failed to initialize menu_driver node: {traceback.format_exc()}")

    GPIO.cleanup()
