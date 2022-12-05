#!/usr/bin/env python3

import traceback
from typing import Dict, List

import rospy
import RPi.GPIO as GPIO
from std_msgs.msg import String


class BabyWalkerMenu:
    def __init__(self):
        """
        Create menu for ROS with special baby walker options
        """
        self._options: Dict[str, List[str]] = {
            "Brake Power": ["1", "2", "3", "4", "5"],
            "New Environment": ["No", "Yes"],
        }
        self._publishers = [
            rospy.Publisher(
                f"baby_walker/settings/{option.replace(' ', '_').lower()}",
                String,
                queue_size=3,
            )
            for option in self._options.keys()
        ]
        self._cur_selection: int = 0
        self._cur_menu_level: int = 0
        self._selected_options: List[int] = [0] * len(self._options)
        self._update_screen()

    def _update_screen(self):
        """
        Refresh screen to what user should be seeing
        """
        # TODO: Update an actual screen instead of logging in ROS
        rospy.loginfo(
            f"{list(self._options)[self._cur_selection]} >"
            if self._cur_menu_level == 0
            else f"< {list(self._options.values())[self._cur_selection][self._selected_options[self._cur_selection]]}"
        )

    def press_button(self, type: str):
        """
        Update menu state based on button press

        @param type: Type of button pressed
        """
        if self._cur_menu_level == 0:
            if type == "up":
                self._cur_selection -= 1
            if type == "down":
                self._cur_selection += 1
            if type == "right":
                self._cur_menu_level = 1
            self._cur_selection %= len(self._options)
        if self._cur_menu_level == 1:
            if type == "up":
                self._selected_options[self._cur_selection] -= 1
            if type == "down":
                self._selected_options[self._cur_selection] += 1
            if type == "left":
                self._publishers[self._cur_selection].publish(
                    list(self._options.values())[self._cur_selection][
                        self._selected_options[self._cur_selection]
                    ]
                )
                self._cur_menu_level = 0
            self._selected_options[self._cur_selection] %= len(
                list(self._options.values())[self._cur_selection]
            )

        self._update_screen()


def button_pressed_callback(pin_num: int) -> None:
    """
    Notify menu which button was called

    @param pin_num: Pin number of digital button
    """
    menu.press_button(pin_to_button_type[pin_num])


def setup_button_gpio(pin_num: int) -> None:
    """
    Set up GPIO and callback for a button on a given pin on the board

    @param pin_num: Pin number of digital button
    """
    GPIO.setup(pin_num, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
    GPIO.add_event_detect(
        pin_num, GPIO.RISING, callback=button_pressed_callback, bouncetime=500
    )


if __name__ == "__main__":
    try:
        rospy.init_node("menu_driver")
        pin_num_up_button = int(rospy.get_param("~pin_num_up_button"))
        pin_num_right_button = int(rospy.get_param("~pin_num_right_button"))
        pin_num_down_button = int(rospy.get_param("~pin_num_down_button"))
        pin_num_left_button = int(rospy.get_param("~pin_num_left_button"))

        # Set up all buttons so pressed = HIGH and not pressed = LOW
        GPIO.setwarnings(False)  # Documentation says this is OK due to bugs in GPIO
        GPIO.setmode(GPIO.BOARD)
        setup_button_gpio(pin_num_up_button)
        setup_button_gpio(pin_num_right_button)
        setup_button_gpio(pin_num_down_button)
        setup_button_gpio(pin_num_left_button)

        pin_to_button_type: Dict[int, str] = {
            pin_num_up_button: "up",
            pin_num_right_button: "right",
            pin_num_down_button: "down",
            pin_num_left_button: "left",
        }

        menu = BabyWalkerMenu()

        rospy.loginfo("Successfully initialized menu_driver node")

        rospy.spin()

    except Exception:
        rospy.logerr(f"Failed to initialize menu_driver node: {traceback.format_exc()}")

    GPIO.cleanup()
