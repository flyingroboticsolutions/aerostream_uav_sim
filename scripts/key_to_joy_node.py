#!/bin/env python3
import rospy
from sensor_msgs.msg import Joy
from pynput import keyboard

AXES_COUNT = 6
joy = Joy()
joy.axes = AXES_COUNT * [0]

key_to_index = {}
key_to_index[keyboard.Key.up]    = 3
key_to_index[keyboard.Key.down]  = 3
key_to_index[keyboard.Key.left]  = 2
key_to_index[keyboard.Key.right] = 2
key_to_index['w'] = 1
key_to_index['s'] = 1
key_to_index['a'] = 0
key_to_index['d'] = 0

key_to_press_value = {}
key_to_press_value[keyboard.Key.up]    =  1.0
key_to_press_value[keyboard.Key.down]  = -1.0
key_to_press_value[keyboard.Key.left]  =  1.0
key_to_press_value[keyboard.Key.right] = -1.0
key_to_press_value['w'] =  1.0
key_to_press_value['s'] = -1.0
key_to_press_value['a'] =  1.0
key_to_press_value['d'] = -1.0

# TODO: asymptotic value change?
def on_press(key):
    try:
        if (key.char in key_to_index):
            joy.axes[key_to_index[key.char]] = key_to_press_value[key.char]
    except AttributeError:
        if (key in key_to_index):
            joy.axes[key_to_index[key]] = key_to_press_value[key]

def on_release(key):
    try:
        if (key.char in key_to_index):
            joy.axes[key_to_index[key.char]] = 0.0
    except AttributeError:
        if (key in key_to_index):
            joy.axes[key_to_index[key]] = 0.0

def main():
    rospy.init_node('key_to_joy_node', anonymous=True)
    pub = rospy.Publisher('joy', Joy, queue_size=1)
    rate = rospy.Rate(10)

    listener = keyboard.Listener(on_press=on_press, on_release=on_release)
    listener.start()

    while not rospy.is_shutdown():
        pub.publish(joy)
        rate.sleep()

if __name__ == "__main__":
    main()
