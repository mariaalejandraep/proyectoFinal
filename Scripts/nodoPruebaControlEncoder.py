#!/usr/bin/env python
import rospy
from std_msgs.msg import Float32MultiArray
import sys


def handle_velocidad_deseada(float):
    print(float)


if __name__ == '__main__':
    try:
        msg = Float32MultiArray()
        msg.data = [1, 1]
        if len(sys.argv) > 2:
            print("Hay parametros")
            try:
                vel1 = float(sys.argv[1])
                vel2 = float(sys.argv[2])
                msg.data = [vel1, vel2]
            except ValueError:
                pass
        rospy.init_node('pruebaEncoder', anonymous=True)
        rospy.Subscriber('velocidad_deseada', Float32MultiArray, handle_velocidad_deseada)
        rate = rospy.Rate (10)
        while not rospy.is_shutdown():
            rate.sleep()
    except rospy.ROSInterruptException:
        pass


## rostopic pub motor/speed std_msgs/Int16MultiArray "{layout: {dim: [{label: '', size: 0, stride: 0}], data_offset: 0}, data: [64, 64]}"
