#!/usr/bin/env python
import rospy
from std_msgs.msg import Float32MultiArray
import sys

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
        pub = rospy.Publisher('velocidad_deseada', Float32MultiArray, queue_size=10)
        rate = rospy.Rate (10)
        while not rospy.is_shutdown():
            pub.publish(msg)
            rate.sleep()
    except rospy.ROSInterruptException:
        pass
