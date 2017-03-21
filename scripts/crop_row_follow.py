import cv2
import numpy as np
import rospy


def test():
    rospy.init_node('crop_row_follow', anonymous=True)
    rate = rospy.Rate(1)
    while not rospy.is_shutdown():
        print 'test'
        rospy.sleep()


if __name__ == '__main__':
    try:
        test()
    except rospy.ROSInterruptException:
        pass
