#!/usr/bin/env python

import rospy
from crop_row_follow import CropRowFind, VisionCV2
from crop_row_follow.srv import VisualDebug


def visual_debug_cb(request):
    return crf.visual_debug(request)


if __name__ == 'main':
    try:
        crf = CropRowFind()
        service = rospy.Service('crf_debug', VisualDebug, visual_debug_cb)
        while not rospy.is_shutdown():
            rospy.spin()
    except rospy.ROSInterruptException:
        pass

