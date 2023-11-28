#!/usr/bin/env python

from jsk_gui_msgs.srv import YesNo
import rospy
from std_srvs.srv import Trigger


def main():
    rospy.wait_for_service('rqt_yn_btn', timeout=None)
    rospy.wait_for_service('data_collection_server/save_request', timeout=None)
    rqt_yn_btn = rospy.ServiceProxy('rqt_yn_btn', YesNo)
    save_req = rospy.ServiceProxy(
        'data_collection_server/save_request', Trigger)

    while not rospy.is_shutdown():
        can_save = False
        while can_save is False or can_save.yes is False:
            try:
                can_save = rqt_yn_btn('Press [Yes] to save image.')
            except rospy.ServiceException as e:
                rospy.logerr('Service call failed. [{}]'.format(e))
        save_res = save_req()
        rospy.loginfo('success: {}, message: {}'.format(
            save_res.success, save_res.message))


if __name__ == '__main__':
    rospy.init_node('save_image_by_btn')
    main()
    rospy.spin()
