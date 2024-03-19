#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge

class collect_image:
    def __init__(self):
        rospy.init_node("collect_imgae_node", anonymous=True)
        self.bridge = CvBridge()
        rospy.Subscriber("/camera/rgb/image_color", Image, self.image_callback)
        self.save_path = "/home/kyo20/office_ws/src/data/"
        self.num = 0

    def image_callback(self, data):
        self.cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        self.resized_image = cv2.resize(self.cv_image, (640, 480))
    
    def loop(self):
        cv2.imwrite(self.save_path+str(self.num)+".png", self.resized_image)
        print(self.num)
        self.num += 1

if __name__ == "__main__":
    ci = collect_image()
    r = rospy.Rate(1)
    rospy.wait_for_message("/camera/rgb/image_color", Image)
    while not rospy.is_shutdown():
        ci.loop()
        r.sleep()
