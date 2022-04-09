# /*
# Software License Agreement

# Copyright (c) 2022 LionsBot International Pte Ltd

# All rights reserved.

# Use in source and binary forms, with or without
# modification, are permitted provided that neither the name of the copyright 
# holder nor the names of its contributors may be used to endorse or promote 
# products derived from this software without specific prior written permission.

# Redistribution of the source code or binaries in any medium is not permitted.

# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
# FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
# DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
# SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
# OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

# Contributor(s): Sunardi Tay

# */


#!/usr/bin/env python3
import rospy
from skimage import measure
import cv2
from cv_bridge import CvBridge
import numpy as np
from imutils import contours
import argparse
import imutils
from sensor_msgs.msg import Image

import message_filters


class BrightSpotDetection:

    def __init__(self):
        rospy.init_node("bright_spot_detection_node")

        # image_sub = message_filters.Subscriber("camera/color/image_raw",        Image)
        # depth_sub = message_filters.Subscriber("/camera/depth/image_rect_raw",  Image)

        # self.ts = message_filters.ApproximateTimeSynchronizer([image_sub, depth_sub], 10, 0.5)
        # self.ts.registerCallback(self.processImage)

        rospy.Subscriber("/camera/color/image_raw", Image, self.processImageRGB)
        rospy.Subscriber("/camera/depth/image_rect_raw", Image, self.processImageDepth)

    def spin(self):
        rospy.spin()



    def processImageDepth(self, msg):
        bridge = CvBridge()
        image = bridge.imgmsg_to_cv2(msg, "passthrough")
        cv2.imshow("ImageDepth", image)
        cv2.waitKey(1)


    def processImageRGB(self, msg):
        bridge = CvBridge()
        image = bridge.imgmsg_to_cv2(msg, "bgr8")

        gray    = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        blurred = cv2.GaussianBlur(gray, (11, 11), 0)

        # threshold the image to reveal light regions in the blurred image
        thresh = cv2.threshold(blurred, 200, 255, cv2.THRESH_BINARY)[1]

        # perform a series of erosions and dilations to remove
        # any small blobs of noise from the thresholded image
        thresh = cv2.erode(thresh, None, iterations=2)
        thresh = cv2.dilate(thresh, None, iterations=4)

        # perform a connected component analysis on the thresholded
        # image, then initialize a mask to store only the "large"
        # components
        labels = measure.label(thresh, connectivity=2, background=0)
        mask = np.zeros(thresh.shape, dtype="uint8")
        print("labels : ",labels[5])
        # loop over the unique components
        print("label : ",np.unique(labels))
        for label in np.unique(labels):
            
            # if this is the background label, ignore it
            if label == 0:
                continue

            # otherwise, construct the label mask and count the
            # number of pixels
            labelMask = np.zeros(thresh.shape, dtype="uint8")
            labelMask[labels == label] = 255
            numPixels = cv2.countNonZero(labelMask)

            # if the number of pixels in the component is sufficiently
            # large, then add it to our mask of "large blobs"
            if numPixels > 300:
                mask = cv2.add(mask, labelMask)

        # find the contours in the mask, then sort them from left to right
        cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL,
                                cv2.CHAIN_APPROX_SIMPLE)
        cnts = imutils.grab_contours(cnts)
        print('--------------1--------------')

        if not cnts:
            print('---------------1.1----------------------')
            print('no bright spots')
        else:
            cnts = contours.sort_contours(cnts)[0]
            print('--------------2--------------')
            #print(cnts)

        # loop over the contours
        for (i, c) in enumerate(cnts):
            
            # draw the bright spot on the image
            (x, y, w, h) = cv2.boundingRect(c)
            ((cX, cY), radius) = cv2.minEnclosingCircle(c)
            cv2.circle(image, (int(cX), int(cY)), int(radius), (0, 0, 255), 3)
            cv2.putText(image, "#{}".format(i + 1), (x, y - 15), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 0, 255), 2)

        # show the output imageter
        cv2.imshow("ImageRGB", image)
        cv2.imshow("blurr", blurred)
        cv2.imshow("gray", gray)
        
        cv2.waitKey(1)
       

if __name__ == '__main__':
    detect_node = BrightSpotDetection()
    detect_node.spin()


