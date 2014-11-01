"""

"""

import cv2
import cv2.cv as cv

capture = cv2.VideoCapture(cv.CV_CAP_OPENNI)
capture.set(cv.CV_CAP_OPENNI_IMAGE_GENERATOR_OUTPUT_MODE, cv.CV_CAP_OPENNI_VGA_30HZ)

print capture.get(cv.CV_CAP_PROP_OPENNI_REGISTRATION)

while True:
    if not capture.grab():
        print "Unable to Grab Frames from camera"
        break
    okay1, depth_map = capture.retrieve(0, cv.CV_CAP_OPENNI_DEPTH_MAP)
    if not okay1:
        print "Unable to Retrieve Disparity Map from camera"
        break
    okay2, gray_image = capture.retrieve(0, cv.CV_CAP_OPENNI_GRAY_IMAGE)
    if not okay2:
        print "Unable to retrieve Gray Image from device"
        break
    cv2.imshow("depth camera", depth_map)
    cv2.imshow("rgb camera", gray_image)
    if cv2.waitKey(10) == 27:
        break
cv2.destroyAllWindows()
capture.release()
