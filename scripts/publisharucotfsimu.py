#this is for publishing the aruco marker's tf
#!/usr/bin/env python
# license removed for brevity
import rospy
from geometry_msgs.msg import Pose
from ur5_moveit.msg import Poses
import numpy as np
import cv2
import tf
import cv2.aruco as aruco
import glob
import sys
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge, CvBridgeError

def calibratecamera():

    # termination criteria for the iterative algorithm
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

    # prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
    # checkerboard of size (7 x 6) is used
    objp = np.zeros((5*7,3), np.float32)
    objp[:,:2] = np.mgrid[0:7,0:5].T.reshape(-1,2)

    # arrays to store object points and image points from all the images.
    objpoints = [] # 3d point in real world space
    imgpoints = [] # 2d points in image plane.

    # iterating through all calibration images
    # in the folder
    images = glob.glob('src/checkerboard/*.jpg')
    print(len(images))
    gray = None
    
    for fname in images:
        print("looping image")
        img = cv2.imread(fname)
        gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
        # find the chess board (calibration pattern) corners
        ret, corners = cv2.findChessboardCorners(gray, (7,5),None)



        # if calibration pattern is found, add object points,
        # image points (after refining them)
        if ret == True:
            objpoints.append(objp)

            # Refine the corners of the detected corners
            corners2 = cv2.cornerSubPix(gray,corners,(11,11),(-1,-1),criteria)
            imgpoints.append(corners2)

            # Draw and display the corners
            img = cv2.drawChessboardCorners(img, (7,5), corners2,ret)

    print("objpoints: " + str(len(objpoints)))
    print("imgpoints: " + str(len(imgpoints)))
    ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1],None,None)
    print(dist)

    return ret, mtx, dist, rvecs, tvecs

def publishtf(cap, mtx, dist):
    # ret, frame = cap
    #if ret returns false, there is likely a problem with the webcam/camera.
    #In that case uncomment the below line, which will replace the empty frame 
    #with a test image used in the opencv docs for aruco at https://www.docs.opencv.org/4.5.3/singlemarkersoriginal.jpg
    # cap = cv2.imread('camera_image.jpeg') 

    # operations on the frame
    gray = cv2.cvtColor(cap, cv2.COLOR_BGR2GRAY)
    
    # set dictionary size depending on the aruco marker selected
    aruco_dict = aruco.Dictionary_get(aruco.DICT_5X5_1000)

    # detector parameters can be set here (List of detection parameters[3])
    parameters = aruco.DetectorParameters_create()
    parameters.adaptiveThreshConstant = 10

    # lists of ids and the corners belonging to each id
    corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)

    # font for displaying text (below)
    font = cv2.FONT_HERSHEY_SIMPLEX

    # check if the ids list is not empty
    # if no check is added the code will crashHeader header
    rvec = None 
    tvec = None
    # if np.all(ids != None):
    #     print ("321")

        # estimate pose of each marker and return the values
        # rvet and tvec-different from camera coefficients
    rvec, tvec ,_ = aruco.estimatePoseSingleMarkers(corners, 0.05, mtx, dist)
    print (tvec)
    #(rvec-tvec).any() # get rid of that nasty numpy value array error

    # for i in range(0, ids.size):
    #     # draw axis for the aruco markersrame_
    #     cv2.drawFrameAxes(cap, mtx, dist, rvec[i], tvec[i], 0.1)

    # draw a square around the markers
    aruco.drawDetectedMarkers(cap, corners)


    # code to show ids of the marker found
    strg = ''
    for i in range(0, ids.size):
        strg += str(ids[i][0])+', '

    cv2.putText(cap, "Id: " + strg, (0,64), font, 1, (0,255,0),2,cv2.LINE_AA)


    # else:
    #     print("123")
    #     # code to show 'No Ids' when no markers are found
    #     cv2.putText(cap, "No Ids", (0,64), font, 1, (0,255,0),2,cv2.LINE_AA)

    # display the resulting framecap = cv2.imread('camera_image.jpeg') 
    cv2.imshow('frame',cap)
    # if cv2.waitKey(1) & 0xFF == ord('q'):
    #     break


    # we need a homogeneous matrix but OpenCV only gives us a 3x3 rotation matrix
    rotation_matrix = np.array([[0, 0, 0, 0],
                                [0, 0, 0, 0],
                                [0, 0, 0, 0],
                                [0, 0, 0, 1]],
                                 dtype=float)
    rotation_matrix[:3, :3], _ = cv2.Rodrigues(rvec)

    # convert the matrix to a quaternion
    quaternion = tf.transformations.quaternion_from_matrix(rotation_matrix)
    return quaternion,tvec,ids

class ImageConvert:

    def __init__(self):
        self.node_name="cv_bridge_demo"
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("ur5/camera1/image_raw",Image,self.callback)

    def callback(self,data):
        try:
            # print("cb triggered")
            self.cv_image = self.bridge.imgmsg_to_cv2(data, "rgb8")
            img = cv2.imwrite('camera_image.jpeg', self.cv_image)
        except CvBridgeError as e:
            print(e)
       
    

        # (rows,cols,channels) = cv_image.shapeargs[
        # if cols > 60 and rows > 60 :
        #     cv2.circle(cv_image, (50,50), 10, 255)

        # cv2.imshow("Image window", cv_image)
        # cv2.waitKey(3)

        # try:
        #     self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
        # except CvBridgeError as e:
        #     print(e)

    def get_data(self):
        if(self.cv_image.any() == None):
            print("get data is null")

        return self.cv_image

# def talker(quaternion, xyz,rvec,tvec):
    
if __name__ == '__main__':
    try:
        image_convert = ImageConvert()
        pub = rospy.Publisher('arucotf', Poses, queue_size=10)
        rospy.init_node('talker', anonymous=True)
        rate = rospy.Rate(10) # 10hz
        ret, mtx, dist, rvecs, tvecs = calibratecamera()
        

        while not rospy.is_shutdown():
            cap=image_convert.get_data()
            cv2.imshow("image",cap)
            # temp_cap=cv2.imread(cv2.samples.findFile("camera_image.jpeg"))
            # cap=np.array(temp_cap, dtype=np.uint8)ids,
            # cap = cv2.imread('camera_image.jpeg') 
            quaternion,tvec,ids = publishtf(cap, mtx, dist)
            # Pose pose
            pose = Pose()
            message = Poses()
            pose.position.x = tvec[0][0][0]
            pose.position.y = tvec[0][0][1]
            pose.position.z = tvec[0][0][2]
            pose.orientation.w = quaternion[3]
            pose.orientation.x = quaternion[0]
            pose.orientation.y = quaternion[1]
            pose.orientation.z = quaternion[2]

            message.Poses.pose = pose
            message.Poses.header.stamp = rospy.Time.now()
            message.Poses.header.frame_id = "camera_link"
            message.Ids=ids


            
            rospy.loginfo(message)
            
            pub.publish(message)
 
    except rospy.ROSInterruptException:
        pass

# When everything done, release the capture
# cap.release()
cv2.destroyAllWindows()