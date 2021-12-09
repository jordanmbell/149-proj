import numpy as np
import cv2
import cv2.aruco as aruco
import glob
import argparse
import math
import time

cam1_id = 1
cam_wait_ms = 1
num_markers = 9
originID = 4
referenceIDs = [5,6]
markerIDs = range(num_markers)

class camera:
    def __init__(self, mc, dc, originID, markerIDs, camID, refIDs):
        self.matrix_coefficients = mc
        self.distortion_coefficients = dc
        self.originID = originID
        self.markerIDs = markerIDs
        self.referenceIDs = refIDs
        self.calib_sq_matrix = np.array([[1,0], [0, 1]])
        self.firstRvec = None
        self.firstTvec = None
        self.origin_yaw = None

        #aruco_marker_size = 0.1
        self.cam = cv2.VideoCapture(camID)
        #self.cam2 = cv2.VideoCapture(2)

        #is this (below) needed?
        criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

    def inversePerspective(self, rvec, tvec):
        R, _ = cv2.Rodrigues(rvec)
        R = np.matrix(R).T
        invTvec = np.dot(-R, np.matrix(tvec))
        invRvec, _ = cv2.Rodrigues(R)
        return invRvec, invTvec


    def relativePosition(self, rvec1, tvec1, rvec2, tvec2):
        tvec1[2][0] = 0
        tvec2[2][0] = 0
        return 0, tvec2 - tvec1
        rvec1, tvec1 = rvec1.reshape((3, 1)), tvec1.reshape(
            (3, 1))
        rvec2, tvec2 = rvec2.reshape((3, 1)), tvec2.reshape((3, 1))

        # Inverse the second marker, the right one in the image
        invRvec, invTvec = self.inversePerspective(rvec2, tvec2)

        orgRvec, orgTvec = self.inversePerspective(invRvec, invTvec)

        info = cv2.composeRT(rvec1, tvec1, invRvec, invTvec)
        composedRvec, composedTvec = info[0], info[1]

        composedRvec = composedRvec.reshape((3, 1))
        composedTvec = composedTvec.reshape((3, 1))
        return composedRvec, composedTvec


    def draw(self, img, corners, imgpts):
        imgpts = np.int32(imgpts).reshape(-1,2)
        for i,j in zip(range(4),range(4)):
            img = cv2.line(img, tuple(imgpts[i]), tuple(imgpts[j]),(255),3)
        return img

    def calibrate_square(self):
        square_dict = self.find_points(self.referenceIDs, True)
        self.calib_sq_matrix = np.array([
            [square_dict[self.referenceIDs[0]][0], square_dict[self.referenceIDs[1]][0]],
            [square_dict[self.referenceIDs[0]][1], square_dict[self.referenceIDs[1]][1]],
        ])
        self.calib_sq_matrix = np.linalg.solve(self.calib_sq_matrix, np.array([[0.22, 0], [0,0.22]]))
        # print(self.calib_sq_matrix @ square_dict[self.referenceIDs[1]][:2])
        # return self.calib_sq_matrix

    def find_points(self, point_markers, isCalibrating):
        tvec_dict = {}
        composedTvec = None
        # firstRvec = None
        # firstTvec = None
        secondRvec = None
        secondTvec = None
        ret1, self.frame = self.cam.read() #originally 1, but the tutorial left it blank?
        gray = cv2.cvtColor(self.frame, cv2.COLOR_BGR2GRAY)  # Change grayscale
        aruco_dict = aruco.Dictionary_get(aruco.DICT_4X4_250)  # Use 5x5 dictionary to find markers
        parameters = aruco.DetectorParameters_create()  # Marker detection parameters

        # lists of ids and the corners beloning to each id
        corners, ids, rejected_img_points = aruco.detectMarkers(gray, aruco_dict,parameters=parameters,cameraMatrix=self.matrix_coefficients, distCoeff=self.distortion_coefficients)

        if np.all(ids is not None):  # If there are markers found by detector
            zipped = zip(ids, corners)
            ids, corners = zip(*(sorted(zipped)))
            axis = np.float32([[-0.01, -0.01, 0], [-0.01, 0.01, 0], [0.01, -0.01, 0], [0.01, 0.01, 0]]).reshape(-1, 3)
            for i in range(0, len(ids)):  # Iterate in markers
                # Estimate pose of each marker and return the values rvec and tvec---different from camera coefficients
                #0.1 below is aruco_marker_size
                aruco_marker_size = 0.2
                rvec, tvec, markerPoints = aruco.estimatePoseSingleMarkers(corners[i], aruco_marker_size, self.matrix_coefficients, self.distortion_coefficients)
                if ids[i] == self.originID:
                    self.firstRvec = rvec
                    self.firstTvec = tvec
                    isFirstMarkerCalibrated = True
                    firstMarkerCorners = corners[i]

                    z1    = math.atan2(rvec[0][0][0], rvec[0][0][1])
                    if isCalibrating:
                        self.origin_yaw = z1

                for mark in point_markers:
                    if ids[i] == mark:
                        secondRvec = rvec
                        secondTvec = tvec
                        isSecondMarkerCalibrated = True
                        secondMarkerCorners = corners[i]

                        if self.originID in ids and self.firstRvec is not None and self.firstTvec is not None:  # If there are two markers, reverse the second and get the difference

                            aruco.drawAxis(self.frame, self.matrix_coefficients, self.distortion_coefficients, secondRvec, secondTvec, 0.1)

                            self.firstRvec, self.firstTvec = self.firstRvec.reshape((3, 1)), self.firstTvec.reshape((3, 1))
                            secondRvec, secondTvec = secondRvec.reshape((3, 1)), secondTvec.reshape((3, 1))

                            composedRvec, composedTvec = self.relativePosition(self.firstRvec, self.firstTvec, secondRvec, secondTvec)
                            #calculating yaw
                            # sin_x = math.sqrt(rvec[2,0] * rvec[2,0] +  rvec[2,1] * rvec[2,1])

                            if not isCalibrating:
                                # print(self.origin_yaw)
                                z1    = math.atan2(rvec[0][0][0], rvec[0][0][1])
                                z1 = z1 - self.origin_yaw
                                z1 = (z1*(-360)/math.pi) % 360
                            pos =  self.calib_sq_matrix @ composedTvec[:2]

                            tvec_dict[mark] = [pos[0][0], pos[1][0], z1]
                            # print(tvec_dict[mark])
                            # tvec_dict[mark] = composedTvec[:2]

                (rvec - tvec).any()  # get rid of that nasty numpy value array error

                aruco.drawDetectedMarkers(self.frame, corners)  # Draw A square around the markers
            return tvec_dict

    def show_image(self):
        cv2.imshow('frame_'+str(self.cam), self.frame)

    def end_capture(self):
        self.cam.release()


def loadCoefficients(filePath):
    # FILE_STORAGE_READ
    cv_file = cv2.FileStorage(filePath, cv2.FILE_STORAGE_READ)
    # cv_file = cv2.FileStorage("images/test.yaml", cv2.FILE_STORAGE_READ)

    # note we also have to specify the type to retrieve other wise we only get a
    # FileNode object back instead of a matrix
    camera_matrix = cv_file.getNode("camera_matrix").mat()
    dist_matrix = cv_file.getNode("dist_coeff").mat()

    # Debug: print the values
    # print("camera_matrix : ", camera_matrix.tolist())
    # print("dist_matrix : ", dist_matrix.tolist())

    cv_file.release()
    return [camera_matrix, dist_matrix]

def track(shared_dict):
     composedRvec, composedTvec = None, None
     composedRvec2, composedTvec2 = None, None

     # mtx, dist = loadCoefficients("../camera_calibration_tests/images_canon_floor/calibrationCoefficients2.yaml")
     mtx, dist = loadCoefficients("../camera_calibration_tests/images_canon/calibrationCoefficients.yaml")
     # mtx, dist = loadCoefficients("images_webcam_black_checkerboard/calibrationCoefficients.yaml")
     # mtx2, dist2 = loadCoefficients("images_sony/calibrationCoefficients.yaml")

     cam1 = camera(mtx, dist, originID, markerIDs, cam1_id, referenceIDs)
     # cam2 = camera(mtx2, dist2, 0, [1], 2)
     cam1.calibrate_square()

     while True:
         vec1_cam1 = cam1.find_points(markerIDs, False)
         # vec1_cam2 = cam2.find_points()
         cam1.show_image()
         # cam2.show_image()

         if vec1_cam1 is not None:
             for id in vec1_cam1.keys():
                 # shared_dict[id] = [vec1_cam1[id][0], vec1_cam1[id][1], 0]  #Need to add rotation
                 shared_dict[id] = vec1_cam1[id]  #Need to add rotation
                 if id in referenceIDs or id == originID:
                     print(id, " ",shared_dict[id][2])
         #else:
             #print("out of frame")

         # if vec1_cam1 is not None and vec1_cam2 is not None:
         #     print((vec1_cam1+vec1_cam2) / 2)
         # elif vec1_cam1 is not None:
         #     print(vec1_cam1)
         # elif vec1_cam2 is not None:
         #     print(vec1_cam2)
         # else:
         #     print("out of frame")

         # Wait 3 miliseconds for an interaction. Check the key and do the corresponding job.
         key = cv2.waitKey(cam_wait_ms) & 0xFF
         if key == ord('q'):  # Quit
             break

     # When everything done, release the captures
     cam1.end_capture()
     # cam2.end_capture()
     cv2.destroyAllWindows()

# Dummy tracking function for debugging
def track_dummy(shared_dict):
    r = 3
    tau = 10
    t = 0
    while True:
        shared_dict[0] = [r * np.cos(t/tau), r * np.sin(t/tau), -1]
        t += 1
        time.sleep(0.1)
