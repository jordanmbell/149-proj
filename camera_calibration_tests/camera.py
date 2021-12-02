import numpy as np
import cv2
import cv2.aruco as aruco
import glob
import argparse
import math

class camera:
    def __init__(self, matrix_coefficients, distortion_coefficients, originID, markerIDs, camera):
        self.matrix_coefficients = matrix_coefficients
        self.distortion_coefficients = distortion_coefficients
        self.originID = originID
        self.markerIDs = markerIDs

        #aruco_marker_size = 0.1
        self.cam = cv2.VideoCapture(camera)
        #self.cam2 = cv2.VideoCapture(2)

        #is this (below) needed?
        #criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

    def inversePerspective(self, rvec, tvec):
        R, _ = cv2.Rodrigues(rvec)
        R = np.matrix(R).T
        invTvec = np.dot(-R, np.matrix(tvec))
        invRvec, _ = cv2.Rodrigues(R)
        return invRvec, invTvec


    def relativePosition(self, rvec1, tvec1, rvec2, tvec2):
        rvec1, tvec1 = rvec1.reshape((3, 1)), tvec1.reshape(
            (3, 1))
        rvec2, tvec2 = rvec2.reshape((3, 1)), tvec2.reshape((3, 1))

        # Inverse the second marker, the right one in the image
        invRvec, invTvec = self.inversePerspective(rvec2, tvec2)

        orgRvec, orgTvec = self.inversePerspective(invRvec, invTvec)
        # print("rvec: ", rvec2, "tvec: ", tvec2, "\n and \n", orgRvec, orgTvec)

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

    def find_points(self):
        composedTvec = None
        firstRvec = None
        firstTvec = None
        secondRvec = None
        secondTvec = None
        ret1, frame = self.cam.read() #originally 1, but the tutorial left it blank?
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)  # Change grayscale
        aruco_dict = aruco.Dictionary_get(aruco.DICT_6X6_250)  # Use 5x5 dictionary to find markers
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
                rvec, tvec, markerPoints = aruco.estimatePoseSingleMarkers(corners[i], 0.1, self.matrix_coefficients,
                                                                           self.distortion_coefficients)

                if ids[i] == self.originID:
                    firstRvec = rvec
                    print("first marker rvec", firstRvec)
                    firstTvec = tvec
                    print("first marker tvec", firstTvec)
                    isFirstMarkerCalibrated = True
                    firstMarkerCorners = corners[i]
                elif ids[i] == self.markerIDs[0]:
                    secondRvec = rvec
                    print("second marker rvec", secondRvec)
                    secondTvec = tvec
                    print("second marker tvec", secondTvec)
                    isSecondMarkerCalibrated = True
                    secondMarkerCorners = corners[i]
                # elif ids[i] == thirdMarkerID:
                #     thirdRvec = rvec
                #     thirdTvec = tvec
                #     isThirdMarkerCalibrated = True
                #     thirdMarkerCorners = corners[i]

                (rvec - tvec).any()  # get rid of that nasty numpy value array error

                aruco.drawDetectedMarkers(frame, corners)  # Draw A square around the markers

            if len(ids) > 1 and firstRvec is not None and firstTvec is not None and secondTvec is not None and secondRvec is not None:  # If there are two markers, reverse the second and get the difference
                firstRvec, firstTvec = firstRvec.reshape((3, 1)), firstTvec.reshape((3, 1))
                # secondRvec, secondTvec = secondRvec.reshape((3, 1)), secondTvec.reshape((3, 1))
                # thirdRvec, thirdTvec = thirdRvec.reshape((3, 1)), thirdTvec.reshape((3, 1))

                composedRvec, composedTvec = self.relativePosition(firstRvec, firstTvec, secondRvec, secondTvec)
                # composedRvec2, composedTvec2 = relativePosition(firstRvec, firstTvec, thirdRvec, thirdTvec)

                # print("ComposedTvec of vec1 - "+str(camera), composedTvec)
                # print("ComposedTvec of vec2 -"+str(camera), composedTvec2)

            cv2.imshow('frame_'+str(self.cam), frame)
            return composedTvec

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

def track():
     composedRvec, composedTvec = None, None
     composedRvec2, composedTvec2 = None, None

     mtx, dist = loadCoefficients("images_canon/calibrationCoefficients.yaml")
     mtx2, dist2 = loadCoefficients("images_sony/calibrationCoefficients.yaml")

     cam1 = camera(mtx, dist, 0, [1], 1)
     cam2 = camera(mtx2, dist2, 0, [1], 2)

     while True:
         vec1_cam1 = cam1.find_points()
         vec1_cam2 = cam2.find_points()
         # print(vec1_cam1)
         # print(vec1_cam2)
         if vec1_cam1 is not None and vec1_cam2 is not None:
             print((vec1_cam1+vec1_cam2) / 2)
         elif vec1_cam1 is not None:
             print(vec1_cam1)
         elif vec1_cam2 is not None:
             print(vec1_cam2)
         else:
             print("out of frame")
         # Wait 3 miliseconds for an interaction. Check the key and do the corresponding job.
         key = cv2.waitKey(300) & 0xFF
         if key == ord('q'):  # Quit
             break

     # When everything done, release the captures
     cam1.end_capture()
     cam2.end_capture()
     cv2.destroyAllWindows()

if __name__ == '__main__':
    track()
