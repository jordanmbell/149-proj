#code from aruco_pos--------------------------------------------------------------------------------------------------------------------

import numpy as np
import cv2
import cv2.aruco as aruco
import glob
import argparse
import math

aruco_marker_size = 0.1
originMarker = 0
markerList = [1, 3, 2]

cap = cv2.VideoCapture(0)

# termination criteria
criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)


firstMarkerID = None
secondMarkerID = None


def calibrate():
    # prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(8,6,0)
    objp = np.zeros((6*9,3), np.float32)
    objp[:, :2] = np.mgrid[0:9, 0:6].T.reshape(-1, 2)

    #adjusting object points
    square_size = .0254
    objp = objp*square_size;

    # Arrays to store object points and image points from all the images.
    objpoints = []  # 3d point in real world space
    imgpoints = []  # 2d points in image plane.

    # images = glob.glob('images_canon/*.jpg')
    # images = glob.glob('images_webcam_black_checkerboard/*.jpg')
    images = glob.glob('images/*.jpg')

    for fname in images:
        img = cv2.imread(fname)
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

        # Find the chess board corners
        ret, corners = cv2.findChessboardCorners(gray, (9, 6), None)

        # If found, add object points, image points (after refining them)
        if ret:
            objpoints.append(objp)

            corners2 = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
            imgpoints.append(corners2)

            # Draw and display the corners
            img = cv2.drawChessboardCorners(img, (9, 6), corners2, ret)

    ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)

    return [ret, mtx, dist, rvecs, tvecs]


def saveCoefficients(mtx, dist):
    # cv_file = cv2.FileStorage("images_canon/calibrationCoefficients.yaml", cv2.FILE_STORAGE_WRITE)
    # cv_file = cv2.FileStorage("images_webcam_black_checkerboard/test.yaml", cv2.FILE_STORAGE_WRITE)
    cv_file = cv2.FileStorage("images/test.yaml", cv2.FILE_STORAGE_WRITE)
    cv_file.write("camera_matrix", mtx)
    cv_file.write("dist_coeff", dist)
    # note you *release* you don't close() a FileStorage object
    cv_file.release()


def loadCoefficients():
    # FILE_STORAGE_READ
    # cv_file = cv2.FileStorage("images_canon/calibrationCoefficients.yaml", cv2.FILE_STORAGE_READ)
    # cv_file = cv2.FileStorage("images_webcam_black_checkerboard/test.yaml", cv2.FILE_STORAGE_READ)
    cv_file = cv2.FileStorage("images/test.yaml", cv2.FILE_STORAGE_READ)

    # note we also have to specify the type to retrieve other wise we only get a
    # FileNode object back instead of a matrix
    camera_matrix = cv_file.getNode("camera_matrix").mat()
    dist_matrix = cv_file.getNode("dist_coeff").mat()

    # Debug: print the values
    # print("camera_matrix : ", camera_matrix.tolist())
    # print("dist_matrix : ", dist_matrix.tolist())

    cv_file.release()
    return [camera_matrix, dist_matrix]


def inversePerspective(rvec, tvec):
    R, _ = cv2.Rodrigues(rvec)
    R = np.matrix(R).T
    invTvec = np.dot(-R, np.matrix(tvec))
    invRvec, _ = cv2.Rodrigues(R)
    return invRvec, invTvec


def relativePosition(rvec1, tvec1, rvec2, tvec2):
    rvec1, tvec1 = rvec1.reshape((3, 1)), tvec1.reshape(
        (3, 1))
    rvec2, tvec2 = rvec2.reshape((3, 1)), tvec2.reshape((3, 1))

    # Inverse the second marker, the right one in the image
    invRvec, invTvec = inversePerspective(rvec2, tvec2)

    orgRvec, orgTvec = inversePerspective(invRvec, invTvec)
    # print("rvec: ", rvec2, "tvec: ", tvec2, "\n and \n", orgRvec, orgTvec)

    info = cv2.composeRT(rvec1, tvec1, invRvec, invTvec)
    composedRvec, composedTvec = info[0], info[1]

    composedRvec = composedRvec.reshape((3, 1))
    composedTvec = composedTvec.reshape((3, 1))
    return composedRvec, composedTvec

#new code----------------------------------------------------------------------------------------------------------------------------------------------------------------

def calculate_coords(origin_ID, marker_list, matrix_coefficients, distortion_coefficients):
    pointCircle = (0, 0)
    coordinate_dict = {}
    composedRvec, composedTvec = None, None
    counter = 0
    while True:
        ret, frame = cap.read(0)
        # operations on the frame come here
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)  # Change grayscale
        aruco_dict = aruco.Dictionary_get(aruco.DICT_6X6_250)  # Use 5x5 dictionary to find markers
        parameters = aruco.DetectorParameters_create()  # Marker detection parameters

        # lists of ids and the corners beloning to each id
        corners, ids, rejected_img_points = aruco.detectMarkers(gray, aruco_dict,
                                                                parameters=parameters,
                                                                cameraMatrix=matrix_coefficients,
                                                                distCoeff=distortion_coefficients)

        if np.all(ids is not None):  # If there are markers found by detector
            coordinate_dict = {}
            isOriginMarkerCalibrated = False
            zipped = zip(ids, corners)
            ids, corners = zip(*(sorted(zipped)))
            axis = np.float32([[-0.01, -0.01, 0], [-0.01, 0.01, 0], [0.01, -0.01, 0], [0.01, 0.01, 0]]).reshape(-1, 3)
            for i in range(0, len(ids)):  # Iterate in markers
                # Estimate pose of each marker and return the values rvec and tvec---different from camera coefficients
                rvec, tvec, markerPoints = aruco.estimatePoseSingleMarkers(corners[i], aruco_marker_size, matrix_coefficients, distortion_coefficients)
                if ids[i] == origin_ID:
                    originRvec = rvec
                    originTvec = tvec
                    isOriginMarkerCalibrated = True
                    orginRvec, originTvec = originRvec.reshape((3, 1)), originTvec.reshape((3, 1))
                else:
                    for markerID in marker_list:
                        if ids[i] == markerID and isOriginMarkerCalibrated:
                            markerRvec = rvec
                            markerTvec = tvec
                            # markerCorners = corners[i]
                            markerRvec, markerTvec = markerRvec.reshape((3, 1)), markerTvec.reshape((3, 1))
                            composedRvec, composedTvec = relativePosition(originRvec, originTvec, markerRvec, markerTvec)
                            coordinate_dict[markerID] = composedTvec

                (rvec - tvec).any()  # get rid of that nasty numpy value array error
                aruco.drawDetectedMarkers(frame, corners)  # Draw A square around the markers

        # Display the resulting frame
        cv2.imshow('frame', frame)
        # Wait 3 milisecoonds for an interaction. Check the key and do the corresponding job.
        key = cv2.waitKey(3) & 0xFF
        if key == ord('q'):  # Quit
            break
        elif key == ord('c'):  # find coordinates
            # print(coordinate_dict)
            if len(coordinate_dict) > 0:
                for key in coordinate_dict.keys():
                    print("tvec for ", key," = ", coordinate_dict[key])


    # When everything done, release the capture
    cap.release()
    cv2.destroyAllWindows()


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Aruco Marker Tracking')
    parser.add_argument('--coefficients', metavar='bool', required=True,
                        help='File name for matrix coefficients and distortion coefficients')

    # Parse the arguments and take action for that.
    args = parser.parse_args()

    if args.coefficients == '1':
        mtx, dist = loadCoefficients()
        ret = True
    else:
        ret, mtx, dist, rvecs, tvecs = calibrate()
        saveCoefficients(mtx, dist)
    print("Calibration is completed. Starting tracking sequence.")
    if ret:
        # track(mtx, dist)
        calculate_coords(originMarker, markerList, mtx, dist)
# view rawpython_aruco_relative_tracker.py hosted with ❤ by GitHub
