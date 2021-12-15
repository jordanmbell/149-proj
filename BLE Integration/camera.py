import numpy as np
import cv2
import cv2.aruco as aruco
import glob
import argparse
import math
import time

cam1_id = 1
cam2_id = None
cam_wait_ms = 1
num_markers = 9
originID = 4
referenceIDs = [5, 6, 7]
markerIDs = range(num_markers)


class camera:
    def __init__(self, mc, dc, originID, markerIDs, camID, refIDs):
        self.matrix_coefficients = mc
        self.distortion_coefficients = dc
        self.originID = originID
        self.markerIDs = markerIDs
        self.referenceIDs = refIDs
        self.calib_sq_matrix = np.array([[1, 0, 0], [0, 1, 0], [0, 0, 1]])
        self.firstRvec = None
        self.firstTvec = None
        self.origin_yaw = None

        # aruco_marker_size = 0.1
        self.cam = cv2.VideoCapture(camID)
        self.camID = camID
        # self.cam2 = cv2.VideoCapture(2)

        # is this (below) needed?
        criteria = (cv2.TERM_CRITERIA_EPS +
                    cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

    def inversePerspective(self, rvec, tvec):
        R, _ = cv2.Rodrigues(rvec)
        R = np.matrix(R).T
        invTvec = np.dot(-R, np.matrix(tvec))
        invRvec, _ = cv2.Rodrigues(R)
        return invRvec, invTvec

    def relativePosition(self, rvec1, tvec1, rvec2, tvec2):
        # tvec1[2][0] = 0
        # tvec2[2][0] = 0
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
        imgpts = np.int32(imgpts).reshape(-1, 2)
        for i, j in zip(range(4), range(4)):
            img = cv2.line(img, tuple(imgpts[i]), tuple(imgpts[j]), (255), 3)
        return img

    def calibrate_square(self):
        square_dict = self.find_points(self.referenceIDs, True)
        while square_dict is None or not (all(id in square_dict.keys() for id in self.referenceIDs)):
            # self.show_image()
            print("\tCam ", self.camID,
                  " trying to find ref ids: ", self.referenceIDs)
            time.sleep(2)
            square_dict = self.find_points(self.referenceIDs, True)
        # print("id0", square_dict[self.referenceIDs[0]])
        # print(square_dict[self.referenceIDs[1]])
        # print(square_dict[self.referenceIDs[2]])
        self.calib_sq_matrix = np.array([
            [square_dict[self.referenceIDs[0]][0], square_dict[self.referenceIDs[1]][0], square_dict[self.referenceIDs[2]][0]],
            [square_dict[self.referenceIDs[0]][1], square_dict[self.referenceIDs[1]][1], square_dict[self.referenceIDs[2]][1]],
            [square_dict[self.referenceIDs[0]][2], square_dict[self.referenceIDs[1]][2], square_dict[self.referenceIDs[2]][2]]
            ]).T
        print("square matrix", self.calib_sq_matrix)
        target_matrix = np.array([[0.22, 0, 0.22], [0, 0.22, 0.22], [0.22, 0.22, 0.22]])
        print("shape of target matrix", target_matrix.shape)
        self.calib_sq_matrix = np.linalg.solve(
            self.calib_sq_matrix, target_matrix)
        # print(self.calib_sq_matrix @ square_dict[self.referenceIDs[1]][:2])
        print("square matrix after calculations", self.calib_sq_matrix)
        # return self.calib_sq_matrix

    def find_points(self, point_markers, isCalibrating):
        tvec_dict = {}
        composedTvec = None
        # firstRvec = None
        # firstTvec = None
        secondRvec = None
        secondTvec = None
        # originally 1, but the tutorial left it blank?
        ret1, self.frame = self.cam.read()
        gray = cv2.cvtColor(self.frame, cv2.COLOR_BGR2GRAY)  # Change grayscale
        # Use 5x5 dictionary to find markers
        aruco_dict = aruco.Dictionary_get(aruco.DICT_4X4_250)
        parameters = aruco.DetectorParameters_create()  # Marker detection parameters

        # lists of ids and the corners beloning to each id
        corners, ids, rejected_img_points = aruco.detectMarkers(
            gray, aruco_dict, parameters=parameters, cameraMatrix=self.matrix_coefficients, distCoeff=self.distortion_coefficients)

        # If there are markers found by detector
        if np.all(ids is not None) and np.all(corners is not None):
            zipped = zip(ids, corners)
            ids, corners = zip(*(sorted(zipped)))
            axis = np.float32([[-0.01, -0.01, 0], [-0.01, 0.01, 0],
                              [0.01, -0.01, 0], [0.01, 0.01, 0]]).reshape(-1, 3)
            for i in range(0, len(ids)):  # Iterate in markers
                # Estimate pose of each marker and return the values rvec and tvec---different from camera coefficients
                # 0.1 below is aruco_marker_size
                aruco_marker_size = 0.2
                rvec, tvec, markerPoints = aruco.estimatePoseSingleMarkers(
                    corners[i], aruco_marker_size, self.matrix_coefficients, self.distortion_coefficients)
                if ids[i] == self.originID:
                    self.firstRvec = rvec
                    self.firstTvec = tvec
                    isFirstMarkerCalibrated = True
                    firstMarkerCorners = corners[i]

                    z1 = math.atan2(rvec[0][0][0], rvec[0][0][1])
                    if isCalibrating:
                        self.origin_yaw = z1

                for mark in point_markers:
                    if ids[i] == mark:
                        secondRvec = rvec
                        secondTvec = tvec
                        isSecondMarkerCalibrated = True
                        secondMarkerCorners = corners[i]

                        # If there are two markers, reverse the second and get the difference
                        if self.originID in ids and self.firstRvec is not None and self.firstTvec is not None:

                            aruco.drawAxis(self.frame, self.matrix_coefficients,
                                           self.distortion_coefficients, secondRvec, secondTvec, 0.1)

                            self.firstRvec, self.firstTvec = self.firstRvec.reshape(
                                (3, 1)), self.firstTvec.reshape((3, 1))
                            secondRvec, secondTvec = secondRvec.reshape(
                                (3, 1)), secondTvec.reshape((3, 1))

                            composedRvec, composedTvec = self.relativePosition(
                                self.firstRvec, self.firstTvec, secondRvec, secondTvec)
                            # calculating yaw
                            # sin_x = math.sqrt(rvec[2,0] * rvec[2,0] +  rvec[2,1] * rvec[2,1])
                            pos = self.calib_sq_matrix @ composedTvec
                            # print("square matrix when making composedTvec", self.calib_sq_matrix)
                            tvec_dict[mark] = [pos[0][0], pos[1][0], pos[2][0]]

                            if not isCalibrating:
                                # print(self.origin_yaw)
                                z1 = math.atan2(rvec[0][0][0], rvec[0][0][1])
                                z1 = z1 - self.origin_yaw
                                z1 = (z1*(360)/math.pi) % 360
                                pos = self.calib_sq_matrix.T @ composedTvec
                                tvec_dict[mark] = [pos[0][0], pos[1][0], z1]
                            # print(tvec_dict[mark])


                (rvec - tvec).any()  # get rid of that nasty numpy value array error

                # Draw A square around the markers
                aruco.drawDetectedMarkers(self.frame, corners)
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


def track(shared_dict, shared_is_calibrated):
    composedRvec, composedTvec = None, None
    composedRvec2, composedTvec2 = None, None

    mtx, dist = loadCoefficients("../camera_calibration_tests/images_canon_floor/calibrationCoefficients2.yaml")
    #mtx, dist = loadCoefficients(
    #    "../camera_calibration_tests/images_canon/calibrationCoefficients.yaml")
    # mtx, dist = loadCoefficients("images_webcam_black_checkerboard/calibrationCoefficients.yaml")
    # mtx, dist = loadCoefficients("../camera_calibration_tests/images_sony/calibrationCoefficients.yaml")

    cam1 = camera(mtx, dist, originID, markerIDs, cam1_id, referenceIDs)
    cam1.calibrate_square()
    if cam2_id is not None:
        # mtx2, dist2 = loadCoefficients("images_webcam_black_checkerboard/calibrationCoefficients.yaml")
        mtx2, dist2 = loadCoefficients(
            "../camera_calibration_tests/images_sony/calibrationCoefficients.yaml")
        cam2 = camera(mtx, dist, originID, markerIDs, cam2_id, referenceIDs)
        cam2.calibrate_square()
    # Update server process that we found the robots
    shared_is_calibrated.value = 1
    rolling_average_dict = dict()
    for i in range(num_markers):
        rolling_average_dict[i] = dict()
        rolling_average_dict[i]['x'] = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
        rolling_average_dict[i]['avgx'] = 0
        rolling_average_dict[i]['y'] = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
        rolling_average_dict[i]['avgy'] = 0
        rolling_average_dict[i]['angsin'] = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
        rolling_average_dict[i]['angcos'] = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
        rolling_average_dict[i]['angsinsum'] = 0
        rolling_average_dict[i]['angcossum'] = 0
        rolling_average_dict[i]['idx'] = 0
    while True:
        vec1_cam1 = cam1.find_points(markerIDs, False)
        if cam2_id is not None:
            vec1_cam2 = cam2.find_points(markerIDs, False)
            cam2.show_image()
        cam1.show_image()

        for id in range(num_markers):
            x = 0
            y = 0
            rot = 0
            in_cam1 = vec1_cam1 is not None and id in vec1_cam1.keys()
            in_cam2 = cam2_id is not None and vec1_cam2 is not None and id in vec1_cam2.keys()
            if in_cam1 and in_cam2:
                x = (vec1_cam1[id][0] + vec1_cam2[id][0])/2
                y = (vec1_cam1[id][1] + vec1_cam2[id][1])/2
                # rot = vec1_cam1[id][2]

                # averaging the rotations
                sine_sum = np.sin(np.radians(
                    vec1_cam1[id][2])) + np.sin(np.radians(vec1_cam2[id][2]))
                cosine_sum = np.cos(np.radians(
                    vec1_cam1[id][2])) + np.cos(np.radians(vec1_cam2[id][2]))
                rot = np.degrees(np.arctan(sine_sum/cosine_sum))

            elif in_cam1:
                x = vec1_cam1[id][0]
                y = vec1_cam1[id][1]
                rot = vec1_cam1[id][2]
            elif in_cam2:
                x = vec1_cam2[id][0]
                y = vec1_cam2[id][1]
                rot = vec1_cam2[id][2]

            if x == 0 and y == 0 and rot == 0:
                rolling_average_dict[id]['idx'] = 0
                continue

            idx = rolling_average_dict[id]['idx']
            prev_x = rolling_average_dict[id]['x'][idx]
            rolling_average_dict[id]['x'][idx] = x

            rolling_average_dict[id]['avgx'] += (x - prev_x) / 10

            prev_y = rolling_average_dict[id]['y'][idx]
            rolling_average_dict[id]['y'][idx] = y

            rolling_average_dict[id]['avgy'] += (y - prev_y) / 10

            prev_ang_sin = rolling_average_dict[id]['angsin'][idx]
            ang_sin = np.sin(np.radians(rot))
            rolling_average_dict[id]['angsin'][idx] = ang_sin

            ang_cos = np.cos(np.radians(rot))
            prev_ang_cos = rolling_average_dict[id]['angcos'][idx]
            rolling_average_dict[id]['angcos'][idx] = ang_cos

            rolling_average_dict[id]['angsinsum'] += (ang_sin - prev_ang_sin)
            rolling_average_dict[id]['angcossum'] += (ang_cos - prev_ang_cos)

            if rolling_average_dict[id]['idx'] == 9:
                avg_ang = np.arctan2(rolling_average_dict[id]['angsinsum'], rolling_average_dict[id]['angcossum'])

                shared_dict[id] = [rolling_average_dict[id]['avgx'],
                                    rolling_average_dict[id]['avgy'],
                                    np.degrees(avg_ang)]

            rolling_average_dict[id]['idx'] = (idx + 1) % 10

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
