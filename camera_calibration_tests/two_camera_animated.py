
import numpy as np
import cv2
import cv2.aruco as aruco
import glob
import argparse
import math
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import numpy as np
from threading import Thread
import asyncio
# Create figure for plotting
plt.style.use('seaborn')
#print(plt.style.available)
fig = plt.figure(figsize=(6, 6))
ax = fig.add_subplot(1, 1, 1)
xs = []
ys = []
xs2 = []
ys2 = []
ax.set_xlim([-20, 20])
ax.set_ylim([-20, 20])

r = 15
b = 0.001
tau = 20
# This function is called periodically from FuncAnimation
def animate(i, xs, ys, xs2, ys2):

   # Add x and y to lists
   x = r * np.cos(i/tau) * (1 - np.exp(-b*i))
   y = r * np.sin(i/tau) * (1 - np.exp(-b*i))

   xs.append(x)
   ys.append(y)

   x2 = r * np.cos(i/tau)
   y2 = r * np.sin(i/tau)

   xs2.append(x2)
   ys2.append(y2)

   # Limit x and y lists to n_items
   n_items = 20
   xs = xs[-n_items:]
   ys = ys[-n_items:]

   xs2 = xs2[-n_items:]
   ys2 = ys2[-n_items:]

   # Draw x and y lists
   ax.clear()
   n_div = 20
   for i in range(n_div):
       x_range, y_range = xs[-(n_items - (n_items//n_div)*i):], ys[-(n_items - (n_items//n_div)*i):]
       ax.plot(x_range, y_range, alpha=0.1*i, c='blue' )

   for i in range(n_div):
       x_range, y_range = xs2[-(n_items - (n_items//n_div)*i):], ys2[-(n_items - (n_items//n_div)*i):]
       ax.plot(x_range, y_range, alpha=0.1*i, c='red' )

   ax.scatter(xs[-1], ys[-1], c='blue')
   ax.scatter(xs2[-1], ys2[-1], c='red')

   ax.plot([xs[-1], xs2[-1]], [ys[-1], ys2[-1]], c='black', alpha=0.3)

   midpoint = [(xs[-1] + xs2[-1])/2 , (ys[-1] + ys2[-1])/2 ]
   distance = np.sqrt((xs[-1] - xs2[-1])**2 + (ys[-1] - ys2[-1])**2)
   plt.annotate('Distance: %2f' % distance, midpoint)

   # Format plot
   plt.title('Position')
   plt.ylabel('Y')
   plt.xlabel('X')
   ax.set_xlim([-20, 20])
   ax.set_ylim([-20, 20])

# Set up plot to call animate() function periodically





aruco_marker_size = 0.1
cam1 = cv2.VideoCapture(1)
cam2 = cv2.VideoCapture(2)
saved_vec = None

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

    images = glob.glob('images_sony/*.jpg')
    # images = glob.glob('images/*.png')

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
    cv_file = cv2.FileStorage("images_canon/calibrationCoefficients.yaml", cv2.FILE_STORAGE_WRITE)
    # cv_file = cv2.FileStorage("images/test.yaml", cv2.FILE_STORAGE_WRITE)
    cv_file.write("camera_matrix", mtx)
    cv_file.write("dist_coeff", dist)
    # note you *release* you don't close() a FileStorage object
    cv_file.release()


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


def draw(img, corners, imgpts):
    imgpts = np.int32(imgpts).reshape(-1,2)
    # draw ground floor in green
    # img = cv2.drawContours(img, [imgpts[:4]],-1,(0,255,0),-3)
    # draw pillars in blue color
    for i,j in zip(range(4),range(4)):
        img = cv2.line(img, tuple(imgpts[i]), tuple(imgpts[j]),(255),3)
    # draw top layer in red color
    return img

def find_points(camera, matrix_coefficients, distortion_coefficients, markerTvecList, markerRvecList, composedRvec, composedTvec):
    ret1, frame = camera.read() #originally 1, but the tutorial left it blank?
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)  # Change grayscale
    aruco_dict = aruco.Dictionary_get(aruco.DICT_6X6_250)  # Use 5x5 dictionary to find markers
    parameters = aruco.DetectorParameters_create()  # Marker detection parameters

    # lists of ids and the corners beloning to each id
    corners, ids, rejected_img_points = aruco.detectMarkers(gray, aruco_dict,parameters=parameters,cameraMatrix=matrix_coefficients, distCoeff=distortion_coefficients)

    if np.all(ids is not None):  # If there are markers found by detector
        del markerTvecList[:]
        del markerRvecList[:]
        zipped = zip(ids, corners)
        ids, corners = zip(*(sorted(zipped)))
        axis = np.float32([[-0.01, -0.01, 0], [-0.01, 0.01, 0], [0.01, -0.01, 0], [0.01, 0.01, 0]]).reshape(-1, 3)
        for i in range(0, len(ids)):  # Iterate in markers
            # Estimate pose of each marker and return the values rvec and tvec---different from camera coefficients
            rvec, tvec, markerPoints = aruco.estimatePoseSingleMarkers(corners[i], aruco_marker_size, matrix_coefficients,
                                                                       distortion_coefficients)

            if ids[i] == firstMarkerID:
                firstRvec = rvec
                firstTvec = tvec
                isFirstMarkerCalibrated = True
                firstMarkerCorners = corners[i]
            elif ids[i] == secondMarkerID:
                secondRvec = rvec
                secondTvec = tvec
                isSecondMarkerCalibrated = True
                secondMarkerCorners = corners[i]
            elif ids[i] == thirdMarkerID:
                thirdRvec = rvec
                thirdTvec = tvec
                isThirdMarkerCalibrated = True
                thirdMarkerCorners = corners[i]

            (rvec - tvec).any()  # get rid of that nasty numpy value array error
            markerRvecList.append(rvec)
            markerTvecList.append(tvec)

            aruco.drawDetectedMarkers(frame, corners)  # Draw A square around the markers

        if len(ids) > 1 and composedRvec is not None and composedTvec is not None:
            info = cv2.composeRT(composedRvec, composedTvec, secondRvec.T, secondTvec.T)
            TcomposedRvec, TcomposedTvec = info[0], info[1]

            imgpts, jac = cv2.projectPoints(axis, TcomposedRvec, TcomposedTvec, matrix_coefficients,
                                            distortion_coefficients)

            aruco.drawAxis(frame, matrix_coefficients, distortion_coefficients, TcomposedRvec, TcomposedTvec,
                           0.01)  # Draw Axis
            relativePoint = (int(imgpts[0][0][0]), int(imgpts[0][0][1]))
            cv2.circle(frame, relativePoint, 2, (255, 255, 0))

            #third vector calculation---------------------------------------------------------------------------
            info = cv2.composeRT(composedRvec2, composedTvec2, thirdRvec.T, thirdTvec.T)
            TcomposedRvec, TcomposedTvec = info[0], info[1]

            # objectPositions = np.array([(0, 0, 0)], dtype=np.float)  # 3D point for projection
            imgpts, jac = cv2.projectPoints(axis, TcomposedRvec, TcomposedTvec, matrix_coefficients,
                                            distortion_coefficients)

            aruco.drawAxis(frame, matrix_coefficients, distortion_coefficients, TcomposedRvec, TcomposedTvec,
                           0.01)  # Draw Axis
            relativePoint = (int(imgpts[0][0][0]), int(imgpts[0][0][1]))
            cv2.circle(frame, relativePoint, 2, (255, 255, 0))
        if len(ids) > 1:  # If there are two markers, reverse the second and get the difference
            firstRvec, firstTvec = firstRvec.reshape((3, 1)), firstTvec.reshape((3, 1))
            secondRvec, secondTvec = secondRvec.reshape((3, 1)), secondTvec.reshape((3, 1))
            thirdRvec, thirdTvec = thirdRvec.reshape((3, 1)), thirdTvec.reshape((3, 1))

            composedRvec, composedTvec = relativePosition(firstRvec, firstTvec, secondRvec, secondTvec)
            composedRvec2, composedTvec2 = relativePosition(firstRvec, firstTvec, thirdRvec, thirdTvec)

            print("ComposedTvec of vec1 - "+str(camera), composedTvec)
            # print("ComposedTvec of vec2 -"+str(camera), composedTvec2)

    cv2.imshow('frame_'+str(camera), frame)
    return composedTvec

def track(matrix_coefficients, distortion_coefficients, matrix_coefficients2, distortion_coefficients2):
    markerTvecList = []
    markerRvecList = []
    composedRvec, composedTvec = None, None

    markerTvecList2 = []
    markerRvecList2 = []
    composedRvec2, composedTvec2 = None, None


    while True:
        find_points(cam1, matrix_coefficients, distortion_coefficients, markerTvecList, markerRvecList, composedRvec, composedTvec)
        find_points(cam2, matrix_coefficients2, distortion_coefficients2, markerTvecList2, markerRvecList2, composedRvec2, composedTvec2)

        # Wait 3 miliseconds for an interaction. Check the key and do the corresponding job.
        key = cv2.waitKey(300) & 0xFF
        if key == ord('q'):  # Quit
            break

    # When everything done, release the captures
    cam1.release()
    cam2.release()
    cv2.destroyAllWindows()


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Aruco Marker Tracking')
    parser.add_argument('--coefficients', metavar='bool', required=True,
                        help='File name for matrix coefficients and distortion coefficients')
    parser.add_argument('--firstMarker', metavar='int', required=True,
                        help='Marker ID for the first marker')
    parser.add_argument('--secondMarker', metavar='int', required=True,
                        help='Marker ID for the second marker')
    parser.add_argument('--thirdMarker', metavar='int', required=True,
                        help='Marker ID for the second marker')


    # Parse the arguments and take action for that.
    args = parser.parse_args()
    firstMarkerID = int(args.firstMarker)
    secondMarkerID = int(args.secondMarker)
    thirdMarkerID = int(args.thirdMarker)

    #will fix this later
    if args.coefficients == '1':
        mtx, dist = loadCoefficients("images_canon/calibrationCoefficients.yaml")
        mtx2, dist2 = loadCoefficients("images_sony/calibrationCoefficients.yaml")
        # ret, mtx2, dist2, rvecs, tvecs = calibrate()
        ret = True
    # else:
    #     ret, mtx, dist, rvecs, tvecs = calibrate()
    #     saveCoefficients(mtx, dist)
    print("Calibration is completed. Starting tracking sequence.")

    if ret:
        asyncio.create_task(track(mtx, dist, mtx2, dist2))

    ani = animation.FuncAnimation(fig, animate, fargs=(xs, ys, xs2, ys2), interval=10)
    plt.show()
# view rawpython_aruco_relative_tracker.py hosted with ??? by GitHub
