# Based on https://www.geeksforgeeks.org/python-opencv-optical-flow-with-lucas-kanade-method/

import numpy as np 
import cv2 
import debugUtils
import math

# Helper functions #

# Function to return a pair of:
#   1. closest "K" points to the origin, i.e. [0, 0], in a list `points` of 
# lists where each list in `points` has 2 elements representing a 2D vector.
# For example,
# ```
# points = [[3, 3], [5, -1], [-2, 4]]
# K = 2
# print(closestKPointsToOrigin(points, K)[0])
# ```
# will print [[3, 3], [-2, 4]] since those are the 2 closest points to [0, 0], the origin.
#   2. indices of those items in the list.
def closestKPointsToOrigin(points, K):
    if len(points) == 0:
        return None
    if K == 1:
        best = points[0]
        bestDist = best[0]**2 + best[1]**2
        bestIndex = 0
        for i in range(0, len(points)):
            p = points[i]
            #print(p)
            #print(points)
            #print(p[0])
            #print(p[0]**2)
            #print(best)
            dist = p[0]**2 + p[1]**2
            if dist < bestDist:
                best = p
                bestDist = dist
                bestIndex = i
        return (best, bestIndex)
    else:
        # Algorithm from https://www.geeksforgeeks.org/find-k-closest-points-to-the-origin/ ,
        # with modifications :
        indicesSorted = np.argsort(points) # https://numpy.org/doc/stable/reference/generated/numpy.argsort.html , https://stackoverflow.com/questions/6422700/how-to-get-indices-of-a-sorted-array-in-python
        points.sort(key = lambda K: K[0]**2 + K[1]**2)
        return (points[:K], indicesSorted)

# Returns the angle in radians between two vectors of any dimension.
def angleBetween(vector_1, vector_2):
    # From https://www.kite.com/python/answers/how-to-get-the-angle-between-two-vectors-in-python :
    unit_vector_1 = vector_1 / np.linalg.norm(vector_1)
    unit_vector_2 = vector_2 / np.linalg.norm(vector_2)
    dot_product = np.dot(unit_vector_1, unit_vector_2)
    angle = np.arccos(dot_product)
    return angle

# "Re-maps a number from one range to another."
# (based on https://processing.org/reference/map_.html and
# https://stackoverflow.com/questions/17134839/how-does-the-map-function-in-processing-work )
# istart/stop: input starting/stopping values of the range of values it can be in.
# ostart/stop: the above range but for the expected/resulting converted value.
def map_(value, istart, istop, ostart, ostop):
    return ostart + (ostop - ostart) * ((value - istart) / (istop - istart))

# Optical flow processing class #

class OpticalFlow:
    def __init__(self, source=0, frame_width=500, frame_height=600, show_debug=False):
        #cap = cv2.VideoCapture('sample.mp4')
        self.cap = cv2.VideoCapture(source)
        # Set video capture properties
        #self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, frame_width)
        #self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, frame_height)

        self.source = source
        self.frame_width = frame_width
        self.frame_height = frame_height
        self.show_debug = show_debug

        # Lists holding good points to track
        self.good_new = None
        self.good_old = None
        self.averagePoint = None
        self.good_new_withNones = None # Holds None for points that were removed
        self.skip = []
        self.originalPoints = None # Holds None for points that were removed
        self.angle = None
        self.angleOffsetFromPreviousPoints = None # Offset to apply to `angle` when
        # we re-prepare due to losing feature points; holds the running sum of all angles
        # we had before.
        
        self.__initializeFlowParams()
        self.p0 = None

    # Call this before beginning to turn the car. It locks onto feature points 
    # in the camera image which will be tracked while turning. 
    # Returns False if an empty frame was found. If an empty frame is found, this
    # will be handled automatically in computeCentermostFlow().
    def prepare(self):
        if self.angle is not None:
            self.angleOffsetFromPreviousPoints += self.angle
            self.angle = None
        self.__setPointChanged(True)

        # Take first frame and find corners in it
        ret, old_frame = self.cap.read() 

        # Check if frame is not empty
        if not ret:
            print("prepare(): Empty frame")
            return False

        self.old_gray = cv2.cvtColor(old_frame, 
                                cv2.COLOR_BGR2GRAY)
        
        if self.show_debug:
            # Create a mask image for drawing purposes 
            self.mask = np.zeros_like(old_frame) 

        self.p0 = cv2.goodFeaturesToTrack(self.old_gray, mask = None, 
                                    **self.feature_params) 
        self.originalPoints = self.p0.copy()
        self.good_new_withNones = self.p0.copy()
    
    # Compute the last optical flow and return the flow vector at the point
    # that was centermost in the frame (i.e., is in-line with the distance sensor
    # on the RPi).
    # @param reprepare -- indicates whether to prepare again or return False when no tracking
    # points are found in the image anymore or if an empty frame 
    # is captured. Recommended to keep True.
    # @param show_debug -- When show_debug was set to True, this method
    # returns True to request exiting the program, or in all cases it may return one of:
    #   1. A pair consisting of:
    #     a. The closest point as a list of two elements representing a 2D vector.
    #     b. The flow vector detected at the closest point to the center.
    #   2. If reprepare is True, returning False indicates that an empty frame 
    #   was all that could be captured
    #   or that no tracking points are left. This means
    #   you need to try again until you get True. cap.isOpened() may be useful too.
    def computeCentermostFlow(self, reprepare=True):
        if self.p0 is None:
            if not reprepare:
                return False
            # Wait until we get a non-empty frame while preparing:
            while self.prepare() == False:
                self.p0 = None
        
        # Capture a frame
        while True:
            ret, frame = self.cap.read() 

            # Check if frame is not empty
            if not ret:
                print("computeCentermostFlow(): Empty frame")
                if not reprepare:
                    return False
            else:
                break

        frame_gray = cv2.cvtColor(frame, 
                                cv2.COLOR_BGR2GRAY) 
    
        # calculate optical flow 
        # https://docs.opencv.org/3.4/dc/d6b/group__video__track.html#ga5d10ebbd59fe09c5f650289ec0ece5af
        p1, st, err = cv2.calcOpticalFlowPyrLK(self.old_gray, 
                                            frame_gray, 
                                            self.p0, None, 
                                            **self.lk_params) 
        # `st` is the "status": array of 0 or 1 where a 0 means we lost that point from self.p0,
        # and 1 means we kept it. (source: https://docs.opencv.org/master/dc/d6b/group__video__track.html#ga473e4b886d0bcc6b65831eb88ed93323 )
        #if len(err) > 0:
        #    print("Error in flow: " + str(err)) # This prints often... strange..

        # Select good points 
        good_new = p1[st == 1] 
        # Go through all points in p0 and keep in good_new_withNones the ones that didn't
        # get set to 0, and update skip counts if so
        for i in range(0, len(st)):
            if st == 0: # Point lost
                # Find where in self.skip that we should place the skip
                outJ = None
                for j in range (0, len(self.skip)):
                    if self.skip[j] == 0:
                        outJ = j
                        break
                if outJ != None:
                    self.skip[outJ] .............
        good_old = self.p0[st == 1] 


        if self.show_debug:
            # draw the tracks 
            #print(good_new)
            for i, (new, old) in enumerate(zip(good_new,  
                                            good_old)): 
                a, b = new.ravel() 
                c, d = old.ravel() 
                self.mask = cv2.line(self.mask, (a, b), (c, d), 
                                self.color[i].tolist(), 2) 
                
                frame = cv2.circle(frame, (a, b), 5, 
                                self.color[i].tolist(), -1) 
                
            img = cv2.add(frame, self.mask) 
        
            cv2.imshow('frame', img) 
        
            k = cv2.waitKey(25) 
            if k == 27: 
                return True

        # Check if we lost any points
        try:
            # Remove points from original array where status was bad
            for i in range(0, len(self.originalPoints)):
                if st[i] == 0: # Point lost
                    self.originalPoints[i] = None
                    self.good_new_withNones[i] = None
            if len(self.good_old) < len(self.good_new):
                self.__setPointChanged(True)
                print("A point was lost")
            #else:
            #    self.__setPointChanged(False)
        except TypeError: # "TypeError: object of type 'NoneType' has no len()"
            self.__setPointChanged(True)
        
        # Check if we have any points left:
        if len(good_new) == 0:
            self.__setPointChanged(True)
            print("computeCentermostFlow(): Re-prepare needed")
            if not reprepare:
                return False
            while self.prepare() == False:
                self.p0 = None
            return self.computeCentermostFlow(reprepare)

        # Find centermost point:
        # Translate all points to be centered
        # in the frame, i.e. relative to width/2, height/2 as the origin.
        good_new_centered = map(lambda l: [l[0] - self.frame_width / 2, 
                                           l[1] - self.frame_height / 2], good_new)
        (centermost, centermostIndex) = closestKPointsToOrigin(list(good_new_centered), 1)

        # Updating Previous frame and points  
        self.old_gray = frame_gray.copy() 
        #print(self.p0)
        self.p0 = good_new.reshape(-1, 1, 2) # Adds a layer of lists.. why?
        #print("Reshaped: " + str(self.p0))

        self.good_new = good_new
        self.good_old = good_old
        return (centermost, np.subtract(centermost, good_old[centermostIndex]))

    # Given a point, this returns how close it is to the center, i.e. how "reliable" it is
    # to use in distance sensor calculations.
    def reliabilityOfPoint(self, p):
        return [p[0] - self.frame_width / 2, 
                p[1] - self.frame_height / 2]
    
    # Returns the change in camera angle in radians since the last call to
    # reset(). Calls to this function must be after having called
    # computeCentermostFlow() at least once.
    def computeRadiansOfCameraRotation(self, sensorDistance, flowVector):
        # Constants
        fovHorizDegrees = 62 # Field of view of the camera, in horizontal degrees
        fovHoriz = math.radians(fovHorizDegrees) # Converted to radians

        if False:
            # Flow/velocity method #

            # Make flowVector into a 3D vector but contained within the plane of the 
            # camera frame, and sensorDistance into a 3D vector going into the frame.
            sensorDistVec3D = [0, 0, sensorDistance]
            #debugUtils.enterREPL(globals(), locals())
            flowVector3D = [flowVector[0], flowVector[1], 0]

            # Get angle in radians between the sensor vector and the sum of the two 
            # vectors defined above:
            angleChange = angleBetween(sensorDistVec3D, np.add(sensorDistVec3D, flowVector3D))

            # Now subtract 90 degrees to make the angle be relevant to the car and not just
            # the angle between those vectors:
            angleChange -= math.pi / 2

            print("Change in angle: " + str(math.degrees(angleChange)) + " degrees")
            return angleChange
        elif False:
            # Point position method: average all movement of all points #

            newAveragePoint = 0
            for i in range(0, len(self.good_old)):
                # Check how much this point has moved
                pnew = self.good_new[i]
                raise Exception("This is broken and needs to use good_new_withNones since some points can be removed")
                pold = self.good_old[i]
                newAveragePoint = np.add(newAveragePoint, np.subtract(pnew, pold))
            newAveragePoint = np.divide(newAveragePoint, len(self.good_old))

            changeInAvg = np.subtract(newAveragePoint, 
                self.averagePoint if self.averagePoint is not None else newAveragePoint)
            # Get the angle using the field of view of the camera
            print("changeInAvg: " + str(changeInAvg))
            # Range of the first value is provided in the second argument here:
            angleChange = map_(changeInAvg[0], 
                                -self.frame_width / 2, self.frame_width / 2,
                                -fovHoriz / 2, fovHoriz / 2)
            # `angle` is now from range -fovHoriz / 2 to fovHoriz / 2.

            # Adjust self.averagePoint:
            self.averagePoint = newAveragePoint

            print("Change in angle: " + str(math.degrees(angleChange)) + " degrees")
            return angleChange
        else:
            # Point position method but keep the offset from the original points instead #

            averageOffsetFromOriginal = [0, 0]
            i = 0
            while i < len(self.originalPoints):
                # Check how much this point has moved
                pnew = self.good_new[i]
                pold = self.originalPoints[i][0] # [0] to remove the nested list.. why?
                if pold is None:
                    i += 1
                    continue # This point was lost in the tracking
                averageOffsetFromOriginal = np.add(averageOffsetFromOriginal, 
                                                    np.subtract(pnew, pold))
                i += 1
            averageOffsetFromOriginal = np.divide(averageOffsetFromOriginal, 
                                                    i)
            #debugUtils.enterREPL(globals(), locals())
            
            # Get the angle using the field of view of the camera
            # Range of the first value is provided in the second argument here:
            self.angle = map_(averageOffsetFromOriginal[0], 
                                -self.frame_width / 2, self.frame_width / 2,
                                -fovHoriz / 2, fovHoriz / 2)
            # `angle` is now from range -fovHoriz / 2 to fovHoriz / 2.
            #print("Angle (radians): " + str(self.angle))

            print("Angle for this prepare(): " + str(math.degrees(self.angle)) + " degrees")
            resAngle = (self.angleOffsetFromPreviousPoints if self.angleOffsetFromPreviousPoints is not None else 0) + self.angle
            print("Angle for this reset(): " + str(math.degrees(resAngle)) + " degrees")
            return resAngle
        

    # Indicates the current frame sequence is no longer being considered.
    def reset(self):
        self.angle = None
        self.angleOffsetFromPreviousPoints = None
        self.__setPointChanged(True)
        self.originalPoints = None
        self.skip = []
    
    # Private methods #

    # "Destructor": called upon garbage collection of this object.
    def __del__(self):
        if self.show_debug:
            cv2.destroyAllWindows() 
        self.cap.release()

    def __initializeFlowParams(self):
        # params for corner detection 
        self.feature_params = dict( maxCorners = 100, 
                            qualityLevel = 0.3, 
                            minDistance = 7, 
                            blockSize = 7 ) 
        
        # Parameters for lucas kanade optical flow 
        self.lk_params = dict( winSize = (15, 15), 
                        maxLevel = 2, 
                        criteria = (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 
                                    10, 0.03)) 
        
        if self.show_debug:
            # Create some random colors 
            self.color = np.random.randint(0, 255, (100, 3))
    
    def __setPointChanged(self, pointChanged):
        if pointChanged:
            self.averagePoint = None