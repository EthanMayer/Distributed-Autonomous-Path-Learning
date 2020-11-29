# Based on https://www.geeksforgeeks.org/python-opencv-optical-flow-with-lucas-kanade-method/

import numpy as np 
import cv2 
import debugUtils

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

# Optical flow processing class #

class OpticalFlow:
    def __init__(self, source=0, frame_width=500, frame_height=600, show_debug=False):
        #cap = cv2.VideoCapture('sample.mp4')
        self.cap = cv2.VideoCapture(source)
        # Set video capture properties
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, frame_width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, frame_height)

        self.source = source
        self.frame_width = frame_width
        self.frame_height = frame_height
        self.show_debug = show_debug

        self.__initializeFlowParams()
        self.p0 = None

    # Call this before beginning to turn the car. It locks onto feature points 
    # in the camera image which will be tracked while turning. 
    def prepare(self):
        # Take first frame and find corners in it 
        ret, old_frame = self.cap.read() 
        self.old_gray = cv2.cvtColor(old_frame, 
                                cv2.COLOR_BGR2GRAY)
        
        if self.show_debug:
            # Create a mask image for drawing purposes 
            self.mask = np.zeros_like(old_frame) 

        self.p0 = cv2.goodFeaturesToTrack(self.old_gray, mask = None, 
                                    **self.feature_params) 
    
    # Compute the last optical flow and return the flow vector at the point
    # that was centermost in the frame (i.e., is in-line with the distance sensor
    # on the RPi).
    # When show_debug was set to True, this method
    # returns False to request exiting the program, otherwise it returns a pair consisting of:
    #   1. The closest point as a list of two elements representing a 2D vector.
    #   2. The flow vector detected at the closest point to the center.
    def computeCentermostFlow(self):
        if self.p0 is None:
            self.prepare()
        
        ret, frame = self.cap.read() 
        frame_gray = cv2.cvtColor(frame, 
                                cv2.COLOR_BGR2GRAY) 
    
        # calculate optical flow 
        # https://docs.opencv.org/3.4/dc/d6b/group__video__track.html#ga5d10ebbd59fe09c5f650289ec0ece5af
        p1, st, err = cv2.calcOpticalFlowPyrLK(self.old_gray, 
                                            frame_gray, 
                                            self.p0, None, 
                                            **self.lk_params) 
    
        # Select good points 
        good_new = p1[st == 1] 
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
                return False

        # Find centermost point:
        # Translate all points to be centered
        # in the frame, i.e. relative to width/2, height/2 as the origin.
        good_new_centered = map(lambda l: [l[0] - self.frame_width / 2, 
                                           l[1] - self.frame_height / 2], good_new)
        (centermost, centermostIndex) = closestKPointsToOrigin(list(good_new_centered), 1)

        # Updating Previous frame and points  
        self.old_gray = frame_gray.copy() 
        self.p0 = good_new.reshape(-1, 1, 2) 
        return (centermost, np.subtract(centermost, good_old[centermostIndex]))

    # Given a point, this returns how close it is to the center, i.e. how "reliable" it is
    # to use in distance sensor calculations.
    def reliabilityOfPoint(self, p):
        return [p[0] - self.frame_width / 2, 
                p[1] - self.frame_height / 2]
    
    def computeRadiansOfCameraRotation(self, sensorDistance, flowVector):
        # Make flowVector into a 3D vector but contained within the plane of the 
        # camera frame, and sensorDistance into a 3D vector going into the frame.
        sensorDistVec3D = [0, 0, sensorDistance]
        debugUtils.enterREPL()
        flowVector3D = [flowVector[0], flowVector[1], 0]

        # Get angle in radians between the sensor vector and the sum of the two 
        # vectors defined above:
        angle = angleBetween(sensorDistVec3D, sensorDistVec3D + flowVector3D)

        print(angle)

        return angle

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