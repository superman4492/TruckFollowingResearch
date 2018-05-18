import numpy as np
import cv2
import rospy
from geometry_msgs.msg import Point32
import sys
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

from NeededFunctions import OrganizeLeft, OrganizeRight, OrganizeTop, LineFromPoints,Intersection,InArea,FindIntersectingLines,DrawLines,TemplateMatch,ResetToBeginning

#source of the camera to get the right image size!
#2 sources so far-- "Portable" - The camera with the long blue cord that we can move everywhere (usb)
# second source - "Car" - the camera mounted in the car already (ethernet)
TargetDist=30
CameraSource='Portable'


###############################################   #import video   ########################################

# make sure to set the width and height of the video first!!!!

class image_converter:
    def __init__(self):

        if CameraSource=='Portable':
            self.HeightOfMainVideo=964
            self.WidthOfMainVideo=1288
        elif CameraSource=='Car':
            self.HeightOfMainVideo = 1200
            self.WidthOfMainVideo = 1600


        self.image_pub = rospy.Publisher("image_topic_2",Image,queue_size=100)
        self.image_pub_full=rospy.Publisher("fullWithTrack",Image,queue_size=100)
        self.pub=rospy.Publisher('chatter', Point32, queue_size=10)

        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/camera/image_color",Image,self.callback)



        TrailerHeight = 112.0  # inches
        TrailerWidth = 93.0  # inches
        self.HeightWidthRatio = (TrailerHeight / TrailerWidth)


        self.WindowScale = 0.5
        self.MinLineLength = int(0.05 * (self.WidthOfMainVideo) * self.WindowScale)
        self.MaxLineGap = int(0.015 * self.WidthOfMainVideo * self.WindowScale)
        self.XrangeLines = int(self.WidthOfMainVideo / 25 * self.WindowScale)

        self.Old = 0
        self.New = 0
        self.BoxFound = 0
        self.UsingTemplate = False
        CurrentTemplate = 0
        self.TrailerWidth = 2.37  # meters
        self.FocalLength = 2850  # pixels
        self.InitialMissCounter = 0

        # Region of Interest
        self.LeftBuffer = self.WidthOfMainVideo / 4
        self.RightBuffer = self.WidthOfMainVideo / 4  # add this to center position
        self.TopBottomBuffer = self.HeightOfMainVideo / 2
        self.BottomBuffer = int(self.HeightOfMainVideo * 0.5)  # add this to the center position

        self.CenterOfTemplateX = self.WidthOfMainVideo / 2
        self.CenterOfTemplateY = self.HeightOfMainVideo / 2
        self.InitialCenterOfTemplateX = self.WidthOfMainVideo / 2
        self.InitialCenterOfTemplateY = self.HeightOfMainVideo / 2

        self.initializeBox = True
        self.LowerThreshold = 50
        self.UpperThreshold = 255
        self.counter=0
        Skipframe = 0
        houghDold = None



    def callback(self,data):

        try:
            BigCap = self.bridge.imgmsg_to_cv2(data, "bgr8")
	    #self.WidthOfMainVideo,self.HeightOfMainVideo=BigCap.shape[:2]
	    #print "shape:", BigCap.shape[:2]


            if BigCap is not 0:

                BigVid = BigCap

                smallfull = cv2.resize(BigVid, (0, 0), fx=self.WindowScale, fy=self.WindowScale)

                # Define ROI for current frame
                ROIxStart = self.CenterOfTemplateX - self.LeftBuffer
                ROIxEnd = self.CenterOfTemplateX + self.RightBuffer


                ROIyStart = self.CenterOfTemplateY - self.TopBottomBuffer
                ROIyEnd = self.CenterOfTemplateY + self.BottomBuffer
                if (self.CenterOfTemplateX - self.LeftBuffer) < 0: ROIxStart = 0
                if (self.CenterOfTemplateX + self.RightBuffer) > self.WidthOfMainVideo: ROIxEnd = self.WidthOfMainVideo
                if (self.CenterOfTemplateY - self.TopBottomBuffer) < 0: ROIyStart = 0
                if (self.CenterOfTemplateY + self.BottomBuffer) > self.HeightOfMainVideo: ROIyEnd = self.HeightOfMainVideo
                SmallVidROI = BigVid[ROIyStart:ROIyEnd, ROIxStart:ROIxEnd]
		#cv2.imshow("roi",SmallVidROI)
                ROIxyStartEnd = [ROIxStart, ROIyStart, ROIxEnd, ROIyEnd]

                HoughD = cv2.resize(SmallVidROI, (0, 0), fx=self.WindowScale, fy=self.WindowScale)
                HoughD2 = cv2.resize(SmallVidROI, (0, 0), fx=self.WindowScale, fy=self.WindowScale)
                HoughDFinal = cv2.resize(SmallVidROI, (0, 0), fx=self.WindowScale, fy=self.WindowScale)
                HoughSelected = cv2.resize(SmallVidROI, (0, 0), fx=self.WindowScale, fy=self.WindowScale)
                height, width = HoughD.shape[:2]
                SmallGrey = cv2.cvtColor(HoughD, cv2.COLOR_BGR2GRAY)
                ROIResized = cv2.resize(SmallVidROI, (0, 0), fx=self.WindowScale, fy=self.WindowScale)
                ROIForFrame = cv2.resize(SmallVidROI, (0, 0), fx=self.WindowScale, fy=self.WindowScale)
                # cv2.imshow('ROIFrame',ROIForFrame)

                edges = cv2.Canny(SmallGrey, self.LowerThreshold, self.UpperThreshold)  # canny edge detection
                # cv2.imshow('canny',edges)
                dilate = cv2.dilate(edges, kernel=(1, 10), iterations=1)  # dilate the edges with a horizontal line
                dilate = cv2.dilate(dilate, kernel=(10, 1), iterations=1)  # dilate edges with a vertical line


                houghD = cv2.HoughLinesP(dilate, 1, np.pi / 180, 50, minLineLength=self.MinLineLength, maxLineGap=self.MaxLineGap)
                # for all horizontal and vertical lines, draw them

                LeftPoints = 0
                RightPoints = 0
                TopPoints = 0
                TrackMissCounter = 0
                DetectMissCounter = 0
                TemplateWidth = 0
                LateralError = 0
                LongDist = TargetDist

                if houghD is not None:  # filtering for straight lines
                    for i in range(len(houghD)):  # each i is one line in this frame
                        for x1, y1, x2, y2 in houghD[i]:  # for the points corresponding to the i lines in this frame
                            angle = np.arctan2(y2 - y1, x2 - x1) * 180. / np.pi
                            if abs(angle) > 85 or abs(angle) < 5:

                                #####################################_______Vertical__________###############

                                # if vertical and both x lie on one side of the screen, and are within 1/5 of the edge
                                if abs(x1 - x2) < self.XrangeLines and (
                                        (x1 > width / 2 and x2 > width / 2) or (x1 < width / 2 and x2 < width / 2)) \
                                        and x1 > width / 5 and x2 < width * 4 / 5:

                                    if (x1 < width / 2 and x2 < width / 2):  # if on the left side
                                        LeftPoints = OrganizeLeft([x1, x2, y1, y2], LeftPoints)

                                    elif (x1 > width / 2 and x2 > width / 2):  # if on the right side
                                        RightPoints = OrganizeRight([x1, x2, y1, y2], RightPoints)


                                ###################################_________ Horizontal___________ ############################

                                # if horizontal and extending to either side of the middle of the screen
                                elif abs(y1 - y2) < self.XrangeLines and x1 < (width / 2) and x2 > (
                                        width / 2) and x1 > width / 5 and x2 < 4 * width / 5 and y1 < height / 2:
                                    TopPoints = OrganizeTop([x1, x2, y1, y2], TopPoints)

                DrawLines(RightPoints, HoughSelected)
                DrawLines(LeftPoints, HoughSelected)
                DrawLines(TopPoints, HoughSelected)

                # Hough Lines without sorting, along with boundaries defining where lines must be
                cv2.rectangle(HoughD, (width / 2 - self.MinLineLength / 2, height / 2 - self.MinLineLength / 2),
                              (width / 2 + self.MinLineLength / 2, height / 2 + self.MinLineLength / 2), (0, 0, 255), 2)
                cv2.line(HoughD, (width / 2, 0), (width / 2, height), (0, 0, 255), 2)
                # cv2.line(HoughD, ( 0,height/2), (width,height/2), (0, 0, 255), 2)
                DistanceBetweenLines=width/2-width/5
                cv2.line(HoughD, (width / 5, 0), (width / 5, height), (0, 0, 255), 2)
                cv2.line(HoughD, (4 * width / 5, 0), (4 * width / 5, height), (0, 0, 255), 2)
		if self.UsingTemplate is False:
		    cv2.line(smallfull, (int(self.WidthOfMainVideo/2*self.WindowScale-DistanceBetweenLines), 0), (int(self.WidthOfMainVideo/2*self.WindowScale-DistanceBetweenLines), self.HeightOfMainVideo), (0, 0, 255), 2)
		    cv2.line(smallfull, (int(self.WidthOfMainVideo/2*self.WindowScale+DistanceBetweenLines), 0), (int(self.WidthOfMainVideo/2*self.WindowScale+DistanceBetweenLines), self.HeightOfMainVideo), (0, 0, 255), 2)
		   
		cv2.line(smallfull, (int(self.WidthOfMainVideo/2*self.WindowScale), 0), (int(self.WidthOfMainVideo/2*self.WindowScale), self.HeightOfMainVideo), (0, 0, 255), 2)



		  

                ##########################________ After at most 6 lines have been selected _________###############

                ########################_____________ For left,right and top lines ____________###############
                if RightPoints is not 0 and TopPoints is not 0 and LeftPoints is not 0:

                    if RightPoints.shape == tuple((4,)):  RightPoints.shape = tuple(
                        (1, 4))  # if 1d array, give second dimension
                    if TopPoints.shape == tuple((4,)):  TopPoints.shape = tuple(
                        (1, 4))  # if 1d array, give second dimension
                    if LeftPoints.shape == tuple((4,)):  LeftPoints.shape = tuple((1, 4))

                    RightSize = RightPoints.size
                    TopSize = TopPoints.size
                    LeftSize = LeftPoints.size
                    # for full right, left, and top lines
                    if RightSize > 0 and TopSize > 0 and LeftSize > 0:
                        IntersectionBounds = 10

                        (self.New, self.Old, HoughDFinal, self.BoxFound, DetectMissCounter) = FindIntersectingLines(RightPoints,
                                                                                                     TopPoints,
                                                                                                     LeftPoints,
                                                                                                     HoughDFinal,
                                                                                                     IntersectionBounds,
                                                                                                     self.UsingTemplate, self.Old,
                                                                                                     self.BoxFound,
                                                                                                     DetectMissCounter,self.HeightWidthRatio)

                    if self.New is not 0:
                        self.Toplength, self.Sidelength, self.CurrentTemplate, self.TopLeftPoints, self.BottomRightPoints, self.UsingTemplate = self.New
                    elif self.New is 0 and self.Old is not 0:
                        self.Toplength, self.Sidelength, self.CurrentTemplate, self.TopLeftPoints, self.BottomRightPoints, self.UsingTemplate = self.Old
                        self.InitialMissCounter += 1
                else:
                    self.New = 0
                    DetectMissCounter += 1
                    self.InitialMissCounter += 1

                # if there's a template and we are tracking
                if self.UsingTemplate is True:

                    CornerOfTemplateXscaled = self.TopLeftPoints[0]
                    CornerOfTemplateYscaled = self.TopLeftPoints[1]

                    (FoundGoodMatch, self.CornerOfTemplateX, self.CornerOfTemplateY, TemplateWidth, TemplateHeight,
                     TrackMissCounter) = TemplateMatch(SmallGrey, self.CurrentTemplate, CornerOfTemplateXscaled,
                                                       CornerOfTemplateYscaled, TrackMissCounter)

                    if FoundGoodMatch is True:
                        cv2.rectangle(ROIResized, (self.CornerOfTemplateX, self.CornerOfTemplateY),
                                      (self.CornerOfTemplateX + TemplateWidth, self.CornerOfTemplateY + TemplateHeight),
                                      (0, 255, 0), 2)
                        # cv2.rectangle(smallfull, (int(CornerOfTemplateX/self.WindowScale+ROIxyStartEnd[0]),int(CornerOfTemplateY/self.WindowScale+ROIxyStartEnd[1])),\
                        #               (int((CornerOfTemplateX+TemplateWidth)/self.WindowScale+ROIxyStartEnd[0]),int((CornerOfTemplateY+TemplateHeight)/self.WindowScale+ROIxyStartEnd[1])),(0,255,0),2)
                        cv2.rectangle(smallfull, (self.CornerOfTemplateX + int(ROIxyStartEnd[0] * self.WindowScale),
                                                  self.CornerOfTemplateY + int(ROIxyStartEnd[1] * self.WindowScale)), \
                                      (self.CornerOfTemplateX + int(ROIxyStartEnd[0] * self.WindowScale) + TemplateWidth,
                                       self.CornerOfTemplateY + int(ROIxyStartEnd[1] * self.WindowScale) + TemplateHeight),
                                      (0, 255, 0), 2)


                        self.CenterOfTemplateX = int((self.CornerOfTemplateX + TemplateWidth / 2) / self.WindowScale) + ROIxyStartEnd[0]
                        self.CenterOfTemplateY = int((self.CornerOfTemplateY + TemplateHeight / 2) / self.WindowScale) + ROIxyStartEnd[1]

                    # convert center of template to the larger frame to update
                    elif FoundGoodMatch is False:
                        #cv2.waitKey()
                        print "cx", self.CenterOfTemplateX
                        print "cy", self.CenterOfTemplateY

                    cv2.rectangle(SmallVidROI, (self.CenterOfTemplateX - 5, self.CenterOfTemplateY - 5),
                                  (self.CenterOfTemplateX + 5, self.CenterOfTemplateY + 5), (0, 0, 255), 2)


                # in the case that the template matching has not enabled yet, essentially on the first few hits of the detector
                elif self.UsingTemplate is False and self.New is not 0:
                    self.CenterOfTemplateX = int((self.TopLeftPoints[0] + self.Toplength / 2) / self.WindowScale) + ROIxyStartEnd[0]
                    self.CenterOfTemplateY = int((self.TopLeftPoints[1] + self.Sidelength / 2) / self.WindowScale) + ROIxyStartEnd[1]
                    # print 'center of template', self.CenterOfTemplateX,self.CenterOfTemplateY
                    # print 'top left points', TopLeftPoints
                    cv2.rectangle(smallfull, (self.CenterOfTemplateX / 2 - 5, self.CenterOfTemplateY / 2 - 5),
                                  (self.CenterOfTemplateX / 2 + 5, self.CenterOfTemplateY / 2 + 5), (0, 255, 0), 2)

                # if both miss, add one to complete
                if TrackMissCounter > 0 and DetectMissCounter > 0:
                    self.CompleteMissCounter += 1
                    # on next iteration, if one of them hits,reset
                elif TrackMissCounter is 0 or DetectMissCounter is 0:
                    self.CompleteMissCounter = 0

                # if missing for longer than a second, reset to middle
                if self.CompleteMissCounter > 40:
                    (self.CenterOfTemplateX, self.CenterOfTemplateY, CompleteMissCounter, self.Old, New,  self.BoxFound, self.UsingTemplate,
                     CurrentTemplate, self.InitialMissCounter) = ResetToBeginning(self)
                if self.UsingTemplate is False and self.InitialMissCounter > 900000:
                    (self.CenterOfTemplateX, self.CenterOfTemplateY, CompleteMissCounter, self.Old, New,  self.BoxFound, self.UsingTemplate,
                     CurrentTemplate, self.InitialMissCounter) = ResetToBeginning(self)

                cv2.imshow('dilate', dilate)
                cv2.imshow('houghdfinal', HoughDFinal)
                #cv2.imshow('ROI', ROIResized)
                cv2.imshow('HoughSelected', HoughSelected)
                cv2.imshow('houghd', HoughD)
                cv2.imshow('full', smallfull)

             #   if self.CenterOfTemplateX > self.WidthOfMainVideo / 2 and self.UsingTemplate == True:
                   
#  print "Target off by %d pixels to the Right" % (self.CenterOfTemplateX - self.WidthOfMainVideo / 2)
              #  elif self.CenterOfTemplateX < self.WidthOfMainVideo / 2 and self.UsingTemplate == True:
                   
			# print "Target off by %d pixels to the Left" % (self.WidthOfMainVideo / 2 - self.CenterOfTemplateX)

                if TemplateWidth is not 0:
                    FullWidthInPixels = TemplateWidth / self.WindowScale

                    PixelWidthRatio = self.TrailerWidth / float(FullWidthInPixels)  # units meters per pixel
                    DistanceToTruck = PixelWidthRatio * float(self.FocalLength)
                    #print "Approximate Distance To Truck:", round(DistanceToTruck, 2), " meters"  # % DistanceToTruck

                if self.UsingTemplate == True and TemplateWidth is not 0 and TrackMissCounter is 0:
                    LateralError = (self.CenterOfTemplateX - self.WidthOfMainVideo / 2) *PixelWidthRatio
                    LateralError = round(LateralError, 2)
                    LongDist = round(DistanceToTruck, 2)

                point = Point32()
                point.x = LateralError
                point.y = LongDist
                self.pub.publish(point)


                if rospy.is_shutdown():
                    cv2.destroyAllWindows()


        except CvBridgeError as e:
            print(e)


        cv2.waitKey(1)

        try:
            self.image_pub.publish(self.bridge.cv2_to_imgmsg(BigCap, "bgr8"))
            self.image_pub_full.publish(self.bridge.cv2_to_imgmsg(smallfull,"bgr8"))
        except CvBridgeError as e:
            print(e)

def main(args):

    rospy.init_node('talker',anonymous=True)
    ic = image_converter()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()


if __name__ == '__main__':
    main(sys.argv)
