import numpy as np
import cv2
import rospy
from geometry_msgs.msg import Point32
import sys
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

TargetDist=25

# this version has been modified for video taken with pointgrey camera
#profile command: Navigate to correct folder then run:
# python -m cProfile -s tottime TruckDetectPointgrey.py > profile2hp.text 2>&1



def OrganizeLeft(CurrentPoints,LeftPoints):
    x1=CurrentPoints[0]
    x2 = CurrentPoints[1]
    y1=CurrentPoints[2]
    y2=CurrentPoints[3]
    if LeftPoints is 0:  # if not defined for this frame
        # now we want to define the points such that x1 corresponds to the top coordinate
        if y1 <= y2:
            LeftPoints = np.array([x1, x2, y1, y2])  # make the first row
        elif y1 > y2:
            LeftPoints = np.array([x2, x1, y2, y1])

    else:  # otherwise stack it onto the last row
        if LeftPoints.size == 4:
            # now we want to define the points such that x1 corresponds to the top coordinate
            if y1 <= y2:
                LeftPoints = np.vstack((LeftPoints, [x1, x2, y1, y2]))  # make the first row
            elif y1 > y2:
                LeftPoints = np.vstack((LeftPoints, [x2, x1, y2, y1]))
            # now, if row 1 is greater than row 2, switch
            if abs(LeftPoints[0, 0] + LeftPoints[0, 1]) / 2 >= abs(LeftPoints[1, 0] + LeftPoints[1, 1]) / 2:
                LeftPoints[1, :] = LeftPoints[0, :]
                if y1 <= y2:
                    LeftPoints[0, :] = np.array([x1, x2, y1, y2])
                elif y1 > y2:
                    LeftPoints[0, :] = np.array([x2, x1, y2, y1])

        elif LeftPoints.size > 4:  # if there is already 2 lines

            # if the new line is left of both the old lines
            if abs(LeftPoints[1, 0] + LeftPoints[1, 1]) / 2 >= abs(x1 + x2) / 2 \
                    and abs(LeftPoints[0, 0] + LeftPoints[0, 1]) / 2 >= abs(x1 + x2) / 2:
                # move the old line down to position 2
                LeftPoints[1, :] = LeftPoints[0, :]
                # place new line in row 1
                # now we want to define the points such that x1 corresponds to the top coordinate
                if y1 <= y2:
                    LeftPoints[0, :] = np.array([x1, x2, y1, y2])
                elif y1 > y2:
                    LeftPoints[0, :] = np.array([x2, x1, y2, y1])


            # else if new line is between previous 2 lines
            elif abs(LeftPoints[1, 0] + LeftPoints[1, 1]) / 2 >= abs(x1 + x2) / 2 \
                    and abs(LeftPoints[0, 0] + LeftPoints[0, 1]) / 2 <= abs(x1 + x2) / 2:
                # replace line 2 with new line
                # now we want to define the points such that x1 corresponds to the top coordinate
                if y1 <= y2:
                    LeftPoints[1, :] = np.array([x1, x2, y1, y2])
                elif y1 > y2:
                    LeftPoints[1, :] = np.array([x2, x1, y2, y1])

    return LeftPoints

def OrganizeRight(CurrentPoints,RightPoints):
    x1 = CurrentPoints[0]
    x2 = CurrentPoints[1]
    y1 = CurrentPoints[2]
    y2 = CurrentPoints[3]
    if RightPoints is 0:  # if not defined, write
        if y1 <= y2:
            RightPoints = np.array([x1, x2, y1, y2])  # make the first row
        elif y1 > y2:
            RightPoints = np.array([x2, x1, y2, y1])

    else:  # otherwise, stack
        if RightPoints.size == 4:
            # now we want to define the points such that x1 corresponds to the top coordinate
            if y1 <= y2:
                RightPoints = np.vstack((RightPoints, [x1, x2, y1, y2]))  # make the first row
            elif y1 > y2:
                RightPoints = np.vstack((RightPoints, [x2, x1, y2, y1]))
            # now, if row 2 is greater than row 1, switch
            if abs(RightPoints[0, 0] + RightPoints[0, 1]) / 2 <= abs(x1 + x2) / 2:
                RightPoints[1, :] = RightPoints[0, :]
                if y1 <= y2:
                    RightPoints[0, :] = np.array([x1, x2, y1, y2])
                elif y1 > y2:
                    RightPoints[0, :] = np.array([x2, x1, y2, y1])

        elif RightPoints.size > 4:  # if there is already 2 lines

            # if the new line is right of both the old lines
            if abs(RightPoints[1, 0] + RightPoints[1, 1]) / 2 <= abs(x1 + x2) / 2 \
                    and abs(RightPoints[0, 0] + RightPoints[0, 1]) / 2 <= abs(x1 + x2) / 2:
                # move the old line down to position 2
                RightPoints[1, :] = RightPoints[0, :]
                # place new line in row 1
                # now we want to define the points such that x1 corresponds to the top coordinate
                if y1 <= y2:
                    RightPoints[0, :] = np.array([x1, x2, y1, y2])
                elif y1 > y2:
                    RightPoints[0, :] = np.array([x2, x1, y2, y1])

            # else if new line is between previous 2 lines
            elif abs(RightPoints[1, 0] + RightPoints[1, 1]) / 2 <= abs(x1 + x2) / 2 \
                    and abs(RightPoints[0, 0] + RightPoints[0, 1]) / 2 >= abs(x1 + x2) / 2:
                # replace line 2 with new line
                if y1 <= y2:
                    RightPoints[1, :] = np.array([x1, x2, y1, y2])
                elif y1 > y2:
                    RightPoints[1, :] = np.array([x2, x1, y2, y1])

    return RightPoints

def OrganizeTop(CurrentPoints,TopPoints):
    x1 = CurrentPoints[0]
    x2 = CurrentPoints[1]
    y1 = CurrentPoints[2]
    y2 = CurrentPoints[3]
    if TopPoints is 0:
        TopPoints = np.array([x1, x2, y1, y2])
    else:
        if TopPoints.size == 4:
            TopPoints = np.vstack((TopPoints, [x1, x2, y1, y2]))
            # now, if row 2 is greater than row 1, switch
            if abs(TopPoints[0, 2] + TopPoints[0, 3]) / 2 < abs(y1 + y2) / 2:
                TopPoints[1, :] = TopPoints[0, :]
                TopPoints[0, :] = np.array([x1, x2, y1, y2])

        elif TopPoints.size > 4:  # if there is already 2 lines

            # if the new line is above of both the old lines
            if abs(TopPoints[1, 2] + TopPoints[1, 3]) / 2 >= abs(y1 + y2) / 2 \
                    and abs(TopPoints[0, 2] + TopPoints[0, 3]) / 2 >= abs(y1 + y2) / 2:
                # move the old line down to position 2
                TopPoints[1, :] = TopPoints[0, :]
                # place new line in row 1
                TopPoints[0, :] = np.array([x1, x2, y1, y2])

            # else if new line is between previous 2 lines
            elif abs(TopPoints[1, 2] + TopPoints[1, 3]) / 2 >= abs(y1 + y2) / 2 \
                    and abs(TopPoints[0, 2] + TopPoints[0, 3]) / 2 <= abs(y1 + y2) / 2:
                # replace line 2 with new line
                TopPoints[1, :] = np.array([x1, x2, y1, y2])

    return TopPoints

def LineFromPoints(xyPoints):
    #takes a set of points defining the endpoints of a line segment and returns the line in homogeneous coordinates

    Line = np.cross([xyPoints[ 0], xyPoints[ 2], 1], [xyPoints[ 1], xyPoints[ 3], 1])
    Line= np.float64(Line)  # convert to float
    Line=Line/Line[2]
    return Line

def Intersection(Line1,Line2):
    Intersection=np.cross(Line1, Line2)
    Intersection = np.int32(Intersection / Intersection[2])
    return Intersection

def InArea(IntersectionPoint,SidePoints,TopPoints,PositionFlag,IntersectionBounds):

    if PositionFlag=='Right':
        RightX = False
        RightY = False
        SidePoints=SidePoints[0,:]
        TopPoints=TopPoints[0,:]
        # right x
        if abs( IntersectionPoint[0] -SidePoints[0])<IntersectionBounds\
            and abs(IntersectionPoint[0]-TopPoints[1])<IntersectionBounds:
            RightX=True
        #right y
        if abs(IntersectionPoint[1] -SidePoints[2])<IntersectionBounds\
            and abs(IntersectionPoint[1]-TopPoints[3])<IntersectionBounds:
            RightY=True

        if RightX==True and RightY==True:
            PointsAreClose=1
        else:
            PointsAreClose=0

        return PointsAreClose

    elif PositionFlag=='Left':
        LeftX=False
        LeftY=False
        SidePoints=SidePoints[0,:]
        TopPoints=TopPoints[0,:]
        #Left X
        if abs(IntersectionPoint[0]-SidePoints[0])<IntersectionBounds\
            and abs(IntersectionPoint[0]-TopPoints[0])<IntersectionBounds:
            LeftX=True

        #left y
        if abs(IntersectionPoint[1]-SidePoints[2])<IntersectionBounds\
            and abs(IntersectionPoint[1]-TopPoints[2])<IntersectionBounds:
            LeftY=True

        if LeftX==True and LeftY==True:
            PointsAreClose=1
        else:
            PointsAreClose=0

        return PointsAreClose

def FindIntersectingLines(RightPoints,TopPoints,LeftPoints,Frame,IntersectionBounds,UsingTemplate,Old,BoxFound,DetectMissCounter,HeightWidthRatio):

    if Old is not 0:
        (oldtoplength, oldsidelength, oldcurrenttemplate, oldtopleftpoints, oldbottomrightpoints,oldusingtemplate) = Old


    PossibleNewTemplate=False
    SizesMatch=False
    RightLine1 = LineFromPoints(RightPoints[0, :])
    TopLine1 = LineFromPoints(TopPoints[0, :])
    LeftLine1 = LineFromPoints(LeftPoints[0, :])
    IntersectionRight = Intersection(RightLine1, TopLine1)
    IntersectionLeft = Intersection(LeftLine1, TopLine1)
    RightPointsClose = InArea(IntersectionRight, RightPoints, TopPoints, 'Right', IntersectionBounds)
    LeftPointsClose = InArea(IntersectionLeft, LeftPoints, TopPoints, 'Left', IntersectionBounds)
    New=0

    if BoxFound>3:
        UsingTemplate=True

    if RightPointsClose == 1 and LeftPointsClose == 1:
        PossibleNewTemplate=True
        Toplength = IntersectionRight[0] - IntersectionLeft[0]
        Sidelength = int(Toplength * HeightWidthRatio)
        TopLeftPoints = (IntersectionLeft[:2])
        BottomRightPoints = (IntersectionRight[0], IntersectionRight[1] + Sidelength)
        NewTemplate = Frame[TopLeftPoints[1]:BottomRightPoints[1], IntersectionLeft[0]:IntersectionRight[0]]
        NewTemplate= cv2.cvtColor(NewTemplate, cv2.COLOR_BGR2GRAY)

        if UsingTemplate == False:
            BoxFound+=1
            New = (Toplength, Sidelength, NewTemplate, TopLeftPoints, BottomRightPoints, UsingTemplate)
            Old = (Toplength, Sidelength, NewTemplate, TopLeftPoints, BottomRightPoints, UsingTemplate)


        elif UsingTemplate == True:
            newtempheight,newtempwidth=NewTemplate.shape[:2]
            #if the new template is within 18% of the size of the old template
            TemplateSizeAccuracy=0.18
            if abs(newtempheight-oldsidelength)<TemplateSizeAccuracy*oldsidelength and\
               abs(newtempwidth-oldtoplength)<TemplateSizeAccuracy*oldtoplength:
                SizesMatch=True
                # ClockPulse+=1
                # if ClockPulse% 2 ==0:
                #     Start=time.time()
                # else:
                #     End=time.time()
                #
                # if End-Start<1:
                #     SizesMatch=False
            else:
                SizesMatch=False


        cv2.rectangle(Frame, tuple((IntersectionRight[:2])), (IntersectionLeft[0], IntersectionLeft[1] + Sidelength),(255, 0, 0), 2)

    #if we find a new template that is the right size, pass new region as new starting region
    if PossibleNewTemplate is True and SizesMatch is True:
        New = (Toplength, Sidelength, NewTemplate, TopLeftPoints, BottomRightPoints, UsingTemplate)
        Old = (Toplength, Sidelength, NewTemplate, TopLeftPoints, BottomRightPoints, UsingTemplate)

    if New is 0:
        DetectMissCounter+=1

    return(New,Old,Frame,BoxFound,DetectMissCounter)

def DrawLines(Points,Frame):
    if Points is not 0:
        if Points.shape == tuple((4,)):  Points.shape = tuple((1, 4))  # if 1d array, give second dimension
        length,otherlength =Points.shape[:2]

        # for i in range(len(Points)):
        #     for x1,x2,y1, y2 in Points:
        #         cv2.line(Frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
        #
        # for i in range(len(Points)):
        Points=Points[0]
        [x1,x2,y1, y2] = Points
        cv2.line(Frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
    return Frame

def TemplateMatch(Frame,Template,oldtemplatecx,oldtemplatecy,TrackMissCounter):

    FoundTemplateAttributes=None
    maxval=0
    TemplateHeight, TemplateWidth=Template.shape[:2]
    FrameHeight,FrameWidth=Frame.shape[:2]


    SearchAreaXStart=oldtemplatecx-int(TemplateWidth*.25)#initialize search area to be very close to the original position
    SearchAreaXEnd=oldtemplatecx+int(TemplateWidth*1.25)
    SearchAreaYStart=oldtemplatecy-int(TemplateHeight*.25)
    SearchAreaYEnd=oldtemplatecy+int(TemplateHeight*1.25)

    if SearchAreaXStart<0:SearchAreaXStart=0
    if SearchAreaXEnd>FrameWidth:SearchAreaXEnd=FrameWidth
    if SearchAreaYStart<0:SearchAreaYStart=0
    if SearchAreaYEnd>FrameHeight:SearchAreaYEnd=FrameHeight

    # if FirstFlag==1:
    #     SearchArea=Frame
    #     SearchAreaXStart=0
    #     SearchAreaYStart=0
    # else:
    SearchArea =Frame[int(SearchAreaYStart):int(SearchAreaYEnd),int(SearchAreaXStart):int(SearchAreaXEnd)]
    SearchAreaRectangle=cv2.resize(SearchArea,(0,0),fx=1,fy=1)
    cv2.imshow('template',Template)
    cv2.imshow('Frame',Frame)
    cv2.imshow('SearchArea',SearchArea)


    for i in np.linspace(0.95,1.05,11):#for increments of 1% size

        Resized=cv2.resize(Template,(0, 0), fx=i, fy=i)#resize
        Ratio=i #keep a ratio to know which one is closest

        rh,rw=Resized.shape[:2]
        sh,sw=SearchArea.shape[:2]
	if rh<sh and rw<sw:
        	Match=cv2.matchTemplate(SearchArea,Resized,cv2.TM_CCOEFF_NORMED)
        	_, maxval, _, maxloc = cv2.minMaxLoc(Match)   #template match location
        # cv2.imshow('Search Area', SearchArea)
        # cv2.imshow('resized',Resized)
        # cv2.rectangle(SearchAreaRectangle,maxloc,(maxloc[0]+int(TemplateWidth*Ratio),maxloc[1]+int(TemplateHeight*Ratio)),(0,255,0),2)
        # cv2.imshow('Found Position',SearchAreaRectangle)


        if FoundTemplateAttributes is None or maxval>FoundTemplateAttributes[0]:
            FoundTemplateAttributes=(maxval,maxloc,Ratio)#if not initialized or if new match is better

    (Finalmaxval,Finalmaxloc,FinalRatio)=FoundTemplateAttributes#after all 10% has been looped through
    # print FinalRatio
    FinalTemplateWidth=int(TemplateWidth*FinalRatio)#new widths and heights
    FinalTemplateHeight=int(TemplateHeight*FinalRatio)



    if Finalmaxval>.8:#if best match is 80% or greater (should be all the time)
        NewCx=Finalmaxloc[0]+SearchAreaXStart
        #new corner is at max location (corner)+search area (bring it back to size of frame) + size of scaled template
        NewCy=Finalmaxloc[1]+SearchAreaYStart
        NewWidth=FinalTemplateWidth
        NewHeight=FinalTemplateHeight
        FoundGoodMatch=True

    else:
        NewCx=oldtemplatecx
        NewCy=oldtemplatecy
        NewWidth=TemplateWidth
        NewHeight=TemplateHeight
        FoundGoodMatch=False
        TrackMissCounter+=1

    return(FoundGoodMatch,NewCx,NewCy,NewWidth,NewHeight,TrackMissCounter)

def ResetToBeginning(self):
    self.CenterOfTemplateX = self.InitialCenterOfTemplateX
    self.CenterOfTemplateY = self.InitialCenterOfTemplateY
    self.CompleteMissCounter = 0
    self.Old = 0
    New = 0
    BoxFound = 0
    self.UsingTemplate = False
    CurrentTemplate = 0
    self.InitialMissCounter=0
    return (self.CenterOfTemplateX,self.CenterOfTemplateY,self.CompleteMissCounter,self.Old,New,BoxFound,self.UsingTemplate,CurrentTemplate,self.InitialMissCounter)







###############################################   #import video   ########################################


class image_converter:
    def __init__(self):
        self.image_pub = rospy.Publisher("image_topic_2",Image,queue_size=100)
        self.pub=rospy.Publisher('chatter', Point32, queue_size=10)

        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/camera/image_color",Image,self.callback)
        self.HeightOfMainVideo = 964
        self.WidthOfMainVideo = 1288

        TrailerHeight = 112.0  # inches
        TrailerWidth = 93.0  # inches
        self.HeightWidthRatio = (TrailerHeight / TrailerWidth)

        TrailerPixelWidth = 200
        TrailerPixelHeight = int(round(TrailerPixelWidth * self.HeightWidthRatio))
        self.WindowScale = 0.5
        self.MinLineLength = int(0.05 * (self.WidthOfMainVideo) * self.WindowScale)
        self.MaxLineGap = int(0.015 * self.WidthOfMainVideo * self.WindowScale)

        self.XrangeLines = int(self.WidthOfMainVideo / 25 * self.WindowScale)

        self.Old = 0
        self.New = 0
        self.BoxFound = 0
        self.UsingTemplate = False
        CurrentTemplate = 0
        Clock = (0, 0, 1)
        self.FocalLength = 2000  # pixels
        self.InitialMissCounter = 0

        # Region of Interest
        self.LeftBuffer = self.WidthOfMainVideo / 4
        self.RightBuffer = self.WidthOfMainVideo / 4  # add this to center position
        self.TopBottomBuffer = self.HeightOfMainVideo / 3
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


            if BigCap is not 0:
            # while (self.initializeBox):

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
                cv2.line(HoughD, (width / 5, 0), (width / 5, height), (0, 0, 255), 2)
                cv2.line(HoughD, (4 * width / 5, 0), (4 * width / 5, height), (0, 0, 255), 2)

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
                    # convert center of template to the larger frame to update

                    self.CenterOfTemplateX = int((self.CornerOfTemplateX + TemplateWidth / 2) / self.WindowScale) + ROIxyStartEnd[0]
                    self.CenterOfTemplateY = int((self.CornerOfTemplateY + TemplateHeight / 2) / self.WindowScale) + ROIxyStartEnd[1]
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
                if self.UsingTemplate is False and self.InitialMissCounter > 90:
                    (self.CenterOfTemplateX, self.CenterOfTemplateY, CompleteMissCounter, self.Old, New,  self.BoxFound, self.UsingTemplate,
                     CurrentTemplate, self.InitialMissCounter) = ResetToBeginning(self)

                cv2.imshow('dilate', dilate)
                cv2.imshow('houghdfinal', HoughDFinal)
                #cv2.imshow('ROI', ROIResized)
                cv2.imshow('HoughSelected', HoughSelected)
                cv2.imshow('houghd', HoughD)
                cv2.imshow('full', smallfull)

                if self.CenterOfTemplateX > self.WidthOfMainVideo / 2 and self.UsingTemplate == True:
                    print "Target off by %d pixels to the Right" % (self.CenterOfTemplateX - self.WidthOfMainVideo / 2)
                elif self.CenterOfTemplateX < self.WidthOfMainVideo / 2 and self.UsingTemplate == True:
                    print "Target off by %d pixels to the Left" % (self.WidthOfMainVideo / 2 - self.CenterOfTemplateX)

                if TemplateWidth is not 0:
                    FullWidthInPixels = TemplateWidth / self.WindowScale
                    TrailerWidth = 2.37  # meters
                    PixelWidthRatio = 2.37 / float(FullWidthInPixels)  # units meters per pixel
                    DistanceToTruck = PixelWidthRatio * float(self.FocalLength)
                    print "Approximate Distance To Truck:", round(DistanceToTruck, 2), " meters"  # % DistanceToTruck

                if self.UsingTemplate == True and TemplateWidth is not 0 and TrackMissCounter is 0:
                    LateralError = (self.CenterOfTemplateX - self.WidthOfMainVideo / 2) / float(FullWidthInPixels)
                    LateralError = round(LateralError, 2)
                    LongDist = round(DistanceToTruck, 2)

                point = Point32()
                point.x = LateralError
                point.y = LongDist
                self.pub.publish(point)


                if rospy.is_shutdown():
                    cv2.destroyAllWindows()
                    cap.release
                    #break

        except CvBridgeError as e:
            print(e)

        #cv2.imshow("Image window", BigCap)
        cv2.waitKey(1)

        try:
            self.image_pub.publish(self.bridge.cv2_to_imgmsg(BigCap, "bgr8"))
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
