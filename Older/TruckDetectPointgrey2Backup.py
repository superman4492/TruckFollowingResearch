import numpy as np
import cv2
import rospy
from geometry_msgs.msg import Point32


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

def DrawLinesWithIntersection(RightPoints,TopPoints,LeftPoints,Frame,IntersectionBounds,UsingTemplate):
    #takes input intersection points, draws lines on "FRAME" if the points are close enough (Intersectionbounds)

    RightLine1 = LineFromPoints(RightPoints[0, :])
    TopLine1 = LineFromPoints(TopPoints[0, :])
    LeftLine1 = LineFromPoints(LeftPoints[0, :])

    IntersectionRight = Intersection(RightLine1, TopLine1)
    IntersectionLeft = Intersection(LeftLine1, TopLine1)

    RightPointsClose=InArea(IntersectionRight,RightPoints,TopPoints,'Right',IntersectionBounds)
    LeftPointsClose=InArea(IntersectionLeft,LeftPoints,TopPoints,'Left',IntersectionBounds)
    Toplength=0
    Sidelength=0
    CurrentTemplate=0
    TopLeftPoints=0
    BottomRightPoints=0
    if RightPointsClose ==1 and LeftPointsClose==1:
        Toplength=IntersectionRight[0]-IntersectionLeft[0]
        Sidelength=int(Toplength*HeightWidthRatio)
        TopLeftPoints=(IntersectionLeft[:2])
        BottomRightPoints=(IntersectionRight[0],IntersectionRight[1]+Sidelength)
        if UsingTemplate==False:
            UsingTemplate=True
            ColorTemplate=Frame[TopLeftPoints[1]:BottomRightPoints[1],IntersectionLeft[0]:IntersectionRight[0]]
            CurrentTemplate=cv2.cvtColor(ColorTemplate,cv2.COLOR_BGR2GRAY)
            #cv2.imshow('curtemplate',CurrentTemplate)
            #cv2.waitKey(1)
        elif UsingTemplate==True:
            ColorTemplate = Frame[TopLeftPoints[1]:BottomRightPoints[1],IntersectionLeft[0]:IntersectionRight[0]]
            CurrentTemplate = cv2.cvtColor(ColorTemplate, cv2.COLOR_BGR2GRAY)
            #cv2.imshow('curtemplate', CurrentTemplate)
            #cv2.waitKey(1)
        cv2.rectangle(Frame,tuple((IntersectionRight[:2])),(IntersectionLeft[0],IntersectionLeft[1]+Sidelength),(255,0,0),2)

    return (Frame,Toplength,Sidelength,CurrentTemplate,TopLeftPoints,BottomRightPoints,UsingTemplate)

def FindIntersectingLines(RightPoints,TopPoints,LeftPoints,Frame,IntersectionBounds,UsingTemplate,Old,BoxFound,DetectMissCounter):

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
    #cv2.waitKey(1)

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
        # cv2.waitKey(1)

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

def ResetToBeginning():
    CenterOfTemplateX = InitialCenterOfTemplateX
    CenterOfTemplateY = InitialCenterOfTemplateY
    CompleteMissCounter = 0
    Old = 0
    New = 0
    BoxFound = 0
    UsingTemplate = False
    CurrentTemplate = 0
    InitialMissCounter=0
    return (CenterOfTemplateX,CenterOfTemplateY,CompleteMissCounter,Old,New,BoxFound,UsingTemplate,CurrentTemplate,InitialMissCounter)


###############################################   #import video   ########################################
BigCap=cv2.VideoCapture(0)
ret,frame=BigCap.read()
HeightOfMainVideo,WidthOfMainVideo=frame.shape[:2]


TrailerHeight=112.0 #inches
TrailerWidth=93.0 #inches
HeightWidthRatio=(TrailerHeight/TrailerWidth)

TrailerPixelWidth=200
TrailerPixelHeight=int(round(TrailerPixelWidth*HeightWidthRatio))
WindowScale=1
MinLineLength = int(0.05*(WidthOfMainVideo) * WindowScale)
MaxLineGap = int(0.015*WidthOfMainVideo*WindowScale)

XrangeLines=int(WidthOfMainVideo/25*WindowScale)

Old=0
New=0
BoxFound=0
UsingTemplate=False
CurrentTemplate=0
Clock=(0,0,1)
FocalLength=2000#pixels
InitialMissCounter=0


#Region of Interest
LeftBuffer=WidthOfMainVideo/4
RightBuffer=WidthOfMainVideo/4#add this to center position
TopBottomBuffer=HeightOfMainVideo/3
BottomBuffer=int(HeightOfMainVideo*0.5)#add this to the center position

CenterOfTemplateX=WidthOfMainVideo/2
CenterOfTemplateY=HeightOfMainVideo/2
InitialCenterOfTemplateX=WidthOfMainVideo/2
InitialCenterOfTemplateY=HeightOfMainVideo/2

initializeBox=True
LowerThreshold = 50
UpperThreshold = 255
Skipframe=0
houghDold=None




################____________program________________##############

pub = rospy.Publisher('chatter', Point32, queue_size=10)
rospy.init_node('talker', anonymous=True)





while(BigCap.isOpened()):


    while (initializeBox):

        ret,BigVid=BigCap.read()

        smallfull=cv2.resize(BigVid,(0,0),fx=WindowScale,fy=WindowScale)

        #Define ROI for current frame
        ROIxStart = CenterOfTemplateX - LeftBuffer
        ROIxEnd = CenterOfTemplateX + RightBuffer
        ROIyStart=CenterOfTemplateY-TopBottomBuffer
        ROIyEnd = CenterOfTemplateY+BottomBuffer
        if (CenterOfTemplateX-LeftBuffer)<0:ROIxStart=0
        if (CenterOfTemplateX+RightBuffer)>WidthOfMainVideo:ROIxEnd=WidthOfMainVideo
        if (CenterOfTemplateY-TopBottomBuffer)<0:ROIyStart=0
        if (CenterOfTemplateY+BottomBuffer)>HeightOfMainVideo:ROIyEnd=HeightOfMainVideo
        SmallVidROI = BigVid[ROIyStart:ROIyEnd, ROIxStart:ROIxEnd]
        ROIxyStartEnd=[ROIxStart,ROIyStart,ROIxEnd,ROIyEnd]




        HoughD = cv2.resize(SmallVidROI, (0, 0), fx=WindowScale, fy=WindowScale)
        HoughD2 = cv2.resize(SmallVidROI, (0, 0), fx=WindowScale, fy=WindowScale)
        HoughDFinal = cv2.resize(SmallVidROI, (0, 0), fx=WindowScale, fy=WindowScale)
        HoughSelected = cv2.resize(SmallVidROI, (0, 0), fx=WindowScale, fy=WindowScale)
        height,width=HoughD.shape[:2]
        SmallGrey=cv2.cvtColor(HoughD,cv2.COLOR_BGR2GRAY)
        ROIResized=cv2.resize(SmallVidROI,(0,0),fx=WindowScale,fy=WindowScale)
        ROIForFrame=cv2.resize(SmallVidROI,(0,0),fx=WindowScale,fy=WindowScale)
        # cv2.imshow('ROIFrame',ROIForFrame)
        # cv2.waitKey(1)

        edges = cv2.Canny(SmallGrey, LowerThreshold, UpperThreshold) #canny edge detection
        #cv2.imshow('canny',edges)
        dilate=cv2.dilate(edges,kernel=(1,10),iterations=1) # dilate the edges with a horizontal line
        dilate=cv2.dilate(dilate,kernel=(10,1),iterations=1)# dilate edges with a vertical line

        #cv2.imshow('dilate',dilate)

        # if UsingTemplate is not True:
        #     houghD = cv2.HoughLinesP(dilate, 1, np.pi / 180, 50, minLineLength=MinLineLength, maxLineGap=MaxLineGap)
        # else:
        #     Skipframe+=1
        #     if Skipframe==2:
        #         houghD=cv2.HoughLinesP(dilate, 1, np.pi / 180, 50, minLineLength=MinLineLength, maxLineGap=MaxLineGap)
        #         Skipframe=0
        #     else:
        #         houghD=None
        houghD = cv2.HoughLinesP(dilate, 1, np.pi / 180, 50, minLineLength=MinLineLength, maxLineGap=MaxLineGap)
        # for all horizontal and vertical lines, draw them
        if houghD is not None:
            for i in range(len(houghD)):
                for x1, y1, x2, y2 in houghD[i]:
                    angle=np.arctan2(y2-y1,x2-x1)*180./np.pi
                    if abs(angle)>85 or abs(angle)<5:
                        cv2.line(HoughD2, (x1, y1), (x2, y2), (0, 255, 0), 2)


        LeftPoints=0
        RightPoints=0
        TopPoints=0
        TrackMissCounter=0
        DetectMissCounter=0
        TemplateWidth=0
	LateralError=0
	LongDist=TargetDist

        if houghD is not None:#filtering for straight lines
            for i in range(len(houghD)):# each i is one line in this frame
                for x1, y1, x2, y2 in houghD[i]: #for the points corresponding to the i lines in this frame
                    angle = np.arctan2(y2 - y1, x2 - x1) * 180. / np.pi
                    if abs(angle) > 85 or abs(angle) < 5:

                        #####################################_______Vertical__________###############

                        # if vertical and both x lie on one side of the screen, and are within 1/5 of the edge
                        if abs(x1 - x2) < XrangeLines and ((x1 > width/2 and x2 > width/2) or (x1 < width/2 and x2 < width/2)) \
                            and x1>width/5 and x2<width*4/5:

                            if (x1 < width/2 and x2 < width/2):# if on the left side
                                LeftPoints=OrganizeLeft([x1,x2,y1,y2],LeftPoints)

                            elif (x1 > width / 2 and x2 > width / 2):  # if on the right side
                                RightPoints=OrganizeRight([x1,x2,y1,y2],RightPoints)


                        ###################################_________ Horizontal___________ ############################

                        # if horizontal and extending to either side of the middle of the screen
                        elif abs(y1-y2)<XrangeLines and x1 < (width / 2 ) and x2 > (width / 2) and x1>width/5 and x2<4*width/5 and y1<height/2:
                            TopPoints=OrganizeTop([x1,x2,y1,y2],TopPoints)


        DrawLines(RightPoints,HoughSelected)
        DrawLines(LeftPoints,HoughSelected)
        DrawLines(TopPoints,HoughSelected)

        #Hough Lines without sorting, along with boundaries defining where lines must be
        cv2.rectangle(HoughD,(width/2-MinLineLength/2,height/2-MinLineLength/2),(width/2+MinLineLength/2,height/2+MinLineLength/2),(0,0,255),2)
        cv2.line(HoughD,(width/2,0),(width/2,height),(0,0,255),2)
        #cv2.line(HoughD, ( 0,height/2), (width,height/2), (0, 0, 255), 2)
        cv2.line(HoughD,(width/5,0),(width/5,height),(0,0,255),2)
        cv2.line(HoughD, (4*width / 5, 0), (4*width / 5, height), (0, 0, 255), 2)



    ##########################________ After at most 6 lines have been selected _________###############


        ########################_____________ For left,right and top lines ____________###############
        if RightPoints is not 0 and TopPoints is not 0 and LeftPoints is not 0:
            if RightPoints.shape == tuple((4,)):  RightPoints.shape = tuple((1, 4))  # if 1d array, give second dimension
            if TopPoints.shape==tuple((4,)):  TopPoints.shape = tuple((1, 4))  # if 1d array, give second dimension
            if LeftPoints.shape==tuple((4,)):  LeftPoints.shape = tuple((1, 4))

            RightSize=RightPoints.size
            TopSize=TopPoints.size
            LeftSize=LeftPoints.size
            # for full right, left, and top lines
            if RightSize>0 and TopSize>0 and LeftSize>0:
                IntersectionBounds=10
                (New,Old,HoughDFinal,BoxFound,DetectMissCounter)=FindIntersectingLines(RightPoints,TopPoints,LeftPoints,HoughDFinal,IntersectionBounds,UsingTemplate,Old,BoxFound,DetectMissCounter)
            if New is not 0:
                Toplength, Sidelength, CurrentTemplate, TopLeftPoints, BottomRightPoints, UsingTemplate=New
            elif New is 0 and Old is not 0:
                Toplength, Sidelength, CurrentTemplate, TopLeftPoints, BottomRightPoints, UsingTemplate=Old
                InitialMissCounter+=1
        else:
            New=0
            DetectMissCounter+=1
            InitialMissCounter+=1

        #if there's a template and we are tracking
        if UsingTemplate is True:

            CornerOfTemplateXscaled= TopLeftPoints[0]
            CornerOfTemplateYscaled = TopLeftPoints[1]

            (FoundGoodMatch,CornerOfTemplateX,CornerOfTemplateY,TemplateWidth,TemplateHeight,TrackMissCounter)=TemplateMatch(SmallGrey,CurrentTemplate,CornerOfTemplateXscaled,CornerOfTemplateYscaled,TrackMissCounter)

            if FoundGoodMatch is True:
                cv2.rectangle(ROIResized,(CornerOfTemplateX,CornerOfTemplateY),(CornerOfTemplateX+TemplateWidth,CornerOfTemplateY+TemplateHeight),(0,255,0),2)
                # cv2.rectangle(smallfull, (int(CornerOfTemplateX/WindowScale+ROIxyStartEnd[0]),int(CornerOfTemplateY/WindowScale+ROIxyStartEnd[1])),\
                #               (int((CornerOfTemplateX+TemplateWidth)/WindowScale+ROIxyStartEnd[0]),int((CornerOfTemplateY+TemplateHeight)/WindowScale+ROIxyStartEnd[1])),(0,255,0),2)
                cv2.rectangle(smallfull,(CornerOfTemplateX+int(ROIxyStartEnd[0]*WindowScale),CornerOfTemplateY+int(ROIxyStartEnd[1]*WindowScale)),\
                                         (CornerOfTemplateX+int(ROIxyStartEnd[0]*WindowScale)+TemplateWidth,CornerOfTemplateY+int(ROIxyStartEnd[1]*WindowScale)+TemplateHeight),(0,255,0),2)
#convert center of template to the larger frame to update

            CenterOfTemplateX=int((CornerOfTemplateX+TemplateWidth/2)/WindowScale)+ROIxyStartEnd[0]
            CenterOfTemplateY=int((CornerOfTemplateY+TemplateHeight/2)/WindowScale)+ROIxyStartEnd[1]
            cv2.rectangle(SmallVidROI,(CenterOfTemplateX-5,CenterOfTemplateY-5),(CenterOfTemplateX+5,CenterOfTemplateY+5),(0,0,255),2)

        #in the case that the template matching has not enabled yet, essentially on the first few hits of the detector
        elif UsingTemplate is False and New is not 0:
            CenterOfTemplateX= int((TopLeftPoints[0]+Toplength/2)/WindowScale)+ROIxyStartEnd[0]
            CenterOfTemplateY=int((TopLeftPoints[1]+Sidelength/2)/WindowScale)+ROIxyStartEnd[1]
            # print 'center of template', CenterOfTemplateX,CenterOfTemplateY
            # print 'top left points', TopLeftPoints
            cv2.rectangle(smallfull,(CenterOfTemplateX/2-5,CenterOfTemplateY/2-5),(CenterOfTemplateX/2+5,CenterOfTemplateY/2+5),(0,255,0),2)


        #if both miss, add one to complete
        if TrackMissCounter >0 and DetectMissCounter>0:
            CompleteMissCounter+=1
            #on next iteration, if one of them hits,reset
        elif TrackMissCounter is 0 or DetectMissCounter is 0:
            CompleteMissCounter=0

        #if missing for longer than a second, reset to middle
        if CompleteMissCounter>40:
            (CenterOfTemplateX, CenterOfTemplateY, CompleteMissCounter, Old, New, BoxFound, UsingTemplate, CurrentTemplate,InitialMissCounter)=ResetToBeginning()
        if UsingTemplate is False and InitialMissCounter > 90:
            (CenterOfTemplateX, CenterOfTemplateY, CompleteMissCounter, Old, New, BoxFound, UsingTemplate,CurrentTemplate,InitialMissCounter) = ResetToBeginning()


        cv2.imshow('dilate',dilate)
        cv2.imshow('houghdfinal', HoughDFinal)
        cv2.imshow('ROI', ROIResized)
        cv2.imshow('HoughSelected',HoughSelected)
        cv2.imshow('houghd', HoughD)
        cv2.imshow('full',smallfull)

        if CenterOfTemplateX>WidthOfMainVideo/2 and UsingTemplate==True:
            print "Target off by %d pixels to the Right" % (CenterOfTemplateX-WidthOfMainVideo/2)
        elif CenterOfTemplateX<WidthOfMainVideo/2 and UsingTemplate ==True:
            print "Target off by %d pixels to the Left" % (WidthOfMainVideo/2-CenterOfTemplateX)

        if TemplateWidth is not 0:
            FullWidthInPixels=TemplateWidth/WindowScale
            TrailerWidth=2.37#meters
            PixelWidthRatio=2.37/float(FullWidthInPixels) #units meters per pixel
            DistanceToTruck=PixelWidthRatio*float(FocalLength)
            print "Approximate Distance To Truck:", round(DistanceToTruck,2)," meters" #% DistanceToTruck

	
	if UsingTemplate==True and TemplateWidth is not 0 and TrackMissCounter is 0:
		LateralError=(CenterOfTemplateX-WidthOfMainVideo/2)/float(FullWidthInPixels)
		LateralError=round(LateralError,2)
		LongDist=round(DistanceToTruck,2)

	point = Point32()	
	point.x=LateralError
	point.y=LongDist
	pub.publish(point)


        k=cv2.waitKey(1) & 0xFF
	if k==27 or rospy.is_shutdown():
		cv2.destroyAllWindows()
		cap.release		
		break


        # if k==27:
        #     (CenterOfTemplateX, CenterOfTemplateY, CompleteMissCounter, Old, New, BoxFound, UsingTemplate, CurrentTemplate)=ResetToBeginning()
