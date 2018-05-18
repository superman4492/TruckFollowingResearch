import numpy as np
import cv2
import rospy
from geometry_msgs.msg import Point32
import sys
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

TargetDist=30

# this version has been modified for video taken with pointgrey camera
#profile command: Navigate to correct folder then run:
# python -m cProfile -s tottime TruckDetectPointgrey.py > profile2hp.text 2>&1



def StackLeft(CurrentPoints,LeftPoints):
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
        # if LeftPoints.size == 4:
        #     # now we want to define the points such that x1 corresponds to the top coordinate
        if y1 <= y2:
            LeftPoints = np.vstack((LeftPoints, [x1, x2, y1, y2]))  # make the first row
        elif y1 > y2:
            LeftPoints = np.vstack((LeftPoints, [x2, x1, y2, y1]))

        #
        #     # now, if row 1 is greater than row 2, switch
        #     if abs(LeftPoints[0, 0] + LeftPoints[0, 1]) / 2 >= abs(LeftPoints[1, 0] + LeftPoints[1, 1]) / 2:
        #         LeftPoints[1, :] = LeftPoints[0, :]
        #         if y1 <= y2:
        #             LeftPoints[0, :] = np.array([x1, x2, y1, y2])
        #         elif y1 > y2:
        #             LeftPoints[0, :] = np.array([x2, x1, y2, y1])
        #
        #
        #
        #
        # elif LeftPoints.size > 4:  # if there is already 2 lines
        #
        #     # if the new line is left of both the old lines
        #     if abs(LeftPoints[1, 0] + LeftPoints[1, 1]) / 2 >= abs(x1 + x2) / 2 \
        #             and abs(LeftPoints[0, 0] + LeftPoints[0, 1]) / 2 >= abs(x1 + x2) / 2:
        #         # move the old line down to position 2
        #         LeftPoints[1, :] = LeftPoints[0, :]
        #         # place new line in row 1
        #         # now we want to define the points such that x1 corresponds to the top coordinate
        #         if y1 <= y2:
        #             LeftPoints[0, :] = np.array([x1, x2, y1, y2])
        #         elif y1 > y2:
        #             LeftPoints[0, :] = np.array([x2, x1, y2, y1])
        #
        #
        #     # else if new line is between previous 2 lines
        #     elif abs(LeftPoints[1, 0] + LeftPoints[1, 1]) / 2 >= abs(x1 + x2) / 2 \
        #             and abs(LeftPoints[0, 0] + LeftPoints[0, 1]) / 2 <= abs(x1 + x2) / 2:
        #         # replace line 2 with new line
        #         # now we want to define the points such that x1 corresponds to the top coordinate
        #         if y1 <= y2:
        #             LeftPoints[1, :] = np.array([x1, x2, y1, y2])
        #         elif y1 > y2:
        #             LeftPoints[1, :] = np.array([x2, x1, y2, y1])

    return LeftPoints

def StackRight(CurrentPoints,RightPoints):
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
        # if RightPoints.size == 4:
        #     # now we want to define the points such that x1 corresponds to the top coordinate
        if y1 <= y2:
            RightPoints = np.vstack((RightPoints, [x1, x2, y1, y2]))  # make the first row
        elif y1 > y2:
            RightPoints = np.vstack((RightPoints, [x2, x1, y2, y1]))
        #     # now, if row 2 is greater than row 1, switch
        #     if abs(RightPoints[0, 0] + RightPoints[0, 1]) / 2 <= abs(x1 + x2) / 2:
        #         RightPoints[1, :] = RightPoints[0, :]
        #         if y1 <= y2:
        #             RightPoints[0, :] = np.array([x1, x2, y1, y2])
        #         elif y1 > y2:
        #             RightPoints[0, :] = np.array([x2, x1, y2, y1])
        #
        # elif RightPoints.size > 4:  # if there is already 2 lines
        #
        #     # if the new line is right of both the old lines
        #     if abs(RightPoints[1, 0] + RightPoints[1, 1]) / 2 <= abs(x1 + x2) / 2 \
        #             and abs(RightPoints[0, 0] + RightPoints[0, 1]) / 2 <= abs(x1 + x2) / 2:
        #         # move the old line down to position 2
        #         RightPoints[1, :] = RightPoints[0, :]
        #         # place new line in row 1
        #         # now we want to define the points such that x1 corresponds to the top coordinate
        #         if y1 <= y2:
        #             RightPoints[0, :] = np.array([x1, x2, y1, y2])
        #         elif y1 > y2:
        #             RightPoints[0, :] = np.array([x2, x1, y2, y1])
        #
        #     # else if new line is between previous 2 lines
        #     elif abs(RightPoints[1, 0] + RightPoints[1, 1]) / 2 <= abs(x1 + x2) / 2 \
        #             and abs(RightPoints[0, 0] + RightPoints[0, 1]) / 2 >= abs(x1 + x2) / 2:
        #         # replace line 2 with new line
        #         if y1 <= y2:
        #             RightPoints[1, :] = np.array([x1, x2, y1, y2])
        #         elif y1 > y2:
        #             RightPoints[1, :] = np.array([x2, x1, y2, y1])

    return RightPoints

def OrganizeTop(CurrentPoints,TopPoints):
    x1 = CurrentPoints[0]
    x2 = CurrentPoints[1]
    y1 = CurrentPoints[2]
    y2 = CurrentPoints[3]
    if TopPoints is 0:
        TopPoints = np.array([x1, x2, y1, y2])
        TopPoints.shape = tuple((1, 4))

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



    Topx1=TopPoints[0,0]
    Topx2=TopPoints[0,1]
    Topy1=TopPoints[0,2]
    Topy2=TopPoints[0,3]
    if Topx1>Topx2:
        TopPoints[0,:]=np.array([Topx2,Topx1,Topy2,Topy1])

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

def TemplateMatch(Frame,Template,oldtemplatecx,oldtemplatecy,TrackMissCounter,ExpandSearchFlag):

    FoundTemplateAttributes=None
    maxval=0
    TemplateHeight, TemplateWidth=Template.shape[:2]
    FrameHeight,FrameWidth=Frame.shape[:2]


    if ExpandSearchFlag is False:
        SearchAreaXStart=oldtemplatecx-int(TemplateWidth*.25)#initialize search area to be very close to the original position
        SearchAreaXEnd=oldtemplatecx+int(TemplateWidth*1.25)
        SearchAreaYStart=oldtemplatecy-int(TemplateHeight*.25)
        SearchAreaYEnd=oldtemplatecy+int(TemplateHeight*1.25)

    elif ExpandSearchFlag is True:
        SearchAreaXStart=oldtemplatecx-int(TemplateWidth*.75)#initialize search area to be very close to the original position
        SearchAreaXEnd=oldtemplatecx+int(TemplateWidth*1.75)
        SearchAreaYStart=oldtemplatecy-int(TemplateHeight*.75)
        SearchAreaYEnd=oldtemplatecy+int(TemplateHeight*1.75)


    if SearchAreaXStart<0:SearchAreaXStart=0
    if SearchAreaXEnd>FrameWidth:SearchAreaXEnd=FrameWidth
    if SearchAreaYStart<0:SearchAreaYStart=0
    if SearchAreaYEnd>FrameHeight:SearchAreaYEnd=FrameHeight



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
    FinalTemplateWidth=int(TemplateWidth*FinalRatio)#new widths and heights
    FinalTemplateHeight=int(TemplateHeight*FinalRatio)


    #print "maxval", Finalmaxval

    if Finalmaxval>.6:#if best match is 80% or greater (should be all the time)
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

def GeoDistance(Point1,Point2):

    x1=Point1[0]
    x2=Point2[0]
    y1=Point1[1]
    y2=Point2[1]

    xdist=(x1-x2)**2
    ydist=(y1-y2)**2

    distance=(xdist+ydist)**0.5

    return distance

def ChooseClosestLeftandRight(LeftPoints,RightPoints,TopLine):

    if TopLine is not 0 and LeftPoints is not 0 and RightPoints is not 0:
        if LeftPoints.shape == tuple((4,)):  LeftPoints.shape = tuple((1, 4))
        if RightPoints.shape == tuple((4,)):  RightPoints.shape = tuple((1, 4))

        ClosestLeftPointIndex=100
        ClosestRightPointIndex=100
        TopLeftPoint=np.array([TopLine[0],TopLine[2]])
        TopRightPoint=np.array([TopLine[1],TopLine[3]])
        LeftError=1000000000
        RightError=1000000000

        for i in range(len(LeftPoints)):
            CurLeftPoint=np.array([LeftPoints[i,0],LeftPoints[i,2]]) # top left x,y for this line
            Distance=GeoDistance(TopLeftPoint,CurLeftPoint)
            if Distance<LeftError:
                ClosestLeftPointIndex=i
                LeftError=Distance

        for i in range(len(RightPoints)):
            CurRightPoint = np.array([RightPoints[i, 0], RightPoints[i, 2]])  # top right x,y for this line
            Distance = GeoDistance(TopRightPoint, CurRightPoint)
            if Distance < RightError:
                ClosestRightPointIndex = i
                RightError = Distance


        LeftPoints=LeftPoints[ClosestLeftPointIndex,:]
        RightPoints=RightPoints[ClosestRightPointIndex,:]

    return(LeftPoints,RightPoints)



