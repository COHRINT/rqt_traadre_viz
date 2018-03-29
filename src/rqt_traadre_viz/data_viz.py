# Software License Agreement (BSD License)
#
# Copyright (c) 2012, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Willow Garage, Inc. nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

import rospy
import tf
from tf.transformations import *

import numpy as np
import random
from math import sqrt, atan, pi, degrees, floor, atan2
from sensor_msgs.msg import *
from nav_msgs.msg import *
from cv_bridge import CvBridge, CvBridgeError

from geometry_msgs.msg import *
from traadre_msgs.msg import *
from traadre_msgs.srv import *

from PyQt5.QtCore import *
from PyQt5.QtGui import *
from PyQt5.QtWidgets import *

from rqt_py_common.topic_helpers import get_field_type

from QLabeledValue import *
import RobotIcon
import ObjectIcon
import QArrow
import QPolicyDirection
from Location import * 

import os, csv
import rospkg
import struct
import cv2
import copy
from Queue import *
from collections import *

def accepted_topic(topic):
    msg_types = [OccupancyGrid, Path, PolygonStamped, PointStamped]
    msg_type, array = get_field_type(topic)

    if not array and msg_type in msg_types:
        return True
    else:
        return False

class DataVizWidget(QSplitter):
    robot_state_changed = Signal()
    goal_changed = Signal()
    goalList_changed = Signal(str)
    
    def __init__(self, map_topic='/map'):
        super(DataVizWidget, self).__init__()

        self._layout = QVBoxLayout()
        self._h_layout = QHBoxLayout()
        self.setAcceptDrops(True)
        self.setWindowTitle('TRAADRE Viz')
        
        self.map = map_topic
        self._tf = tf.TransformListener()

        vNavLayout = QVBoxLayout()
     
        self._map_view = DEMView(map_topic, tf = self._tf, parent = self)
        #self._wordBank = HRIWordBank(parent = self)
        self._doneButton = QPushButton('Done!')
#        self._doneButton.clicked.connect(self._map_view.savePoses)
        
        #Add the word bank to the left side
        #hNavLayout.addWidget(self._wordBank)
        self.setOrientation(Qt.Vertical)
        self.addWidget(self._map_view)


        hNavLayout = QHBoxLayout()
               
        poseGroup = QGroupBox("Current Pose")
        poseLayout = QVBoxLayout()
        
        traverseGroup = QGroupBox("Traverses")
        traverseLayout = QVBoxLayout()

        goalGroup = QGroupBox('Goals')
        goalLayout = QVBoxLayout()

        optionsGroup = QGroupBox("Options")
        optionsLayout = QVBoxLayout()
        
        self.poseLabels = [QLabeledValue("X"),
                QLabeledValue("Y"),
                QLabeledValue("Z"),
                QLabeledValue("Roll"),
                QLabeledValue("Pitch"),
                QLabeledValue("Yaw")]
        poseLayout.setSpacing(0)
        for label in self.poseLabels:
            poseLayout.addWidget(label)

        poseGroup.setLayout(poseLayout)
        hNavLayout.addWidget(poseGroup)

        self.goalList = QListView()
        self.goalList.setMaximumHeight(self.height() / 5)
        self.goalList.clicked.connect(self.lstGoal_clicked)
        self.goalModel = QStandardItemModel(self.goalList)
        self.goalModel.itemChanged.connect(self.lstGoal_itemChanged)
        
        goalLayout.addWidget(self.goalList)
        self.btnLoadPolicy = QPushButton('Load policy')
        self.btnLoadPolicy.clicked.connect(self.btnLoadPolicy_onclick)
        self.btnLoadPolicy.setDisabled(True)
        goalLayout.addWidget(self.btnLoadPolicy)

        self.chkPolicyVisible = QCheckBox('Show policy?')
        self.chkPolicyVisible.stateChanged.connect(self.chkPolicyVisible_stateChanged)
        self.chkPolicyVisible.setCheckState(Qt.Checked)
        goalLayout.addWidget(self.chkPolicyVisible)
        
        goalGroup.setLayout(goalLayout)
        
        self.traverseList = QListView()
        self.traverseList.setMaximumHeight(self.height() / 5)
        
        self.traverseModel = QStandardItemModel(self.traverseList)
        self.traverseModel.itemChanged.connect(self.lstTraverse_itemChanged)
        traverseLayout.addWidget(self.traverseList)
        self.btnNewTraverse = QPushButton('New traverse')
        self.btnNewTraverse.clicked.connect(self.btnNewTraverse_onclick)
        
        traverseLayout.addWidget(self.btnNewTraverse)
        traverseGroup.setLayout(traverseLayout)

        btnSave = QPushButton('Save')
        optionsLayout.addWidget(btnSave)
        btnSave.clicked.connect(self.btnSave_clicked)
        optionsGroup.setLayout(optionsLayout)
        
        hNavLayout.addWidget(goalGroup)
        hNavLayout.addWidget(traverseGroup)
        hNavLayout.addWidget(optionsGroup)
        
        vWidget = QWidget()
        vWidget.setLayout(hNavLayout)
        self.addWidget(vWidget)
        
        #vNavLayout.addWidget(vWidget)
        #self._layout.addWidget(vNavLayout)
        #self._layout.addWidget(self._doneButton)
        #self.setLayout(vNavLayout)
        
        #self.robot_state_changed.connect(self._updateState)
        self.stateTimer = QTimer()
        self.stateTimer.timeout.connect(self._updateState)
        self.stateTimer.start(100)
        
        self.btnNewTraverse_onclick() #start with an active traverse
        self.odom_sub = rospy.Subscriber('state', RobotState, self.robot_state_cb)

        #Resize the splitter thing
        self.setSizes([self.height()*0.7, self.height()*0.3])
        self.stateQueue = deque() #Queue() #to handle high volume state updates from bag playback

        self.addGoals(self.getGoals())

    def btnSave_clicked(self):
        
        fileName = QFileDialog.getSaveFileName(caption='Save File',
                                               initialFilter='view.png',
                                               filter='JPEG (*.jpg);;PNG( *.png)')
        print 'Saving to: ', fileName
        #self._map_view.setSceneRect(self._map_view._scene.itemsBoundingRect())
        sceneImage = QImage(self._map_view.sceneRect().size().toSize(), QImage.Format_ARGB32)
        sceneImage.fill(Qt.transparent)
        painter = QPainter(sceneImage)

        self._map_view._scene.render(painter)
        painter.end()
        sceneImage.save(fileName[0])
        
        #pixMap = self._map_view.grab()
        #pixMap.save(str(fileName[0]), '' ,100)
        
    def chkPolicyVisible_stateChanged(self, state):
        self._map_view.setPolicyVisible(False if state == Qt.Unchecked else True)

    def getGoals(self):
        try:
            goal = rospy.ServiceProxy('/policy/policy_server/GetGoalList', GetGoalList)
	    response = goal()
	    row = response.goals
	    return zip(response.ids, row)
	except rospy.ServiceException, e:
	    print "Service call failed: %s"%e

    def addGoals(self, goalList):
        allGoals = sorted(goalList, key=lambda param: param[0]) #Sort the combined list by the goal ID
        
        for goal in allGoals:
            item = QStandardItem()
            itemKey = str(goal[0])
            item.setText(itemKey)
            item.setCheckable(True)
            item.setCheckState(Qt.Checked)
        
            self.goalModel.appendRow(item)
            self.goalList.setModel(self.goalModel)
            self._map_view.addGoal(goal[0], goal[1].x, goal[1].y)
            #            self._map_view.goals[itemKey] = goal[1]
            self._map_view.goalVisible[itemKey] = True

    def btnLoadPolicy_onclick(self):
        item = self.lstGoal_selectedItem
        goalID = self.goalModel.item(item.row()).text()
        print 'Loading new policy for goal:', goalID
        #Make a service call to select the goal so that the policy is published
        self.setCurrentGoal(goalID)

    def btnNewTraverse_onclick(self):
        #Add a new item to the list box model for this traverse
        item = QStandardItem()
        itemKey = str(len(self._map_view.traverses.keys()))
        item.setText(itemKey)
        item.setCheckable(True)
        item.setCheckState(Qt.Checked)
        
        self.traverseModel.appendRow(item)
        self.traverseList.setModel(self.traverseModel)
        self._map_view.traverses[int(itemKey)] = []
        self._map_view.traverseVisible[int(itemKey)] = True

    def lstTraverse_itemChanged(self, item):
        print 'Item changed!', item.text(), ' ', item.checkState()
        self._map_view.setTraverseState(int(item.text()), True if item.checkState() == 2 else False)

    def lstGoal_clicked(self, item):
        self.lstGoal_selectedItem = item
        self.btnLoadPolicy.setDisabled(False)
       
        
    def setCurrentGoal(self,id):
	try:
	    goal = rospy.ServiceProxy('/policy/policy_server/SetCurrentGoal', SetCurrentGoal)
            response = goal(id)
	    return response.goal
	except rospy.ServiceException, e:
	    print "Service call failed: %s"%e
            
    def lstGoal_itemChanged(self, item):
        self.btnLoadPolicy.setDisabled(False)
        self._map_view.setGoalState(item.text(), True if item.checkState() == 2 else False)
        
    def _updateState(self):

        #process every message in the queue so far:
        #print 'State queue size:', len(self.stateQueue)
        while True:
            try:
                self._robotState = self.stateQueue.pop()
            except IndexError, e:
                return

            for idx, val in enumerate(self._robotState):
                self.poseLabels[idx].updateValue(val)

                #self.fuelLabel.updateValue(self._robotFuel)

                #Ping unity with a steer if enabled
                '''
                if not self.lastSteerMsg is None:
                self.steer_pub.publish(self.lastSteerMsg)
                '''
                #Add to the current traverse

            currentIndex = len(self._map_view.traverses.keys()) - 1
            if currentIndex >= 0:
                self._map_view.addTraversePoint(currentIndex, (self._robotState[0], self._robotState[1]))

    def _updateSteer(self, steer):
        steerMsg = Steering()
        steerMsg.header.stamp = rospy.Time.now()
        steerMsg.steer= steer  * 180 / math.pi

        steerMsg.id = self._goal[0]
        steerMsg.goal.x = self._goal[1]
        steerMsg.goal.y = self._goal[2]
        self.lastSteerMsg = steerMsg
        
        self.steer_pub.publish(steerMsg)
        
        #print 'Updating main widgets to ', steer
    

        
    def robot_state_cb(self, msg):
        #Resolve the odometry to a screen coordinate for display

        worldX = msg.pose.position.x
        worldY = msg.pose.position.y
        worldZ = msg.pose.position.z

        
        worldRoll, worldPitch, worldYaw = euler_from_quaternion([msg.pose.orientation.x,
                                                                 msg.pose.orientation.y,
                                                                 msg.pose.orientation.z,
                                                                 msg.pose.orientation.w],'sxyz')


        #TODO: Wrap the inv rpy to [-pi, pi]
        #print 'Orientation:', msg.pose.orientation, ' RPY:', worldRoll, worldPitch, worldYaw
        
        self.stateQueue.appendleft([worldX, worldY, worldZ, worldRoll, worldPitch, worldYaw])
        
        #print 'Robot State:', self._robotState
        #self._robotFuel = msg.fuel

        self.robot_state_changed.emit()
        
    def goal_cb(self, msg):
         #Resolve the odometry to a screen coordinate for display

        worldX = msg.pose.x
        worldY = msg.pose.y

        print 'Got Goal at: ' + str(worldX) + ',' + str(worldY)

        self._goal = [msg.id, worldX, worldY]
        self.goal_changed.emit()
        
    def save_settings(self, plugin_settings, instance_settings):
        self._map_view.save_settings(plugin_settings, instance_settings)

    def restore_settings(self, plugin_settings, instance_settings):
        self._map_view.restore_settings(plugin_settings, instance_settings)
        
class OccupancyMember(object):
    def __init__(self, m_x=0, m_y=0, m_traverseID=0):
        self.x = m_x
        self.y = m_y
        self.traverseID = m_traverseID
        
class DEMView(QGraphicsView):
    dem_changed = Signal()
    robot_odom_changed = Signal()
    
    hazmap_changed = Signal()
    policy_changed = Signal(Policy)
    
    def __init__(self, dem_topic='dem',
                 tf=None, parent=None):
        super(DEMView, self).__init__()
        self._parent = parent
        self.demDownsample = 4
        self.dem_changed.connect(self._update)
        self.hazmap_changed.connect(self._updateHazmap)
        
        self._dem_item = None
        self._robotIcon = None
        
        self.setDragMode(QGraphicsView.NoDrag)

        self._addedItems = dict()
        self.w = 4000
        self.h = 4000

        self._colors = [(238, 34, 116),
                        (68, 134, 252),
                        (236, 228, 46),
                        (102, 224, 18),
                        (242, 156, 6),
                        (240, 64, 10),
                        (196, 30, 250),
                        (int(random.random()*255),int(random.random()*255),int(random.random()*255)),
                        (int(random.random()*255),int(random.random()*255),int(random.random()*255)),
                        (int(random.random()*255),int(random.random()*255),int(random.random()*255)),
                        (int(random.random()*255),int(random.random()*255),int(random.random()*255)),
                        (int(random.random()*255),int(random.random()*255),int(random.random()*255)) ]
        
        self._scene = QGraphicsScene()
        self.setScene(self._scene)
        
        self.traverses = dict()
        self.traverseVisible = dict()
        
        self.dem_sub = rospy.Subscriber('dem', Image, self.dem_cb)
        self.odom_sub = rospy.Subscriber('state', RobotState, self.robot_odom_cb)

        self.policy_changed.connect(self.updatePolicy)
        
        self._robotLocation = [0,0,0,0,0,0]
        self.goalIcons = dict()
        self.goalVisible = dict()
        
        self.setAcceptDrops(True)
        self.policyArrows = None
        
    def updatePolicy(self, msg):
        print 'Got new policy to draw'
        '''
        Drawing a policy:
        1. Figure out the scale for the arrows to overlay enough QPolicyDirections: demview.width / demDownsampled items
        2. Center each QPolicyDirection over the midpoint
        3. Set the transformOrigin
        4. Rotate according to the policy enumeration
        '''

        actionMap_raw = np.array(struct.unpack('<%dB' % (msg.width*msg.height), msg.policy), dtype=np.uint8, copy=False, order='C')

        actionMap = np.reshape(actionMap_raw, (msg.height, msg.width))
        
        scaledHeight = self.h // actionMap.shape[0]
        scaledWidth = self.w // actionMap.shape[1]
        print 'Base size:', (self.w, self.h)
        print 'Scaled size:', (scaledWidth, scaledHeight)

        if not self.policyArrows is None: 
            for row in range(0, len(self.policyArrows)):
                for col in range(0,len(self.policyArrows[0])):
                    self._scene.removeItem(self.policyArrows[row][col])
                
        self.policyArrows = []
        
        for row in range(0, actionMap.shape[1]):
            self.policyArrows.append([])
            for col in range(0, actionMap.shape[0]):
                thisDir = QPolicyDirection.QPolicyDirection(color=Qt.white)
                self.policyArrows[row].append(thisDir)
                self._scene.addItem(thisDir)

                bounds = thisDir.boundingRect()
                scale_w = scaledWidth / bounds.width()
                scale_h = scaledHeight / bounds.height()
                thisDir.setTransformOriginPoint(QPointF(bounds.width() / 2, bounds.height() / 2))

                
                thisDir.setScale(scale_w)
                bounds = thisDir.boundingRect()
                thisDir.setPos(QPointF(col * scaledWidth - scaledWidth/2, row * scaledHeight - scaledHeight/2))

                #Rotate to match the policy:
                polVal = Location(actionMap[row][col])
                
                #Convert to degrees:
                if(polVal == Location.Forward):
                    polAngle = 0.0
                elif(polVal == Location.FwdRight):
                    polAngle = 45.0
                elif(polVal == Location.Right):
                    polAngle = 90.0
                elif(polVal == Location.BackRight):
                    polAngle = 135.0
                elif(polVal == Location.Back):
                    polAngle = 180.0
                elif(polVal == Location.BackLeft):
                    polAngle = -135.0
                elif(polVal == Location.Left):
                    polAngle = -90.0
                elif(polVal == Location.FwdLeft):
                    polAngle = -45.0
                else:
                    #print 'Unknown policy value:', polVal
                    #Current = goal
                    thisDir.setVisible(False)
                    polAngle = 0.0
                thisDir.setTransformationMode(Qt.SmoothTransformation)
                thisDir.setRotation(polAngle)
                
    def setPolicyVisible(self, state):
        if not self.policyArrows is None: 
            for row in range(0, len(self.policyArrows)):
                for col in range(0,len(self.policyArrows[0])):
                    self.policyArrows[row][col].setVisible(state)

    def policy_cb(self, msg):
        self.policy_changed.emit(msg)
        
    def l2dist(self, src, dest):
        return math.sqrt((dest[0] - src[0])**2 + (dest[1] - src[1]) ** 2)
    
    def addTraversePoint(self, index, point):
        theColor = QColor(self._colors[index][0], self._colors[index][1], self._colors[index][2])
        theColor.setAlpha(80)
        thePen = QPen(theColor)
        thePen.setWidthF(5)

        theBrush = QBrush(theColor)
        theBrush.setStyle(Qt.SolidPattern)
        
        if self._dem_item is None:
            return

        point_scaled = (point[0] / self.demDownsample, point[1] / self.demDownsample)
        
        if len(self.traverses[index]) == 0:
            
            marker = self._scene.addEllipse(0.0, 0.0, 1.0, 1.0, thePen, theBrush)
            bounds = marker.boundingRect()
            
            #marker.setPos(QPointF(point_scaled[0] - bounds.width()/2, point_scaled[1] - bounds.height()/2))
            marker.setPos(QPointF(point_scaled[0], point_scaled[1]))
            
            self.traverses[index].append(marker)
            marker.setVisible(self.traverseVisible[index])

        elif self.l2dist((self.traverses[index][-1].x(), self.traverses[index][-1].y()),
                         point_scaled) > 1.0:
            marker = self._scene.addEllipse(0.0, 0.0, 1.0, 1.0, thePen, theBrush)
            bounds = marker.boundingRect()
            
            marker.setPos(QPointF(point_scaled[0], point_scaled[1]))
            #marker.setTransformOriginPoint(QPoint(bounds.width()/2, bounds.height()/2))
            self.traverses[index].append(marker)
            marker.setVisible(self.traverseVisible[index])

    def setGoalState(self, index, state):
        #Alter the visibility of the given goal
        if index not in self.goalVisible.keys():
            print 'DEMView: Goal index ', index, ' not in:', self.goalVisible.keys()
            return

        print 'Setting goal ', index, ' to visible state:', state
        self.goalVisible[index] = state
        
        #Go through the GraphicsItems that make up the goals and set their visibility as needed:
        self.goalIcons[index].setVisible(state)

            
    def setTraverseState(self, index, state):
        #Alter the visibility of the given traverse
        if index >= len(self.traverses):
            print 'DEMView: Traverse index ', index, ' out of bounds:', len(self.traverses)
            return

        print 'Setting traverse ', index, ' to visible state:', state
        self.traverseVisible[index] = state
        
        #Go through the GraphicsItems that make up the traverse and set their visibility as needed:
        for item in self.traverses[index]:
            item.setVisible(state)
    
    def addGoal(self, id, x, y):
        #Draw a new goal icon per the directions...
        if id not in self.goalIcons.keys():
            thisGoal = RobotIcon.RobotWidget(str(id), QColor(self._colors[1][0], self._colors[1][1], self._colors[1][2]))
            thisGoal.setFont(QFont("SansSerif", max(self.h / 20.0,3), QFont.Bold))
            thisGoal.setBrush(QBrush(QColor(self._colors[1][0], self._colors[1][1], self._colors[1][2])))          
            self.goalIcons[id] = thisGoal
            #self._scene.addItem(thisGoal)
            #Update the label's text:
            self.goalIcons[id].setText(str(id))
            
            #Pick up the world coordinates
            world = [x, y]
            #print 'Goal raw coord:', world
            iconBounds = self.goalIcons[id].boundingRect()

            world[0] /= self.demDownsample
            world[1] /= self.demDownsample
            
            #Adjust the world coords so that the icon is centered on the goal
            #Since the font may change, we adjust this later
            #world[0] = world[0] - iconBounds.width()/2 
            #world[1] = world[1] - iconBounds.height()/2 #mirror the y coord
            
            #print 'Drawing goal ', id, ' at ', world
            self.goalIcons[id].setPos(QPointF(world[0], world[1]))
            self.goalVisible[id] = True
            
    def robot_odom_cb(self, msg):

        #Resolve the odometry to a screen coordinate for display from a RobotState message

        worldX = msg.pose.position.x
        worldY = msg.pose.position.y
        worldZ = msg.pose.position.z
        
        worldRoll, worldPitch, worldYaw = euler_from_quaternion([msg.pose.orientation.x,
                                                                 msg.pose.orientation.y,
                                                                 msg.pose.orientation.z,
                                                                 msg.pose.orientation.w],'sxyz')

       
        self._robotLocation = [worldX, worldY, worldZ, worldRoll, worldPitch, worldYaw]
        #print 'Robot at: ', self._robotLocations[0] 
        
        self.robot_odom_changed.emit()
        
    def _updateRobot(self):
        #Redraw the robot locations
        #print 'Updating robot locations'
        #If this is the first time we've seen this robot, create its icon
        if self._robotIcon == None:
            #thisRobot = RobotIcon.RobotWidget('R', color=QColor(self._colors[0][0], self._colors[0][1], self._colors[0][2]))
            #thisRobot.setFont(QFont("SansSerif", max(self.h / 20.0,3), QFont.Bold))
            #thisRobot.setBrush(QBrush(QColor(self._colors[0][0], self._colors[0][1], self._colors[0][2])))
            thisRobot = QArrow.QArrow(color=QColor(self._colors[0][0], self._colors[0][1], self._colors[0][2]))
            self._robotIcon = thisRobot
            self._scene.addItem(thisRobot)
            
        #Pick up the world coordinates - copy so we can change in place
        world = copy.deepcopy(self._robotLocation)
        #print 'Raw robot loc: ', world[0], world[1]
        
        iconBounds = self._robotIcon.boundingRect()

        world[0] /= self.demDownsample
        world[1] /= self.demDownsample
        
        #Adjust the world coords so that the icon is centered on the robot, rather than top-left
        world[0] = world[0] - iconBounds.width()/2 
        world[1] = world[1] - iconBounds.height()/2 #mirror the y coord

        #print 'Drawing robot at ', world[0], world[1]
        
        self._robotIcon.setPos(QPointF(world[0], world[1]))
        
        #print 'Rotating:', world[5]
        self._robotIcon.setRotation(world[5]*180/math.pi + 90)
        
        #Set the origin so that the caret rotates around its center(ish)
        self._robotIcon.setTransformOriginPoint(QPoint(iconBounds.width()/2, iconBounds.height()/2))

        #print 'Transform origin:', self._robotIcon.transformOriginPoint()
                          
    def add_dragdrop(self, item):
        # Add drag and drop functionality to all the items in the view
        def c(x, e):
            self.dragEnterEvent(e)
        def d(x, e):
            self.dropEvent(e)
        item.setAcceptDrops(True)
        item.dragEnterEvent = c
        item.dropEvent = d

    def dragEnterEvent(self, e):
        print 'map enter event:'
        print 'text:', e.mimeData().hasText()
        print 'url:', e.mimeData().hasUrls()
        print 'html:', e.mimeData().hasHtml()
        e.accept()
        
    def dropEvent(self, e):
        print 'map drop event:', e
        print 'text:', e.mimeData().hasText()
        print 'url:', e.mimeData().hasUrls()
        print 'html:', e.mimeData().hasHtml()
        
    def mousePressEvent(self,e):
        return
    
    def hazmap_cb(self, msg):
        #Unlike the dem, the hazmap is pretty standard - gray8 image
        hazmap = CvBridge().imgmsg_to_cv2(msg, desired_encoding="passthrough")

        self.hazmapImage = QImage(hazmap, msg.width, msg.height, QImage.Format_Grayscale8)
        self.hazmap_changed.emit()

    def _updateHazmap(self):
        print 'Rendering hazmap'

        hazTrans = QImage(self.hazmapImage.width(), self.hazmapImage.height(), QImage.Format_ARGB32)
        hazTrans.fill(Qt.transparent)
        
        for row in range(0, self.hazmapImage.height()):
            for col in range(0, self.hazmapImage.width()):
                #Change the colormap to be clear for clear areas, red translucent for obstacles
                pixColor = self.hazmapImage.pixelColor(col, row)

                if pixColor.rgba() == 0xff000000:
                    hazTrans.setPixelColor(col, row, QColor(255, 0, 0, 32))
                else:
                    hazTrans.setPixelColor(col, row, QColor(0, 0, 0, 0))

        self.hazmapItem = self._scene.addPixmap(QPixmap.fromImage(hazTrans)) #.scaled(self.w*100,self.h*100))
        self.hazmapItem.setPos(QPointF(0, 0))
        trans = QTransform()
        #print 'Translating by:', bounds.width()
        
        trans.scale(self.w/hazTrans.width(),self.h/hazTrans.height())
        #trans.translate(0, -bounds.height())
        self.hazmapItem.setTransform(trans)
        
        # Everything must be mirrored
        #self._mirror(self.hazmapItem)
        
    def dem_cb(self, msg):
        #self.resolution = msg.info.resolution
        self.w = msg.width
        self.h = msg.height

        print 'Got DEM encoded as:', msg.encoding
        print 'message length:', len(msg.data), 'type:', type(msg.data)
        print 'width:', msg.width
        print 'height:', msg.height
        
        a = np.array(struct.unpack('<%dd' % (msg.width*msg.height), msg.data), dtype=np.float64, copy=False, order='C')

        rawDEM = a.reshape((self.h, self.w))
        rawDEM = cv2.resize(rawDEM, (self.h//self.demDownsample, self.w//self.demDownsample), interpolation = cv2.INTER_LINEAR)
        self.h = rawDEM.shape[0]
        self.w = rawDEM.shape[1]
        
        '''
        if self.w % 4:
            e = np.zeros((self.h, 4 - self.w % 4), dtype=rawDEM.dtype, order='C')
            rawDEM = np.append(rawDEM, e, axis=1)
            self.h = rawDEM.shape[0]
            self.w = rawDEM.shape[1]
        ''' 

        #Scale to a 8-bit grayscale image:
        self.grayDEM = np.zeros(rawDEM.shape, dtype=np.uint8)
        minZ = np.min(np.min(rawDEM))
        maxZ = np.max(np.max(rawDEM))
        dynRange = maxZ - minZ

        print 'Max Z:', maxZ
        print 'Min Z:', minZ
        
        for i in range(0, self.h):
            for j in range(0, self.w):
                self.grayDEM[i][j] = (rawDEM[i][j] - minZ) * 255/dynRange

        #use OpenCV2 to interpolate the dem into something reasonably sized
        print 'Grayscale conversion complete'

        #Needs to be a class variable so that at QImage built on top of this numpy array has
        #a valid underlying buffer - local ars
        #self.resizedDEM = cv2.resize(self.grayDEM, (500,500), interpolation = cv2.INTER_LINEAR)

        print 'Image resize complete'
        
        self.h = self.grayDEM.shape[0]
        self.w = self.grayDEM.shape[1]
        image = QImage(self.grayDEM.reshape((self.h*self.w)), self.w, self.h, QImage.Format_Grayscale8)

        #        for i in reversed(range(101)):
        #            image.setColor(100 - i, qRgb(i* 2.55, i * 2.55, i * 2.55))
        #        image.setColor(101, qRgb(255, 0, 0))  # not used indices
        #        image.setColor(255, qRgb(200, 200, 200))  # color for unknown value -1

        self._dem = image       
        self.dem_changed.emit()

    def close(self):

        if self.dem_sub:
            self.dem_sub.unregister()
        if self.pose_sub:
            self.pose_sub.unregister()
            
        super().close()
            
    def mousePressEvent(self, e):
        scenePt = self.mapToScene(e.pos())
        print 'Click at:', scenePt.x(), ' ', scenePt.y()
        print 'Sizes:', self.parent().sizes()
        
    def dragMoveEvent(self, e):
        print('Scene got drag move event')
        
    def resizeEvent(self, evt=None):
        #Resize map to fill window
        scale = 1
        bounds = self._scene.sceneRect()
        if bounds:
            self._scene.setSceneRect(0, 0, self.w*scale, self.h*scale)
            self.fitInView(self._scene.sceneRect(), Qt.KeepAspectRatio)
            #self.centerOn(self._dem_item)
            self.show()
 
    def _update(self):
        if self._dem_item:
            self._scene.removeItem(self._dem_item)

        pixmap = QPixmap.fromImage(self._dem)
        self._dem_item = self._scene.addPixmap(pixmap) #.scaled(self.w*100,self.h*100))
        self._dem_item.setPos(QPointF(0, 0))
        # Everything must be mirrored
        #self._mirror(self._dem_item)

        # Add drag and drop functionality
        self.add_dragdrop(self._dem_item)

        #Resize map to fill window
        scale = 1
        self.setSceneRect(0, 0, self.w*scale, self.h*scale)
        self.fitInView(self._scene.sceneRect(), Qt.KeepAspectRatio)
        #self.centerOn(self._dem_item)
        self.show()
        bounds = self._scene.sceneRect()
        #print 'Bounds:', bounds
        #Allow the robot position to be drawn on the DEM 
        #self.robot_odom_changed.connect(self._updateRobot)

        #Overlay the hazmap now that the dem is loaded
        self.hazmap_sub = rospy.Subscriber('hazmap', Image, self.hazmap_cb)

        #Allow goals to be drawn as well
        #self.goalList_changed.connect(self._updateGoalList)

        #Add the policy as well
        self.policy_sub = rospy.Subscriber('policy', Policy, self.policy_cb)

        #Make sure the goals stack above this
        for item in self.goalIcons.items():
            self._scene.addItem(item[1])
            item[1].setFont(QFont("SansSerif", max(self.h / 20.0,3), QFont.Bold))
            #When the font size changes, the boundingRect changes - adjust the position:
            bounds = item[1].boundingRect()
            pos = item[1].pos()
            item[1].setPos(QPointF(pos.x() - bounds.width()/2.0, pos.y() - bounds.height()/2.0))
    
    def _mirror(self, item):
        #Get the width from the item's bounds...
        bounds = item.sceneBoundingRect()
        #print 'Bounds:', bounds
        #print 'item:', item

        trans = QTransform()
        #print 'Translating by:', bounds.width()
        
        trans.scale(1,-1)
        trans.translate(0, -bounds.height())
        item.setTransform(trans)

        #bounds = item.sceneBoundingRect()
        #print 'Bounds:', bounds

    def save_settings(self, plugin_settings, instance_settings):
        # TODO add any settings to be saved
        pass

    def restore_settings(self, plugin_settings, instance_settings):
        # TODO add any settings to be restored
        pass    

