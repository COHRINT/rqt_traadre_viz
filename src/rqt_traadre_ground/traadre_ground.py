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
from math import sqrt, atan, pi, degrees, floor
from sensor_msgs.msg import Image
from nav_msgs.msg import *
from geometry_msgs.msg import *
from traadre_msgs.msg import *
from traadre_msgs.srv import *

from PyQt5.QtCore import *
from PyQt5.QtGui import *
from PyQt5.QtWidgets import *

from rqt_py_common.topic_helpers import get_field_type

import RobotIcon
import ObjectIcon
import os, csv
import rospkg
import struct
import cv2

def accepted_topic(topic):
    msg_types = [OccupancyGrid, Path, PolygonStamped, PointStamped]
    msg_type, array = get_field_type(topic)

    if not array and msg_type in msg_types:
        return True
    else:
        return False

class TraadreGroundWidget(QWidget):

    def __init__(self, map_topic='/map'):
        super(TraadreGroundWidget, self).__init__()
        self._layout = QVBoxLayout()
        self._h_layout = QHBoxLayout()
        self.setAcceptDrops(True)
        self.setWindowTitle('TRAADRE Ground Station')

        self.map = map_topic
        self._tf = tf.TransformListener()

        hNavLayout = QHBoxLayout()
        vVidLayout = QVBoxLayout()

     
        self._map_view = DEMView(map_topic, tf = self._tf, parent = self)
        #self._wordBank = HRIWordBank(parent = self)
        self._doneButton = QPushButton('Done!')
#        self._doneButton.clicked.connect(self._map_view.savePoses)
        
        #Add the word bank to the left side
        #hNavLayout.addWidget(self._wordBank)
        hNavLayout.addWidget(self._map_view)

        
        self._layout.addLayout(hNavLayout)
        self._layout.addWidget(self._doneButton)
        self.setLayout(self._layout)

        
    def dragEnterEvent(self, e):
        if not e.mimeData().hasText():
            if not hasattr(e.source(), 'selectedItems') or len(e.source().selectedItems()) == 0:
                qWarning('HRIGetPos.dragEnterEvent(): not hasattr(event.source(), selectedItems) or len(event.source().selectedItems()) == 0')
                return
            item = e.source().selectedItems()[0]
            topic_name = item.data(0, Qt.UserRole)
            if topic_name == None:
                qWarning('HRIGetPos.dragEnterEvent(): not hasattr(item, ros_topic_name_)')
                return

        else:
            topic_name = str(e.mimeData().text())

        if accepted_topic(topic_name):
            e.accept()
            e.acceptProposedAction()

    def dropEvent(self, e):
        if e.mimeData().hasText():
            topic_name = str(e.mimeData().text())
        else:
            droped_item = e.source().selectedItems()[0]
            topic_name = str(droped_item.data(0, Qt.UserRole))

        topic_type, array = get_field_type(topic_name)

        #Add the word bank icon to the map at the desired position

    def save_settings(self, plugin_settings, instance_settings):
        self._map_view.save_settings(plugin_settings, instance_settings)

    def restore_settings(self, plugin_settings, instance_settings):
        self._map_view.restore_settings(plugin_settings, instance_settings)

class DEMView(QGraphicsView):
    dem_changed = Signal()
    robot_odom_changed = Signal()
    goal_changed = Signal()
    
    def __init__(self, dem_topic='/dem',
                 tf=None, parent=None):
        super(DEMView, self).__init__()
        self._parent = parent

        self._goal_mode = True

        self.dem_changed.connect(self._update)

        self._dem_item = None
        self._goalIcon = None
        self._robotIcon = None
        
        self.setDragMode(QGraphicsView.NoDrag)

        self._addedItems = dict()
        self.w = 0
        self.h = 0

        self._colors = [(238, 34, 116), (68, 134, 252), (236, 228, 46), (102, 224, 18), (242, 156, 6), (240, 64, 10), (196, 30, 250)]
        self._scene = QGraphicsScene()

       
        self.dem_sub = rospy.Subscriber('/dem', Image, self.dem_cb)
        self.odom_sub = rospy.Subscriber('/pose', PoseStamped, self.robot_odom_cb)
        self.goal_sub = rospy.Subscriber('/current_goal', Pose2D, self.goal_cb)
        self._robotLocations = [(0,0)]
        self._goalLocations = [(0,0)]

        self.setScene(self._scene)

    def goal_cb(self, msg):
         #Resolve the odometry to a screen coordinate for display

        worldX = msg.x
        worldY = msg.y

        print 'Got Goal at: ' + str(worldX) + ',' + str(worldY)

        self._goalLocations[0] = [worldX, worldY]
        self.goal_changed.emit()

    def _updateGoal(self):
        #Redraw the goal locations
        #print 'Updating goal locations'
        #If this is the first time we've seen this robot, create its icon
        if self._goalIcon is None:
            thisGoal = RobotIcon.RobotWidget(str('X'), self._colors[1])
            thisGoal.setFont(QFont("SansSerif", max(self.h / 20.0,3), QFont.Bold))
            thisGoal.setBrush(QBrush(QColor(self._colors[1][0], self._colors[1][1], self._colors[1][2])))          
            self._goalIcon = thisGoal
            self._scene.addItem(thisGoal)

        #Pick up the world coordinates
        world = self._goalLocations[0]

        iconBounds = self._goalIcon.boundingRect()
        #Adjust the world coords so that the icon is centered on the robot, rather than top-left
        world[0] = world[0]  - iconBounds.width()/2 + 0.75
        
        world[1] = self.h - (world[1] + iconBounds.height()/2) #mirror the y coord
        print 'Drawing goal at ', world
        self._goalIcon.setPos(QPointF(world[0], world[1]))


    def robot_odom_cb(self, msg):

        #Resolve the odometry to a screen coordinate for display

        worldX = msg.pose.position.x
        worldY = msg.pose.position.y
        worldZ = msg.pose.position.z
        
        worldRoll, worldPitch, worldYaw = euler_from_quaternion([msg.pose.orientation.w,
                                                                 msg.pose.orientation.x,
                                                                 msg.pose.orientation.y,
                                                                 msg.pose.orientation.z],'sxyz')

       
        self._robotLocations[0] = [worldX, worldY, worldZ, worldRoll, worldPitch, worldYaw]
        #print 'Robot at: ', self._robotLocations[0] 
        
        self.robot_odom_changed.emit()


    def _updateRobot(self):
        #Redraw the robot locations
        #print 'Updating robot locations'
        #If this is the first time we've seen this robot, create its icon
        if self._robotIcon == None:
            thisRobot = RobotIcon.RobotWidget(str('^'), self._colors[0])
            thisRobot.setFont(QFont("SansSerif", max(self.h / 20.0,3), QFont.Bold))
            thisRobot.setBrush(QBrush(QColor(self._colors[0][0], self._colors[0][1], self._colors[0][2])))
            self._robotIcon = thisRobot
            self._scene.addItem(thisRobot)

        #Pick up the world coordinates
        world = self._robotLocations[0]

        iconBounds = self._robotIcon.boundingRect()
        #Adjust the world coords so that the icon is centered on the robot, rather than top-left
        world[0] = world[0]  - iconBounds.width()/2 + 0.75
        
        world[1] = self.h - (world[1] + iconBounds.height()/2) #mirror the y coord
        #print 'Drawing robot at ', world
        
        self._robotIcon.setPos(QPointF(world[0], world[1]))
        #print 'Rotating:', world[5]
        self._robotIcon.setRotation(-world[5]*180/math.pi + 90)

        #Set the origin so that the caret rotates around its centroid(ish)
        self._robotIcon.setTransformOriginPoint(QPoint(iconBounds.width()/2, iconBounds.height()/3))
        
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
        print 'map enter event'

    def dropEvent(self, e):
        print 'Nav drop event'
            
    def mousePressEvent(self,e):
        return
    
        p = self.mapToScene(e.x(), e.y())
        if self._parent._wordBank._currentItem:
            modelName = self._parent._wordBank._currentItem._modelName
            wordItem = self._parent._wordBank._currentItem
            if self._addedItems.has_key(modelName):
                self._scene.removeItem(self._addedItems[modelName])
                
            #Draw a pixmap of the current item on the map
            newPM = ObjectIcon.ObjectIcon(None, None, wordItem)
            self._addedItems[modelName] = newPM
            self._scene.addItem(newPM)
            #newPM.rescale(1)
            newPM.setPos(QPointF(p.x() - 50, p.y() - 50))
   
            
    def dem_cb(self, msg):
        #self.resolution = msg.info.resolution
        self.w = msg.width
        self.h = msg.height

        print 'Got DEM encoded as:', msg.encoding
        print 'message length:', len(msg.data), 'type:', type(msg.data)
        print 'width:', msg.width
        print 'height:', msg.height
        
        a = np.array(struct.unpack('<%dd' % (msg.width*msg.height), msg.data), dtype=np.float64, copy=False, order='C')

        self.demDownsample = 4
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
        super().close()
        
    def dragMoveEvent(self, e):
        print('Scene got drag move event')
        
    def resizeEvent(self, evt=None):
        #Resize map to fill window
        bounds = self._scene.sceneRect()
        if bounds:
            self.fitInView(self._scene.sceneRect(), Qt.KeepAspectRatio)
            self.centerOn(self._dem_item)
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
        self.centerOn(self._dem_item)
        self.show()

        #Allow the robot position to be drawn on the DEM 
        self.robot_odom_changed.connect(self._updateRobot)
        self.goal_changed.connect(self._updateGoal)
        

    def _mirror(self, item):
        pass
        #item.scale((-1, 1))
        #item.translate(-self.w, 0)

    def save_settings(self, plugin_settings, instance_settings):
        # TODO add any settings to be saved
        pass

    def restore_settings(self, plugin_settings, instance_settings):
        # TODO add any settings to be restored
        pass    
            
class HRIGetPos(QGraphicsView):
    map_changed = Signal()
   
    
    def __init__(self, map_topic='/map',
                 tf=None, parent=None):
        super(HRIGetPos, self).__init__()
        self._parent = parent

        self._goal_mode = True

        self.map_changed.connect(self._updateMap)
       
        
        self.setDragMode(QGraphicsView.NoDrag)

        self._map = None
        self._map_item = None
        self._addedItems = dict()
        self.w = 0
        self.h = 0

        self._colors = [(238, 34, 116), (68, 134, 252), (236, 228, 46), (102, 224, 18), (242, 156, 6), (240, 64, 10), (196, 30, 250)]
        self._scene = QGraphicsScene()

       
        self.map_sub = rospy.Subscriber('/map', OccupancyGrid, self.map_cb)

        self.setScene(self._scene)

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
        print 'map enter event'

    def dropEvent(self, e):
        print 'Nav drop event'
            
    def mousePressEvent(self,e):

        p = self.mapToScene(e.x(), e.y())
        if self._parent._wordBank._currentItem:
            modelName = self._parent._wordBank._currentItem._modelName
            wordItem = self._parent._wordBank._currentItem
            if self._addedItems.has_key(modelName):
                self._scene.removeItem(self._addedItems[modelName])
                
            #Draw a pixmap of the current item on the map
            newPM = ObjectIcon.ObjectIcon(None, None, wordItem)
            self._addedItems[modelName] = newPM
            self._scene.addItem(newPM)
            #newPM.rescale(1)
            newPM.setPos(QPointF(p.x() - 50, p.y() - 50))

    def savePoses(self, msg):
        #Save the position of each of the pixmaps for the selected objects
        #Also save object names
        rospack = rospkg.RosPack()
        print 'Working dir:' + rospack.get_path('ramrod')

        outFile = open(rospack.get_path('ramrod') + '/user_output/model_pos.txt','w')
        
        for model in self._addedItems:
            xCoord = (3100 + self._addedItems[model].x() + 50)/100
            yCoord = (3100 - self._addedItems[model].y() + 50)/100
            outFile.write(model + ',' + str(xCoord) + ',' + str(yCoord) + '\n')
        outFile.close()    
            
    def map_cb(self, msg):
        self.resolution = msg.info.resolution
        self.w = msg.info.width
        self.h = msg.info.height

        a = np.array(msg.data, dtype=np.uint8, copy=False, order='C')
        a = a.reshape((self.h, self.w))
        if self.w % 4:
            e = np.empty((self.h, 4 - self.w % 4), dtype=a.dtype, order='C')
            a = np.append(a, e, axis=1)
        image = QImage(a.reshape((a.shape[0] * a.shape[1])), self.w, self.h, QImage.Format_Indexed8)

        for i in reversed(range(101)):
            image.setColor(100 - i, qRgb(i* 2.55, i * 2.55, i * 2.55))
        image.setColor(101, qRgb(255, 0, 0))  # not used indices
        image.setColor(255, qRgb(200, 200, 200))  # color for unknown value -1
        self._map = imaeg
        
        self.map_changed.emit()

    def close(self):
        if self.map_sub:
            self.map_sub.unregister()
        super(HRIGetPos, self).close()
        
    def dragMoveEvent(self, e):
        print('Scene got drag move event')
        
    def resizeEvent(self, evt=None):
        #Resize map to fill window
        bounds = self._scene.sceneRect()
        if bounds:
            self.fitInView(self._scene.sceneRect(), Qt.KeepAspectRatio)
            self.centerOn(self._map_item)
            self.show()
 
    def _updateMap(self):
        if self._map_item:
            self._scene.removeItem(self._map_item)

        pixmap = QPixmap.fromImage(self._map)
        self._map_item = self._scene.addPixmap(pixmap.scaled(self.w*100,self.h*100))
        self._map_item.setPos(QPointF(0, 0))
        # Everything must be mirrored
        self._mirror(self._map_item)

        # Add drag and drop functionality
        self.add_dragdrop(self._map_item)

        #Resize map to fill window
        self.setSceneRect(-self.w*100, 0, self.w*100, self.h*100)

        self.fitInView(self._scene.sceneRect(), Qt.KeepAspectRatio)
        self.centerOn(self._map_item)
        self.show()


    def _mirror(self, item):
        item.scale(-1, 1)
        item.translate(-self.w, 0)

    def save_settings(self, plugin_settings, instance_settings):
        # TODO add any settings to be saved
        pass

    def restore_settings(self, plugin_settings, instance_settings):
        # TODO add any settings to be restored
        pass
