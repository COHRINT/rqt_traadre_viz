from PyQt5.QtGui import *
from PyQt5.QtCore import *
from PyQt5.QtWidgets import *

class ObjectIcon(QGraphicsPixmapItem):
    def __init__(self, filePath, modelName, src=None):
        if src:
            self.initCopy(src)
            return
        
        self._modelName = modelName
        self.theImage = QImage(filePath + modelName + '.png')
        thePM = QPixmap(self.theImage)
        QGraphicsPixmapItem.__init__(self, thePM.scaled(100,100))
        self._selected = False
        self._color = QColor(Qt.green)
        self._color.setAlpha(100)

    def initCopy(self,src):
        self._modelName = src._modelName
        self._selected = False
        self._color = src._color
        QGraphicsPixmapItem.__init__(self,src.pixmap())
        
    def selected(self, m_sel):
        self._selected = m_sel
        self.update()
        
    def getPixelRGB(self, x,y):
         position = QPoint(x,  y)
         color = QColor.fromRgb(self.theImage.pixel( position ) )
         if color.isValid():
             rgbColor = '('+str(color.red())+','+str(color.green())+','+str(color.blue())+','+str(color.alpha())+')'
            
             print 'RGB: ' + rgbColor
             
    def rescale(self, scale):
        thePM = self.pixmap().scaled(scale,scale)
        self.setPixmap(thePM)
        
    def mouseReleaseEvent(self, ev):
        self.emit(SIGNAL('clicked()'))
        
    def paint(self, qp, options, widget):
        QGraphicsPixmapItem.paint(self,qp, options, widget)
        if self._selected:
            qp.setBrush(QBrush(self._color)) #light transparent version of the text color
            qp.drawRect(0,0,98,98)
        
        
