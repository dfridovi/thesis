"""
Skeleton UI for load manager.
"""

import sys
from PyQt4 import QtGui

class LoadManagerUI(QtGui.QWidget):
    
    def __init__(self):
        super(LoadManagerUI, self).__init__()
        self.initUI()
                
    def initUI(self):      

        FRAME_H = 600
        FRAME_W = 600

        self.red = QtGui.QColor(255, 0, 0)
        self.green = QtGui.QColor(0, 255, 0)

        self.square1 = QtGui.QFrame(self)
        self.square1.setGeometry(FRAME_H - 20, FRAME_H, 100, 100)
        self.square1.setStyleSheet("QWidget { background-color: %s }" %  
            self.red.name())

        self.square2 = QtGui.QFrame(self)
        self.square2.setGeometry(150, 20, 100, 100)
        self.square2.setStyleSheet("QWidget { background-color: %s }" %  
            self.green.name())
        
        self.setGeometry(300, 300, FRAME_H, FRAME_W)
        self.setWindowTitle('Toggle button')
        self.show()
        
        
    def setColor(self, pressed):
        
        source = self.sender()
        
        if pressed:
            val = 255
        else: val = 0
                        
        if source.text() == "Red":
            self.col.setRed(val)                
        elif source.text() == "Green":
            self.col.setGreen(val)             
        else:
            self.col.setBlue(val) 
            
        self.square.setStyleSheet("QFrame { background-color: %s }" %
            self.col.name())  
            
        
def main():
    
    app = QtGui.QApplication(sys.argv)
    ex = LoadManagerUI()
    sys.exit(app.exec_())


if __name__ == '__main__':
    main()    
