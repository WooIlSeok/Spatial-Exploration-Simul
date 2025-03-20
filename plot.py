import pyqtgraph as pg
from pyqtgraph.Qt import QtWidgets, QtGui



class Plot:
    def __init__(self, fieldArray, robot):
        # Create Qt application
        self.app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])

        self.window = pg.GraphicsLayoutWidget(show=True, title="Real-Time Image Update")
        self.window.resize(1200, 800)  # Increased window size
        self.window.setWindowTitle("PyQtGraph Plot")

        self.view1 = self.window.addViewBox(row=0, col=0)
        self.view2 = self.window.addViewBox(row=0, col=1)

        self.imageField = pg.ImageItem(fieldArray)
        self.imageMap = pg.ImageItem(robot.map)

        # Rotate images for correct orientation
        transform = QtGui.QTransform().rotate(-90)
        self.imageField.setTransform(transform)
        self.imageMap.setTransform(transform)

        self.view1.addItem(self.imageField)
        self.view2.addItem(self.imageMap)
        
        self.view1.setAspectLocked(True)
        self.view2.setAspectLocked(True)
        # (wall, empty space(path), unknown space, object's position)
        self.imageMap.setLevels([-1, 2])

    def UpdateMap(self, mapArray):

        self.imageMap.setImage(mapArray, autoLevels=False)
        QtWidgets.QApplication.processEvents()

    def End(self):
        # Run Qt event loop (blocking)
        QtWidgets.QApplication.exec_()
