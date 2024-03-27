import sys
from PyQt5 import QtWidgets
from PyQt5.QtCore import QTimer
from vtkmodules.qt.QVTKRenderWindowInteractor import QVTKRenderWindowInteractor
import vtk
import rclpy
from threading import Thread
from MainWindow import Ui_MainWindow
from lidar_visualization import LidarVisualizer
from ros_connection import LidarSubscriber

class MainWindow(QtWidgets.QMainWindow, Ui_MainWindow):
    def __init__(self, *args, **kwargs):
        super(MainWindow, self).__init__(*args, **kwargs)
        self.setupUi(self)
        self.addVTKWidget()

        self.timer = QTimer(self)
        self.timer.timeout.connect(self.updateVTK)
        self.timer.start(100)  # Update every 100 ms
        
    def addVTKWidget(self):
        self.vtkWidget = QVTKRenderWindowInteractor(self.vtk_frame)
        self.vtkWidget.setMinimumSize(300, 300)

        self.renderer = vtk.vtkRenderer()
        self.vtkWidget.GetRenderWindow().AddRenderer(self.renderer)

        self.lidarVisualizer = LidarVisualizer(self.renderer)
        self.vtkWidget.Initialize()

        camera = self.renderer.GetActiveCamera()
        camera.Zoom(0.5)
        camera.SetPosition(0, 0, 15)
        self.verticalLayout.addWidget(self.vtkWidget)

        rclpy.init()
        self.lidarSubscriber = LidarSubscriber(self.lidarVisualizer)
        self.rclpyThread = Thread(target=rclpy.spin, args=(self.lidarSubscriber,), daemon=True)
        self.rclpyThread.start()
    
    def updateVTK(self):
        self.vtkWidget.GetRenderWindow().Render()

def main():
    app = QtWidgets.QApplication(sys.argv)
    window = MainWindow()
    window.show()
    app.exec_()

    # Properly handle ROS2 shutdown
    rclpy.shutdown()

if __name__ == '__main__':
    main()