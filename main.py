import os

# Sprawdzenie, czy sesja jest uruchomiona na Waylandzie
if os.getenv('XDG_SESSION_TYPE') == 'wayland':
    # Ustawienie platformy Qt na 'xcb', aby używać X11 przez XCB (XWayland)
    os.environ['QT_QPA_PLATFORM'] = 'xcb'

import sys
from PyQt5 import QtWidgets
from PyQt5.QtCore import QTimer
from vtkmodules.qt.QVTKRenderWindowInteractor import QVTKRenderWindowInteractor
import vtk
import rclpy
from threading import Thread
from MainWindow import Ui_MainWindow  # Zaimportuj interfejs użytkownika z Qt Designer.
from lidar_visualization import LidarVisualizer
from ros_connection import LidarSubscriber

class MainWindow(QtWidgets.QMainWindow, Ui_MainWindow):
    def __init__(self, *args, **kwargs):
        super(MainWindow, self).__init__(*args, **kwargs)
        self.setupUi(self)  # Inicjalizacja interfejsu użytkownika.
        self.addVTKWidget()  # Dodaj widget VTK do okna.

        # Timer do aktualizacji widoku VTK.
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.updateVTK)
        self.timer.start(100)  # Aktualizacja co 100 ms.
        
    def addVTKWidget(self):
        # Konfiguracja widgetu VTK do wyświetlania wizualizacji.
        self.vtkWidget = QVTKRenderWindowInteractor(self.vtk_frame)
        self.vtkWidget.setMinimumSize(300, 300)

        self.renderer = vtk.vtkRenderer()
        self.vtkWidget.GetRenderWindow().AddRenderer(self.renderer)

        # Ustawienie wizualizera lidaru i kamery.
        self.lidarVisualizer = LidarVisualizer(self.renderer)
        self.vtkWidget.Initialize()

        camera = self.renderer.GetActiveCamera()
        camera.Zoom(0.5)
        camera.SetPosition(0, 0, 15)
        self.verticalLayout.addWidget(self.vtkWidget)

        # Inicjalizacja i uruchomienie wątku dla ROS2.
        rclpy.init()
        self.lidarSubscriber = LidarSubscriber(self.lidarVisualizer)
        self.rclpyThread = Thread(target=rclpy.spin, args=(self.lidarSubscriber,), daemon=True)
        self.rclpyThread.start()
    
    def updateVTK(self):
        # Renderowanie sceny VTK.
        self.vtkWidget.GetRenderWindow().Render()

def main():
    app = QtWidgets.QApplication(sys.argv)
    window = MainWindow()
    window.show()
    app.exec_()

    rclpy.shutdown()  # Poprawne zamknięcie ROS2.

if __name__ == '__main__':
    main()