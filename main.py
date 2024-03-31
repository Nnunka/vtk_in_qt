import os

# Sprawdzenie, czy sesja jest uruchomiona na Waylandzie
if os.getenv('XDG_SESSION_TYPE') == 'wayland':
    # Ustawienie platformy Qt na 'xcb', aby używać X11 przez XCB (XWayland)
    os.environ['QT_QPA_PLATFORM'] = 'xcb'

import sys
from PyQt5 import QtWidgets
from PyQt5.QtCore import QTimer
from vtkmodules.qt.QVTKRenderWindowInteractor import QVTKRenderWindowInteractor
import subprocess
from MainWindow import Ui_MainWindow  # Zaimportuj interfejs użytkownika z Qt Designer

class MainWindow(QtWidgets.QMainWindow, Ui_MainWindow):
    def __init__(self, *args, **kwargs):
        super(MainWindow, self).__init__(*args, **kwargs)
        self.setupUi(self)  # Inicjalizacja interfejsu użytkownika

        self.program_running = False  # Flaga informująca, czy program został już uruchomiony

        # Przypisanie funkcji do przycisku
        self.pushButton.clicked.connect(self.openVTK)
        self.pushButton.setText("Wizualizacja Lidaru")
        
    def openVTK(self):
        if not self.program_running:  # Sprawdź, czy program nie został już uruchomiony
            self.program_running = True  # Ustaw flagę na True, aby oznaczyć, że program został uruchomiony
            subprocess.Popen(["python3", "lidar_visualization.py"])
            self.pushButton.setEnabled(False)  # Wyszarza przycisk po naciśnięciu

def main():
    app = QtWidgets.QApplication(sys.argv)
    window = MainWindow()
    window.show()
    app.exec_()


if __name__ == '__main__':
    main()
