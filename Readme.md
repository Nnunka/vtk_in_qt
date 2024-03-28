## venv

Aby stworzyć wirtualne środowisko należy użyć komendy:

```
virtualenv venv
```

Aktywacja wirtualnego środowiska:

```
source venv/bin/activate
```

Aby opuścić wirtualne środowisko należy użyć komendy:

```
deactivate
```

## Pre requirements

```
pip install vtk
pip install PyQt5
```

Należy również uruchomić symulację piper bot z gazebo.

## Run

Pliki wykonawcze to main. Aby go uruchomić należy wpisać komendę:

```
python main.py
```

## Files

W Paczce znajdują się pliki takie jak:

**mainwindow ui** - Można go odpalić w Qt Designerze i edytować bezpośrednio w nim. Po edycji należy wykonać komendę:

```
pyuic5 mainwindow.ui -o MainWindow.py
```

**MainWindow py** - Jest to plik pythona konwertowany z pliku xml ui. Nie należy w nim nic edytować!

**main py** - Jest to plik główny, tworzy on okno na podstawie ui z pliku MainWindow py.

**lidar_visualization py** - W tym pliku znajduje się logika wizualizacji punktów lidaru w vtk.

**ros_connection py** - W tym pliku znajduje się logika połączenia się z topikiem rosa.
