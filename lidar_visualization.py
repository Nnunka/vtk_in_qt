import math
import vtk

class LidarVisualizer:
    def __init__(self, renderer):
        # Inicjalizacja renderera i aktora VTK.
        self.renderer = renderer
        self.points = vtk.vtkPoints()  # Punkty do wyświetlenia.
        self.vertices = vtk.vtkCellArray()  # Komórki dla punktów.
        self.polyData = vtk.vtkPolyData()  # Struktura danych dla geometrii.
        self.polyData.SetPoints(self.points)
        self.polyData.SetVerts(self.vertices)
        
        # Mapper i aktor do renderowania punktów.
        self.mapper = vtk.vtkPolyDataMapper()
        self.mapper.SetInputData(self.polyData)
        self.actor = vtk.vtkActor()
        self.actor.SetMapper(self.mapper)
        
        # Ustawienia wyglądu punktów.
        self.actor.GetProperty().SetPointSize(5)
        self.actor.GetProperty().SetColor(1.0, 0.0, 0.0)  # Czerwone punkty.
        self.renderer.AddActor(self.actor)

    def update_points(self, ranges, angle_min, angle_increment):
        # Aktualizacja punktów na podstawie danych z lidaru.
        self.points.Reset()
        self.vertices.Reset()
        for i, range in enumerate(ranges):
            if range == float('inf') or range == 0.0:
                continue  # Pomijanie nieprawidłowych danych.
            angle = angle_min + i * angle_increment
            x = range * math.cos(angle)
            y = range * math.sin(angle)
            z = 0
            pt_id = self.points.InsertNextPoint([x, y, z])
            self.vertices.InsertNextCell(1)
            self.vertices.InsertCellPoint(pt_id)
        
        # Oznaczanie zmian w danych, aby odświeżyć wizualizację.
        self.points.Modified()
        self.vertices.Modified()
        self.polyData.Modified()
