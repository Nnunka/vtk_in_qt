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

        # Inicjalizacja tablicy kolorów
        self.colors = vtk.vtkUnsignedCharArray()
        self.colors.SetNumberOfComponents(3)
        self.colors.SetName("Colors")  # Ustawienie nazwy dla tablicy kolorów

        # Powiązanie tablicy kolorów z danymi punktowymi
        self.polyData.GetPointData().SetScalars(self.colors)

        # Mapper i aktor do renderowania punktów.
        self.mapper = vtk.vtkPolyDataMapper()
        self.mapper.SetInputData(self.polyData)
        self.actor = vtk.vtkActor()
        self.actor.SetMapper(self.mapper)
        
        # Ustawienia wyglądu punktów.
        self.actor.GetProperty().SetPointSize(5)
        self.actor.GetProperty().SetColor(1.0, 0.0, 0.0)  # Czerwone punkty
        self.renderer.AddActor(self.actor)

    def update_points(self, ranges, angle_min, angle_increment):
        # Aktualizacja punktów na podstawie danych z lidaru.
        self.points.Reset()
        self.vertices.Reset()
        self.colors.Reset()

        # Indeks odpowiadający kątowi 0
        zero_angle_index = round(abs(angle_min) / angle_increment)

        for i, range in enumerate(ranges):
            if range == float('inf') or range == 0.0:
                continue  # Pomijanie nieprawidłowych danych.
            angle = angle_min + i * angle_increment
            # Przesunięcie punktów tak, aby kąt zero był na dole (oś X)
            x = range * math.cos(angle - math.pi/2)  # Odejmujemy pi/2, aby obrócić kąt zero na oś X
            y = range * math.sin(angle - math.pi/2)  # Odejmujemy pi/2, aby obrócić kąt zero na oś X
            z = 0
            pt_id = self.points.InsertNextPoint([x, y, z])
            self.vertices.InsertNextCell(1)
            self.vertices.InsertCellPoint(pt_id)

            # Ustawienie koloru punktu na zielony, jeśli kąt odpowiada kątowi 0
            if i == zero_angle_index:
                self.colors.InsertNextTuple([0, 255, 0])  # Kolor zielony
            else:
                self.colors.InsertNextTuple([255, 0, 0])  # Domyślny kolor czerwony
        
        # Oznaczanie zmian w danych, aby odświeżyć wizualizację.
        self.points.Modified()
        self.vertices.Modified()
        self.polyData.Modified()
