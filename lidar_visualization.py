import math
import vtk

class LidarVisualizer:
    def __init__(self, renderer):
        self.renderer = renderer
        self.points = vtk.vtkPoints()
        self.vertices = vtk.vtkCellArray()
        self.polyData = vtk.vtkPolyData()
        self.polyData.SetPoints(self.points)
        self.polyData.SetVerts(self.vertices)
        self.mapper = vtk.vtkPolyDataMapper()
        self.mapper.SetInputData(self.polyData)
        self.actor = vtk.vtkActor()
        self.actor.SetMapper(self.mapper)
        self.actor.GetProperty().SetPointSize(5)
        self.actor.GetProperty().SetColor(1.0, 0.0, 0.0)
        self.renderer.AddActor(self.actor)

    def update_points(self, ranges, angle_min, angle_increment):
        self.points.Reset()
        self.vertices.Reset()
        for i, range in enumerate(ranges):
            if range == float('inf') or range == 0.0:
                continue
            angle = angle_min + i * angle_increment
            x = range * math.cos(angle)
            y = range * math.sin(angle)
            z = 0
            pt_id = self.points.InsertNextPoint([x, y, z])
            self.vertices.InsertNextCell(1)
            self.vertices.InsertCellPoint(pt_id)

        self.points.Modified()
        self.vertices.Modified()
        self.polyData.Modified()
