'''
Creado por: Stanislav Gatin
    https://github.com/stas-gatin
    https://www.linkedin.com/stanislav-gatin
'''

import numpy as np
from typing import List, Tuple
import sys
import open3d as o3d

# Configuraciones globales
CONFIG = {
    'CELL_SIZE': 1.0,  # Tamaño de celda en metros
    'PCD_FILE': "poli000.pcd",
    'VISUALIZATION': {
        'enabled': True,  # Habilitar/deshabilitar visualización
        'show_empty_cells': True,  # Habilitar/deshabilitar visualización de celdas vacías
        'background_color': [0, 0, 0],  # Color de fondo [R,G,B]
        'point_size': 1.0,  # Tamaño de los puntos
        'occupied_cell_color': [1, 0, 0],  # Color de celdas ocupadas [R,G,B]
        'empty_cell_color': [0, 0, 1]  # Color de celdas vacías [R,G,B]
    }
}

# Clase para representar un punto en el espacio 3D
class Point:
    def __init__(self, x: float, y: float, z: float):
        self.x = x
        self.y = y
        self.z = z

# Clase para representar una rejilla de ocupación
class OccupancyGrid:
    def __init__(self, cell_size: float, bounds: Tuple[float, float, float, float, float, float]):
        self.cell_size = cell_size
        self.bounds = bounds  # (min_x, max_x, min_y, max_y, min_z, max_z)
        
        # Calcular dimensiones de la rejilla
        self.nx = int((bounds[1] - bounds[0]) / cell_size) + 1
        self.ny = int((bounds[3] - bounds[2]) / cell_size) + 1
        self.nz = int((bounds[5] - bounds[4]) / cell_size) + 1
        
        # Inicializar arrays para contar puntos y acumular coordenadas
        self.point_count = np.zeros((self.nx, self.ny, self.nz), dtype=int)
        self.sum_points = np.zeros((self.nx, self.ny, self.nz, 3))

    # Método para agregar un punto a la rejilla
    def add_point(self, point: Point):
        # Convertir coordenadas a índices de la rejilla
        ix = int((point.x - self.bounds[0]) / self.cell_size)
        iy = int((point.y - self.bounds[2]) / self.cell_size)
        iz = int((point.z - self.bounds[4]) / self.cell_size)
        
        if 0 <= ix < self.nx and 0 <= iy < self.ny and 0 <= iz < self.nz:
            self.point_count[ix, iy, iz] += 1
            self.sum_points[ix, iy, iz] += [point.x, point.y, point.z]

    # Método para obtener estadísticas de la rejilla
    def get_statistics(self):
        total_cells = self.nx * self.ny * self.nz
        occupied_cells = np.sum(self.point_count > 0)
        empty_cells = total_cells - occupied_cells
        avg_points = np.mean(self.point_count[self.point_count > 0]) if occupied_cells > 0 else 0
        
        return {
            "total_cells": total_cells,
            "occupied_cells": occupied_cells,
            "empty_cells": empty_cells,
            "avg_points_per_occupied_cell": avg_points,
            "memory_usage": sys.getsizeof(self.point_count) + sys.getsizeof(self.sum_points)
        }

# Clase para representar un nodo en un octree
class OctreeNode:
    def __init__(self, center: Point, size: float):
        self.center = center
        self.size = size
        self.children = [None] * 8
        self.point_count = 0
        self.sum_points = [0, 0, 0]
        self.is_leaf = True

# Clase para representar un octree
class Octree:
    def __init__(self, min_cell_size: float, bounds: Tuple[float, float, float, float, float, float]):
        self.min_cell_size = min_cell_size
        self.bounds = bounds
        center_x = (bounds[0] + bounds[1]) / 2
        center_y = (bounds[2] + bounds[3]) / 2
        center_z = (bounds[4] + bounds[5]) / 2
        size = max(bounds[1]-bounds[0], bounds[3]-bounds[2], bounds[5]-bounds[4])
        self.root = OctreeNode(Point(center_x, center_y, center_z), size)

    # Método para agregar un punto al octree
    def add_point(self, point: Point):
        self._insert(self.root, point)

    # Método interno para insertar un punto en el octree
    def _insert(self, node: OctreeNode, point: Point):
        if node.size / 2 < self.min_cell_size:
            node.point_count += 1
            node.sum_points[0] += point.x
            node.sum_points[1] += point.y
            node.sum_points[2] += point.z
            return

        # Determinar el octante
        octant = self._get_octant(node.center, point)
        
        if node.children[octant] is None:
            new_size = node.size / 2
            new_center = self._get_child_center(node.center, octant, new_size)
            node.children[octant] = OctreeNode(new_center, new_size)
            node.is_leaf = False

        self._insert(node.children[octant], point)

    # Método para determinar el octante de un punto
    def _get_octant(self, center: Point, point: Point) -> int:
        octant = 0
        if point.x >= center.x: octant |= 4
        if point.y >= center.y: octant |= 2
        if point.z >= center.z: octant |= 1
        return octant

    # Método para obtener el centro de un hijo en un octante específico
    def _get_child_center(self, parent_center: Point, octant: int, size: float) -> Point:
        x = parent_center.x + (size/2 if octant & 4 else -size/2)
        y = parent_center.y + (size/2 if octant & 2 else -size/2)
        z = parent_center.z + (size/2 if octant & 1 else -size/2)
        return Point(x, y, z)

    # Método para obtener estadísticas del octree
    def get_statistics(self):
        stats = {"total_cells": 0, "occupied_cells": 0, "empty_cells": 0, "total_points": 0}
        self._count_nodes(self.root, stats)
        avg_points = stats["total_points"] / stats["occupied_cells"] if stats["occupied_cells"] > 0 else 0
        
        return {
            "total_cells": stats["total_cells"],
            "occupied_cells": stats["occupied_cells"],
            "empty_cells": stats["empty_cells"],
            "avg_points_per_occupied_cell": avg_points,
            "memory_usage": stats["total_cells"] * sys.getsizeof(OctreeNode(Point(0,0,0), 0))
        }

    # Método interno para contar nodos en el octree
    def _count_nodes(self, node: OctreeNode, stats: dict):
        if node is None:
            return
        
        stats["total_cells"] += 1
        if node.point_count > 0:
            stats["occupied_cells"] += 1
            stats["total_points"] += node.point_count
        else:
            stats["empty_cells"] += 1

        if not node.is_leaf:
            for child in node.children:
                self._count_nodes(child, stats)

# Método para leer un archivo PCD y obtener una lista de puntos
def read_pcd_file(filename: str) -> List[Point]:
    points = []
    data_section = False
    
    with open(filename, 'r') as file:
        for line in file:
            if line.startswith('DATA ascii'):
                data_section = True
                continue
            
            if data_section:
                values = line.strip().split()
                if len(values) >= 3:
                    x, y, z = map(float, values[:3])
                    points.append(Point(x, y, z))
    
    return points

# Método para visualizar el octree y los puntos
def visualize_octree(octree, points):
    vis = o3d.visualization.Visualizer()
    vis.create_window()

    # Convertir los puntos originales a nube de puntos de Open3D
    pcd = o3d.geometry.PointCloud()
    points_array = np.array([[p.x, p.y, p.z] for p in points])
    pcd.points = o3d.utility.Vector3dVector(points_array)
    
    vis.add_geometry(pcd)

    # Función recursiva para crear cajas que representan los nodos del octree
    def create_boxes(node, geometries):
        if node is None:
            return

        # Solo crear caja si el nodo está ocupado o si show_empty_cells está activado
        if node.point_count > 0 or CONFIG['VISUALIZATION']['show_empty_cells']:
            box = o3d.geometry.AxisAlignedBoundingBox(
                min_bound=np.array([
                    node.center.x - node.size/2,
                    node.center.y - node.size/2,
                    node.center.z - node.size/2
                ]),
                max_bound=np.array([
                    node.center.x + node.size/2,
                    node.center.y + node.size/2,
                    node.center.z + node.size/2
                ])
            )
            
            # Asignar color según configuración
            if node.point_count > 0:
                box.color = np.array(CONFIG['VISUALIZATION']['occupied_cell_color'])
            else:
                box.color = np.array(CONFIG['VISUALIZATION']['empty_cell_color'])
            
            geometries.append(box)

        if not node.is_leaf:
            for child in node.children:
                create_boxes(child, geometries)

    geometries = []
    create_boxes(octree.root, geometries)
    for geometry in geometries:
        vis.add_geometry(geometry)

    # Configurar la visualización según los parámetros
    opt = vis.get_render_option()
    opt.background_color = np.asarray(CONFIG['VISUALIZATION']['background_color'])
    opt.point_size = CONFIG['VISUALIZATION']['point_size']

    vis.run()
    vis.destroy_window()

# Método para comparar los métodos de rejilla de ocupación y octree
def compare_methods(pcd_file: str, cell_size: float):
    # Leer puntos
    points = read_pcd_file(pcd_file)
    
    # Calcular límites
    min_x = min(p.x for p in points)
    max_x = max(p.x for p in points)
    min_y = min(p.y for p in points)
    max_y = max(p.y for p in points)
    min_z = min(p.z for p in points)
    max_z = max(p.z for p in points)
    bounds = (min_x, max_x, min_y, max_y, min_z, max_z)
    
    # Crear estructuras
    grid = OccupancyGrid(cell_size, bounds)
    octree = Octree(cell_size, bounds)
    
    # Agregar puntos
    for point in points:
        grid.add_point(point)
        octree.add_point(point)
    
    # Obtener estadísticas
    grid_stats = grid.get_statistics()
    octree_stats = octree.get_statistics()
    
    print("Comparación de métodos:")
    print("\nRejilla de ocupación:")
    for key, value in grid_stats.items():
        print(f"{key}: {value}")
    
    print("\nOctree:")
    for key, value in octree_stats.items():
        print(f"{key}: {value}")

    # Visualizar si está habilitado
    if CONFIG['VISUALIZATION']['enabled']:
        visualize_octree(octree, points)

# Punto de entrada del programa
if __name__ == "__main__":
    compare_methods(CONFIG['PCD_FILE'], CONFIG['CELL_SIZE'])
