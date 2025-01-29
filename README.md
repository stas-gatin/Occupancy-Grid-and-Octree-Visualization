# 3D Occupancy Grid and Octree Visualization

This project provides an implementation of a 3D occupancy grid and an octree for efficient spatial representation and visualization of point cloud data. The program reads a `.pcd` file, processes the point cloud data, and allows visualization using Open3D.

## Features
- **3D Occupancy Grid**: Represents space as a uniform grid and classifies cells as occupied or empty.
- **Octree Structure**: Implements an adaptive hierarchical structure for efficient point cloud representation.
- **Point Cloud Processing**: Reads `.pcd` files and extracts 3D points.
- **Visualization**: Uses Open3D to display the occupancy grid and octree with customizable colors and parameters.
- **Statistics Computation**: Provides metrics such as total cells, occupied cells, empty cells, and memory usage.

## Installation
### Requirements
Ensure you have Python 3 installed along with the required dependencies:
```sh
pip install numpy open3d
```

## Usage
1. Place your `.pcd` file in the project directory.
2. Modify the `CONFIG` dictionary to adjust visualization settings.
3. Run the script:
```sh
python main.py
```

## Configuration
Modify the `CONFIG` dictionary to customize parameters:
```python
CONFIG = {
    'CELL_SIZE': 1.0,
    'PCD_FILE': "poli000.pcd",
    'VISUALIZATION': {
        'enabled': True,
        'show_empty_cells': True,
        'background_color': [0, 0, 0],
        'point_size': 1.0,
        'occupied_cell_color': [1, 0, 0],
        'empty_cell_color': [0, 0, 1]
    }
}
```

## Author
Created by [Stanislav Gatin](https://github.com/stas-gatin)

## License
This project is open-source and available under the MIT License.

