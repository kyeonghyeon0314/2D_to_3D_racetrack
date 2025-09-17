#!/usr/bin/env python3
"""
F1TENTH 2D Map to 3D Isaac Sim Converter
Converts PNG/YAML map format to 3D mesh for Isaac Sim
"""

import cv2
import numpy as np
import yaml
import trimesh
from scipy.ndimage import binary_erosion, binary_dilation
import argparse
import os

class Map3DConverter:
    def __init__(self, map_png_path, map_yaml_path, wall_height=1.0):
        """
        Initialize the 3D map converter

        Args:
            map_png_path: Path to PNG map file
            map_yaml_path: Path to YAML map metadata
            wall_height: Height of walls in meters (default: 1.0m)
        """
        self.map_png_path = map_png_path
        self.map_yaml_path = map_yaml_path
        self.wall_height = wall_height

        # Load map metadata
        with open(map_yaml_path, 'r') as f:
            self.map_metadata = yaml.safe_load(f)

        self.resolution = self.map_metadata['resolution']  # meters per pixel
        self.origin = self.map_metadata['origin'][:2]  # [x, y] in meters
        self.occupied_thresh = self.map_metadata.get('occupied_thresh', 0.45)
        self.free_thresh = self.map_metadata.get('free_thresh', 0.196)

        print(f"Map resolution: {self.resolution} m/pixel")
        print(f"Map origin: {self.origin}")
        print(f"Wall height: {self.wall_height} m")

    def load_and_process_map(self):
        """Load PNG map and convert to binary occupancy grid"""
        # Load map image (grayscale)
        map_img = cv2.imread(self.map_png_path, cv2.IMREAD_GRAYSCALE)
        if map_img is None:
            raise FileNotFoundError(f"Could not load map image: {self.map_png_path}")

        print(f"Map image shape: {map_img.shape}")

        # Convert to occupancy grid (0=free, 1=occupied)
        # PNG values: 0=occupied(black), 255=free(white), 127=unknown(gray)
        occupancy_grid = np.zeros_like(map_img, dtype=np.uint8)

        # Normalize pixel values to 0-1 range
        normalized = map_img.astype(float) / 255.0

        # Apply thresholds
        occupancy_grid[normalized <= (1.0 - self.occupied_thresh)] = 1  # Occupied
        occupancy_grid[normalized >= (1.0 - self.free_thresh)] = 0     # Free
        occupancy_grid[(normalized > (1.0 - self.occupied_thresh)) &
                      (normalized < (1.0 - self.free_thresh))] = 1     # Unknown -> treat as occupied

        return occupancy_grid

    def create_wall_mesh(self, occupancy_grid):
        """Create 3D wall mesh from 2D occupancy grid"""
        height, width = occupancy_grid.shape

        # Find wall edges using morphological operations
        # Erode to find inner boundaries, then subtract to get edges
        eroded = binary_erosion(occupancy_grid, structure=np.ones((3,3)))
        wall_edges = occupancy_grid.astype(bool) & ~eroded

        vertices = []
        faces = []
        vertex_count = 0

        print("Generating 3D mesh from occupancy grid...")

        # For each wall pixel, create a vertical wall segment
        wall_pixels = np.where(wall_edges)
        total_pixels = len(wall_pixels[0])

        for i, (row, col) in enumerate(zip(wall_pixels[0], wall_pixels[1])):
            if i % 1000 == 0:
                print(f"Processing pixel {i}/{total_pixels}")

            # Convert pixel coordinates to world coordinates
            # Note: Image row=0 is at top, but world y increases upward
            world_x = self.origin[0] + col * self.resolution
            world_y = self.origin[1] + (height - row - 1) * self.resolution

            # Create a small cube for this wall pixel
            cube_size = self.resolution * 0.9  # Slightly smaller to avoid gaps

            # Define cube vertices (8 vertices per cube)
            x_min, x_max = world_x, world_x + cube_size
            y_min, y_max = world_y, world_y + cube_size
            z_min, z_max = 0.0, self.wall_height

            cube_vertices = [
                [x_min, y_min, z_min],  # 0: bottom-left-front
                [x_max, y_min, z_min],  # 1: bottom-right-front
                [x_max, y_max, z_min],  # 2: bottom-right-back
                [x_min, y_max, z_min],  # 3: bottom-left-back
                [x_min, y_min, z_max],  # 4: top-left-front
                [x_max, y_min, z_max],  # 5: top-right-front
                [x_max, y_max, z_max],  # 6: top-right-back
                [x_min, y_max, z_max],  # 7: top-left-back
            ]

            vertices.extend(cube_vertices)

            # Define cube faces (12 triangles, 2 per face)
            base_idx = vertex_count
            cube_faces = [
                # Bottom face (z=0)
                [base_idx+0, base_idx+1, base_idx+2],
                [base_idx+0, base_idx+2, base_idx+3],
                # Top face (z=wall_height)
                [base_idx+4, base_idx+6, base_idx+5],
                [base_idx+4, base_idx+7, base_idx+6],
                # Front face (y=y_min)
                [base_idx+0, base_idx+4, base_idx+5],
                [base_idx+0, base_idx+5, base_idx+1],
                # Back face (y=y_max)
                [base_idx+3, base_idx+2, base_idx+6],
                [base_idx+3, base_idx+6, base_idx+7],
                # Left face (x=x_min)
                [base_idx+0, base_idx+3, base_idx+7],
                [base_idx+0, base_idx+7, base_idx+4],
                # Right face (x=x_max)
                [base_idx+1, base_idx+5, base_idx+6],
                [base_idx+1, base_idx+6, base_idx+2],
            ]

            faces.extend(cube_faces)
            vertex_count += 8

        print(f"Generated mesh with {len(vertices)} vertices and {len(faces)} faces")

        # Create trimesh object
        if len(vertices) == 0:
            print("Warning: No wall pixels found!")
            return None

        mesh = trimesh.Trimesh(vertices=np.array(vertices), faces=np.array(faces))

        # Clean up mesh (remove duplicates, fix normals)
        mesh.remove_duplicate_faces()
        mesh.fix_normals()

        return mesh

    def create_floor_mesh(self, occupancy_grid):
        """Create floor mesh for the entire map area"""
        height, width = occupancy_grid.shape

        # Create floor vertices
        x_min = self.origin[0]
        y_min = self.origin[1]
        x_max = self.origin[0] + width * self.resolution
        y_max = self.origin[1] + height * self.resolution

        floor_vertices = [
            [x_min, y_min, 0.0],
            [x_max, y_min, 0.0],
            [x_max, y_max, 0.0],
            [x_min, y_max, 0.0]
        ]

        floor_faces = [
            [0, 1, 2],
            [0, 2, 3]
        ]

        floor_mesh = trimesh.Trimesh(vertices=np.array(floor_vertices),
                                   faces=np.array(floor_faces))

        return floor_mesh

    def convert_to_3d(self, output_path=None):
        """Main conversion function"""
        if output_path is None:
            base_name = os.path.splitext(os.path.basename(self.map_png_path))[0]
            output_path = f"{base_name}_3d.obj"

        print("Loading and processing 2D map...")
        occupancy_grid = self.load_and_process_map()

        print("Creating wall mesh...")
        wall_mesh = self.create_wall_mesh(occupancy_grid)

        print("Creating floor mesh...")
        floor_mesh = self.create_floor_mesh(occupancy_grid)

        # Combine meshes
        if wall_mesh is not None:
            combined_mesh = wall_mesh + floor_mesh
        else:
            combined_mesh = floor_mesh

        # Export mesh
        print(f"Exporting 3D mesh to: {output_path}")
        combined_mesh.export(output_path)

        # Also export as STL for Isaac Sim
        stl_path = output_path.replace('.obj', '.stl')
        combined_mesh.export(stl_path)
        print(f"Also exported as STL: {stl_path}")

        print("Conversion completed successfully!")
        return output_path

def main():
    parser = argparse.ArgumentParser(description='Convert F1TENTH 2D map to 3D mesh for Isaac Sim')
    parser.add_argument('track_name', help='Track name (e.g., Austin) - will look in tracks/TRACK_NAME/ directory')
    parser.add_argument('--height', type=float, default=1.0, help='Wall height in meters (default: 1.0)')
    parser.add_argument('--output', '-o', help='Output mesh file path (optional)')

    args = parser.parse_args()

    track_name = args.track_name

    # 자동 경로 설정
    input_dir = f"tracks/{track_name}"
    output_dir = f"output/{track_name}"

    # 입력 파일 경로 구성
    map_png = f"{input_dir}/{track_name}_map.png"
    map_yaml = f"{input_dir}/{track_name}_map.yaml"

    # 출력 파일 경로 구성
    if args.output:
        output_path = args.output
    else:
        # 출력 디렉토리 생성
        os.makedirs(output_dir, exist_ok=True)
        output_path = f"{output_dir}/{track_name}_track_3d.obj"

    print(f"🏁 F1TENTH 3D 맵 변환기")
    print(f"📍 트랙: {track_name}")
    print(f"📂 입력 디렉토리: {input_dir}")
    print(f"📂 출력 디렉토리: {output_dir}")
    print("-" * 50)

    # 입력 파일 검증
    if not os.path.exists(map_png):
        print(f"❌ PNG 파일을 찾을 수 없습니다: {map_png}")
        print(f"💡 다음 위치에 파일이 있는지 확인해주세요:")
        print(f"   - {map_png}")
        print(f"   - {map_yaml}")
        return 1

    if not os.path.exists(map_yaml):
        print(f"❌ YAML 파일을 찾을 수 없습니다: {map_yaml}")
        return 1

    print(f"✅ 입력 파일 확인 완료")
    print(f"   PNG: {map_png}")
    print(f"   YAML: {map_yaml}")

    # 변환기 생성 및 실행
    converter = Map3DConverter(map_png, map_yaml, args.height)
    try:
        final_output_path = converter.convert_to_3d(output_path)

        print(f"\n🎉 변환 성공!")
        print(f"📁 3D 모델 저장 위치:")
        print(f"   OBJ: {final_output_path}")
        print(f"   STL: {final_output_path.replace('.obj', '.stl')}")
        print(f"\n📋 사용 방법:")
        print(f"   Isaac Sim: STL 파일 사용")
        print(f"   Blender/Maya: OBJ 파일 사용")
        return 0
    except Exception as e:
        print(f"❌ 변환 중 오류 발생: {e}")
        return 1

if __name__ == "__main__":
    exit(main())