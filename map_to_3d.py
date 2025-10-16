#!/usr/bin/env python3
"""
F1TENTH 2D Map to 3D Isaac Sim Converter
Converts PNG/YAML map format to 3D mesh for Isaac Sim
"""

import cv2
import numpy as np
import yaml
import trimesh
from scipy.ndimage import binary_erosion
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
        """Create 3D wall mesh from 2D occupancy grid using contours"""
        height, width = occupancy_grid.shape

        print("Extracting wall edges using morphological operations...")
        # 1단계: occupied 영역의 경계 추출
        eroded_occupied = binary_erosion(occupancy_grid, structure=np.ones((3,3)))
        occupied_edges = occupancy_grid.astype(bool) & ~eroded_occupied

        # 2단계: free space 영역의 경계 추출 (트랙 안쪽 벽을 위해)
        free_space = ~occupancy_grid.astype(bool)
        eroded_free = binary_erosion(free_space, structure=np.ones((3,3)))
        free_edges = free_space & ~eroded_free

        # 3단계: 두 경계를 합침 (바깥벽 + 안쪽벽)
        wall_edges = occupied_edges | free_edges

        print("Finding wall contours from edges...")
        # 4단계: wall_edges를 uint8로 변환하여 contour 추출
        wall_edges_uint8 = (wall_edges * 255).astype(np.uint8)
        contours, _ = cv2.findContours(wall_edges_uint8, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        if not contours:
            print("Warning: No wall contours found!")
            return None

        mesh_list = []

        print(f"Found {len(contours)} contours. Extruding to 3D walls...")
        for i, contour in enumerate(contours):
            if i % 100 == 0:
                print(f"Processing contour {i}/{len(contours)}")

            # 너무 작은 contour는 노이즈일 수 있으니 필터링
            if cv2.contourArea(contour) < 10:
                continue

            # 3D 변환을 위해 불필요한 차원 제거 (N, 1, 2) -> (N, 2)
            contour_points = np.squeeze(contour, axis=1)

            # 최소 3개의 점이 필요함
            if len(contour_points) < 3:
                continue

            # 픽셀 좌표를 월드 좌표로 변환
            # Y축 방향이 이미지 좌표계와 월드 좌표계에서 반대인 것을 보정
            world_points_x = self.origin[0] + contour_points[:, 0] * self.resolution
            world_points_y = self.origin[1] + (height - contour_points[:, 1] - 1) * self.resolution

            # 수동으로 벽면 생성 (천장 없이 옆면만)
            try:
                num_points = len(world_points_x)
                vertices = []
                faces = []

                # 하단 vertices (z=0)와 상단 vertices (z=wall_height) 생성
                for j in range(num_points):
                    # 하단 점
                    vertices.append([world_points_x[j], world_points_y[j], 0.0])
                    # 상단 점
                    vertices.append([world_points_x[j], world_points_y[j], self.wall_height])

                # 각 edge를 수직 사각형(2개 삼각형)으로 변환
                for j in range(num_points):
                    # 현재 점과 다음 점 인덱스
                    curr_bottom = j * 2
                    curr_top = j * 2 + 1
                    next_bottom = ((j + 1) % num_points) * 2
                    next_top = ((j + 1) % num_points) * 2 + 1

                    # 사각형을 2개 삼각형으로 분할
                    # 삼각형 1: bottom_curr, top_curr, bottom_next
                    faces.append([curr_bottom, curr_top, next_bottom])
                    # 삼각형 2: top_curr, top_next, bottom_next
                    faces.append([curr_top, next_top, next_bottom])

                # Trimesh 객체 생성
                mesh = trimesh.Trimesh(vertices=np.array(vertices), faces=np.array(faces))
                mesh_list.append(mesh)
            except Exception as e:
                # 가끔 매우 작거나 잘못된 형태의 contour는 에러를 발생시킬 수 있습니다.
                print(f"Warning: Could not create wall mesh for contour {i}. Skipping. Error: {e}")

        if not mesh_list:
            print("Warning: No valid meshes created!")
            return None

        # 모든 메쉬를 한 번에 결합 (메모리 효율적)
        print(f"Combining {len(mesh_list)} meshes...")
        combined_wall_mesh = trimesh.util.concatenate(mesh_list)

        print(f"Generated mesh with {len(combined_wall_mesh.vertices)} vertices and {len(combined_wall_mesh.faces)} faces")

        # 메쉬 정리 (중복 제거 및 법선 벡터 수정)
        combined_wall_mesh.remove_duplicate_faces()
        combined_wall_mesh.fix_normals()

        return combined_wall_mesh

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