# 2D to 3D 레이스트랙 변환기

F1TENTH 2D 맵을 3D 메시로 변환하여 Isaac Sim에서 사용할 수 있도록 하는 도구입니다.

<div align="center">
  <div style="margin-bottom: 10px;">
    <img src="/img/isaacsim.png" width="90%">
    <p style="text-align: center;">isaac sim</p>
  </div>
</div>

## 개요

이 프로젝트는 PNG/YAML 형식의 2D 맵을 읽어 3D 벽과 바닥이 있는 메시로 변환합니다. Isaac Sim, Blender, Maya 등 다양한 3D 환경에서 사용할 수 있습니다.

## 필요 라이브러리

```bash
pip install opencv-python numpy pyyaml trimesh scipy
```

## 사용법

### 기본 사용법

```bash
python map_to_3d.py <트랙이름>
```

### 예시

```bash
# Austin 트랙 변환 (기본 벽 높이 1.0m)
python map_to_3d.py Austin

# 벽 높이 2.0m로 설정
python map_to_3d.py Austin --height 2.0

# 특정 출력 경로 지정
python map_to_3d.py Austin --output my_track.obj
```

### 명령어 옵션

- `track_name`: 변환할 트랙 이름 (필수)
- `--height`: 벽 높이 (미터 단위, 기본값: 1.0)
- `--output`: 출력 파일 경로 (선택사항)

## 디렉토리 구조

```
2D_to_3D_racetrack/
├── map_to_3d.py          # 메인 변환 스크립트
├── tracks/               # 입력 트랙 데이터
│   ├── Austin/
│   │   ├── Austin_map.png
│   │   └── Austin_map.yaml
│   ├── Monza/
│   │   ├── Monza_map.png
│   │   └── Monza_map.yaml
│   └── ...
└── output/               # 변환된 3D 모델 (자동 생성)
    ├── Austin/
    │   ├── Austin_track_3d.obj
    │   └── Austin_track_3d.stl
    └── ...
```

## 지원 트랙

현재 다음 트랙들이 포함되어 있습니다:

- Austin
- BrandsHatch
- Budapest
- Catalunya
- Hockenheim
- IMS
- Melbourne
- MexicoCity
- Montreal
- Monza
- MoscowRaceway
- Nuerburgring
- Oschersleben
- Sakhir
- SaoPaulo
- Sepang
- Shanghai
- Silverstone
- Sochi
- Spa
- Spielberg
- YasMarina
- Zandvoort

## 출력 형식

- **OBJ 파일**: Blender, Maya 등 3D 모델링 소프트웨어용
- **STL 파일**: Isaac Sim, 3D 프린팅용

## 변환 과정

1. PNG 맵 이미지와 YAML 메타데이터 로드
2. 픽셀 값을 점유 격자로 변환 (검은색=벽, 흰색=자유공간)
3. 형태학적 연산으로 벽 경계 검출
4. 각 벽 픽셀을 3D 큐브로 변환
5. 바닥 메시 생성
6. 벽과 바닥 메시 결합
7. OBJ/STL 형식으로 내보내기

## Isaac Sim에서 사용하기

1. 변환된 STL 파일을 Isaac Sim 프로젝트에 임포트
2. mesh를 rigid body 와 collider 설정 추가
3. Default Material에 rigid body material 설정 추가 Static Friction과 Dynamic Friction 값을 1로 설정
4. F1TENTH 차량 모델과 함께 시뮬레이션 실행

## 문제 해결

### 파일을 찾을 수 없는 경우
- `tracks/트랙이름/` 디렉토리에 PNG와 YAML 파일이 있는지 확인
- 파일명이 `트랙이름_map.png`, `트랙이름_map.yaml` 형식인지 확인

### 메시가 너무 크거나 작은 경우
- YAML 파일의 resolution 값 확인
- 벽 높이 조절 (`--height` 옵션 사용)

### Isaac Sim에서 로딩 문제
- STL 파일 사용 (OBJ 대신)
- 메시 단위가 미터인지 확인
- Static Collider 속성 설정

## 라이선스

이 프로젝트는 F1TENTH 커뮤니티를 위한 오픈소스 도구입니다.
