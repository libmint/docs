---
layout: default
title: real2sim-eval 물제 만들기
parent: 나의 공부 게시판
---

갤럭시 S25 울트라는 2026년 현재 최상급 카메라 성능과 프로세서(스냅드래곤 8 Gen 4 등)를 탑재하고 있어, 아이폰의 LiDAR 센서가 없더라도 사진 기반의 3D 가우시안 스플래팅(3DGS) 스캔을 매우 훌륭하게 수행할 수 있습니다.

본인의 물체를 `real2sim-eval` 시뮬레이션 환경에 넣기 위한 전체 과정을 A부터 Z까지 상세히 안내해 드립니다. 처음이시라면 형태가 변하지 않는 **'단단한 물체(강체, Rigid Body)'**로 시작하는 것을 권장합니다. (예: 머그컵, 플라스틱 장난감, 상자 등)

---

### 1단계: 스마트폰으로 물체 스캔하기 (데이터 획득)

해당 프로젝트에서 공식적으로 권장하며 완전 무료로 기기 내(On-device) 처리가 가능한 **Scaniverse** 앱을 사용합니다.

1. **앱 설치:** 구글 플레이스토어에서 `Scaniverse - 3D Scanner` (Niantic 개발)를 설치합니다.
2. **환경 세팅:** * 빛이 골고루 비치는 밝은 곳에 물체를 놓습니다. (그림자가 너무 강하거나 반사가 심한 유리/거울 재질은 피하십시오.)
* 물체 주변을 360도 빙빙 돌 수 있는 공간을 확보합니다.


3. **스캔 진행:**
* 앱을 실행하고 하단의 십자(+) 버튼을 눌러 **'Splat (Gaussian Splatting)'** 모드를 선택합니다.
* 물체를 중심에 두고, 위, 중간, 아래 세 가지 다른 높이에서 천천히 원을 그리며 물체의 모든 면이 카메라에 담기도록 촬영합니다. (약 1~2분 소요)


4. **처리 및 저장:** 촬영을 마치면 'Process'를 누릅니다. S25 울트라의 내부 연산을 통해 약 1분 내외로 3D 결과물이 생성됩니다.

### 2단계: 필수 파일 추출하기 (Export)

시뮬레이션에는 '시각 데이터'와 '물리 데이터' 두 가지가 모두 필요합니다. Scaniverse 앱에서 만들어진 결과물을 열고 다음 두 가지 포맷으로 내보내기(Export)를 수행하여 PC로 전송합니다.

* **Splat 데이터 (`.ply`):** 화면 렌더링에 쓰일 시각 데이터입니다. Export 메뉴에서 `Gaussian Splat (.ply)` 형식을 선택합니다.
* **Mesh 데이터 (`.obj` 또는 `.ply`):** 물리 엔진이 물체의 부피와 충돌을 계산할 뼈대입니다. Export 메뉴에서 `Mesh (.obj)` 형식을 선택합니다.

### 3단계: PC 환경에 데이터 배치 및 정렬

PC의 `real2sim-eval` 프로젝트 폴더로 이동하여 데이터를 세팅합니다. (물체 이름을 `my_cup`이라고 가정하겠습니다.)

1. **폴더 생성:** `log/gs/scans/my_cup/` 폴더를 만들고, 스마트폰에서 가져온 두 파일을 넣습니다. (예: `splat.ply`, `mesh.obj`)
2. **위치 및 크기 정렬 (SuperSplat 활용):**
* 스마트폰으로 스캔한 데이터는 삐뚤어져 있거나 크기가 제각각일 수 있습니다.
* 웹 브라우저에서 [SuperSplat](https://playcanvas.com/supersplat/editor) 사이트에 접속하여 `splat.ply` 파일을 엽니다.
* 편집 툴을 이용해 불필요한 배경(바닥 등)을 지우고, 물체가 정중앙 원점(0,0,0)에 오도록 위치와 회전을 맞춘 뒤 적절한 크기(Scale)로 조절합니다.
* 수정된 파일을 다시 내보내어 덮어씁니다. (`mesh.obj` 파일도 3D 모델링 툴(Blender 등)을 이용해 동일한 위치와 크기로 맞춰주면 가장 좋습니다.)



### 4단계: 물리 트윈(PhysTwin) 생성

로봇이 이 물체를 건드렸을 때 물리적으로 반응하도록 강체 물리 모델을 생성합니다. 터미널을 열고 프로젝트 루트 경로에서 다음 스크립트를 실행합니다.

```bash
python experiments/utils/create_rigid_phystwin.py \
  --rigid_mesh_path log/gs/scans/my_cup/mesh.obj \
  --ckpt_path log/phystwin/my_cup \
  --case_name my_cup_0001 \
  --cfg_path experiments/cfg/physics/default.yaml

```

이 작업이 끝나면 `log/phystwin/my_cup/` 폴더 아래에 물리 연산을 위한 체크포인트 파일들이 생성됩니다.

### 5단계: 환경 설정(Config) 추가

마지막으로 시뮬레이터가 이 새로운 물체를 인식하도록 Hydra 설정 파일을 만들어야 합니다.

1. `cfg/gs/` 폴더 안에 `my_cup.yaml` 파일을 새로 만듭니다.
2. 기존에 있던 `rope.yaml`이나 `T.yaml`의 내용을 복사하여 붙여넣은 뒤, 경로 부분만 내 물체에 맞게 수정합니다.
```yaml
# cfg/gs/my_cup.yaml 예시
mesh_path: log/gs/scans/my_cup/mesh.obj
gs_path: log/gs/scans/my_cup/splat.ply
# ... (나머지 스케일 및 초기 위치 좌표는 물체 크기에 맞게 조정)

```



### 6단계: 시뮬레이션에서 확인하기

이제 키보드 원격 제어 스크립트를 실행하여, 내가 스캔한 물체가 가상 환경에 잘 나타나고 로봇으로 건드렸을 때 정상적으로 밀리는지 확인합니다.

```bash
CUDA_VISIBLE_DEVICES=0 python experiments/keyboard_teleop.py \
  gs=my_cup \
  env=xarm_gripper \
  physics.ckpt_path=log/phystwin/my_cup \
  physics.case_name=my_cup_0001

```

현재 스캔하려고 계획 중인 물체는 구체적으로 어떤 것입니까? 물체의 재질이나 형태에 따라 스캔 시 주의해야 할 점이나 물리 설정 시 팁을 추가로 제공해 드릴 수 있습니다.
