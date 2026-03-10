---
layout: default
title: Jianghanxiao/PhysTwin 프로젝트 상세 분석
parent: 나의 공부 게시판
---

**Jianghanxiao/PhysTwin 프로젝트 상세 분석**

PhysTwin(Physics-Informed Reconstruction and Simulation of Deformable Objects from Videos)은 희소한(sparse) 동영상 데이터를 입력받아 변형 가능한 객체(옷, 인형, 밧줄, 택배 상자 등)의 물리적 디지털 트윈을 구축하는 프레임워크입니다. 컬럼비아 대학교와 일리노이 대학교 어바나-샴페인(UIUC) 연구진이 개발하였으며, ICCV 2025에 채택된 연구입니다.

해당 GitHub 레포지토리의 핵심 내용과 기술적 특징은 다음과 같습니다.

### 1. 핵심 기술 및 아키텍처

PhysTwin은 객체의 시각적 외형, 3D 기하학, 그리고 물리적 동역학을 동시에 재구성하기 위해 두 가지 주요 구성 요소를 사용합니다.

* **물리 기반 표현 (Physics-Informed Representation)**
* **스프링-질량 모델 (Spring-Mass Models):** 객체를 질량 노드와 가상의 스프링으로 연결된 그래프 형태로 모델링하여 실시간 물리 시뮬레이션을 수행합니다.
* **생성형 형상 모델 (Generative Shape Models):** 제한된 시점과 가려짐(Occlusion)이 있는 단일 혹은 소수의 영상 프레임으로부터 객체의 완전한 3D 기하학적 형태를 복원합니다.
* **3D 가우시안 스플래팅 (3D Gaussian Splatting):** 시각적 렌더링을 담당합니다. 질량 노드에 가우시안 커널을 부착하고 선형 블렌드 스키닝(LBS, Linear Blend Skinning)을 통해 프레임 간 자연스러운 외형 변형을 렌더링합니다.


* **다단계 최적화 기반 역모델링 (Multi-Stage Inverse Modeling)**
* **영차 최적화 (Zero-order Optimization, CMA-ES):** 미분 불가능한 위상(Topology)이나 전역 물리 파라미터(충돌 계수 등)를 초기화합니다. 약 12분이 소요됩니다.
* **일차 최적화 (First-order Optimization):** Nvidia Warp 기반의 미분 가능(Differentiable) 시뮬레이터를 사용하여 스프링 강도(Stiffness)와 같은 밀집된 연속형 파라미터를 기울기 하강법으로 미세 조정합니다. 시각화를 비활성화할 경우 약 5분 내외로 매우 빠르게 완료됩니다.
* **외형 최적화:** 렌더링된 이미지가 실제 RGB-D 관측값과 일치하도록 가우시안 파라미터를 업데이트합니다.



### 2. 레포지토리 주요 기능 (Features)

* **인터랙티브 플레이그라운드 (`interactive_playground.py`):** 구축된 디지털 트윈과 실시간으로 상호작용할 수 있는 환경을 제공합니다. Gradio를 통한 웹 기반 조작 및 원격 가상 키보드 입력을 지원합니다. LBS 초기화 연산을 CPU로 오프로드하여 GPU 메모리 사용량을 약 2GB(Sloth 예제 기준 4GB) 수준으로 최적화했습니다.
* **로봇 물리 제어 지원 (Real2Sim-Eval):** 키보드 조작뿐만 아니라 Gello 기반의 로봇 제어를 지원하여, 로보틱스 시뮬레이터 및 모션 플래닝(Motion Planning)에 활용할 수 있습니다.
* **자기 충돌 연산 가속 (Self-Collision Acceleration):** 옷이나 천과 같은 객체 시뮬레이션 시, 위상학적으로 인접한 입자 쌍의 충돌 검사를 무시하는 알고리즘을 도입하여 최적화 및 추론 속도를 크게 향상시켰습니다.
* **시각화 도구:** 동영상에서 객체에 가해진 힘을 분석하는 힘 시각화(`visualize_force.py`)와 스프링-질량 모델을 통해 근사된 재질 속성 시각화(`visualize_material.py`) 기능을 제공합니다.

### 3. 주요 코드 구조

* `process_data.py` / `script_process_data.py`: 입력 동영상 처리 스크립트. Grounding-SAM-2, TRELLIS 등의 사전 학습 모델을 활용하여 배경을 분리하고 초기 포인트 클라우드를 추출합니다.
* `optimize_cma.py` / `script_optimize.py`: CMA-ES 알고리즘과 Nvidia Warp 기반 시뮬레이터를 활용한 물리 파라미터 최적화 코드입니다.
* `gs_train.py` / `gs_render_dynamics.py`: 3D 가우시안 스플래팅 모델을 학습하고, 물리 시뮬레이션 결과와 결합하여 동적 객체를 렌더링합니다.
* `configs/`: 모델 학습 및 데이터 처리에 필요한 파라미터를 정의한 설정 파일 디렉토리입니다.
* `docker_scripts/`, `env_install/`: 환경 구축 스크립트.

### 4. 환경 설정 및 구동 방식

* **OS 지원:** Linux(기본) 및 Windows(`windows_setup` 브랜치)를 지원합니다.
* **의존성:** CUDA 12.1 이상을 요구하며, Python 3.10 환경에서 Conda를 통해 설치합니다. 특정 하드웨어(예: RTX 5090 + CUDA 12.8)를 위한 맞춤형 설치 스크립트(`5090_env_install.sh`)를 제공합니다.
* **Docker 지원:** `docker_scripts/build.sh`를 통해 NVIDIA RTX 환경에 맞는 컨테이너 빌드를 공식적으로 지원합니다.

해당 레포지토리는 단순한 3D 재구성을 넘어 XR, VR, 로보틱스 분야에서 활용 가능한 'Real-to-Sim(현실에서 시뮬레이션으로)' 물리 엔진 플랫폼으로 확장되는 것을 장기적인 목표로 삼고 활발히 업데이트되고 있습니다.
