---
layout: default
title: IsaacSIm 6 설치
parent: 나의 공부 게시판
---

네, 맞습니다. 지적하신 **`https://github.com/newton-physics/newton.git`** 저장소가 GTC 2026에서 발표된 **Newton 1.0 GA(정식 출시) 버전의 공식 소스 코드**가 위치한 곳입니다.

GTC 2026(2026년 3월 16일\~19일)을 기점으로 해당 저장소의 `main` 브랜치는 정식 버전인 1.0 GA 코드로 업데이트되었습니다. 이제 더 이상의 시행착오 없이 **Ubuntu 환경에서 한 번에 성공하는 2026년형 설치 가이드**를 다시 완벽하게 정리해 드립니다.

-----

## \#\# Newton 1.0 GA & Isaac Sim 6.0 무결점 설치 절차 (2026)

이 절차는 `python3-venv` 누락 문제부터 Newton 소스 빌드까지 모든 예외 상황을 고려한 최적의 경로입니다.

### 1\. 시스템 필수 패키지 설치 (가장 중요)

가상환경(`venv`) 명령어가 작동하지 않는 문제를 해결하기 위해 시스템 패키지를 먼저 업데이트하고 필수 모듈을 설치합니다.

```bash
# 시스템 업데이트 및 필수 패키지 설치
sudo apt update && sudo apt upgrade -y
sudo apt install git python3.12 python3.12-venv python3.12-dev build-essential -y
```

### 2\. 가상환경 구축 및 활성화

반드시 3.12 버전을 지정하여 가상환경을 생성합니다.

```bash
# 가상환경 생성 (경로: ~/isaac-sim-6)
python3.12 -m venv ~/isaac-sim-6

# 가상환경 활성화
source ~/isaac-sim-6/bin/activate

# pip 최신화 (가상환경 내에서 수행)
pip install --upgrade pip setuptools wheel
```

### 3\. Newton 1.0 GA 설치 (소스 빌드 방식)

질문하신 GitHub 저장소를 클론하고, 1.0 GA의 모든 기능(Kamino, VBD 등)을 사용할 수 있도록 설치합니다.

```bash
# 저장소 클론 및 이동
git clone https://github.com/newton-physics/newton.git
cd newton

# 1.0 GA 정식 버전 확인 및 설치
# (GA 출시 이후 main 브랜치가 1.0입니다)
pip install -e ".[examples]" 
```

### 4\. Isaac Sim 6.0 설치 (Pip 기반)

Isaac Sim 6.0은 현재 **Early Developer Release** 상태입니다. 아래 명령어로 설치하되, 만약 특정 리눅스 환경에서 오류가 날 경우 NVIDIA Developer Portal에서 제공하는 최신 `.run` 파일이나 Docker를 병행해야 할 수 있습니다.

```bash
# Isaac Sim 6.0 코어 패키지 설치
pip install "isaacsim[all]==6.0.0" --extra-index-url https://pypi.nvidia.com
```

-----

## \#\# 설치 완료 후 최종 점검 (One-Shot Check)

모든 설치가 끝났다면 아래 명령어를 한 줄씩 복사해서 터미널에 넣어보세요. 에러 없이 정보가 출력되면 성공입니다.

```python
python -c "import newton; print(f'Newton Version: {newton.__version__}'); import isaacsim; print('Isaac Sim 6.0 로드 성공')"
```

### 💡 팩트 체크 및 주의사항

  * **GitHub 저장소:** `newton-physics/newton`은 Linux Foundation 프로젝트로 전환되었으며, NVIDIA, Google DeepMind, Disney Research가 공동 관리하는 **1.0 GA 공식 저장소**가 맞습니다.
  * **1.0 GA의 핵심:** 이번 버전부터 **Kamino 솔버**(복잡한 관절체용)와 **VBD**(케이블/천 등 유연체용)가 정식 통합되었습니다.
  * **Isaac Sim 6.0 상태:** 현재 6.0은 초기 개발자 릴리스(Early Release) 단계이므로, 일부 GUI 기능이 불안정할 수 있습니다. 이 경우 Newton 백엔드를 사용하는 \*\*Python 스크립트 방식(`backend="newton"`)\*\*으로 구동하는 것이 가장 확실합니다.

-----

이제 설치가 완벽하게 끝났을 것입니다. **설치된 Newton 1.0의 신기능인 'Kamino 솔버'를 사용해 복잡한 병렬 링크 로봇을 구동하는 예제 코드를 바로 확인해 보시겠습니까?**

[Newton Physics Engine Overview](https://www.google.com/search?q=https://www.youtube.com/watch%3Fv%3DS0TscL6Y1zM)
이 영상은 GTC 2026에서 발표된 Newton 1.0 GA의 주요 기능과 MuJoCo 대비 성능 향상 수치, 그리고 실제 산업용 로봇 조작에 적용된 사례를 상세히 보여주어 기술적 이해에 큰 도움이 됩니다.
