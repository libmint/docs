---
layout: default
title: ply 크기 줄이는 방법
parent: 나의 공부 게시판
---

`pose` 행렬에서 물체의 크기(Scale)를 0.005 이하로 줄일 때 에러가 발생한다면, 이는 3D 가우시안 스플래팅(3DGS) 렌더러 또는 물리 엔진(PhysTwin) 내부 연산에서 **부동소수점 언더플로우(Floating-point underflow)**나 **0으로 나누기(Divide by zero)**와 같은 수치적 불안정성(Numerical Instability)이 발생했기 때문입니다.

시뮬레이션 엔진은 보통 1.0(1m)을 기준으로 최적화되어 있어, 0.005(5mm)보다 작은 단위의 행렬 곱셈이 반복되면 가우시안 포인트의 공분산(크기 및 형태) 행렬이 수학적으로 붕괴하여 렌더링을 멈추게 됩니다.

따라서 yaml 설정 파일에서 강제로 0.005 이하로 줄이는 방식은 불가능하며, **원본 파일 자체의 크기를 줄여서 시뮬레이터에 1.0 비율로 넣는 근본적인 해결책**을 사용해야 합니다.

다음의 두 가지 방법 중 하나를 선택하여 해결할 수 있습니다.

### 방법 1: SuperSplat에서 원본 크기 줄이기 (가장 쉽고 권장됨)

이전에 원점 정렬을 위해 안내해 드렸던 웹 기반 편집기를 사용하여 `.ply` 파일 자체의 크기 데이터를 수정하는 방식입니다.

1. **SuperSplat 접속:** 웹 브라우저에서 [SuperSplat](https://playcanvas.com/supersplat/editor)을 엽니다.
2. **파일 열기:** 크기를 줄이고자 하는 `splat.ply` 원본 파일을 드래그하여 엽니다.
3. **크기 조절 (Scale):**
* 물체를 전체 선택합니다.
* 화면의 **Scale(크기 조절) 도구**를 선택합니다.
* 마우스를 드래그하여 물체의 크기를 시각적으로 아주 작게 줄입니다. (격자 눈금을 보며 대략적인 크기를 가늠합니다.)


4. **저장 및 덮어쓰기:**
* 크기를 줄인 상태로 Export(내보내기)하여 기존 `splat.ply` 파일을 덮어씁니다.


5. **yaml 설정 복구:**
* `my_object.yaml` 파일의 `pose` 행렬 크기 값을 다시 안전한 범위(예: 1.0 또는 0.1)로 되돌려놓고 실행합니다.
```yaml
pose: [
  1.0, 0.0, 0.0, 0.36,
  0.0, 1.0, 0.0, 0.0,
  0.0, 0.0, 1.0, 0.05,
  0.0, 0.0, 0.0, 1.0
]

```





### 방법 2: 파이썬 스크립트로 크기 변환 후 저장하기

SuperSplat 편집기가 익숙하지 않거나 정확한 배율(예: 정확히 1/1000 크기로 축소)로 줄이고 싶다면, 파이썬 코드 몇 줄로 `.ply` 파일 내부의 스케일 속성을 직접 조작할 수 있습니다.

1. `scale_down.py`라는 파이썬 파일을 프로젝트 폴더에 만들고 아래 코드를 붙여넣습니다.
```python
from plyfile import PlyData, PlyElement
import numpy as np

# 1. 파일 경로 설정
input_file = 'log/gs/scans/my_object/splat.ply'
output_file = 'log/gs/scans/my_object/splat_scaled.ply'
scale_factor = 0.001 # 1000분의 1로 축소

# 2. 파일 읽기
plydata = PlyData.read(input_file)
vertex = plydata['vertex']

# 3. 위치(x, y, z) 및 가우시안 크기(scale_0, scale_1, scale_2) 축소
vertex['x'] = vertex['x'] * scale_factor
vertex['y'] = vertex['y'] * scale_factor
vertex['z'] = vertex['z'] * scale_factor

# 가우시안 크기는 보통 로그 스케일로 저장되므로 선형 변환이 필요할 수 있으나, 
# 단순 스케일 다운 시에는 아래와 같이 더하기 연산으로 처리합니다. (지수법칙: log(a*b) = log(a) + log(b))
vertex['scale_0'] = vertex['scale_0'] + np.log(scale_factor)
vertex['scale_1'] = vertex['scale_1'] + np.log(scale_factor)
vertex['scale_2'] = vertex['scale_2'] + np.log(scale_factor)

# 4. 새 파일로 저장
PlyData([PlyElement.describe(vertex, 'vertex')], text=False).write(output_file)
print("축소 완료!")

```


2. 터미널에서 `uv pip install plyfile numpy`를 입력하여 필요한 라이브러리를 설치합니다.
3. `python scale_down.py`를 실행합니다.
4. 생성된 `splat_scaled.ply` 파일 이름을 `splat.ply`로 변경하고 시뮬레이터를 다시 실행합니다.

원본 파일(`.ply`)과 물리 엔진 뼈대(`.obj`)의 크기를 1번 방법(SuperSplat)으로 1차 축소하신 뒤, yaml에서는 안전한 `1.0` 스케일 값으로 테스트를 이어나가시는 것을 권장합니다.
