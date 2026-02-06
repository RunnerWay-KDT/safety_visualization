"""
역삼역 중심 ~1km 순환 산책경로에 대해 safety_score를 계산하고
시각화에 필요한 모든 데이터를 JSON으로 출력한다.
"""
from __future__ import annotations

import json
import math
import os
import sys

# safety_score.py 가 같은 폴더에 있으므로 경로 추가
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from safety_score import (
    SafetyParams,
    compute_safety_score,
    load_cctv_points,
    load_lamp_points,
)

# ── 1. 역삼역 중심 ~1km 순환 산책경로 생성 ──────────────────────────
# 역삼역 좌표
CENTER_LAT = 37.50068
CENTER_LNG = 127.03647

def make_circular_route(
    center_lat: float,
    center_lng: float,
    radius_m: float = 160.0,   # 반지름 160m → 둘레 ≈ 1005m
    n_points: int = 40,
) -> list[dict[str, float]]:
    """원형 순환경로를 WGS84 좌표로 생성한다."""
    # 위도 1도 ≈ 111,320m, 경도 1도 ≈ 111,320 * cos(lat) m
    lat_per_m = 1.0 / 111_320.0
    lng_per_m = 1.0 / (111_320.0 * math.cos(math.radians(center_lat)))

    coords = []
    for i in range(n_points + 1):  # +1 으로 시작점=끝점 닫기
        angle = 2.0 * math.pi * i / n_points
        dlat = radius_m * math.cos(angle) * lat_per_m
        dlng = radius_m * math.sin(angle) * lng_per_m
        coords.append({
            "lat": round(center_lat + dlat, 7),
            "lng": round(center_lng + dlng, 7),
        })
    return coords


# ── 2. 데이터 로드 ─────────────────────────────────────────────────
HERE = os.path.dirname(os.path.abspath(__file__))
CCTV_CSV = os.path.join(HERE, "Seoul_CCTV.csv")
LAMP_CSV = os.path.join(HERE, "서울특별시_강남구_보안등정보_20250103.csv")

print(f"[INFO] CCTV CSV : {CCTV_CSV}  (exists={os.path.exists(CCTV_CSV)})")
print(f"[INFO] LAMP CSV : {LAMP_CSV}  (exists={os.path.exists(LAMP_CSV)})")

cctv_raw = load_cctv_points(CCTV_CSV)
lamp_raw = load_lamp_points(LAMP_CSV)

print(f"[INFO] CCTV 데이터 수: {len(cctv_raw)}")
print(f"[INFO] 보안등 데이터 수: {len(lamp_raw)}")

# ── 3. 경로 주변(±0.01도 ≈ 1.1km)만 필터링하여 속도 향상 ──────────
MARGIN = 0.008  # 약 800m
min_lat = CENTER_LAT - MARGIN
max_lat = CENTER_LAT + MARGIN
min_lng = CENTER_LNG - MARGIN
max_lng = CENTER_LNG + MARGIN

cctv_near = [
    p for p in cctv_raw
    if min_lat <= p["lat"] <= max_lat and min_lng <= p["lon"] <= max_lng
]
lamp_near = [
    p for p in lamp_raw
    if min_lat <= p["lat"] <= max_lat and min_lng <= p["lon"] <= max_lng
]

print(f"[INFO] 경로 주변 CCTV: {len(cctv_near)}")
print(f"[INFO] 경로 주변 보안등: {len(lamp_near)}")

# infra_points 합치기 (type 키 보존)
infra_points = cctv_near + lamp_near

# ── 4. 순환 경로 생성 ──────────────────────────────────────────────
route_coords = make_circular_route(CENTER_LAT, CENTER_LNG, radius_m=160.0, n_points=40)
print(f"[INFO] 경로 좌표 수: {len(route_coords)}")

# ── 5. 안전점수 계산 (debug=True 로 상세 데이터) ──────────────────
params = SafetyParams()
result = compute_safety_score(
    route_coords=route_coords,
    infra_points=infra_points,
    params=params,
    debug=True,
)

print(f"\n{'='*50}")
print(f"  안전 점수: {result['score']} / 100")
print(f"{'='*50}")
print(f"  경로 길이      : {result['features']['route_len_m']:.1f} m")
print(f"  CCTV 수 (회랑) : {result['features']['cctv_count']}")
print(f"  보안등 수(회랑) : {result['features']['lamp_count']}")
print(f"  밀도 점수       : {result['features']['density_score']:.4f}")
print(f"  커버리지 점수   : {result['features']['coverage_score']:.4f}")
print(f"  최대 공백(m)    : {result['features']['max_gap_m']:.1f}")
print(f"  공백 페널티     : {result['features']['gap_penalty']:.4f}")
print(f"{'='*50}\n")

# ── 6. JSON 출력 ───────────────────────────────────────────────────
output = {
    "center": {"lat": CENTER_LAT, "lng": CENTER_LNG},
    "route": route_coords,
    "score": result["score"],
    "raw01": result["raw01"],
    "features": result["features"],
    "params": {
        "sample_step_m": params.sample_step_m,
        "lamp_radius_m": params.lamp_radius_m,
        "cctv_radius_m": params.cctv_radius_m,
        "density_k_per_km": params.density_k_per_km,
        "gap_G_m": params.gap_G_m,
        "w_density": params.w_density,
        "w_coverage": params.w_coverage,
        "w_gap": params.w_gap,
        "rescale_max": params.rescale_max,
    },
    "debug": result.get("debug", {}),
    "infra_cctv": [{"lat": p["lat"], "lng": p["lon"]} for p in cctv_near],
    "infra_lamp": [{"lat": p["lat"], "lng": p["lon"]} for p in lamp_near],
}

out_path = os.path.join(HERE, "safety_data.json")
with open(out_path, "w", encoding="utf-8") as f:
    json.dump(output, f, ensure_ascii=False, indent=2)

print(f"[OK] {out_path} 저장 완료  ({os.path.getsize(out_path):,} bytes)")
