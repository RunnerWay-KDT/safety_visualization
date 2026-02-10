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

# ── 1. 실제 산책경로 데이터 ──────────────────────────────────────
# 사용자 제공 실제 경로
ROUTE_COORDS = [{"lat": 37.503623, "lng": 127.0361686}, {"lat": 37.5034744, "lng": 127.0357109}, {"lat": 37.5032757, "lng": 127.0349808}, {"lat": 37.5032214, "lng": 127.034856}, {"lat": 37.5037225, "lng": 127.034624}, {"lat": 37.5034661, "lng": 127.0337318}, {"lat": 37.5033264, "lng": 127.0332763}, {"lat": 37.5032076, "lng": 127.0328883}, {"lat": 37.5035132, "lng": 127.032747}, {"lat": 37.5037725, "lng": 127.0326456}, {"lat": 37.5037952, "lng": 127.0325951}, {"lat": 37.5035371, "lng": 127.0317236}, {"lat": 37.5037149, "lng": 127.0317203}, {"lat": 37.5037024, "lng": 127.031294}, {"lat": 37.5036979, "lng": 127.0310975}, {"lat": 37.5032575, "lng": 127.030711}, {"lat": 37.5035326, "lng": 127.0302366}, {"lat": 37.5036472, "lng": 127.0301775}, {"lat": 37.5035368, "lng": 127.0300972}, {"lat": 37.5032782, "lng": 127.0298768}, {"lat": 37.5035018, "lng": 127.0294633}, {"lat": 37.5036461, "lng": 127.0291964}, {"lat": 37.5034817, "lng": 127.0290595}, {"lat": 37.5031468, "lng": 127.0287642}, {"lat": 37.503442, "lng": 127.0282061}, {"lat": 37.5027424, "lng": 127.027595}, {"lat": 37.5022177, "lng": 127.027138}, {"lat": 37.5020422, "lng": 127.0269904}, {"lat": 37.5017209, "lng": 127.0267333}, {"lat": 37.5017411, "lng": 127.0268272}, {"lat": 37.5017209, "lng": 127.0267333}, {"lat": 37.5012818, "lng": 127.0263057}, {"lat": 37.5012183, "lng": 127.0263331}, {"lat": 37.5010877, "lng": 127.0263962}, {"lat": 37.5004716, "lng": 127.0266942}, {"lat": 37.5004435, "lng": 127.0267069}, {"lat": 37.5004035, "lng": 127.0267249}, {"lat": 37.500051, "lng": 127.0268888}, {"lat": 37.4996172, "lng": 127.0270963}, {"lat": 37.4995861, "lng": 127.0271093}, {"lat": 37.4997601, "lng": 127.02767}, {"lat": 37.4995366, "lng": 127.0277645}, {"lat": 37.4992816, "lng": 127.0278926}, {"lat": 37.4991826, "lng": 127.0279418}, {"lat": 37.4987722, "lng": 127.0281278}, {"lat": 37.4992073, "lng": 127.0296209}, {"lat": 37.4988163, "lng": 127.0297916}, {"lat": 37.4988333, "lng": 127.0298487}, {"lat": 37.4991249, "lng": 127.0307769}, {"lat": 37.4990249, "lng": 127.0308263}, {"lat": 37.4988829, "lng": 127.0308964}, {"lat": 37.4986872, "lng": 127.0302498}, {"lat": 37.4982365, "lng": 127.0304695}, {"lat": 37.4971109, "lng": 127.0310202}, {"lat": 37.4967805, "lng": 127.0311708}, {"lat": 37.4964854, "lng": 127.0313073}, {"lat": 37.4965869, "lng": 127.0316469}, {"lat": 37.4964854, "lng": 127.0313073}, {"lat": 37.4959701, "lng": 127.031551}, {"lat": 37.4960817, "lng": 127.0319315}, {"lat": 37.4955886, "lng": 127.0321567}, {"lat": 37.4957224, "lng": 127.0326281}, {"lat": 37.4959974, "lng": 127.0335442}, {"lat": 37.4964603, "lng": 127.0333328}, {"lat": 37.496724, "lng": 127.0341508}, {"lat": 37.4971143, "lng": 127.0339614}, {"lat": 37.4972897, "lng": 127.0345686}, {"lat": 37.4968629, "lng": 127.0347624}, {"lat": 37.4968024, "lng": 127.0349762}, {"lat": 37.4973519, "lng": 127.0352056}, {"lat": 37.497408, "lng": 127.0355944}, {"lat": 37.4978355, "lng": 127.0354016}, {"lat": 37.4979438, "lng": 127.0357635}, {"lat": 37.4984944, "lng": 127.0354893}, {"lat": 37.4985488, "lng": 127.0355684}, {"lat": 37.4990545, "lng": 127.0359986}, {"lat": 37.4991442, "lng": 127.0357973}, {"lat": 37.4996819, "lng": 127.0355492}, {"lat": 37.4998095, "lng": 127.0359777}, {"lat": 37.5002179, "lng": 127.0357568}, {"lat": 37.5003255, "lng": 127.0356986}, {"lat": 37.5003987, "lng": 127.0359367}, {"lat": 37.5004348, "lng": 127.0359783}, {"lat": 37.5004976, "lng": 127.0360252}, {"lat": 37.5005391, "lng": 127.0360735}, {"lat": 37.5005721, "lng": 127.0361486}, {"lat": 37.5006114, "lng": 127.0362733}, {"lat": 37.5006202, "lng": 127.0362961}, {"lat": 37.5006306, "lng": 127.0363162}, {"lat": 37.5006458, "lng": 127.0363256}, {"lat": 37.5006604, "lng": 127.0363283}, {"lat": 37.5007244, "lng": 127.0362948}, {"lat": 37.5005054, "lng": 127.0355884}, {"lat": 37.5006178, "lng": 127.0355339}, {"lat": 37.500985, "lng": 127.0353415}, {"lat": 37.5012147, "lng": 127.036036}, {"lat": 37.50197, "lng": 127.0357051}, {"lat": 37.5021483, "lng": 127.0363132}, {"lat": 37.5025503, "lng": 127.0361212}, {"lat": 37.5030402, "lng": 127.0359037}, {"lat": 37.5034744, "lng": 127.0357109}, {"lat": 37.503623, "lng": 127.0361686}, {"lat": 37.5037358, "lng": 127.0365393}]


# 경로 중심점 계산
CENTER_LAT = sum(p["lat"] for p in ROUTE_COORDS) / len(ROUTE_COORDS)
CENTER_LNG = sum(p["lng"] for p in ROUTE_COORDS) / len(ROUTE_COORDS)


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

# ── 4. 안전점수 계산에 사용할 경로 설정 ──────────────────────────────
route_coords = ROUTE_COORDS
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
