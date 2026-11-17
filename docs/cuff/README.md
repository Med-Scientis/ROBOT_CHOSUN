# 팔 받침 도면 — 가공 보류

**현재 파일로 금속 가공·최종 봉제를 진행하지 않는다.** [자체 검토](../CUFF_SELF_REVIEW.md)에서 아래팔 테이퍼/연결 높이와 작은 받침의 찍찍이 길이 문제를 확인했다. 완성 형상·블록 높이·패치 길이는 미정이다.

DXF 3종은 기존 원통형 제안의 이력을 보존한 형상이다. `CUT_REVIEW`는 기존 절단 윤곽, `FORM_REFERENCE`는 성형 참고선, `REVIEW_NOTE`는 가공 보류 안내다. 참고선과 안내 문구를 절단선으로 사용하지 않는다.

JSON의 `original_*`와 `withdrawn_defaults`는 과거 계산 기록이다. `null`인 제작값은 아직 정하지 않았다. `strap`은 여유 재료량이며 완성 봉제 길이가 아니다.

`python check_self_review.py`로 이상화된 경로·높이 계산을 재현할 수 있다. 출력은 `SELF_REVIEW_CALCULATIONS.json`과 대응하며 실물 적합·강도·압박·동력 안전을 판정하지 않는다.
