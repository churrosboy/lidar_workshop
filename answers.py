# 1단계 — 군집화 cluster_scan
# 파일: src/gl5_detection/gl5_detection/detection_core.py
# 구멍: 89–101행, `for i, distance in enumerate(ranges):` 루프 본문 (들여쓰기 8칸)

        if not math.isfinite(distance) or distance <= 0 or not range_min <= distance <= range_max:
            continue
        angle = angle_min + i * angle_increment
        point = (distance * math.cos(angle), distance * math.sin(angle))
        if polygon is not None and not inside(point, polygon):
            flush()
            continue
        if current and math.dist(current[-1], point) > max_gap:
            flush()
        current.append(point)


# 2단계 — 배경 차분 BackgroundModel.foreground
# 파일: src/gl5_detection/gl5_detection/background.py
# 구멍: 91–103행, `frame = np.asarray(ranges, dtype=float)` 다음 (들여쓰기 8칸)

        distance, _ = self.target.tree.query(aligned)
        keep = distance > self.margin + self.ratio * frame[index]
        heading = np.arctan2(aligned[:, 1], aligned[:, 0])
        low, high = self.map_sector
        keep &= (heading >= low + 0.02) & (heading <= high - 0.02)
        out = np.full(len(frame), np.inf)
        out[index[keep]] = frame[index[keep]]
        return out.tolist()


# 3단계 — 진입 예측 predict_entry
# 파일: src/gl5_detection/gl5_detection/prediction.py
# 구멍: 9–21행, 함수 본문 전체 (들여쓰기 4칸)

    if math.hypot(*velocity) < min_speed or inside(center, polygon):
        return None
    steps = int(round(horizon / step))
    for k in range(1, steps + 1):
        t = k * step
        point = (center[0] + velocity[0] * t, center[1] + velocity[1] * t)
        if inside(point, polygon):
            return t, point
    return None


# 4단계 — ICP 반복 스텝 icp
# 파일: src/gl5_localization/gl5_localization/icp.py
# 구멍: 85–97행, `for _ in range(iterations):` 루프 안, p / q / n을 구한 다음 (들여쓰기 8칸)

        jacobian = np.column_stack((n[:, 0], n[:, 1], n[:, 0] * -p[:, 1] + n[:, 1] * p[:, 0]))
        residual = np.einsum('ij,ij->i', n, q - p)
        (dx, dy, dtheta), *_ = np.linalg.lstsq(jacobian, residual, rcond=1e-6)
        if (not all(map(math.isfinite, (dx, dy, dtheta))) or math.hypot(dx, dy) > MAX_STEP_M
                or abs(dtheta) > MAX_STEP_RAD):
            return initial, 0.0
        step = make_transform(dx, dy, dtheta)
        transform = step @ transform
        current = apply(step, current)
        if math.hypot(dx, dy) < tolerance and abs(dtheta) < tolerance:
            break
