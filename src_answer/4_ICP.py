# 점-대-선 ICP: src를 dst에 맞추는 변환과 대응 비율(fitness) 반환
def icp(src: np.ndarray, dst, init=None, iterations=20, max_dist=0.5, tolerance=1e-4,
        min_pairs=10) -> tuple[np.ndarray, float]:
    target = dst if isinstance(dst, Target) else Target(dst)
    initial = np.eye(3) if init is None else np.array(init, dtype=float)
    if len(src) < min_pairs or not np.all(np.isfinite(initial)):
        return initial, 0.0
    transform = initial
    current = apply(transform, src)

    # 현재 점들의 최근접 대응과 max_dist 이내 여부
    def matches():
        distances, neighbours = target.tree.query(current, distance_upper_bound=max_dist)
        matched = np.isfinite(distances)
        return matched, neighbours

    for _ in range(iterations):
        matched, neighbours = matches()
        if matched.sum() < min_pairs:
            break
        p = current[matched]
        q = target.points[neighbours[matched]]
        n = target.normals[neighbours[matched]]
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
    matched, _ = matches()
    return transform, float(matched.mean())