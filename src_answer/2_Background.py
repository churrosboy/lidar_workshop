    # 스캔을 지도에 정렬 후 지도에서 떨어진 점만 유지, 나머지는 inf
    def foreground(self, ranges, angle_min, angle_increment) -> list[float]:
        if not self.ready:
            return list(ranges)
        index, points = self._points(ranges, angle_min, angle_increment)
        if len(points) == 0:
            return list(ranges)
        self._align(points)
        aligned = icp.apply(self.pose, points)
        distance, _ = self.target.tree.query(aligned)
        frame = np.asarray(ranges, dtype=float)
        keep = distance > self.margin + self.ratio * frame[index]
        heading = np.arctan2(aligned[:, 1], aligned[:, 0])
        low, high = self.map_sector
        keep &= (heading >= low + 0.02) & (heading <= high - 0.02)
        out = np.full(len(frame), np.inf)
        out[index[keep]] = frame[index[keep]]
        return out.tolist()