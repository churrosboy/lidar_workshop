"""Short-term association of observed boxes; velocities are relative to the sensor."""
import math
from collections import deque
from roi_geometry import bounds


class BoxTracker:
    def __init__(self, match_distance=0.4, max_age=0.5, window=0.4):
        self.match_distance, self.max_age, self.window = match_distance, max_age, window
        self.next_id = 1
        self.tracks = {}

    def reset(self):
        self.tracks.clear()

    def update(self, groups, now):
        self.tracks = {key:t for key,t in self.tracks.items() if 0 <= now-t['time'] <= self.max_age}
        boxes = [bounds(group) for group in groups]
        centers = [((a+c)/2, (b+d)/2) for a,b,c,d in boxes]
        candidates = []
        for key, track in self.tracks.items():
            dt = now - track['time']
            predicted = tuple(track['center'][i] + track['velocity'][i] * dt for i in (0,1))
            for index, center in enumerate(centers):
                distance = math.dist(predicted, center)
                if distance <= self.match_distance:
                    candidates.append((distance, key, index))
        used_tracks, assignments = set(), {}
        for _, key, index in sorted(candidates):
            if key not in used_tracks and index not in assignments:
                used_tracks.add(key)
                assignments[index] = key
        visible = []
        for index, center in enumerate(centers):
            if index not in assignments:
                key = self.next_id
                self.next_id += 1
                track = {'id':key, 'history':deque(), 'velocity':(0.0,0.0), 'time':now}
                self.tracks[key] = track
            else:
                track = self.tracks[assignments[index]]
                if now - track['time'] > 0.2:
                    track['history'].clear()
                    track['velocity'] = (0.0,0.0)
            history = track['history']
            if not history or now > history[-1][0]:
                history.append((now, *center))
            while len(history) > 1 and now - history[0][0] > self.window:
                history.popleft()
            speed = None
            if len(history) >= 3 and history[-1][0] - history[0][0] >= 0.15:
                # Regression over a short window reduces frame-to-frame range noise.
                times = [sample[0] - history[0][0] for sample in history]
                mean_t = sum(times)/len(times)
                denominator = sum((t-mean_t)**2 for t in times)
                velocities = []
                for axis in (1,2):
                    mean_p = sum(sample[axis] for sample in history)/len(history)
                    velocities.append(sum((t-mean_t)*(sample[axis]-mean_p)
                                          for t,sample in zip(times,history))/denominator)
                track['velocity'] = tuple(velocities)
                speed = math.hypot(*velocities)
            track.update(center=center, time=now, box=boxes[index], speed=speed)
            visible.append(track.copy())
        return visible
