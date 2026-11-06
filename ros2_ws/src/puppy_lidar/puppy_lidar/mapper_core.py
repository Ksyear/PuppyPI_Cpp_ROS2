"""오도메트리 없는 2D 점유격자 지도작성(ROS 비의존, 단독 시험 가능).

팀의 이전 ROS 2 노드 lidar_mapping_cpp(이 저장소의 이전 버전, Apache-2.0, tinySLAM 계열)의 방식을 numpy 로 옮겼다:
  1) 셀마다 로그오즈(int16, HIT +24 / MISS -6, ±120 포화)
  2) 새 스캔마다 언덕오르기 스캔매칭(±x, ±y, ±θ, 개선 없으면 보폭 절반, 4단계)으로 자세 보정
  3) 5 cm 또는 0.05 rad 이상 움직였을 때만 지도에 통합(광선 경로 비움, 끝점 점유)
바꾼 점(이유):
  - 점수는 점유 로그오즈를 가우시안으로 흐린 지도에서 계산 — 원래 방식(끝점 셀 값 합)은 수렴 범위가 셀 1~2개라
    보행 흔들림·빠른 이동(25 cm/s → 스캔당 2.5 cm)에서 쉽게 놓친다
  - 직전 두 자세로 등속 예측한 값에서 탐색 시작, 한 스캔에 0.3 m·0.35 rad 넘게 튀면 예측값 유지
한계: 루프 클로저가 없어 큰 공간을 오래 돌면 오차가 쌓인다(팀 README 와 같음).
"""

import math
import time
from collections import deque

import numpy as np

HIT, MISS, LO_MIN, LO_MAX = 24, -6, -120, 120


class OccupancyMapper(object):
    def __init__(self, size_m=30.0, resolution=0.05, max_range=11.5, min_range=0.12,
                 min_travel=0.05, min_turn=0.05, lin_step=0.05, ang_step=0.03, halvings=4, max_iters=200,
                 max_jump=0.3, max_jump_ang=0.35, blur_sigma_cells=1.0):
        self.res = float(resolution)
        self.n = int(round(size_m / resolution))
        self.origin = -self.n * self.res / 2.0  # 시작 위치가 지도 중앙
        self.lo = np.zeros((self.n, self.n), dtype=np.int16)  # [cy, cx]
        self.known = np.zeros((self.n, self.n), dtype=bool)
        self.score_map = np.zeros((self.n, self.n), dtype=np.float32)
        self.max_range, self.min_range = max_range, min_range
        self.min_travel, self.min_turn = min_travel, min_turn
        self.lin_step, self.ang_step, self.halvings, self.max_iters = lin_step, ang_step, halvings, max_iters
        self.max_jump, self.max_jump_ang = max_jump, max_jump_ang
        self.blur = blur_sigma_cells
        self.pose = np.zeros(3)  # x, y, θ (map 기준)
        self.prev_pose = np.zeros(3)
        self.last_integrated = np.zeros(3)
        self.initialized = False
        self.scans = 0
        self.integrations = 0
        self.rejected_jumps = 0
        self.trail = []
        self.last_world = np.zeros((0, 2))  # 가장 최근 스캔(지도 좌표) — live 표시용
        self.recent = deque(maxlen=60)  # (시각, 지도 좌표 점) — live 표시용 최근 스캔(2026-11-05)

    # ── 좌표 ──
    def to_cells(self, xy):
        return np.floor((xy - self.origin) / self.res).astype(np.int64)

    @staticmethod
    def scan_points(ranges, angle_min, angle_inc, min_range, max_range, laser_xyyaw=(0.0, 0.0, 0.0)):
        r = np.asarray(ranges, dtype=np.float64)
        a = angle_min + angle_inc * np.arange(r.size)
        ok = np.isfinite(r) & (r >= min_range) & (r <= max_range)
        r, a = r[ok], a[ok] + laser_xyyaw[2]
        return np.stack([laser_xyyaw[0] + r * np.cos(a), laser_xyyaw[1] + r * np.sin(a)], axis=1)

    def _world(self, pose, pts):
        c, s = math.cos(pose[2]), math.sin(pose[2])
        return np.stack([pose[0] + c * pts[:, 0] - s * pts[:, 1], pose[1] + s * pts[:, 0] + c * pts[:, 1]], axis=1)

    # ── 점수·매칭 ──
    def score(self, pose, pts):
        cells = self.to_cells(self._world(pose, pts))
        ok = (cells[:, 0] >= 0) & (cells[:, 0] < self.n) & (cells[:, 1] >= 0) & (cells[:, 1] < self.n)
        cells = cells[ok]
        return float(self.score_map[cells[:, 1], cells[:, 0]].sum())

    def match(self, init, pts):
        best = np.array(init, dtype=np.float64)
        best_score = self.score(best, pts)
        lin, ang, level, iters = self.lin_step, self.ang_step, 0, 0
        while level <= self.halvings and iters < self.max_iters:
            improved = False
            for d in ((lin, 0, 0), (-lin, 0, 0), (0, lin, 0), (0, -lin, 0), (0, 0, ang), (0, 0, -ang)):
                cand = best + d
                sc = self.score(cand, pts)
                if sc > best_score:
                    best, best_score, improved = cand, sc, True
            if not improved:
                lin, ang, level = lin / 2.0, ang / 2.0, level + 1
            iters += 1
        return best, best_score

    # ── 통합 ──
    def integrate(self, pose, pts):
        end = self._world(pose, pts)
        x0, y0 = pose[0], pose[1]
        d = np.hypot(end[:, 0] - x0, end[:, 1] - y0)
        steps = np.maximum(1, np.ceil(d / (self.res * 0.7)).astype(np.int64))
        total = int(steps.sum())
        ray = np.repeat(np.arange(len(end)), steps)
        k = np.arange(total) - np.repeat(np.cumsum(steps) - steps, steps)
        f = k / steps[ray]  # [0, 1) — 끝점 제외
        free = self.to_cells(np.stack([x0 + f * (end[ray, 0] - x0), y0 + f * (end[ray, 1] - y0)], axis=1))
        hit = self.to_cells(end)
        n = self.n
        inb = lambda c: c[(c[:, 0] >= 0) & (c[:, 0] < n) & (c[:, 1] >= 0) & (c[:, 1] < n)]
        free_i = np.unique(inb(free)[:, 1] * n + inb(free)[:, 0]) if total else np.zeros(0, np.int64)
        hit_i = np.unique(inb(hit)[:, 1] * n + inb(hit)[:, 0])
        free_i = np.setdiff1d(free_i, hit_i, assume_unique=True)
        lo = self.lo.reshape(-1)
        kn = self.known.reshape(-1)
        lo[free_i] = np.clip(lo[free_i].astype(np.int32) + MISS, LO_MIN, LO_MAX).astype(np.int16)
        lo[hit_i] = np.clip(lo[hit_i].astype(np.int32) + HIT, LO_MIN, LO_MAX).astype(np.int16)
        kn[free_i] = True
        kn[hit_i] = True
        self._refresh_score()
        self.integrations += 1

    def _refresh_score(self):
        occ = np.clip(self.lo, 0, None).astype(np.float32)
        if self.blur > 0:
            import cv2
            occ = cv2.GaussianBlur(occ, (0, 0), self.blur)
        self.score_map = occ

    # ── 한 스캔 처리 ──
    def process(self, pts, now=None):
        """base 기준 점(N,2) → 갱신된 자세. 점이 30개 미만이면 무시. now = 단조 시각(live 표시용, 없으면 현재)."""
        if len(pts) < 30:
            return self.pose.copy()
        now = time.monotonic() if now is None else now
        self.scans += 1
        if not self.initialized:
            self.last_world = self._world(self.pose, pts)
            self.recent.append((now, self.last_world))
            self.integrate(self.pose, pts)
            self.last_integrated = self.pose.copy()
            self.initialized = True
            self.trail.append(self.pose[:2].copy())
            return self.pose.copy()
        pred = self.pose + (self.pose - self.prev_pose)  # 등속 예측
        est, _ = self.match(pred, pts)
        jump = math.hypot(est[0] - self.pose[0], est[1] - self.pose[1])
        dth = abs(math.atan2(math.sin(est[2] - self.pose[2]), math.cos(est[2] - self.pose[2])))
        if jump > self.max_jump or dth > self.max_jump_ang:
            self.rejected_jumps += 1
            est = self.pose.copy()
        self.prev_pose, self.pose = self.pose, est
        self.last_world = self._world(est, pts)
        self.recent.append((now, self.last_world))
        moved = math.hypot(est[0] - self.last_integrated[0], est[1] - self.last_integrated[1])
        turned = abs(math.atan2(math.sin(est[2] - self.last_integrated[2]), math.cos(est[2] - self.last_integrated[2])))
        if moved >= self.min_travel or turned >= self.min_turn:
            self.integrate(est, pts)
            self.last_integrated = est.copy()
            self.trail.append(est[:2].copy())
            if len(self.trail) > 5000:
                del self.trail[:1000]
        return self.pose.copy()

    # ── 출력 ──
    def occupancy(self):
        """ROS OccupancyGrid data(행 = y, -1 미탐사, 0~100)."""
        p = 1.0 / (1.0 + np.exp(-self.lo.astype(np.float32) * 0.05))
        occ = np.rint(p * 100).astype(np.int8)
        occ[~self.known] = -1
        return occ

    def pgm(self):
        """map_server 규약 PGM(위쪽 = y 최대): 점유 0, 빈 254, 미탐사 205."""
        occ = self.occupancy()
        img = np.full(occ.shape, 205, np.uint8)
        img[(occ >= 0) & (occ <= 25)] = 254
        img[occ >= 65] = 0
        return np.flipud(img)

    def explored_bounds(self, margin_m=1.0):
        ys, xs = np.nonzero(self.known)
        if xs.size == 0:
            return 0, self.n, 0, self.n
        m = int(margin_m / self.res)
        return (max(0, xs.min() - m), min(self.n, xs.max() + m + 1), max(0, ys.min() - m), min(self.n, ys.max() + m + 1))

    def render_live_with_info(self, max_px=640, window_s=1.0, now=None):
        """live 표시(2026-11-05): 최근 window_s 초 안의 스캔 점과 로봇만 그린다(오래된 점일수록 흐리게).
        누적 지도·지나온 길은 그리지 않는다(위치 추정용 지도는 내부에만 둔다). 창·좌표 정보는 render_with_info 와 같은 규칙.
        한 장만 그리면 보행 흔들림·측정 잡음으로 점이 깜빡여서 짧게 겹친다. 사라진 물체는 window_s 안에 지워진다."""
        import cv2
        x0, x1, y0, y1 = self.explored_bounds()
        w, hgt = x1 - x0, y1 - y0
        scale = max(1, int(max_px / max(w, hgt)))
        img = np.full((hgt * scale, w * scale, 3), 235, np.uint8)
        h = img.shape[0]

        def px(x, y):
            cx = (x - self.origin) / self.res - x0
            cy = (y - self.origin) / self.res - y0
            return int(cx * scale), int(h - cy * scale)

        grid = max(1, int(round(0.5 / self.res)))  # 0.5 m 격자(거리 감)
        for gx in range(0, w, grid):
            cv2.line(img, (gx * scale, 0), (gx * scale, h - 1), (215, 215, 215), 1)
        for gy in range(0, hgt, grid):
            cv2.line(img, (0, h - gy * scale), (img.shape[1] - 1, h - gy * scale), (215, 215, 215), 1)
        # 점 그리기(배열 연산): 새 점일수록 진하게(30), 창 끝에 가까울수록 흐리게(170). 255 = 점 없음.
        now = time.monotonic() if now is None else now
        window_s = max(0.05, float(window_s))
        canvas = np.full((h, img.shape[1]), 255, np.uint8)
        scans = [(t, w) for t, w in self.recent if now - t <= window_s] or ([(now, self.last_world)] if len(self.last_world) else [])
        for t, wpts in scans:
            if len(wpts) == 0:
                continue
            age = min(1.0, max(0.0, (now - t) / window_s))
            val = np.uint8(30 + 140 * age)
            u = (((wpts[:, 0] - self.origin) / self.res - x0) * scale).astype(np.int64)
            v = (h - ((wpts[:, 1] - self.origin) / self.res - y0) * scale).astype(np.int64)
            ok = (u >= 0) & (u < canvas.shape[1]) & (v >= 0) & (v < h)
            u, v = u[ok], v[ok]
            canvas[v, u] = np.minimum(canvas[v, u], val)
        r = max(1, scale // 2)
        canvas = cv2.erode(canvas, np.ones((2 * r + 1, 2 * r + 1), np.uint8))
        mask = canvas < 255
        img[mask] = np.repeat(canvas[mask][:, None], 3, axis=1)
        p0 = px(self.pose[0], self.pose[1])
        p1 = px(self.pose[0] + 0.35 * math.cos(self.pose[2]), self.pose[1] + 0.35 * math.sin(self.pose[2]))
        cv2.circle(img, p0, max(3, scale), (0, 0, 230), -1)
        cv2.arrowedLine(img, p0, p1, (0, 0, 230), max(2, scale // 2), tipLength=0.35)
        info = {"width_px": int(img.shape[1]), "height_px": int(img.shape[0]),
                "left_m": float(self.origin + x0 * self.res), "bottom_m": float(self.origin + y0 * self.res),
                "px_per_m": float(scale / self.res), "resolution_m": float(self.res), "display": "live",
                "robot": {"x_m": float(self.pose[0]), "y_m": float(self.pose[1]), "yaw_rad": float(self.pose[2]),
                          "u_px": p0[0], "v_px": p0[1]}}
        return img, info

    def render(self, max_px=640):
        """보기용 BGR 이미지: 탐사 범위만 잘라 확대, 지나온 길(파랑)·현재 자세(빨강 화살표)."""
        return self.render_with_info(max_px)[0]

    def render_with_info(self, max_px=640):
        """(이미지, 좌표 정보). 정보로 지도 좌표(m) ↔ 이미지 픽셀을 바꾼다(README 'PuppyPi Media v1'):
        u = (x - left_m) * px_per_m,  v = height_px - (y - bottom_m) * px_per_m   (u 오른쪽, v 아래쪽, 원점 = 왼쪽 위)."""
        import cv2
        x0, x1, y0, y1 = self.explored_bounds()
        gray = np.flipud(self.pgm())  # 행 = y 증가 방향
        crop = gray[y0:y1, x0:x1]
        scale = max(1, int(max_px / max(crop.shape)))
        img = cv2.cvtColor(np.ascontiguousarray(np.flipud(crop)), cv2.COLOR_GRAY2BGR)
        img = cv2.resize(img, (crop.shape[1] * scale, crop.shape[0] * scale), interpolation=cv2.INTER_NEAREST)
        h = img.shape[0]

        def px(x, y):
            cx = (x - self.origin) / self.res - x0
            cy = (y - self.origin) / self.res - y0
            return int(cx * scale), int(h - cy * scale)

        if len(self.trail) > 1:
            pts = np.array([px(x, y) for x, y in self.trail[-2000:]], np.int32)
            cv2.polylines(img, [pts], False, (255, 140, 0), max(1, scale // 2))
        p0 = px(self.pose[0], self.pose[1])
        p1 = px(self.pose[0] + 0.35 * math.cos(self.pose[2]), self.pose[1] + 0.35 * math.sin(self.pose[2]))
        cv2.circle(img, p0, max(3, scale), (0, 0, 230), -1)
        cv2.arrowedLine(img, p0, p1, (0, 0, 230), max(2, scale // 2), tipLength=0.35)
        info = {"width_px": int(img.shape[1]), "height_px": int(img.shape[0]),
                "left_m": float(self.origin + x0 * self.res), "bottom_m": float(self.origin + y0 * self.res),
                "px_per_m": float(scale / self.res), "resolution_m": float(self.res),
                "robot": {"x_m": float(self.pose[0]), "y_m": float(self.pose[1]), "yaw_rad": float(self.pose[2]),
                          "u_px": p0[0], "v_px": p0[1]}}
        return img, info
