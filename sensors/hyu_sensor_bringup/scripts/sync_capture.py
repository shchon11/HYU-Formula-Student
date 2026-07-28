#!/usr/bin/env python3
"""
LiDAR(RS16) + ZED 실시간 동기 캡처 도구 (방식 A: 호스트 클럭 + ApproximateTime)

창 구성:  [ 카메라 영상 | LiDAR 탑다운(BEV) ]  나란히 실시간 표시
  - 좌: ZED 영상, 우: LiDAR 포인트 top-down (전방=위, ±BEV_RANGE m)
  - 상단 오버레이: LiDAR 포인트수 / 현재 Δt / Δt 통계(mean±std, min~max) / 저장수
키:
  [SPACE]/[C] = 그 순간 동기 쌍 저장  ·  [Q]/[ESC] = 종료
저장(captures/<세션>/):
  NNNN_L.png/_R.png  좌/우 스테레오 이미지 (right 토픽이 켜져 있을 때; launch.sh 기본)
  NNNN.png           right 토픽이 없으면 기존처럼 모노(left)만 저장
  NNNN.pcd           원본 포인트클라우드 (x y z intensity [+ring +timestamp])
  NNNN_roi.npy       캡처 직후 그린 라이다 보드 ROI 마스크 (calibrate 자동 사용)
  NNNN_view.png      화면과 동일한 [카메라|BEV] 합성 뷰 (빠른 확인용)
  index.csv          idx, lidar_stamp, cam_stamp, dt_ms, npoints, png, png_R, pcd
자가검증:  python3 sync_capture.py --selftest

전제: rslidar `use_lidar_clock:false`, ZED `enable_ipc:=false` (둘 다 호스트 클럭).
"""
import os, sys, csv, time
from collections import deque
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
import message_filters
from sensor_msgs.msg import PointCloud2, Image
from sensor_msgs_py import point_cloud2 as pc2
from cv_bridge import CvBridge
import cv2

# 토픽/저장 경로는 환경변수로 덮어쓸 수 있다 — calib.sh 가 워크스페이스에
# 맞춰 넘긴다. 기본값은 hyu_sensor_bringup 이 발행하는 /sensors 계약.
LIDAR_TOPIC = os.environ.get('CAPTURE_LIDAR_TOPIC', '/sensors/lidar/points')
CAM_TOPIC   = os.environ.get('CAPTURE_CAM_TOPIC',
                             '/sensors/zed/left/color/rect/image')   # 좌측 rectified
RIGHT_TOPIC = os.environ.get('CAPTURE_RIGHT_TOPIC',
                             '/sensors/zed/right/color/rect/image')  # 있으면 스테레오 저장
LR_MATCH_MS = 25.0                 # 좌/우 같은 프레임 판정 허용 오차
SLOP_SEC    = 0.05
BEV_RANGE_M = 10.0                 # BEV 표시 반경(m)
OUT_ROOT    = os.environ.get(
    'CAPTURE_OUT_ROOT',
    os.path.join(os.environ.get('EUFS_MASTER', os.path.expanduser('~/fsk')),
                 'captures'))
# ZED·rslidar 모두 RELIABLE 퍼블리셔 → 반드시 RELIABLE로 구독해야 프레임 드롭 없음
# (best_effort로 받으면 부하 시 프레임이 버려져 "불규칙"해 보임)
QOS = QoSProfile(reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST, depth=10)


def stamp_sec(h):
    return h.stamp.sec + h.stamp.nanosec * 1e-9


def cloud_xyzi(cloud):
    names = [f.name for f in cloud.fields]
    fn = ('x', 'y', 'z', 'intensity') if 'intensity' in names else ('x', 'y', 'z')
    return np.array([tuple(float(v) for v in p)
                     for p in pc2.read_points(cloud, field_names=fn, skip_nans=True)],
                    dtype=np.float32), fn


def cloud_all_fields(cloud):
    """저장용: x y z intensity에 더해 ring/timestamp가 있으면 함께 추출.
    (timestamp는 epoch 초 — float32로는 정밀도가 깨져 float64 유지 필수)"""
    names = [f.name for f in cloud.fields]
    fn = tuple(n for n in ('x', 'y', 'z', 'intensity', 'ring', 'timestamp')
               if n in names)
    return np.array([tuple(float(v) for v in p)
                     for p in pc2.read_points(cloud, field_names=fn, skip_nans=True)],
                    dtype=np.float64), fn


def write_pcd(path, cloud):
    """pcd 저장 후 (점수, 포인트배열)을 반환 — 배열은 파일과 같은 행 순서라
    캡처 직후 ROI 마스크(NNNN_roi.npy)의 인덱스 기준으로 그대로 쓸 수 있다."""
    arr, fn = cloud_all_fields(cloud)
    n = arr.shape[0]
    sizes = ['8' if f == 'timestamp' else '4' for f in fn]
    with open(path, 'w') as f:
        f.write("# .PCD v0.7\nVERSION 0.7\n")
        f.write(f"FIELDS {' '.join(fn)}\nSIZE {' '.join(sizes)}\n")
        f.write(f"TYPE {' '.join(['F']*len(fn))}\nCOUNT {' '.join(['1']*len(fn))}\n")
        f.write(f"WIDTH {n}\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\nPOINTS {n}\nDATA ascii\n")
        for r in arr:
            f.write(' '.join(f"{v:.6f}" if fl == 'timestamp' else f"{v:.4f}"
                             for v, fl in zip(r, fn)) + '\n')
    return n, arr


def render_bev(cloud, size, range_m=BEV_RANGE_M):
    """LiDAR 포인트를 탑다운(BEV) 이미지로. 전방(x)=위, 좌(y)=왼쪽, 높이(z)로 색."""
    img = np.full((size, size, 3), 18, np.uint8)
    c = size // 2
    scale = size / (2.0 * range_m)
    for r in range(2, int(range_m) + 1, 2):                     # 거리 링(2m 간격)
        cv2.circle(img, (c, c), int(r * scale), (45, 45, 45), 1)
    cv2.line(img, (c, 0), (c, size), (45, 45, 45), 1)
    cv2.line(img, (0, c), (size, c), (45, 45, 45), 1)
    arr, _ = cloud_xyzi(cloud)
    if arr.shape[0]:
        x, y, z = arr[:, 0], arr[:, 1], arr[:, 2]
        px = (c - y * scale).astype(np.int32)
        py = (c - x * scale).astype(np.int32)
        m = (px >= 0) & (px < size) & (py >= 0) & (py < size)
        zc = np.clip((z + 2.0) / 5.0, 0, 1)                     # -2..3m → 0..1
        col = cv2.applyColorMap((zc * 255).astype(np.uint8).reshape(-1, 1),
                                cv2.COLORMAP_JET).reshape(-1, 3)
        img[py[m], px[m]] = col[m]
        img = cv2.dilate(img, np.ones((2, 2), np.uint8))        # 포인트 살짝 굵게
    cv2.circle(img, (c, c), 4, (255, 255, 255), -1)             # ego
    cv2.putText(img, f"LiDAR BEV +-{range_m:.0f}m  (forward=up)", (8, size - 10),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (170, 170, 170), 1, cv2.LINE_AA)
    return img


def put(img, text, org, color=(0, 255, 0)):
    cv2.putText(img, text, org, cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 0), 4, cv2.LINE_AA)
    cv2.putText(img, text, org, cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 1, cv2.LINE_AA)


class SyncCapture(Node):
    def __init__(self, session_dir):
        super().__init__('sync_capture')
        self.dir = session_dir
        self.bridge = CvBridge()
        self.latest = None                 # (cloud, image, dt_ms)
        self.count = 0
        self.dts = deque(maxlen=200)       # 최근 Δt 통계용
        self._bev = (None, None)           # (stamp, bev img) 캐시
        self.right_buf = deque(maxlen=60)  # (stamp, Image) 우측 최근 프레임 버퍼
                                           # (sync 쌍은 최신보다 1~3프레임 과거일 수
                                           #  있어 최신 1장만으론 매칭이 어긋남)
        self.csv = open(os.path.join(self.dir, 'index.csv'), 'w', newline='')
        self.w = csv.writer(self.csv)
        self.w.writerow(['idx', 'lidar_stamp', 'cam_stamp', 'dt_ms', 'npoints', 'png', 'png_R', 'pcd'])

        lidar = message_filters.Subscriber(self, PointCloud2, LIDAR_TOPIC, qos_profile=QOS)
        cam   = message_filters.Subscriber(self, Image, CAM_TOPIC, qos_profile=QOS)
        self.sync = message_filters.ApproximateTimeSynchronizer([lidar, cam], queue_size=10, slop=SLOP_SEC)
        self.sync.registerCallback(self.on_pair)
        # 우측은 별도 구독(토픽이 없으면 그냥 안 들어옴 → 모노 저장으로 동작)
        self.create_subscription(Image, RIGHT_TOPIC, self.on_right, QOS)
        self.get_logger().info(f'sync 시작: {LIDAR_TOPIC} + {CAM_TOPIC} (slop={SLOP_SEC*1000:.0f}ms)'
                               f' / 우측: {RIGHT_TOPIC} (있으면 스테레오 저장)')

    def on_pair(self, cloud, image):
        dt_ms = (stamp_sec(cloud.header) - stamp_sec(image.header)) * 1000.0
        self.dts.append(dt_ms)
        self.latest = (cloud, image, dt_ms)

    def on_right(self, image):
        self.right_buf.append((stamp_sec(image.header), image))

    def match_right(self, cam_stamp):
        """cam_stamp와 같은 그랩의 우측 프레임(허용오차 내 최근접)을 찾는다."""
        best, best_dt = None, LR_MATCH_MS
        for st, msg in self.right_buf:
            dt = abs(st - cam_stamp) * 1000.0
            if dt <= best_dt:
                best, best_dt = msg, dt
        return best

    def stats(self):
        if not self.dts:
            return None
        a = np.array(self.dts)
        return a.mean(), a.std(), a.min(), a.max()

    def bev_of(self, cloud, size):
        st = stamp_sec(cloud.header)
        if self._bev[0] == st and self._bev[1] is not None and self._bev[1].shape[0] == size:
            return self._bev[1]
        bev = render_bev(cloud, size)
        self._bev = (st, bev)
        return bev

    def compose(self):
        """현재 최신 쌍 → [카메라 | BEV] 합성 + 오버레이. (없으면 None)
        같은 쌍이면 캐시 재사용 — GUI 루프가 매 프레임 3.7MB 변환을 반복하며
        수신 스레드를 굶기지 않게 한다."""
        if self.latest is None:
            return None
        cloud, image, dt_ms = self.latest
        key = (stamp_sec(cloud.header), stamp_sec(image.header), self.count)
        cached = getattr(self, '_view_cache', None)
        if cached is not None and cached[0] == key:
            return cached[1]
        cam = self.bridge.imgmsg_to_cv2(image, desired_encoding='bgr8')
        bev = self.bev_of(cloud, cam.shape[0])
        view = cv2.hconcat([cam, bev])
        npts = cloud.width * cloud.height
        put(view, f"LiDAR pts: {npts}", (12, 30))
        put(view, f"sync dt: {dt_ms:+.1f} ms", (12, 60))
        st = self.stats()
        if st:
            put(view, f"dt stats: {st[0]:+.1f}+-{st[1]:.1f} ms  ({st[2]:+.0f}..{st[3]:+.0f})", (12, 90), (0, 220, 255))
        stereo_on = self.match_right(stamp_sec(image.header)) is not None
        put(view, f"stereo(R): {'ON' if stereo_on else 'OFF (mono save)'}", (12, 120),
            (0, 255, 0) if stereo_on else (0, 140, 255))
        put(view, f"captured: {self.count}", (12, 150))
        self._view_cache = (key, view)
        return view

    def snapshot(self, do_roi=True):
        if self.latest is None:
            return None
        cloud, image, dt_ms = self.latest
        idx = self.count
        base = os.path.join(self.dir, f'{idx:04d}')

        # 우측 프레임이 좌측과 같은 그랩이면 스테레오(_L/_R)로 저장
        cam_stamp = stamp_sec(image.header)
        right = self.match_right(cam_stamp)

        if right is not None:
            png_l = base + '_L.png'
            png_r = base + '_R.png'
            cv2.imwrite(png_l, self.bridge.imgmsg_to_cv2(image, desired_encoding='bgr8'))
            cv2.imwrite(png_r, self.bridge.imgmsg_to_cv2(right, desired_encoding='bgr8'))
        else:
            png_l = base + '.png'
            png_r = ''
            cv2.imwrite(png_l, self.bridge.imgmsg_to_cv2(image, desired_encoding='bgr8'))

        npts, arr = write_pcd(base + '.pcd', cloud)

        # ── 캡처 직후 라이다 ROI: 정면 뷰에 보드 둘레 다각형을 바로 그림 ──
        #    (ENTER 확정 → NNNN_roi.npy 저장, calibrate가 자동 사용.
        #     ESC 스킵 → 저장 안 함, calibrate에서 기존 방식으로 ROI 지정)
        roi_pts = 0
        if do_roi:
            try:
                from charuco_lidar_calib import lidar_board as LB
                mask = LB.select_polygon(arr, {})
                np.save(base + '_roi.npy', mask)
                roi_pts = int(mask.sum())
            except RuntimeError:
                self.get_logger().info(f'#{idx}: ROI 스킵됨 (calibrate에서 지정)')
            except Exception as e:
                self.get_logger().warn(f'#{idx}: ROI 실패 — {e}')

        view = self.compose()
        if view is not None:
            cv2.imwrite(base + '_view.png', view)
        self.w.writerow([idx, f'{stamp_sec(cloud.header):.6f}', f'{cam_stamp:.6f}',
                         f'{dt_ms:.1f}', npts, png_l, png_r, base + '.pcd']); self.csv.flush()
        self.count += 1
        mode = 'L+R' if right is not None else 'mono'
        roi_note = f", roi={roi_pts}pts" if roi_pts else ""
        self.get_logger().info(f"저장 #{idx} ({mode}): img+{npts}pts{roi_note}, "
                               f"dt={dt_ms:+.1f}ms → {png_l}")
        return dict(idx=idx, dt_ms=dt_ms, npts=npts, stereo=(right is not None),
                    roi_pts=roi_pts)


def gui_loop(node):
    """ROS 수신은 전용 스레드(executor)로 돌리고, 이 루프는 렌더링만 한다.
    (기존처럼 루프당 spin_once 1회면 이미지 2스트림+라이다 ≈ 70msg/s를 못
    따라가 수신이 굶고, right 버퍼에 구멍이 나 stereo ON/OFF가 깜빡인다.)"""
    import threading
    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(node)
    spin_t = threading.Thread(target=executor.spin, daemon=True)
    spin_t.start()

    win = 'LiDAR+ZED sync capture   [SPACE/C]=capture   [Q/ESC]=quit'
    cv2.namedWindow(win, cv2.WINDOW_NORMAL)
    try:
        while rclpy.ok():
            view = node.compose()
            if view is None:
                view = np.zeros((240, 640, 3), np.uint8)
                put(view, 'waiting for synced LiDAR+camera...', (20, 120), (0, 255, 255))
            cv2.imshow(win, view)
            k = cv2.waitKey(30) & 0xFF          # ~33fps 렌더링이면 충분
            if k in (ord(' '), ord('c'), ord('C')):
                r = node.snapshot()
                print('  ✓ 저장:', r if r else '(아직 동기 쌍 없음)')
            elif k in (ord('q'), ord('Q'), 27):
                break
    finally:
        executor.shutdown()
        cv2.destroyAllWindows()


def selftest(node):
    print('[selftest] 동기 쌍 대기...')
    for _ in range(200):
        rclpy.spin_once(node, timeout_sec=0.05)
        if node.latest is not None:
            break
    saved = 0
    for _ in range(2):
        for _ in range(10):
            rclpy.spin_once(node, timeout_sec=0.05)
        r = node.snapshot(do_roi=False)
        if r:
            saved += 1
            print(f'[selftest] 저장됨: idx={r["idx"]} pts={r["npts"]} dt={r["dt_ms"]:+.1f}ms')
    s = node.stats()
    if s:
        print(f'[selftest] Δt 통계: mean={s[0]:+.1f} std={s[1]:.1f} min={s[2]:+.1f} max={s[3]:+.1f} ms')
    print(f'[selftest] 총 {saved}장 저장, 폴더: {node.dir}')
    return saved > 0


def main():
    rclpy.init()
    session_dir = os.path.join(OUT_ROOT, time.strftime('%Y%m%d_%H%M%S'))
    os.makedirs(session_dir, exist_ok=True)
    node = SyncCapture(session_dir)
    ok = True
    try:
        if '--selftest' in sys.argv:
            ok = selftest(node)
        else:
            print(f'\n▶ 캡처 폴더: {session_dir}\n▶ [SPACE]=저장, [Q]=종료\n')
            gui_loop(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.csv.close()
        node.destroy_node()
        rclpy.try_shutdown()
    return ok


if __name__ == '__main__':
    sys.exit(0 if main() else 1)
