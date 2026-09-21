#!/usr/bin/env python3
# 2.5D 高程图: PCD -> 确定性地面基准 -> 离地高度栅格 -> 可通行性 costmap
# 输出 /elevation_costmap (OccupancyGrid): 0=平地 80=小台阶/缓坡(需切腿) 100=障碍(绕行)
# 供 Nav2 costmap 层 / 模态切换判断消费
#
# ★ 2026-09-11 参数化订正（2.5D 阶段① 前置，否则 step_max 无法标定）:
#   原版把判级阈值硬编码在 _build() 内 —— occ_lo/occ_hi=0.15/1.8、step_max=0.50；
#   而 declare 出来的 ground_thresh/h_flat/h_step/cost_step 虽然传进了 _build，
#   却一个都没被使用（h = pts.z - GZ 算完就丢，h_flat/h_step 从未参与判级）。
#   后果：这些参数在 launch/yaml 里怎么改都不生效，标定无从下手。
#   本次改为全部从参数读取，并在启动日志打印【实际生效值 + 分级统计】。
#
# 两套坐标语义（不要混用）:
#   occ_lo / occ_hi   绝对 z。与 pcd_to_map_node.py 的 z_min/z_max 对齐是硬约束，
#                     保证高程图与 /map 的"占用"判据一致（否则两图互相打架）。
#   h_flat / step_max 离地高度（pts.z - 地面基准 GZ）。物理量，可标定，换场地不失效。
import numpy as np
import open3d as o3d

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Header

# 与 pcd_to_map_node.py 对齐的默认占用带（绝对 z）
DEFAULT_OCC_LO = 0.15
DEFAULT_OCC_HI = 1.8


class ElevationMapNode(Node):
    def __init__(self):
        super().__init__('elevation_map')
        self.declare_parameter('pcd_path', '/home/nvidia/dog_ws/src/dog_brain/maps/lab_3d_map.pcd')
        self.declare_parameter('resolution', 0.05)
        # --- 占用判据: 绝对 z, 必须与 pcd_to_map 的 z_min/z_max 一致 ---
        self.declare_parameter('occ_lo', DEFAULT_OCC_LO)
        self.declare_parameter('occ_hi', DEFAULT_OCC_HI)
        # --- 地面基准: z 的确定性分位(%), 原硬编码 3 ---
        self.declare_parameter('ground_percentile', 3.0)
        # --- 分级阈值: 离地高度(m) ---
        self.declare_parameter('h_flat', 0.06)     # 最高点离地 < 此值 => 平地(0), 滤掉贴地噪点/薄地毯
        self.declare_parameter('step_max', 0.50)   # 离地 < 此值 => 可越台阶(cost_step); 否则障碍(100)
        self.declare_parameter('cost_step', 80)
        self.declare_parameter('publish_interval', 3.0)

        path = self.get_parameter('pcd_path').value
        res = self.get_parameter('resolution').value
        self.occ_lo = float(self.get_parameter('occ_lo').value)
        self.occ_hi = float(self.get_parameter('occ_hi').value)
        self.ground_pct = float(self.get_parameter('ground_percentile').value)
        self.h_flat = float(self.get_parameter('h_flat').value)
        self.step_max = float(self.get_parameter('step_max').value)
        self.cost_step = int(self.get_parameter('cost_step').value)

        self.get_logger().info(
            f'[params] res={res} occ_z=[{self.occ_lo},{self.occ_hi}](绝对) '
            f'ground_pct={self.ground_pct} h_flat={self.h_flat} '
            f'step_max={self.step_max}(离地) cost_step={self.cost_step}')

        msg = self._build(path, res)
        if msg is None:
            self.get_logger().error('Elevation map build failed')
            return
        self.msg = msg

        qos = QoSProfile(depth=1)
        qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        qos.reliability = ReliabilityPolicy.RELIABLE
        self.pub = self.create_publisher(OccupancyGrid, '/elevation_costmap', qos)
        self.pub.publish(self.msg)
        self.create_timer(self.get_parameter('publish_interval').value, self._repub)

    def _build(self, path, res):
        pcd = o3d.io.read_point_cloud(path)
        pts = np.asarray(pcd.points)
        if len(pts) == 0:
            self.get_logger().error(f'No points in {path}')
            return None

        # 确定性地面基准: 水平地面假设, 取 z 的 ground_percentile 分位
        # (RANSAC segment_plane 随机采样导致每次拟合平面不稳, 故改用确定性基准)
        GZ = float(np.percentile(pts[:, 2], self.ground_pct))
        if abs(GZ) > 0.5:
            self.get_logger().warn(
                f'Ground base z={GZ:.3f} 偏离 0 较大, 请确认 PCD 的 odom 原点与地面关系')
        self.get_logger().info(
            f'Ground base z={GZ:.3f} (p{self.ground_pct}), '
            f'z range [{pts[:, 2].min():.2f},{pts[:, 2].max():.2f}], points={len(pts)}')

        # 占用判据过滤: 与 pcd_to_map_node.py L29-30 完全对齐 (硬约束, 见文件头注释)
        #   ★ FixB 2026-09-20: 原实现先用【全体点】算包围盒、之后才过滤,
        #     导致 925 个离群鬼影点(占万分之六)把 x 原点从 -12.81 拉到 -46.62,
        #     /elevation_costmap 变成 2060x975, 与 /map 的 1380x910 冲突,
        #     global_costmap 每 30s 在两尺寸间来回 Resize => NavFn 规划失败。
        #     修法: 先过滤再算包围盒, 与 pcd_to_map 顺序一致。
        obs_all = pts[(pts[:, 2] >= self.occ_lo) & (pts[:, 2] <= self.occ_hi)]
        # 栅格化
        min_x, min_y = float(obs_all[:, 0].min()), float(obs_all[:, 1].min())
        # 取整方式也必须与 pcd_to_map_node.py L34-35 一致(int 截断会少 1 格,
        # 1379x909 vs 1380x910 仍会触发 StaticLayer resize)
        W = int(np.ceil((obs_all[:, 0].max() - min_x) / res)) + 1
        H = int(np.ceil((obs_all[:, 1].max() - min_y) / res)) + 1

        occ = np.zeros((H, W), dtype=np.int8)   # 默认平地(可走)
        obs = obs_all
        n_cand = int(len(obs))
        if n_cand:
            oix = ((obs[:, 0] - min_x) / res).astype(np.int64)
            oiy = ((obs[:, 1] - min_y) / res).astype(np.int64)
            np.clip(oix, 0, W - 1, out=oix)
            np.clip(oiy, 0, H - 1, out=oiy)
            # 每格取最高点（向量化，替代原先逐点 Python dict 循环）
            cellmax = np.full(H * W, -1e9, dtype=np.float64)
            np.maximum.at(cellmax, oiy * W + oix, obs[:, 2])
            cellmax = cellmax.reshape(H, W)
            has = cellmax > -1e8
            dz = np.where(has, cellmax - GZ, 0.0)    # 离地高度
            with np.errstate(invalid='ignore'):
                occ[has & (dz >= self.h_flat) & (dz < self.step_max)] = self.cost_step
                occ[has & (dz >= self.step_max)] = 100
            # dz < h_flat 保持 0: 贴地噪点/薄地毯不判为台阶

        n_flat = int((occ == 0).sum())
        n_step = int((occ == self.cost_step).sum())
        n_wall = int((occ == 100).sum())
        if n_cand == 0:
            self.get_logger().warn(
                f'占用带 [{self.occ_lo},{self.occ_hi}] 内无点（检查 PCD 的 z 原点）')
        self.get_logger().info(
            f'Elevation map {W}x{H} res={res}: 占用候选点={n_cand} '
            f'flat={n_flat} step(可越,{self.cost_step})={n_step} obstacle(绕行,100)={n_wall} '
            f'({100.0 * n_flat / max(W * H, 1):.1f}% flat)')

        msg = OccupancyGrid()
        msg.header = Header(); msg.header.frame_id = 'map'
        msg.info.resolution = float(res); msg.info.width = int(W); msg.info.height = int(H)
        msg.info.origin.position.x = min_x; msg.info.origin.position.y = min_y
        msg.info.origin.orientation.w = 1.0
        msg.data = occ.flatten().astype(int).tolist()
        return msg

    def _repub(self):
        self.msg.header.stamp = self.get_clock().now().to_msg()
        self.pub.publish(self.msg)


def main():
    rclpy.init()
    node = ElevationMapNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
