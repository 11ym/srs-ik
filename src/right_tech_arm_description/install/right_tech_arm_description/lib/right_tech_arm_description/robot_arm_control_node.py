#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import numpy as np
import math
import pandas as pd
from scipy.spatial.transform import Rotation as R

# 机械臂参数
dBS = 0.0
dSE = 0.3
dEW = 0.246
dWT = 0.024

def secJudge(Zmin, Zmax, fai):
    # 处理边界情况
    Zmin = max(Zmin, -1)
    Zmax = min(Zmax, 1)

    if Zmin > Zmax:
        return np.array([]).reshape(2, 0)  # 无效区间

    # 单个区间情况（前三个条件）：转换为二维数组（2,1）
    if Zmin < -1 and Zmax <= 1 and Zmax >= -1:
        return np.array([[-np.pi - np.arcsin(Zmax) - fai], [np.arcsin(Zmax) - fai]])  # 添加维度

    elif Zmin < -1 and Zmax > 1:
        return np.array([[-np.pi], [np.pi]])  # 二维数组

    # 两个区间情况（第四个条件）：保持二维数组（2,2）
    elif Zmin < 1 and Zmin >= -1 and Zmax <= 1 and Zmax >= -1:
        return np.array([[np.pi - np.arcsin(Zmax) - fai, np.arcsin(Zmin) - fai],
                         [np.pi - np.arcsin(Zmin) - fai, np.arcsin(Zmax) - fai]])

    # 单个区间情况（最后一个条件）：转换为二维数组（2,1）
    else:
        return np.array([[np.arcsin(Zmin) - fai], [np.pi - np.arcsin(Zmin) - fai]])


def comTwoInterval(interval1, interval2):
    if interval1.size == 0 or interval2.size == 0:
        return np.array([]).reshape(2, 0)  # 空区间返回空数组

    # 确保都是二维列向量
    interval1 = interval1.reshape(2, -1)
    interval2 = interval2.reshape(2, -1)

    res = []
    for i in range(interval1.shape[1]):
        for j in range(interval2.shape[1]):
            a1, a2 = interval1[0, i], interval1[1, i]
            b1, b2 = interval2[0, j], interval2[1, j]
            if a2 < b1 or b2 < a1:
                continue  # 无交集
            else:
                res.append(np.array([[max(a1, b1)], [min(a2, b2)]]))

    if not res:
        return np.array([]).reshape(2, 0)
    return np.hstack(res)


def comMultiInterval(A, B):
    if A.size == 0:
        return B
    if B.size == 0:
        return A

    A = A.reshape(2, -1)
    B = B.reshape(2, -1)

    res = []
    for i in range(A.shape[1]):
        for j in range(B.shape[1]):
            com = comTwoInterval(A[:, i:i + 1], B[:, j:j + 1])
            if com.size > 0:
                res.append(com)

    if not res:
        return np.array([]).reshape(2, 0)
    return np.hstack(res)


def compressIntervals(intervals, percent=0.8):
    if intervals.size == 0:
        return intervals

    intervals = intervals.reshape(2, -1)
    res = np.zeros_like(intervals)

    for i in range(intervals.shape[1]):
        length = intervals[1, i] - intervals[0, i]
        if length <= 0:
            continue
        delta = (1 - percent) * length / 2
        res[0, i] = intervals[0, i] + delta
        res[1, i] = intervals[1, i] - delta

    return res


def mergeIntervals(intervals):
    if intervals.size == 0:
        return np.array([]).reshape(2, 0)

    intervals = intervals.reshape(2, -1).T
    intervals = intervals[intervals[:, 0].argsort()]  # 按左端点排序

    merged = []
    for interval in intervals:
        if not merged:
            merged.append(interval)
        else:
            last = merged[-1]
            if interval[0] <= last[1]:
                # 合并区间
                new_interval = [last[0], max(last[1], interval[1])]
                merged[-1] = new_interval
            else:
                merged.append(interval)

    if not merged:
        return np.array([]).reshape(2, 0)
    return np.array(merged).T


def getFeasibleArmAngle_Case1(T, node):
    nx, ox, ax, px = T[0, :]
    ny, oy, ay, py = T[1, :]
    nz, oz, az, pz = T[2, :]

    R07 = np.array([[nx, ox, ax], [ny, oy, ay], [nz, oz, az]])
    pSW = np.array([-dWT * ax + px, -dWT * ay + py, -dWT * az + pz - dBS])
    dSW = np.linalg.norm(pSW)

    if dSW >= (dSE + dEW):
        return np.array([]).reshape(2, 0)

    theta_SEW = np.arccos((dSE ** 2 + dEW ** 2 - dSW ** 2) / (2 * dSE * dEW))
    q4 = np.pi - theta_SEW

    if not (-np.pi <= q4 <= np.pi) or abs(np.degrees(q4)) > 120:
        return np.array([]).reshape(2, 0)

    uSW = pSW / dSW if dSW != 0 else np.array([1, 0, 0])
    k = np.cross(uSW, [0, 0, 1])
    k = k / np.linalg.norm(k) if np.linalg.norm(k) != 0 else np.array([1, 0, 0])
    K = np.array([[0, -k[2], k[1]], [k[2], 0, -k[0]], [-k[1], k[0], 0]])

    As = np.cross(uSW, np.column_stack([
        (pSW + np.cos(q4) * uSW * dEW + uSW * dSE) / (np.sin(q4) * dEW),
        -uSW,
        np.cross((pSW + np.cos(q4) * uSW * dEW + uSW * dSE) / (np.sin(q4) * dEW), -uSW)
    ]))

    a2 = -As[2, 1]
    b2 = -As[1, 1]
    c2 = -As[0, 1]
    fai_2 = np.arctan2(b2, a2) if (a2 != 0 or b2 != 0) else 0

    Zmin_2 = (np.cos(np.deg2rad(120)) - c2) / np.hypot(a2, b2) if np.hypot(a2, b2) != 0 else -2
    Zmax_2 = (1 - c2) / np.hypot(a2, b2) if np.hypot(a2, b2) != 0 else 2
    feapsi_2 = secJudge(Zmin_2, Zmax_2, fai_2)

    Aw = np.dot(np.array([[np.cos(q4), 0, np.sin(q4)], [np.sin(q4), 0, -np.cos(q4)], [0, 1, 0]]).T, As.T @ R07)
    a6 = Aw[2, 2]
    b6 = Aw[1, 2]
    c6 = Aw[0, 2]
    fai_6 = np.arctan2(b6, a6) if (a6 != 0 or b6 != 0) else 0

    Zmin_6 = (np.cos(np.deg2rad(120)) - c6) / np.hypot(a6, b6) if np.hypot(a6, b6) != 0 else -2
    Zmax_6 = (1 - c6) / np.hypot(a6, b6) if np.hypot(a6, b6) != 0 else 2
    feapsi_6 = secJudge(Zmin_6, Zmax_6, fai_6)

    return comMultiInterval(feapsi_2, feapsi_6)


def getFeasibleArmAngle_Case2(T, node):
    nx, ox, ax, px = T[0, :]
    ny, oy, ay, py = T[1, :]
    nz, oz, az, pz = T[2, :]

    R07 = np.array([[nx, ox, ax], [ny, oy, ay], [nz, oz, az]])
    pSW = np.array([-dWT * ax + px, -dWT * ay + py, -dWT * az + pz - dBS])
    dSW = np.linalg.norm(pSW)

    if dSW >= (dSE + dEW):
        return np.array([]).reshape(2, 0)

    theta_SEW = -np.arccos((dSE ** 2 + dEW ** 2 - dSW ** 2) / (2 * dSE * dEW))
    q4 = np.pi - theta_SEW

    if not (-np.pi <= q4 <= np.pi) or abs(np.degrees(q4)) > 120:
        return np.array([]).reshape(2, 0)

    uSW = pSW / dSW if dSW != 0 else np.array([1, 0, 0])
    k = np.cross(uSW, [0, 0, 1])
    k = k / np.linalg.norm(k) if np.linalg.norm(k) != 0 else np.array([1, 0, 0])
    K = np.array([[0, -k[2], k[1]], [k[2], 0, -k[0]], [-k[1], k[0], 0]])

    As = np.cross(uSW, np.column_stack([
        (pSW + np.cos(q4) * uSW * dEW + uSW * dSE) / (np.sin(q4) * dEW),
        -uSW,
        np.cross((pSW + np.cos(q4) * uSW * dEW + uSW * dSE) / (np.sin(q4) * dEW), -uSW)
    ]))

    a2 = -As[2, 1]
    b2 = -As[1, 1]
    c2 = -As[0, 1]
    fai_2 = np.arctan2(b2, a2) if (a2 != 0 or b2 != 0) else 0

    Zmin_2 = (np.cos(np.deg2rad(120)) - c2) / np.hypot(a2, b2) if np.hypot(a2, b2) != 0 else -2
    Zmax_2 = (1 - c2) / np.hypot(a2, b2) if np.hypot(a2, b2) != 0 else 2
    feapsi_2 = secJudge(Zmin_2, Zmax_2, fai_2)

    Aw = np.dot(np.array([[np.cos(q4), 0, np.sin(q4)], [np.sin(q4), 0, -np.cos(q4)], [0, 1, 0]]).T, As.T @ R07)
    a6 = Aw[2, 2]
    b6 = Aw[1, 2]
    c6 = Aw[0, 2]
    fai_6 = np.arctan2(b6, a6) if (a6 != 0 or b6 != 0) else 0

    Zmin_6 = (np.cos(np.deg2rad(120)) - c6) / np.hypot(a6, b6) if np.hypot(a6, b6) != 0 else -2
    Zmax_6 = (1 - c6) / np.hypot(a6, b6) if np.hypot(a6, b6) != 0 else 2
    feapsi_6 = secJudge(Zmin_6, Zmax_6, fai_6)

    return comMultiInterval(feapsi_2, feapsi_6)


def getFeasibleArmAngle(T, node):
    case1 = getFeasibleArmAngle_Case1(T, node)
    case2 = getFeasibleArmAngle_Case2(T, node)

    # 合并区间并压缩
    merged = mergeIntervals(np.hstack([case1, case2]))
    compressed = compressIntervals(merged)

    return merged if compressed.size == 0 else compressed


class RobotArmController(Node):
    def __init__(self):
        super().__init__('robot_arm_controller')
        self.declare_parameters(
            namespace='',
            parameters=[
                ('csv_path', '/home/mym/ros2_ws/src/right_tech_arm_description/data_csv/both_arm_endpose.csv'),
                ('publish_freq', 10.0),
                ('initial_psi', np.pi/3)
            ]
        )
        
        # 获取参数
        self.csv_path = self.get_parameter('csv_path').value
        self.publish_freq = self.get_parameter('publish_freq').value
        self.initial_psi = self.get_parameter('initial_psi').value
        
        # 初始化变量
        self.current_index = 0
        self.current_psi = self.initial_psi
        self.current_q = np.zeros(7)
        self.joint_names = ['r_joint1', 'r_joint2', 'r_joint3',
                            'r_joint4', 'r_joint5', 'r_joint6', 'r_joint7']
        
        # 读取CSV数据
        self.df, self.timestamps = self.csv_to_pose_data()
        
        # 创建发布器和定时器
        self.joint_pub = self.create_publisher(JointState, '/joint_states', 10)
        self.timer = self.create_timer(1.0/self.publish_freq, self.timer_callback)
        
        self.get_logger().info(f"Loaded {len(self.df)} poses from CSV")

    def csv_to_pose_data(self):
        df = pd.read_csv(self.csv_path)
        poses = []
        timestamps = []
        
        for idx, row in df.iterrows():
            xyz = np.array([float(x) for x in row['right_ee_xyz'].split(',')])
            quat = np.array([float(x) for x in row['right_ee_quat(xyzw)'].split(',')])
            rot_mat = R.from_quat(quat).as_matrix()
            
            T = np.eye(4)
            T[:3, :3] = rot_mat
            T[:3, 3] = xyz
            poses.append(T)
            timestamps.append(row['timestamp'])
        
        return np.array(poses), np.array(timestamps)

    def is_joint_valid(self, q):
        joint_limits = [
            (-np.radians(170), np.radians(170)),  # r_joint1
            (-np.radians(120), np.radians(120)),  # r_joint2
            (-np.radians(170), np.radians(170)),  # r_joint3
            (-np.pi, np.pi),                    # r_joint4
            (-np.radians(170), np.radians(170)),  # r_joint5
            (-np.radians(120), np.radians(120)),  # r_joint6
            (-np.radians(170), np.radians(170))   # r_joint7
        ]
        return all(low <= qi <= high for qi, (low, high) in zip(q, joint_limits))

    def compute_inverse_kinematics(self, T, psi):
        nx, ox, ax, px = T[0, 0], T[0, 1], T[0, 2], T[0, 3]
        ny, oy, ay, py = T[1, 0], T[1, 1], T[1, 2], T[1, 3]
        nz, oz, az, pz = T[2, 0], T[2, 1], T[2, 2], T[2, 3]

        R07 = np.array([[nx, ox, ax], [ny, oy, ay], [nz, oz, az]])
        pSW = np.array([px - dWT * ax, py - dWT * ay, pz - dWT * az - dBS])
        dSW = np.linalg.norm(pSW)

        if dSW >= (dSE + dEW):
            self.get_logger().warn("位姿超出可达范围")
            return None

        theta_SEW_1 = np.arccos((dSE**2 + dEW**2 - dSW**2) / (2*dSE*dEW))
        theta_SEW_2 = -theta_SEW_1
        q4_list = [np.pi - theta_SEW_1, np.pi - theta_SEW_2]

        valid_solutions = []
        for q4 in q4_list:
            if not (-np.pi <= q4 <= np.pi) or abs(np.degrees(q4)) > 120:
                continue

            uSW = pSW / dSW if dSW != 0 else np.array([1, 0, 0])
            z0 = np.array([0, 0, 1])
            k = np.cross(uSW, z0)
            k = k / np.linalg.norm(k) if np.linalg.norm(k) != 0 else np.array([1, 0, 0])
            K = np.array([[0, -k[2], k[1]], [k[2], 0, -k[0]], [-k[1], k[0], 0]])

            R_k_theta = np.eye(3) + np.sin(q4 - np.pi) * K + (1 - np.cos(q4 - np.pi)) * K @ K
            y3 = -R_k_theta @ uSW
            x3 = (pSW + y3*dSE + np.cos(q4)*y3*dEW) / (np.sin(q4)*dEW) if np.sin(q4) != 0 else np.array([1, 0, 0])
            z3 = np.cross(x3, y3)
            R03 = np.column_stack((x3, y3, z3))

            R34 = np.array([[np.cos(q4), 0, np.sin(q4)],
                           [np.sin(q4), 0, -np.cos(q4)],
                           [0, 1, 0]])
            Aw = R34.T @ R03.T @ R07

            try:
                q2 = np.arctan2(-Aw[1, 1], Aw[0, 1])
                q1 = np.arctan2(Aw[2, 1], Aw[2, 0])
                q3 = np.arctan2(-Aw[2, 2], Aw[2, 0])
                q5 = np.arctan2(-Aw[1, 2], -Aw[0, 2])
                q6 = np.arctan2(np.sqrt(Aw[0, 2]**2 + Aw[1, 2]**2), Aw[2, 2])
                q7 = np.arctan2(Aw[2, 0], -Aw[2, 1])
            except:
                continue

            q = np.array([q1, q2, q3, q4, q5, q6, q7])
            if self.is_joint_valid(q):
                valid_solutions.append(q)

        if not valid_solutions:
            return None

        best_q = min(valid_solutions, key=lambda x: np.linalg.norm(x - self.current_q))
        return best_q

    def opt_psi_selector(self, current_psi, current_q, T):
        feasible_psi = getFeasibleArmAngle(T, self)
        if feasible_psi.size == 0:
            self.get_logger().warn("无可行臂角")
            return None

        psi_values = []
        for i in range(feasible_psi.shape[1]):
            psi_values.append(feasible_psi[0, i])
            psi_values.append(feasible_psi[1, i])

        closest_psi = min(psi_values, key=lambda x: abs(x - current_psi))
        q = self.compute_inverse_kinematics(T, closest_psi)
        if q is not None:
            return closest_psi

        for psi in psi_values:
            if psi == closest_psi:
                continue
            q = self.compute_inverse_kinematics(T, psi)
            if q is not None:
                return psi

        self.get_logger().warn("无法找到有效臂角")
        return None

    def timer_callback(self):
        if self.current_index >= len(self.df):
            self.get_logger().info("处理完成")
            return

        current_T = self.df[self.current_index]
        current_timestamp = self.timestamps[self.current_index]

        feasible_psi = getFeasibleArmAngle(current_T, self)
        if feasible_psi.size == 0:
            self.get_logger().warn(f"位姿 {current_timestamp} 无可行臂角")
            self.current_index += 1
            return

        next_psi = self.opt_psi_selector(self.current_psi, self.current_q, current_T)
        if next_psi is None:
            self.current_index += 1
            return

        joint_angles = self.compute_inverse_kinematics(current_T, next_psi)
        if joint_angles is None:
            self.get_logger().warn(f"位姿 {current_timestamp} 逆解失败")
            self.current_index += 1
            return

        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self.joint_names
        msg.position = joint_angles.tolist()
        self.joint_pub.publish(msg)

        self.get_logger().info(f"发布关节角度: {np.round(np.degrees(joint_angles), 2)} (deg)")
        self.current_psi, self.current_q = next_psi, joint_angles
        self.current_index += 1

def main(args=None):
    rclpy.init(args=args)
    node = RobotArmController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
