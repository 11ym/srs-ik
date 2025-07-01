import numpy as np
import csv

def rotate_point_around_x(point, angle=np.pi):
    """绕X轴旋转三维点"""
    x, y, z = point
    cos_angle = np.cos(angle)
    sin_angle = np.sin(angle)
    new_y = y * cos_angle - z * sin_angle
    new_z = y * sin_angle + z * cos_angle
    return [x, new_y, new_z]

def rotate_quaternion_around_x(quat, angle=np.pi):
    """绕X轴旋转四元数"""
    rotation_axis = np.array([1, 0, 0])
    half_angle = angle / 2
    rotation_quat = np.array([
        np.sin(half_angle) * rotation_axis[0],
        np.sin(half_angle) * rotation_axis[1],
        np.sin(half_angle) * rotation_axis[2],
        np.cos(half_angle)
    ])
    
    q = quat.copy()
    q_rot = rotation_quat
    
    new_x = q_rot[3]*q[0] + q_rot[0]*q[3] + q_rot[1]*q[2] - q_rot[2]*q[1]
    new_y = q_rot[3]*q[1] - q_rot[0]*q[2] + q_rot[1]*q[3] + q_rot[2]*q[0]
    new_z = q_rot[3]*q[2] + q_rot[0]*q[1] - q_rot[1]*q[0] + q_rot[2]*q[3]
    new_w = q_rot[3]*q[3] - q_rot[0]*q[0] - q_rot[1]*q[1] - q_rot[2]*q[2]
    
    return [new_x, new_y, new_z, new_w]

def process_csv(input_file, output_file):
    """处理CSV文件，旋转左侧数据并去除引号"""
    with open(input_file, 'r') as infile, open(output_file, 'w', newline='') as outfile:
        reader = csv.reader(infile)
        writer = csv.writer(outfile)
        
        # 写入标题行
        header = next(reader)
        writer.writerow(header)
        
        # 处理每一行数据
        for row in reader:
            timestamp = row[0]
            
            # 解析右侧数据（去除引号）
            right_ee_xyz = [float(x.strip('"')) for x in row[1].split(',')]
            right_ee_quat = [float(x.strip('"')) for x in row[2].split(',')]
            
            # 解析左侧数据
            left_ee_xyz = [float(x.strip('"')) for x in row[3].split(',')]
            left_ee_quat = [float(x.strip('"')) for x in row[4].split(',')]
            
            # 绕X轴旋转180度
            rotated_left_ee_xyz = rotate_point_around_x(left_ee_xyz)
            rotated_left_ee_quat = rotate_quaternion_around_x(left_ee_quat)
            
            # 格式化输出（不使用引号）
            right_ee_xyz_str = ",".join([f"{x:.4f}" for x in right_ee_xyz])
            right_ee_quat_str = ",".join([f"{x:.4f}" for x in right_ee_quat])
            left_ee_xyz_str = ",".join([f"{x:.4f}" for x in rotated_left_ee_xyz])
            left_ee_quat_str = ",".join([f"{x:.4f}" for x in rotated_left_ee_quat])
            
            # 写入新行（数值无引号）
            writer.writerow([timestamp, right_ee_xyz_str, right_ee_quat_str, left_ee_xyz_str, left_ee_quat_str])

if __name__ == "__main__":
    input_file = "all.csv"  # 替换为实际输入文件名
    output_file = "output.csv"  # 替换为实际输出文件名
    process_csv(input_file, output_file)
    print(f"处理完成，结果已保存到 {output_file}")
