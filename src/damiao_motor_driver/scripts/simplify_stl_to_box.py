#!/usr/bin/env python3
"""
STL 文件简化脚本
将 STL 文件简化为最小包围长方体（bounding box）

使用方法:
    python3 simplify_stl_to_box.py <input_folder> <output_folder>

示例:
    python3 simplify_stl_to_box.py /path/to/meshes /path/to/simplified_meshes
"""

import os
import sys
import argparse
from pathlib import Path
import numpy as np

try:
    from stl import mesh
except ImportError:
    print("错误: 需要安装 numpy-stl 库")
    print("安装命令: pip install numpy-stl")
    sys.exit(1)


def create_bounding_box_stl(vertices):
    """
    创建包围顶点的最小长方体 STL

    Args:
        vertices: numpy array, shape (N, 3) - 原始 mesh 的所有顶点

    Returns:
        numpy-stl mesh 对象: 长方体 mesh
    """
    if len(vertices) == 0:
        raise ValueError("顶点列表为空")

    # 计算边界框
    min_bounds = np.min(vertices, axis=0)
    max_bounds = np.max(vertices, axis=0)

    # 计算尺寸和中心
    size = max_bounds - min_bounds
    center = (min_bounds + max_bounds) / 2.0

    # 创建长方体的8个顶点
    # 相对于中心点的偏移
    offsets = np.array([
        [-1, -1, -1],
        [1, -1, -1],
        [1,  1, -1],
        [-1,  1, -1],
        [-1, -1,  1],
        [1, -1,  1],
        [1,  1,  1],
        [-1,  1,  1],
    ]) * (size / 2.0)

    # 将顶点移动到正确位置
    vertices_box = center + offsets

    # 定义长方体的12个三角形面（每个面2个三角形）
    # 顶点索引：0=(-,-,-), 1=(+,-,-), 2=(+,+,-), 3=(-,+,-)
    #           4=(-,-,+), 5=(+,-,+), 6=(+,+,+), 7=(-,+,+)
    # 每个面的三角形顶点必须按逆时针顺序（从外部看），法向量指向外部
    faces = np.array([
        # 底面 (z = min, 法向量向下 -z)
        [0, 3, 2], [0, 2, 1],  # 从下往上看，逆时针
        # 顶面 (z = max, 法向量向上 +z)
        [4, 5, 6], [4, 6, 7],  # 从上往下看，逆时针
        # 前面 (y = min, 法向量向前 -y)
        [0, 1, 5], [0, 5, 4],  # 从前面看，逆时针
        # 后面 (y = max, 法向量向后 +y)
        [2, 3, 7], [2, 7, 6],  # 从后面看，逆时针
        # 左面 (x = min, 法向量向左 -x)
        [0, 4, 7], [0, 7, 3],  # 从左面看，逆时针
        # 右面 (x = max, 法向量向右 +x)
        [1, 2, 6], [1, 6, 5],  # 从右面看，逆时针
    ])

    # 创建 STL mesh
    box_mesh = mesh.Mesh(np.zeros(faces.shape[0], dtype=mesh.Mesh.dtype))

    for i, face in enumerate(faces):
        # 获取三角形的三个顶点
        v0 = vertices_box[face[0]]
        v1 = vertices_box[face[1]]
        v2 = vertices_box[face[2]]

        # 计算法向量（确保指向外部）
        # 使用右手定则：从 v0 到 v1 到 v2 的叉积
        edge1 = v1 - v0
        edge2 = v2 - v0
        normal = np.cross(edge1, edge2)
        norm = np.linalg.norm(normal)

        # 避免除零错误
        if norm > 1e-10:
            normal = normal / norm
        else:
            # 如果法向量为零，使用默认值
            normal = np.array([0.0, 0.0, 1.0])

        # 设置 STL 数据
        box_mesh.vectors[i] = np.array([v0, v1, v2])
        box_mesh.normals[i] = normal

    return box_mesh


def simplify_stl_file(input_path, output_path):
    """
    简化单个 STL 文件为包围长方体

    Args:
        input_path: 输入 STL 文件路径
        output_path: 输出 STL 文件路径

    Returns:
        bool: 是否成功
    """
    try:
        # 读取 STL 文件
        print(f"  处理: {input_path.name}")
        original_mesh = mesh.Mesh.from_file(str(input_path))

        # 提取所有顶点
        # STL 文件中的每个三角形有3个顶点
        vertices = original_mesh.vectors.reshape(-1, 3)

        # 计算边界框（用于体积计算）
        min_bounds = np.min(vertices, axis=0)
        max_bounds = np.max(vertices, axis=0)
        box_size = max_bounds - min_bounds

        # 创建包围长方体
        box_mesh = create_bounding_box_stl(vertices)

        # 确保输出目录存在
        output_path.parent.mkdir(parents=True, exist_ok=True)

        # 导出简化后的 STL
        box_mesh.save(str(output_path))

        # 计算体积
        # 对于原始 mesh，尝试获取体积（如果 mesh 是封闭的）
        try:
            original_volume = original_mesh.get_mass_properties()[0]
            if original_volume < 0:
                # 如果体积为负，说明 mesh 可能有问题，使用边界框体积作为参考
                original_volume = np.prod(box_size)
        except:
            # 如果无法计算，使用边界框体积
            original_volume = np.prod(box_size)

        # 对于长方体，直接计算体积（长×宽×高）
        box_volume = np.prod(box_size)

        # 计算减少百分比
        reduction = (1 - box_volume / original_volume) * \
            100 if original_volume > 0 else 0

        # 打印信息
        print(f"    原始体积: {original_volume:.6f}")
        print(f"    长方体体积: {box_volume:.6f}")
        print(f"    体积减少: {reduction:.2f}%")
        print(f"    保存到: {output_path}")

        return True

    except Exception as e:
        print(f"  错误: 处理 {input_path.name} 时出错: {str(e)}")
        import traceback
        traceback.print_exc()
        return False


def process_folder(input_folder, output_folder):
    """
    处理文件夹中的所有 STL 文件

    Args:
        input_folder: 输入文件夹路径
        output_folder: 输出文件夹路径
    """
    input_path = Path(input_folder)
    output_path = Path(output_folder)

    # 检查输入文件夹是否存在
    if not input_path.exists():
        print(f"错误: 输入文件夹不存在: {input_folder}")
        return False

    if not input_path.is_dir():
        print(f"错误: 输入路径不是文件夹: {input_folder}")
        return False

    # 创建输出文件夹
    output_path.mkdir(parents=True, exist_ok=True)

    # 查找所有 STL 文件（不区分大小写）
    stl_files = list(input_path.glob("*.stl")) + list(input_path.glob("*.STL"))

    # 递归查找子文件夹中的 STL 文件
    stl_files.extend(input_path.rglob("*.stl"))
    stl_files.extend(input_path.rglob("*.STL"))

    # 去重
    stl_files = list(set(stl_files))

    if not stl_files:
        print(f"警告: 在 {input_folder} 中未找到 STL 文件")
        return False

    print(f"找到 {len(stl_files)} 个 STL 文件")
    print(f"输入文件夹: {input_folder}")
    print(f"输出文件夹: {output_folder}")
    print("-" * 60)

    success_count = 0
    fail_count = 0

    # 处理每个 STL 文件
    for stl_file in stl_files:
        # 保持相对路径结构
        relative_path = stl_file.relative_to(input_path)
        output_file = output_path / relative_path

        if simplify_stl_file(stl_file, output_file):
            success_count += 1
        else:
            fail_count += 1
        print()

    # 打印总结
    print("-" * 60)
    print(f"处理完成!")
    print(f"成功: {success_count}")
    print(f"失败: {fail_count}")
    print(f"总计: {len(stl_files)}")

    return fail_count == 0


def main():
    parser = argparse.ArgumentParser(
        description="将 STL 文件简化为最小包围长方体",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
示例:
  %(prog)s /path/to/meshes /path/to/simplified_meshes
  %(prog)s ./urdf/meshes ./urdf/simplified_meshes
        """
    )

    parser.add_argument(
        "input_folder",
        type=str,
        help="包含 STL 文件的输入文件夹路径"
    )

    parser.add_argument(
        "output_folder",
        type=str,
        help="保存简化后 STL 文件的输出文件夹路径"
    )

    args = parser.parse_args()

    # 处理文件夹
    success = process_folder(args.input_folder, args.output_folder)

    sys.exit(0 if success else 1)


if __name__ == "__main__":
    main()
