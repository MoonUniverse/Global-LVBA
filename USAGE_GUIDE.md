# Odin1 → Global-LVBA 使用流程

## 概览

将 Manifold Odin1 LiDAR-相机传感器的录制数据（recorddata）转换为 Global-LVBA 格式，
然后运行 LiDAR-Visual Bundle Adjustment 管线优化点云和位姿。

```
录制数据 → odin2lvba.py 转换 → undistort_odin.py 去畸变 → Global-LVBA 运行
```

---

## 1. 环境准备（首次）

### 1.1 系统要求

- Ubuntu 22.04 + ROS2 Humble
- NVIDIA GPU（SiftGPU 需要 OpenGL）
- 至少 32GB 内存（7000帧数据峰值约 27GB）

### 1.2 安装依赖

```bash
# 基础依赖
sudo apt install libceres-dev freeglut3-dev libgoogle-glog-dev \
    libgflags-dev libglew-dev libsqlite3-dev python3-numpy python3-opencv

# Sophus（header-only，需从源码安装）
cd /tmp && git clone https://github.com/strasdat/Sophus.git
cd Sophus && mkdir build && cd build
# Ubuntu 22.04 cmake 版本需要 patch
sed -i 's/cmake_minimum_required(VERSION 3\.[0-9]*)/cmake_minimum_required(VERSION 3.14)/' ../CMakeLists.txt
cmake .. -DBUILD_TESTS=OFF -DBUILD_EXAMPLES=OFF
make -j$(nproc) && sudo make install

# Sophus 新版用 .hpp 后缀，项目用 .h，需要创建符号链接
for f in /usr/local/include/sophus/*.hpp; do
  base=$(basename "$f" .hpp)
  sudo ln -sf "$f" "/usr/local/include/sophus/${base}.h"
done
```

### 1.3 编译 SiftGPU

```bash
cd /home/asus/odin1/Global-LVBA/src/SiftGPU
git submodule update --init
mkdir -p build && cd build
cmake .. && make -j$(nproc)
# 确认生成: src/SiftGPU/build/src/SiftGPU/libsiftgpu.a
```

### 1.4 编译 Global-LVBA

```bash
cd /home/asus/odin1
mkdir -p lvba_ws/src
ln -sfn /home/asus/odin1/Global-LVBA lvba_ws/src/global_lvba
source /opt/ros/humble/setup.bash
cd lvba_ws
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
```

---

## 2. 数据转换

### 2.1 录制数据结构

Odin recorddata 目录（所有文件在根目录，扁平结构）：

```
20260307_100516/
├── OdinPose.bin            # 6DoF 位姿
├── OdinImage.bin           # JPEG 图像流
├── MT20260307_100516.olx   # SLAM 点云（每帧 N 个 XYZRGBA 点）
├── OdinIMU.bin             # IMU 数据
├── OdinRotate.bin          # 旋转数据
├── image/                  # 已提取的 JPEG 图像（可忽略，脚本从 .bin 读取）
│   ├── cam_in_ex.txt       # 相机内外参（供参考）
│   └── img0_*.JPEG
├── calib_online.yaml       # 在线标定结果
└── project_info.json       # 项目信息
```

### 2.2 运行转换

> ⚠️ **重要：`MT*.olx` 里的 SLAM cloud 默认是 `odom/world` 坐标系，不是 `body` 坐标系。**
> Global-LVBA 的 `all_pcd_body/*.pcd` 必须是 **LiDAR 机体系点云**。因此使用 Odin 驱动录制的数据时，
> 转换脚本需要先按对应位姿把点云从 `odom/world` 反变换回 `body` 再写入 `all_pcd_body/`。
> 当前脚本已经支持这个流程，推荐显式传入 `--cloud-frame odom`。

```bash
# 推荐：保留 image-cloud 1:1 对齐，并把 SLAM cloud 从 odom/world 转回 body
python3 /home/asus/odin1/odin_ros_driver/script/odin2lvba.py \
    /home/asus/Documents/odin1_data/20260307_100516 \
    /home/asus/Documents/odin1_lvba/20260307_100516_align20ms_fixbody \
    --align-nearest-ms 20 \
    --cloud-frame odom
```

常用参数：
- `--align-nearest-ms 20`
  用图像和点云做互为最近邻的 1:1 配对，只保留时间差在 20ms 内的帧对，推荐用于 Odin 录制数据。
- `--cloud-frame odom`
  表示 `MT*.olx` 中的点云已经在 `odom/world` 系，脚本会自动转换回 `body` 系后再输出给 LVBA。
- `--cloud-frame body`
  仅当你确认输入点云本来就是机体系时才使用。

输出结构（直接在输出目录下生成）：
```
20260307_100516_align20ms_fixbody/
├── all_image/
│   ├── 113.757220.png       # 以时间戳命名的 PNG
│   ├── ...
│   └── image_poses.txt      # TUM 格式位姿: ts tx ty tz qx qy qz qw
└── all_pcd_body/
    ├── 113.757220.pcd       # 二进制 PCD (PointXYZINormal)
    ├── ...
    └── lidar_poses.txt      # TUM 格式位姿
```

### 2.3 图像去畸变

Odin 使用多项式鱼眼模型（含 A12 skew），必须先去畸变再给 LVBA（LVBA 用 pinhole 模型）：

```bash
python3 /home/asus/odin1/odin_ros_driver/script/undistort_odin.py \
    --img_dir /home/asus/Documents/odin1_lvba/20260307_100516/all_image \
    --jobs 8
```

> ⚠️ 这会**原地覆盖**图像文件。如需保留原始图像，使用 `--out_dir` 参数。

> 📝 默认参数适用于设备 #O1-P010100014。如果换了设备，需要从 `image/cam_in_ex.txt` 中
> 读取新的 A11/A12/A22/u0/v0/k2-k7 参数，通过命令行传入：
> `--A11 xxx --A12 xxx --A22 xxx --u0 xxx --v0 xxx --k2 xxx ...`

---

## 3. 配置 Global-LVBA

### 3.1 创建数据集符号链接

```bash
ln -sfn /home/asus/Documents/odin1_lvba/20260307_100516 \
    /home/asus/odin1/Global-LVBA/dataset/20260307_100516
```

### 3.2 修改 config/config.yaml

编辑 `/home/asus/odin1/Global-LVBA/config/config.yaml`：

```yaml
lv_ba:
  ros__parameters:
    cam_model:
      cam_width: 1600           # Odin 图像宽
      cam_height: 1296          # Odin 图像高
      scale: 0.5                # 内部缩放比（降低 SiftGPU 显存占用）
      cam_fx: 733.347088        # A11 — 来自 cam_in_ex.txt
      cam_fy: 733.652700        # A22
      cam_cx: 768.670454        # u0
      cam_cy: 624.177971        # v0
      cam_d0: 0.0               # 已去畸变，全部设 0
      cam_d1: 0.0
      cam_d2: 0.0
      cam_d3: 0.0

    extrin_calib:
      extrinsic_T: [0.0, 0.0, 0.0]       # IMU-LiDAR（Odin 位姿已在 LiDAR 坐标系，设为零）
      extrinsic_R: [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
      Rcl: [-0.012330, -0.999830, -0.013650,   # Camera→LiDAR 旋转（Tcl 前3x3）
            -0.001750,  0.013670, -0.999910,
             0.999920, -0.012310, -0.001920]
      Pcl: [0.039420, 0.015750, -0.007440]      # Camera→LiDAR 平移（Tcl 第4列）

    data_config:
      data_path: "dataset/20260307_100516_align20ms_fixbody/"  # ← 改成你的数据集名称
      colmap_db_path: "no_colmap.db"
      image_sample_step: 5
      enable_lidar_ba: true
      enable_visual_ba: true

    window_ba:
      enable: true
      size: 20
      anchor_leaf_size: 0.01
      use_window_ba_rel: true

    BALM_stage1:
      enable: true
      root_voxel_size: 1.0
      eigen_ratio_array: [0.2, 0.2, 0.2, 0.2]

    BALM_stage2:
      root_voxel_size: 0.5
      eigen_ratio_array: [0.08, 0.08, 0.08, 0.08]

    track_fusion:
      min_view_angle: 8.0
      reproj_mean_thr: 15.0       # 重投影误差阈值，建议 10-20

    colmap_output:
      enable: false
      filter_size_points3D: 0.01
```

> 📝 **换设备时需要修改**: cam_fx/fy/cx/cy（对应 A11/A22/u0/v0）和 Rcl/Pcl（对应 Tcl）

### 3.3 关键参数说明

| 参数 | 含义 | 建议值 |
|------|------|--------|
| `image_sample_step` | 图像采样步长 | 5-10（越小越多图像，越慢） |
| `window_ba.size` | 滑窗 BA 窗口大小 | 20 |
| `reproj_mean_thr` | Track 重投影误差阈值 | 15.0（实测值）|
| `BALM_stage2.root_voxel_size` | 精细 BA 体素大小 | 0.5（越小越精细） |
| `scale` | 图像缩放 | 0.5（降低 GPU 显存） |

---

## 4. 运行管线

```bash
cd /home/asus/odin1/lvba_ws
source /opt/ros/humble/setup.bash
source install/setup.bash

CONFIG=/home/asus/odin1/Global-LVBA/config/config.yaml
echo "1" | ros2 run global_lvba lidar_visual_ba \
    --ros-args --params-file $CONFIG -p runtime_config_path:=$CONFIG
```

> ⚠️ 程序会出现确认提示，`echo "1" |` 自动输入 1 跳过。

### 管线阶段

```
1. 数据加载        → 读取图像、PCD、位姿
2. Window LiDAR BA → 滑窗 LiDAR Bundle Adjustment
3. Global BALM     → Stage1 + Stage2 全局 LiDAR BA
4. Visual BA:
   ├─ buildGridMap           → 构建全局体素地图
   ├─ updateCameraPoses      → 从优化后的 LiDAR 位姿更新相机位姿
   ├─ generateDepthWithVoxel → 为每张图像生成深度图
   ├─ extractAndMatchGPU     → SiftGPU 特征提取 + 匹配
   ├─ BuildTracksAndFuse3D   → 建立特征轨迹 + 三角化
   ├─ optimizeCameraPoses    → Ceres 优化相机位姿
   ├─ visualizeProj          → 可视化重投影
   └─ pubRGBCloud            → 发布彩色点云
```

### 运行时间参考（RTX 4060, 32GB RAM, ~6000帧）

| 阶段 | 预计耗时 |
|------|----------|
| LiDAR BA (Window + BALM) | ~5 分钟 |
| Depth Generation | ~3 分钟 |
| SiftGPU 匹配 | ~30 分钟 |
| Track 建立 + 优化 | ~5 分钟 |
| **总计** | **~45 分钟** |

---

## 5. 输出结果

管线在数据集目录下生成：

```
dataset/20260307_100516/
├── depth/              # 每帧深度图 (PNG, 16bit)
├── result/             # 特征匹配可视化
├── track_features/     # Track 可视化
├── reproj/             # 重投影可视化（优化前后对比）
└── Colmap/             # (如果 enable) COLMAP 格式输出
    ├── sparse/         # cameras.txt, images.txt, points3D.txt
    └── images/
```

---

## 6. 完整示例（端到端）

以新录制 `20260307_100516` 为例：

```bash
# Step 1: 转换数据
python3 /home/asus/odin1/odin_ros_driver/script/odin2lvba.py \
    /home/asus/Documents/odin1_data/20260307_100516 \
    /home/asus/Documents/odin1_lvba/20260307_100516_align20ms_fixbody \
    --align-nearest-ms 20 \
    --cloud-frame odom

# Step 2: 去畸变
python3 /home/asus/odin1/odin_ros_driver/script/undistort_odin.py \
    --img_dir /home/asus/Documents/odin1_lvba/20260307_100516_align20ms_fixbody/all_image \
    --jobs 8

# Step 3: 创建数据集链接
ln -sfn /home/asus/Documents/odin1_lvba/20260307_100516_align20ms_fixbody \
    /home/asus/odin1/Global-LVBA/dataset/20260307_100516_align20ms_fixbody

# Step 4: 修改 config.yaml 中的 data_path
#   data_path: "dataset/20260307_100516_align20ms_fixbody/"

# Step 5: 运行管线
cd /home/asus/odin1/lvba_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
CONFIG=/home/asus/odin1/Global-LVBA/config/config.yaml
printf '1\n' | ros2 run global_lvba lidar_visual_ba \
    --ros-args --params-file $CONFIG -p runtime_config_path:=$CONFIG
```

---

## 7. 常见问题

### SiftGPU 上下文创建失败
```
[SiftGPU] Not supported or context creation failed.
```
需要 X11 显示：`export DISPLAY=:0`，或使用虚拟帧缓冲 `Xvfb :99 &`

### 内存不足被 Kill (exit 247)
7000帧全局地图约 244M 点，峰值占 ~27GB。解决方案：
- 增大 `image_sample_step`（减少处理的图像数）
- 截取部分数据进行测试

### Track 过滤后剩余很少
`[TrackFilter] kept=X dropped=Y`
- 增大 `reproj_mean_thr`（建议 15-30）
- 检查相机外参 Rcl/Pcl 是否与 cam_in_ex.txt 一致
- 确认已执行去畸变

### LiDAR `before` 点云明显错位
- 优先检查 `odin2lvba.py` 是否使用了 `--cloud-frame odom`
- Odin 驱动录到的 `MT*.olx` 是 `SLAM cloud`，默认在 `odom/world` 系
- 如果直接把它当作 `all_pcd_body` 输入给 LVBA，会被再次乘位姿，导致 `before` 地图严重错位
- 建议重新转换为 `*_align20ms_fixbody/` 这类数据集后再跑

### 编译错误: sophus/se3.h not found
确认创建了 `.h → .hpp` 符号链接（见 1.2 节）

### 编译错误: cannot find -lSophus
Sophus 是 header-only。检查 CMakeLists.txt 中 `set(Sophus_LIBRARIES "")` 是否设置。

---

## 8. 文件清单

```
odin1/
├── odin_ros_driver/script/
│   ├── odin2lvba.py          # 数据转换脚本
│   └── undistort_odin.py     # 图像去畸变脚本
├── Global-LVBA/
│   ├── config/config.yaml    # 主配置文件
│   ├── launch/lvba.launch.py # ROS2 Launch 文件
│   ├── dataset/              # 数据集目录（放符号链接）
│   └── src/SiftGPU/          # SiftGPU 子模块
└── lvba_ws/                  # colcon 工作空间
    ├── build/
    └── install/
```
