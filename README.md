# Industrial Calibration

![Industrial Calibration](docs/static/extrinsic_hand_eye_calibration.png)

## Description
A set of tools for performing calibration between cameras and robots and analyzing the accuracy of the calibration.

See the [documentation](https://ros-industrial.github.io/industrial_calibration) for more information on the project.

## 眼在手外参标定 GUI 使用说明

本节记录使用非对称圆阵列标定板完成 `TCP <-> Camera` 外参标定的完整流程。已经验收过的示例数据位于：

```text
examples/data/vision_align_left_eye_in_hand/
├── images/          # 去畸变后的完整实拍图
├── poses/           # 拍照时对应的机器人 Base -> TCP 位姿
├── cal_data.yaml    # GUI 配置及观测列表
└── output/          # 保存的标定结果
```

### 1. 点序从哪里来

点序不是逐点保存在 `cal_data.yaml` 中的。处理过程如下：

1. `cal_data.yaml` 定义标定板类型、行数、列数、间距和圆检测参数。
2. GUI 在每张 `images/*.png` 中自动检测圆心。
3. OpenCV 的非对称圆阵列检测器自动对圆心排序并分配编号。
4. 程序根据相同的行列顺序生成标定板上的 3D 点，并建立 2D--3D 对应关系。

因此，标定前必须逐帧点击 Observation，确认：

- 每帧都检测到 20 个点（当前标定板为 5 行 x 4 列）。
- 编号方向和原点方向正确。
- 所有帧的点序方向保持一致，没有某一帧整体翻转。
- 圆心标注确实落在对应圆的中心。

`cal_data.yaml` 中与点序相关的是：

```yaml
target_finder:
  type: ModifiedCircleGridTargetFinder
  asymmetric: true
  rows: 5
  cols: 4
  spacing: 0.1
```

其中 `spacing` 必须使用标定板的真实尺寸，单位为米。

### 2. 启动和退出 GUI

Compose 文件位于 `docker/`，因此需要在该目录启动：

```bash
cd /mnt/w/Github/industrial_calibration/docker
export CURRENT_UID="$(id -u):$(id -g)"
docker compose up --build
```

前台运行时按 `Ctrl+C` 退出。需要彻底停止并删除 Compose 容器时执行：

```bash
cd /mnt/w/Github/industrial_calibration/docker
docker compose down
```

Windows、WSL 和容器中的项目目录是同一份数据：

```text
Windows: W:\Github\industrial_calibration
WSL:     /mnt/w/Github/industrial_calibration
Docker:  /workspace
```

### 3. 采集新数据

建议采集至少 15 帧。每一帧必须同时保存：

- 一张包含完整标定板的清晰图像。
- 拍照时刻对应的 `T_base_tcp` 机器人位姿。

采集姿态应包含明显的平移和绕不同轴的旋转，不要只改变一个方向。图像与位姿使用相同编号，例如：

```text
images/1.png  <->  poses/1.yaml
images/2.png  <->  poses/2.yaml
```

单个位姿文件格式如下，平移单位为米，四元数顺序为 `qx qy qz qw`：

```yaml
x: 0.5
y: 0.1
z: 0.4
qx: 0.0
qy: 0.0
qz: 0.0
qw: 1.0
```

### 4. 对原图去畸变

使用 `examples/undistort_calibration_images.py`。下面是当前相机参数的完整示例：

```bash
cd /mnt/w/Github/industrial_calibration

python3 examples/undistort_calibration_images.py \
  /path/to/new_capture/raw_images \
  /path/to/new_calibration/images \
  --fx 616.0441 \
  --fy 616.1274 \
  --cx 639.7722 \
  --cy 410.804 \
  --distortion -0.787511 0.063795 0.00044492 -0.000076 0.114368 -0.766721 0.034261 0.125889 \
  --new-fx 611.611450195 \
  --new-fy 611.404785156 \
  --new-cx 639.113524472 \
  --new-cy 410.726658058 \
  --strip-suffix _Color
```

例如，`1_Color.png` 会输出为 `images/1.png`。如果原始文件已经叫 `1.png`，省略 `--strip-suffix _Color`。

脚本结束时会打印去畸变图像对应的 `fx/fy/cx/cy`。这些值必须写入后续的 `cal_data.yaml`，不能继续使用与原始畸变图对应的其他内参。

如果相机或 SDK 输出的图像已经去畸变，可以跳过此步骤，直接把图像放入新数据集的 `images/`。

### 5. 生成 poses 和 cal_data.yaml

如果机器人位姿已经保存为 `poses/1.yaml`、`poses/2.yaml` 等文件：

```bash
python3 examples/generate_hand_eye_cal_data.py \
  /path/to/new_calibration \
  --template examples/data/vision_align_left_eye_in_hand/cal_data.yaml \
  --previous-result examples/data/vision_align_left_eye_in_hand/output/test_cali_output
```

如果位姿是当前采集程序生成的 `samples_left_arm.csv`，可由同一个脚本生成 `poses/*.yaml`：

```bash
python3 examples/generate_hand_eye_cal_data.py \
  /path/to/new_calibration \
  --pose-csv /path/to/new_capture/samples_left_arm.csv \
  --template examples/data/vision_align_left_eye_in_hand/cal_data.yaml \
  --previous-result examples/data/vision_align_left_eye_in_hand/output/test_cali_output
```

CSV 必须包含以下列：

```text
image_id,tcp_x_m,tcp_y_m,tcp_z_m,tcp_qx,tcp_qy,tcp_qz,tcp_qw
```

`--previous-result` 会把上一次标定结果作为本次优化初值。同一套相机和 TCP 安装关系可以复用；如果标定板移动距离很大，还应检查 `target_mount_to_target_guess` 是否仍是合理初值。

生成后打开 `/path/to/new_calibration/cal_data.yaml`，重点检查：

```yaml
intrinsics:
  fx: ...
  fy: ...
  cx: ...
  cy: ...

target_finder:
  rows: 5
  cols: 4
  spacing: 0.1
```

### 6. 在 GUI 中执行标定

1. 选择 **Load configuration file**，加载新数据集的 `cal_data.yaml`。
2. 选择 **Load observations**，再次加载同一个 `cal_data.yaml`。
3. 逐帧点击 Observation，检查点数、圆心位置和点序。
4. 有误检、漏检、点序翻转或图像模糊时，先修正或删除该帧。
5. 点击 **Calibrate**。
6. 确认结果中 `converged: true`，并检查最终成本、重投影误差和各帧一致性。
7. 点击 **Save**，建议保存为带 `.yaml` 扩展名的文件。

当出现 `Failed to find asymmetric circle centers` 时，说明当前图片没有被识别成完整的非对称圆阵列。优先检查：

- `rows/cols` 是否正确。
- 圆是白色还是黑色，`circleColor` 是否匹配。
- `minArea/maxArea` 是否覆盖实拍图中的圆面积。
- 图像是否模糊、过曝或标定板不完整。

当前示例实拍图已经验证可用的关键参数为：

```yaml
filterByArea: true
minArea: 300.0
maxArea: 15000.0
filterByCircularity: true
minCircularity: 0.6
```

### 7. 标定结果和矩阵方向

眼在手模式下，GUI 保存结果时会同时输出位姿和两个便于交接的 4x4 矩阵：

```yaml
camera_mount_to_camera:  # 相机在 TCP 下的位姿（平移 + 四元数）
T_tcp_camera:            # Camera -> TCP
T_camera_tcp:            # TCP -> Camera
```

矩阵使用方式：

```text
p_tcp    = T_tcp_camera * p_camera
p_camera = T_camera_tcp * p_tcp
T_base_camera = T_base_tcp * T_tcp_camera
```

其中点使用齐次坐标 `[x, y, z, 1]^T`，平移单位为米。交接给其他程序时优先使用这两个 4x4 矩阵，可以避免四元数顺序和变换方向产生歧义。
