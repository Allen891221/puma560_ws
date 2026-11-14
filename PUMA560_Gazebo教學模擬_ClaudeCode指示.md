# 任務：建立 PUMA560 正向運動學教學模擬（ROS 2 Humble + Gazebo，Ubuntu 22.04）

## 0. 背景與目標

我是研究所「高等機器人學」的學生，教科書是 John J. Craig《Introduction to Robotics》4th ed.。請幫我做一套**輔助教材**：

1. 在 Gazebo 中顯示 PUMA560 機械手臂。
2. 一個 GUI 視窗，有 **6 個拉桿（slider）**，分別調整 θ1～θ6，Gazebo 中的手臂即時跟著動。
3. 同一個視窗**即時顯示 ⁰₆T（4×4 齊次轉換矩陣）**，數值隨拉桿變動而更新。
4. 所有座標系與參數**嚴格依照 Craig 的修正式 DH（modified DH）**，不可使用 Spong 等書的標準 DH。

這是教學用途：正確性比外觀重要。手臂外觀用簡單幾何形狀即可，但**每個 link 的座標系必須和 Craig DH 座標系 {0}～{6} 完全一致**。

---

## 1. 環境

- 作業系統：Ubuntu 22.04
- ROS 2：Humble
- Gazebo：優先使用 Humble 官方搭配的 **Gazebo Fortress（gz sim / ignition）**，透過 `ros_gz` 連接。
- 程式語言：Python 3（GUI 與運動學），URDF 用 xacro。

### 安裝注意事項（請先檢查，不要猜套件名稱）

- 先用 `apt-cache search` / `apt list --installed` 確認實際可用的套件名稱再安裝。Humble 搭配 Fortress 的 ros2_control 外掛名稱可能是 `ign_ros2_control` 或 `gz_ros2_control`，請**實際查證**後再寫進 launch 與 URDF。
- 預計需要（以實際查證結果為準）：`ros-humble-desktop`、`ros-humble-ros-gz`、ros2_control 的 Gazebo 外掛、`ros-humble-ros2-controllers`、`ros-humble-xacro`、`ros-humble-robot-state-publisher`、`python3-numpy`、`python3-pyqt5`。
- 若安裝或執行遇到問題，停下來告訴我錯誤訊息與你建議的解法，不要擅自改用完全不同的架構。
- 若虛擬機或無 GPU 導致 Gazebo 無法顯示，提供 `LIBGL_ALWAYS_SOFTWARE=1` 等軟體渲染的替代方式，並寫進 README。

---

## 2. PUMA560 的 DH 參數（Craig 修正式 DH，必須完全照此）

連桿轉換：

```
^{i-1}_i T = RotX(α_{i-1}) · TransX(a_{i-1}) · RotZ(θ_i) · TransZ(d_i)
```

展開後：

```
[ cθ        -sθ        0      a_{i-1}     ]
[ sθ·cα     cθ·cα     -sα    -sα·d_i      ]
[ sθ·sα     cθ·sα      cα     cα·d_i      ]
[ 0          0          0      1          ]
```

（α = α_{i-1}，θ = θ_i）

| i | α_{i-1} | a_{i-1} | d_i | θ_i |
|---|---|---|---|---|
| 1 | 0 | 0 | 0 | θ1 |
| 2 | −90° | 0 | 0 | θ2 |
| 3 | 0 | a2 | d3 | θ3 |
| 4 | −90° | a3 | d4 | θ4 |
| 5 | +90° | 0 | 0 | θ5 |
| 6 | −90° | 0 | 0 | θ6 |

尺寸（URDF 內用公尺，GUI 顯示預設用 mm，可切換）：

| 參數 | mm | m |
|---|---|---|
| a2 | 431.8 | 0.4318 |
| a3 | 20.32 | 0.02032 |
| d3 | 149.09 | 0.14909 |
| d4 | 433.07 | 0.43307 |

慣例：{0} 在 θ1 = 0 時與 {1} 重合；{6} 原點在腕心（三條腕軸交點），d6 = 0。

### 2.1 URDF 的關鍵：讓 link frame 等於 DH frame

URDF 的 joint 是「先套 `<origin>`，再繞 `<axis>` 轉」。Craig 的轉換是 RotX(α)·TransX(a)·RotZ(θ)·TransZ(d)，而 TransZ(d) 和 RotZ(θ) 可交換，所以每個 joint 可以寫成：

```xml
<joint name="joint_i" type="revolute">
  <parent link="link_{i-1}"/>
  <child  link="link_i"/>
  <origin xyz="${a_{i-1}}  ${-sin(α_{i-1})*d_i}  ${cos(α_{i-1})*d_i}"
          rpy="${α_{i-1}} 0 0"/>
  <axis xyz="0 0 1"/>
</joint>
```

也就是 origin 的位置 = [a, −sα·d, cα·d]，姿態 = 繞 X 轉 α，旋轉軸固定為子座標系的 Z。

- 請用 xacro 寫一個巨集，**從 DH 表自動產生** joint，不要手算每個數字。
- `link_0` 就是 {0}（base），`link_6` 就是 {6}。
- 這樣 TF 上 `link_0 → link_6` 的轉換，就必須等於 ⁰₆T。這是整個教材的正確性核心，後面要驗證（見第 5 節）。

### 2.2 外觀（visual）

- 用 cylinder / box 畫出：底座、腰部、上臂（長 a2）、手肘偏移、前臂（長 d4）、手腕、小法蘭。
- 外觀只是示意，**visual 的 origin 可以自由調整**，但不可以改動 joint 的 origin。
- 每個 link 加上小型座標軸指示（紅 X、綠 Y、藍 Z 的細長圓柱），方便上課講解座標系；提供一個參數可開關。
- 加上合理的 inertial，避免 Gazebo 物理引擎出錯。

---

## 3. Gazebo 與關節控制

- 建立一個 world：**關閉重力**（教學用，只看運動學），有地板與光源。
- 用 ros2_control 的 Gazebo 外掛，每個關節開 **position command interface**，使用 `forward_command_controller`（position 型），topic 例如 `/forward_position_controller/commands`（`std_msgs/Float64MultiArray`，6 個值，單位 rad）。
- 也要啟動 `joint_state_broadcaster` 與 `robot_state_publisher`，讓 TF 可用。
- 若 ros2_control 外掛在 Fortress 上無法運作，替代方案是 Gazebo 內建的 `JointPositionController` system + `ros_gz_bridge`。改用替代方案前先告訴我原因。
- 關節範圍：預設 −180°～180°。另提供一個選項，改用 PUMA560 實際關節範圍；若不確定實際數值，請註明「僅供參考」，不要捏造來源。

---

## 4. GUI（PyQt5 + rclpy，獨立視窗）

### 4.1 必要功能

1. **6 個拉桿**：θ1～θ6，單位度（°），解析度 0.1°。每個拉桿旁邊有可直接輸入數字的欄位，兩者同步。
2. 拉桿變動時，**即時**發布關節命令到 Gazebo（rad），並更新矩陣顯示。
3. **⁰₆T 即時顯示**：
   - 4×4 表格，等寬字型，小數 4 位。
   - 旋轉部分（3×3）和位置欄（第 4 欄）用不同底色區分。
   - 位置單位可切換 mm / m（預設 mm）。
   - 旁邊標示每一欄的意義：第 1～3 欄 = {6} 的 X̂、Ŷ、Ẑ 軸在 {0} 中的方向；第 4 欄 = {6} 原點（腕心）位置。
4. **DH 參數表**：顯示上面那張表，θ 欄即時顯示目前角度。
5. **預設姿態按鈕**：
   - 全零姿態 (0, 0, 0, 0, 0, 0)
   - 練習題姿態 (60, −30, 30, 40, 45, −20)
   - 奇異姿態 θ5 = 0，例如 (30, −20, 40, 50, 0, −30)
   - 歸零（Reset）
6. **驗證區**：用 tf2 查詢 `link_0 → link_6` 的實際轉換，和 GUI 自己用 DH 公式算出的 ⁰₆T 比較，顯示最大誤差（mm）。誤差小於 0.01 mm 顯示綠色「一致」，否則紅色。

### 4.2 加分功能（核心完成後再做）

7. **展開各連桿轉換**：可切換顯示 ⁰₁T、¹₂T、…、⁵₆T 的數值，以及中間乘積 ¹₃T、³₆T、¹₆T（對應講義的分組乘法）。
8. **工具中心點（TCP）**：輸入工具長度 d6（mm），額外顯示 P_TCP = p + d6·Ẑ6。
9. **奇異點提示**：當 |sin θ5| < 0.01 時，顯示提示「θ5 ≈ 0：Ẑ4 與 Ẑ6 共線，只能決定 θ4+θ6」。

### 4.3 反向運動學面板（第二階段，等我確認第一階段 OK 再做）

- 按「以目前 ⁰₆T 反解」：用講義的 PUMA560 閉合解算出**全部 8 組解**，列成表格，點任一組即把拉桿設成那組角度。
- 公式使用以下**已修正**版本（講義原文有印刷錯誤）：
  - θ1 = atan2(py, px) − atan2(d3, ±√(px²+py²−d3²))
  - K = (px²+py²+**pz²**−a2²−a3²−d3²−d4²) / (2a2)
  - θ3 = atan2(a3, d4) − atan2(K, ±√(a3²+d4²−K²))
  - 令 u = c1·px + s1·py：
    θ23 = atan2( (−a3−a2c3)·pz − u·(d4−a2s3),  (a2s3−d4)·pz **+** (a3+a2c3)·u )，θ2 = θ23 − θ3
  - θ4 = atan2(−r13·s1 + r23·c1, −r13·c1·c23 − r23·s1·c23 + r33·s23)
  - θ5 由 s5、c5 求出：
    −s5 = r13(c1c23c4 + s1s4) + r23(s1c23c4 − c1s4) − r33(s23c4)
    c5 = −r13·c1·s23 − r23·s1·s23 − r33·c23
  - θ6 由 s6、c6 求出：
    s6 = −r11(c1c23s4 − s1c4) − r21(s1c23s4 + c1c4) + r31(s23s4)
    c6 = r11[(c1c23c4 + s1s4)c5 − c1s23s5] + r21[(s1c23c4 − c1s4)c5 − s1s23s5] − r31(s23c4c5 + c23s5)
  - 手腕翻轉：(θ4+180°, −θ5, θ6+180°)
- 每組解都要代回正向運動學驗算，顯示誤差。

---

## 5. 驗證與測試（必做）

把運動學寫成獨立、不依賴 ROS 的 Python 模組（例如 `puma560_kinematics.py`），並用 `pytest` 測試：

1. **全零姿態**：位置應為 (452.12, 149.09, −433.07) mm，旋轉部分應為 diag(1, −1, −1)。
2. **練習題姿態** (60, −30, 30, 40, 45, −20)：

```
[ 0.5074   0.5487  -0.6645    68.0192 ]
[ 0.5487  -0.8003  -0.2418   415.9926 ]
[-0.6645  -0.2418  -0.7071  -217.1700 ]
[ 0        0        0          1      ]
```

   （位置單位 mm，容許誤差 1e-3）

3. **(90, 0, −90, 0, 0, 0)**：位置應為 (−149.09, 864.87, 20.32) mm。
4. 任意 100 組隨機角度：旋轉部分滿足 RᵀR = I、det R = +1。
5. （第二階段）任意隨機姿態 → IK 得到的 8 組解，代回 FK 都和原 ⁰₆T 一致（誤差 < 1e-6）。
6. 啟動整套系統後，確認 GUI 的「驗證區」顯示 TF 與 DH 公式一致。

---

## 6. 專案結構（建議，可微調但請說明理由）

```
~/puma560_ws/src/
├── puma560_description/     # xacro、DH 參數 yaml、RViz 設定
├── puma560_gazebo/          # world、controllers.yaml、launch
└── puma560_teaching_gui/    # PyQt5 GUI node、puma560_kinematics.py、tests/
```

- DH 參數放在**一個** yaml 檔，URDF（xacro）和 GUI 都讀同一份，避免兩邊不一致。
- 一個主要 launch 檔：`ros2 launch puma560_gazebo teaching.launch.py`，同時啟動 Gazebo、生成手臂、啟動控制器與 GUI。
- 參數 `rviz:=true` 時另外開 RViz2，顯示 TF（每個 link 的座標軸），這對講解座標系很有幫助。

---

## 7. 文件

寫一份**繁體中文** README，包含：

1. 安裝步驟（實際用到的 apt 套件）
2. 編譯：`colcon build`、`source install/setup.bash`
3. 啟動指令與參數（`rviz:=true`、軟體渲染）
4. GUI 每個區塊的說明
5. DH 參數表，以及「URDF joint origin = [a, −sα·d, cα·d]、rpy = [α, 0, 0]」的原理說明
6. 常見問題（Gazebo 黑畫面、控制器沒啟動、TF 查不到）

程式碼請附上中文註解，特別是 DH 轉換、URDF 巨集、IK 公式的部分。

---

## 8. 工作方式

1. 先列出計畫與要安裝的套件，等我確認後再安裝。
2. **依序完成並各自驗證**：運動學模組 + pytest → URDF（先用 RViz2 + `joint_state_publisher_gui` 確認座標系正確）→ Gazebo + 控制器 → GUI → 驗證區。
3. 每完成一個階段，告訴我怎麼執行、預期會看到什麼，再進下一階段。
4. 不確定的地方（套件名稱、外掛相容性、PUMA 實際關節範圍）請直說並查證，不要編造。
5. 第 4.3 節的反向運動學面板，等我確認第一階段可以使用後再開始。
