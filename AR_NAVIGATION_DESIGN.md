# 视觉 AR 室内导航改造方案

> 基于 Android_ORB-SLAM2s 项目的 AR 导航系统设计文档
> 适用分支：`claude/ar-navigation-design-dP6Zi`

---

## 一、现有系统能力分析

### 1.1 已具备的核心能力

| 能力 | 现有实现 | 代码位置 |
|------|---------|---------|
| 单目视觉里程计 | `System::TrackMonocular()` | `cpp/include/System.h` |
| 6-DOF 实时位姿 | `getV()` 返回视图矩阵 | `NativeHelper.java:178` |
| 平面检测（SVD） | `Plane` 类 + `detect()` | `cpp/Plane.h` |
| 地图持久化 | `saveMap/loadMap` 二进制序列化 | `NativeHelper.MapManager` |
| 重定位 | DBoW2 词袋 + PnP 求解器 | `cpp/include/Tracking.h` |
| 关键帧图 | 协同可见性图（Covisibility Graph） | `cpp/include/KeyFrame.h` |
| 3D 点云 | `getAllMapPoints()` | `cpp/include/Map.h` |
| AR 物体渲染 | OBJ + MTL + OpenGL ES 2.0 | `rendering/render/ObjectRenderer.java` |
| IMU 回退（3DOF） | `OrientationSensor` + `lib3dof.so` | `sensors/OrientationSensor.java` |

### 1.2 关键数据流（现有）

```
Camera → ORBextractor → ORBmatcher → PnPsolver
                                          ↓
                                    Tcw (6-DOF Pose)
                                          ↓
                              getV() → View Matrix (Java)
                                          ↓
                              OpenGL MVP → AR 物体渲染
```

### 1.3 导航改造的先天优势

- **地图可保存/加载**：可提前建图，导航时加载地图实现室内定位
- **重定位机制**：用户进入已建图区域可自动定位
- **关键帧协同可见性图**：天然可作为导航拓扑图的基础
- **平面检测**：可将导航路径投影到地面平面
- **OBJ 渲染管线**：可复用渲染箭头、路径线等导航元素

---

## 二、AR 导航系统总体架构

```
┌─────────────────────────────────────────────────────────┐
│                   AR 导航系统架构                         │
│                                                         │
│  ┌──────────┐  ┌──────────────┐  ┌──────────────────┐  │
│  │  Phase 1 │  │   Phase 2    │  │    Phase 3       │  │
│  │  建图阶段  │  │   标注阶段    │  │    导航阶段       │  │
│  │          │  │              │  │                  │  │
│  │ 用户漫游  │  │ 设置路点/目标  │  │ 实时路径 + AR 引导 │  │
│  │ 自动建图  │  │ 保存带路点地图 │  │ 方向箭头 + HUD   │  │
│  └──────────┘  └──────────────┘  └──────────────────┘  │
│                                                         │
│  ┌───────────────────────────────────────────────────┐  │
│  │                  核心模块层                         │  │
│  │                                                   │  │
│  │  NavigationGraph  PathPlanner  NavigationEngine   │  │
│  │  (拓扑图构建)      (A* 路径规划)  (实时导航计算)      │  │
│  │                                                   │  │
│  │  WaypointManager  MapMetadataEx  LocalizerBridge  │  │
│  │  (路点管理)        (地图元数据扩展) (重定位桥接)      │  │
│  └───────────────────────────────────────────────────┘  │
│                                                         │
│  ┌───────────────────────────────────────────────────┐  │
│  │                  渲染层                             │  │
│  │                                                   │  │
│  │  ArrowRenderer   PathLineRenderer   NavHudOverlay │  │
│  │  (3D 方向箭头)    (地面路径线)        (HUD 叠加层)   │  │
│  └───────────────────────────────────────────────────┘  │
└─────────────────────────────────────────────────────────┘
```

---

## 三、详细模块设计

### 3.1 导航拓扑图（NavigationGraph）— C++ 层

**目的**：将 ORB-SLAM2 的关键帧协同可见性图转化为可路径规划的导航图。

**设计原理**：
- 每个关键帧的位姿中心 = 图中一个节点（3D 坐标）
- 两关键帧间存在协同可见性关系（共享地图点 ≥ 阈值）= 图中一条边
- 边权重 = 两关键帧光心的欧氏距离

**新增文件**：`app/src/main/cpp/navigation/NavigationGraph.h/.cc`

```cpp
namespace Navigation {

struct NavNode {
    long unsigned int kfId;     // KeyFrame ID
    cv::Mat position;           // 3D 世界坐标 (3x1)
    std::vector<long unsigned int> neighbors;  // 相邻节点 ID
    std::string label;          // 可选语义标签（如"走廊入口"）
};

class NavigationGraph {
public:
    // 从当前 SLAM 地图构建拓扑图
    void buildFromMap(ORB_SLAM2::Map* pMap);

    // 查询最近节点
    long unsigned int findNearestNode(const cv::Mat& pos3D) const;

    // A* 路径规划，返回节点 ID 序列
    std::vector<long unsigned int> findPath(
        long unsigned int startId,
        long unsigned int goalId
    );

    // 添加路点节点（不绑定关键帧）
    long unsigned int addWaypointNode(const cv::Mat& pos3D, const std::string& label);

    // 序列化/反序列化（随地图一起持久化）
    void serialize(std::ofstream& ofs) const;
    void deserialize(std::ifstream& ifs);

private:
    std::map<long unsigned int, NavNode> mNodes;
    float heuristic(long unsigned int a, long unsigned int b) const;
};

} // namespace Navigation
```

**A\* 算法要点**：
- 启发函数 `h(n)` = 节点 n 到目标的欧氏距离
- 代价函数 `g(n)` = 起点到节点 n 的实际路径长度
- 优先队列按 `f(n) = g(n) + h(n)` 排序

---

### 3.2 路点管理器（WaypointManager）— C++ + Java

**目的**：用户在建图阶段标注兴趣点（出口、电梯、房间等），保存为路点。

**C++ 层**：

```cpp
// app/src/main/cpp/navigation/WaypointManager.h

struct Waypoint {
    std::string id;             // 唯一 ID（UUID）
    std::string name;           // 用户命名（如"会议室"）
    cv::Mat worldPos;           // 3D 世界坐标
    std::string iconType;       // 图标类型："exit"/"room"/"elevator"/"custom"
    long long createTime;       // 创建时间戳
    long unsigned int anchorKfId; // 锚定关键帧 ID（用于重定位后重建位置）
};

class WaypointManager {
public:
    // 在当前摄像机前方指定距离处创建路点
    std::string addWaypoint(
        const cv::Mat& cameraPose,   // 当前 Tcw
        float distanceAhead,          // 前方距离（米）
        const std::string& name,
        const std::string& iconType
    );

    // 通过触摸射线与地面平面交点创建路点
    std::string addWaypointAtRayIntersection(
        const cv::Mat& rayOrigin,
        const cv::Mat& rayDir,
        const Plane& groundPlane,
        const std::string& name
    );

    std::vector<Waypoint> getAllWaypoints() const;
    bool deleteWaypoint(const std::string& id);

    void serialize(std::ofstream& ofs) const;
    void deserialize(std::ifstream& ifs);

private:
    std::map<std::string, Waypoint> mWaypoints;
};
```

**Java 层扩展（NativeHelper.java）**：

```java
// 新增 JNI 接口
public native String addWaypoint(float distanceAhead, String name, String iconType);
public native String[] getWaypointList();    // JSON 数组
public native boolean deleteWaypoint(String waypointId);
public native float[] getWaypointWorldPos(String waypointId);

// 新增 MapManager.MapInfo 字段扩展
// 在 JSON 元数据中追加 waypoints 数组
```

---

### 3.3 路径规划器（PathPlanner）— C++

**目的**：给定起点和目标路点，计算最优路径。

```cpp
// app/src/main/cpp/navigation/PathPlanner.h

struct NavPath {
    std::vector<cv::Mat> waypoints3D;   // 路径中的 3D 路点序列
    std::vector<std::string> instructions; // 语音/文字指令（"前进""左转""到达"）
    float totalDistance;                // 总路径长度（米）
};

class PathPlanner {
public:
    PathPlanner(NavigationGraph* graph, WaypointManager* waypointMgr);

    // 计算从当前位置到目标路点的路径
    NavPath planPath(
        const cv::Mat& currentPos3D,
        const std::string& goalWaypointId
    );

    // 重规划（当偏离路径超过阈值时触发）
    NavPath replan(
        const cv::Mat& currentPos3D,
        const std::string& goalWaypointId,
        float deviationThreshold = 1.5f  // 米
    );

private:
    NavigationGraph* mpGraph;
    WaypointManager* mpWaypointMgr;
};
```

---

### 3.4 实时导航引擎（NavigationEngine）— C++

**目的**：逐帧计算当前位姿与导航路径的关系，输出导航指令。

```cpp
// app/src/main/cpp/navigation/NavigationEngine.h

enum class TurnDirection { STRAIGHT, TURN_LEFT, TURN_RIGHT, U_TURN, ARRIVED };

struct NavInstruction {
    TurnDirection direction;        // 转向指令
    float angleToNextWaypoint;      // 到下一路点的偏转角（度，正=右，负=左）
    float distanceToNextWaypoint;   // 到下一路点的距离（米）
    float totalRemainingDistance;   // 剩余总路程
    float progressPercent;          // 完成百分比 [0,1]
    cv::Mat nextWaypointWorld;      // 下一路点世界坐标（用于渲染箭头）
    std::string instruction;        // 文字说明（"向前走 3 米后左转"）
    bool isArrived;                 // 是否到达目的地
};

class NavigationEngine {
public:
    void setPath(const NavPath& path);
    void clearPath();
    bool hasActivePath() const;

    // 每帧调用（在 TrackMonocular 返回后）
    // currentTcw: 当前相机位姿
    // returns: 当前导航指令
    NavInstruction update(const cv::Mat& currentTcw);

    // 获取用于 AR 渲染的箭头变换矩阵（OpenGL 列主序）
    void getArrowModelMatrix(float* outMatrix16, float floatInFrontDist = 1.5f) const;

    // 获取路径线顶点（地面投影，用于 OpenGL 线绘制）
    std::vector<float> getPathLineVertices(const Plane& groundPlane) const;

private:
    NavPath mCurrentPath;
    int mCurrentSegment = 0;        // 当前所在路段索引
    float mWaypointReachThreshold = 0.8f; // 到达判定阈值（米）

    TurnDirection classifyDirection(float angleDeg) const;
    void advanceToNextSegment();
};
```

---

### 3.5 AR 导航渲染层 — Java/OpenGL ES 2.0

#### 3.5.1 方向箭头渲染器（ArrowRenderer）

复用现有 `ObjectRenderer.java` 的着色器管线，加载内置箭头 OBJ 模型（程序化生成或内嵌资源）。

```java
// app/src/main/java/com/orb/slam2s/rendering/render/ArrowRenderer.java

public class ArrowRenderer {
    private ObjectRenderer mArrowModel;
    private float[] mArrowColor = {0.2f, 0.8f, 1.0f, 0.9f}; // 青蓝色半透明

    // 动画参数（上下浮动 + 旋转）
    private float mBobPhase = 0f;
    private static final float BOB_AMPLITUDE = 0.05f;  // 5cm 浮动幅度
    private static final float BOB_SPEED = 2.0f;        // 浮动频率

    public void init(Context context) { /* 加载内置箭头模型 */ }

    // 根据 NavigationEngine 输出的矩阵渲染箭头
    public void draw(float[] mvpMatrix, float[] arrowModelMatrix, float deltaTime);

    // 设置箭头颜色（到达时变绿色）
    public void setIndicatorColor(TurnDirection direction);
}
```

**箭头动画效果**：
- 上下轻微浮动（`sin` 波形，幅度 5cm，周期约 2 秒）
- 沿路径方向平滑旋转过渡（`slerp` 插值）
- 接近目标时缩放动画（脉冲效果）

#### 3.5.2 路径线渲染器（PathLineRenderer）

在地面平面上绘制从当前位置到目标的路径线。

```java
// app/src/main/java/com/orb/slam2s/rendering/render/PathLineRenderer.java

public class PathLineRenderer {
    private int mVbo;           // 顶点缓冲对象
    private int mShaderProgram;

    // 路径线样式
    private float[] mLineColor = {0.2f, 0.9f, 0.4f, 0.8f};  // 绿色
    private float mLineWidth = 0.08f;   // 8cm 线宽（渲染为扁平几何体）
    private boolean mDashAnimation = true; // 流动虚线动画
    private float mDashOffset = 0f;

    public void updatePath(float[] pathVertices3D); // 从 NavigationEngine 获取
    public void draw(float[] vpMatrix, float deltaTime);
}
```

#### 3.5.3 导航 HUD 覆盖层（NavHudOverlay）

纯 Android View 叠加层（不依赖 OpenGL），显示导航状态。

```
┌─────────────────────────────────────────────────────────┐
│  [←] 向左转    距下一路点: 12.3m    剩余: 45.6m  [85%] │  ← 顶部状态栏
│                                                         │
│                    (相机画面)                             │
│                                                         │
│                  ↑ (3D 方向箭头 AR 叠加)                  │
│                                                         │
│         ══════════════════════                          │
│         ║        (路径线)         ║                      │
│                                                         │
│ ┌──────────────────────────────┐                        │
│ │  🎯 目标: 会议室              │                        │  ← 底部信息卡
│ │  当前指令: 前进后右转          │                        │
│ │  ████████████████░░░░  [退出]│                        │
│ └──────────────────────────────┘                        │
└─────────────────────────────────────────────────────────┘
```

```java
// app/src/main/java/com/orb/slam2s/ui/NavHudOverlay.java

public class NavHudOverlay extends FrameLayout {
    // UI 组件
    private TextView tvDirection;       // 转向文字（"向左转"）
    private TextView tvDistance;        // 到下一路点距离
    private TextView tvRemaining;       // 剩余总距离
    private ProgressBar pbProgress;     // 导航进度条
    private TextView tvInstruction;     // 详细指令
    private ImageView ivDirectionArrow; // 2D 转向箭头图标（左/直/右）
    private TextView tvDestination;     // 目标名称

    // 更新 HUD 内容（在主线程调用）
    public void updateNavInstruction(NavInstruction instruction);

    // 显示/隐藏（进入/退出导航模式）
    public void setNavigating(boolean navigating, String destinationName);

    // 到达目的地动画
    public void showArrivalAnimation();
}
```

---

### 3.6 导航模式 Activity 集成（ArCamUIActivity 扩展）

在现有 `ArCamUIActivity` 中新增导航模式支持：

```java
// 新增状态
private enum AppMode { AR_OBJECT, NAV_MAPPING, NAVIGATION }
private AppMode currentMode = AppMode.AR_OBJECT;

// 新增组件
private NavHudOverlay navHudOverlay;
private ArrowRenderer arrowRenderer;   // 集成进 ObjRendererWrapper 流程
private PathLineRenderer pathRenderer;
private String navDestinationId = null;

// 导航状态回调接口（扩展 OnMVPUpdatedCallback）
interface OnNavUpdatedCallback {
    void onNavInstructionUpdated(NavInstruction instruction);
}

// 新增按钮
// [建图] [标注路点] [选择目标] [开始导航] [停止导航]
```

**模式切换流程**：

```
默认模式（AR 物体放置）
        ↓ 点击[建图]
建图模式（录制地图，禁用 AR 物体）
        ↓ 点击[标注路点]（多次）
        ↓ 点击[保存地图]
路点标注完成
        ↓ 点击[选择目标]
目标选择对话框（路点列表）
        ↓ 点击[开始导航]
导航模式（实时 AR 箭头 + HUD）
        ↓ 到达目的地 或 点击[停止]
导航结束
```

---

## 四、C++ 层新增 JNI 接口

在 `native-lib.cpp` 中新增以下 JNI 函数：

```cpp
// ===================== 导航图 =====================
// 从当前地图构建导航图（在地图加载或建图完成后调用）
JNIEXPORT void JNICALL Java_..._nativeBuildNavGraph(JNIEnv*, jobject);

// ===================== 路点管理 =====================
// 在当前摄像机前方 distance 米处添加路点，返回路点 ID 字符串
JNIEXPORT jstring JNICALL Java_..._nativeAddWaypoint(
    JNIEnv* env, jobject, jfloat distance, jstring name, jstring iconType);

// 获取所有路点（JSON 格式）
JNIEXPORT jstring JNICALL Java_..._nativeGetWaypointsJson(JNIEnv*, jobject);

// 删除路点
JNIEXPORT jboolean JNICALL Java_..._nativeDeleteWaypoint(JNIEnv* env, jobject, jstring id);

// ===================== 路径规划 =====================
// 规划到目标路点的路径，返回成功/失败
JNIEXPORT jboolean JNICALL Java_..._nativePlanPathToWaypoint(
    JNIEnv* env, jobject, jstring waypointId);

// 清除当前路径
JNIEXPORT void JNICALL Java_..._nativeClearNavPath(JNIEnv*, jobject);

// ===================== 导航状态查询 =====================
// 获取当前导航指令（float 数组格式）
// 返回: [angleDeg, distToNext, totalRemain, progress, directionEnum, isArrived,
//         arrowMat16(16 floats), pathVerts...]
JNIEXPORT jfloatArray JNICALL Java_..._nativeGetNavInstruction(JNIEnv*, jobject);

// 获取路径线顶点（用于 PathLineRenderer）
JNIEXPORT jfloatArray JNICALL Java_..._nativeGetPathLineVertices(JNIEnv*, jobject);
```

---

## 五、地图元数据扩展

在现有 `MapManager.MapInfo` JSON 中新增路点字段：

```json
{
  "name": "office_floor2",
  "keyFrames": 342,
  "mapPoints": 8921,
  "createTime": 1739800000000,
  "hasPlane": true,
  "fileSize": 2048576,
  "waypoints": [
    {
      "id": "wp_001",
      "name": "会议室入口",
      "iconType": "room",
      "worldPos": [1.23, -0.05, 4.56],
      "anchorKfId": 127,
      "createTime": 1739800100000
    },
    {
      "id": "wp_002",
      "name": "紧急出口",
      "iconType": "exit",
      "worldPos": [-3.45, -0.05, 8.12],
      "anchorKfId": 289,
      "createTime": 1739800200000
    }
  ]
}
```

---

## 六、导航流程的用户体验设计

### 6.1 建图阶段（首次使用）

```
1. 用户打开 App → 进入[建图模式]
2. 缓慢移动手机，覆盖目标区域（走廊、房间）
3. SLAM 自动构建点云地图（屏幕显示绿色点云）
4. 到达兴趣点时，点击[+ 标注路点] 按钮
   → 弹出对话框输入名称（"会议室"）和选择图标
5. 重复步骤 4，标注所有导航目标
6. 点击[保存地图]，输入地图名（"办公室二层"）
```

### 6.2 导航阶段（日常使用）

```
1. 用户打开 App → 加载已有地图
2. 系统自动重定位（DBoW2 识别场景）→ 状态栏显示"已定位"
3. 用户点击[导航] → 弹出路点列表（选择"会议室"）
4. 系统计算路径，进入 AR 导航模式：
   - 摄像机画面中出现青色 3D 箭头
   - 地面显示绿色路径线
   - 顶部 HUD 显示方向和距离
5. 用户跟随箭头行走
6. 到达目的地 → 屏幕显示"已到达会议室！"动画
```

### 6.3 重定位失败处理

```
SLAM 丢失 → 切换 3DOF 模式（现有 OrientationSensor）
         → HUD 显示"正在重定位，请缓慢转动手机"
         → 重定位成功后恢复完整导航
```

---

## 七、实施路线图

### Phase 1：基础导航图构建（2-3 周）

**目标**：能从已建地图构建拓扑图并运行 A* 路径规划。

- [ ] 实现 `NavigationGraph` 类（节点 + 边 + A* 算法）
- [ ] 实现 `WaypointManager` 类（添加/删除/序列化路点）
- [ ] 实现 `PathPlanner` 类（A* 规划 + 路段转折点平滑）
- [ ] 扩展 `NativeHelper.java` 增加路点 JNI 接口
- [ ] 扩展 `MapManager` 支持路点 JSON 持久化
- [ ] 单元测试：在已保存地图上验证路径规划正确性

**关键文件修改**：
- `app/src/main/cpp/CMakeLists.txt` 添加 navigation 模块
- `app/src/main/cpp/native-lib.cpp` 添加导航 JNI 函数
- `app/src/main/java/.../slamar/NativeHelper.java` 添加导航接口

### Phase 2：实时导航引擎（2-3 周）

**目标**：实时输出准确的导航指令（方向、距离）。

- [ ] 实现 `NavigationEngine` 类（逐帧 update）
- [ ] 实现路段进度追踪和自动前进逻辑
- [ ] 实现偏航检测和重规划触发
- [ ] 实现 `getArrowModelMatrix()` 输出（箭头放置在前方 1.5m 处）
- [ ] 实现 `getPathLineVertices()` 输出（地面投影路径线）
- [ ] 在帧处理流程中集成：`processCameraFrame()` → `NavigationEngine::update()`

**验证标准**：
- 直线行走时箭头方向误差 < 10°
- 路径切换延迟 < 100ms

### Phase 3：AR 渲染层（2-3 周）

**目标**：完整的 3D AR 导航视觉效果。

- [ ] 程序化生成内置箭头 OBJ 模型（或内嵌 assets）
- [ ] 实现 `ArrowRenderer`（动画浮动 + 颜色变化）
- [ ] 实现 `PathLineRenderer`（流动虚线动画）
- [ ] 修改 `ObjRendererWrapper` 支持导航渲染模式
- [ ] 实现 `NavHudOverlay`（Android View 叠加）
- [ ] 集成进 `ArCamUIActivity`（模式切换 + UI 按钮）

### Phase 4：用户体验与稳定性（1-2 周）

**目标**：流畅的用户体验和边界情况处理。

- [ ] 路点标注 UI（长按相机画面，射线与地面求交）
- [ ] 导航路点选择对话框（带图标和距离）
- [ ] SLAM 丢失时的降级处理（切换 3DOF + 重定位提示）
- [ ] 到达目的地动画效果
- [ ] 多语言支持（在现有 strings.xml 中添加导航相关字符串）
- [ ] 性能优化：导航计算不阻塞渲染线程

---

## 八、技术挑战与解决方案

| 挑战 | 问题描述 | 解决方案 |
|------|---------|---------|
| **单目尺度不确定性** | 单目 SLAM 无法获得绝对尺度，距离显示不准确 | 建图时标定实际距离（用户输入两路点间真实距离），或使用已知尺寸物体初始化尺度 |
| **地图漂移** | 长时间使用后地图点积累误差 | 利用现有闭环检测修正；建图时走回起点触发闭环 |
| **路径在垂直方向的噪声** | 地面路径线受 SLAM 噪声影响上下浮动 | 检测地面平面后，将所有路径顶点投影到检测到的平面 |
| **重定位时间** | DBoW2 重定位需要 0.5-2 秒，导航暂停 | 重定位期间保持最后已知路径，切换 3DOF 模式估计位移 |
| **室内光照变化** | 暗光环境 ORB 特征少，追踪失败 | 现有的暗帧检测逻辑 + 提示用户改善光照 |
| **行人遮挡** | 移动物体干扰特征匹配 | ORB-SLAM2 本身对动态点有一定鲁棒性；可加运动一致性检测 |

---

## 九、与现有代码的接口对照

| 现有代码 | 导航改造中的角色 | 改动类型 |
|---------|--------------|---------|
| `System::TrackMonocular()` | 提供每帧 Tcw（相机位姿），导航引擎输入 | **无需修改** |
| `Map::GetAllMapPoints()` | 为 NavigationGraph 提供关键帧节点 | **无需修改** |
| `KeyFrame::GetCovisibles()` | 提供节点间边关系 | **无需修改** |
| `Plane` 类 | 地面平面检测，路径线投影基准 | **无需修改** |
| `ObjRendererWrapper` | 改造为支持多渲染器（AR 物体 + 箭头 + 路径线） | **需扩展** |
| `NativeHelper.java` | 新增导航 JNI 接口 | **需扩展** |
| `ArCamUIActivity.java` | 新增导航模式 UI 和流程控制 | **需扩展** |
| `MapManager.MapInfo` | JSON 中新增 waypoints 数组 | **需扩展** |
| `native-lib.cpp` | 新增导航相关 JNI 函数实现 | **需扩展** |
| `CMakeLists.txt` | 添加 navigation 目录编译 | **需修改** |
| `GlobalConstant.java` | 添加导航相关常量 | **需扩展** |

---

## 十、验收标准

| 指标 | 目标值 |
|------|-------|
| 重定位成功率（已建图区域） | > 90% |
| 导航方向角误差 | < 15° |
| 路径规划延迟 | < 200ms（地图 < 500 关键帧） |
| AR 箭头渲染帧率 | ≥ 25 FPS（中端设备） |
| 路点标注精度 | < 0.5m（相对于真实位置） |
| 地图加载+重定位总时间 | < 5 秒 |

---

## 附录：目录结构规划

```
app/src/main/
├── cpp/
│   ├── navigation/                    ← 新增目录
│   │   ├── NavigationGraph.h/.cc      ← 拓扑图 + A*
│   │   ├── WaypointManager.h/.cc      ← 路点管理
│   │   ├── PathPlanner.h/.cc          ← 路径规划
│   │   └── NavigationEngine.h/.cc    ← 实时导航
│   ├── native-lib.cpp                 ← 扩展导航 JNI
│   └── ...（现有文件不变）
│
├── java/com/orb/slam2s/
│   ├── navigation/                    ← 新增包
│   │   ├── NavInstruction.java        ← 导航指令数据类
│   │   └── WaypointData.java          ← 路点数据类
│   ├── rendering/render/
│   │   ├── ArrowRenderer.java         ← 新增：箭头渲染器
│   │   └── PathLineRenderer.java      ← 新增：路径线渲染器
│   ├── ui/
│   │   ├── NavHudOverlay.java         ← 新增：导航 HUD
│   │   └── ArCamUIActivity.java       ← 扩展：导航模式
│   └── slamar/
│       └── NativeHelper.java          ← 扩展：导航 JNI 接口
│
└── res/
    ├── layout/
    │   └── nav_hud_overlay.xml        ← 新增：HUD 布局
    ├── drawable/
    │   ├── ic_nav_straight.xml        ← 直行图标
    │   ├── ic_nav_left.xml            ← 左转图标
    │   └── ic_nav_right.xml           ← 右转图标
    └── values/
        └── strings.xml                ← 扩展：导航相关字符串
```

---

*文档版本：v1.0 | 创建于 2026-02-18*
