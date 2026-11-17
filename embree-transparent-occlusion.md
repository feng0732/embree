# Embree 透明表面遍历：过滤回调与遮挡查询的契约分析

本文基于仓库内 Embree 4 源码（`include/embree4`、`kernels/`）、官方 API 文档（`doc/src/api/`）以及现有测试与教程材料（`tutorials/verify`、`tutorials/intersection_filter`、`tutorials/next_hit`、`tests/`），分析透明表面参与射线遍历时三条查询路径的行为差异：

1. **普通最近命中**：`rtcIntersect1` 不挂过滤回调；
2. **过滤后继续遍历**：`rtcIntersect1` 挂载 intersection filter，回调拒绝（`valid[0]=0`）透明命中；
3. **遮挡查询早停**：`rtcOccluded1` 挂载 occlusion filter，首个被接受的命中立即终止遍历。

文中所有结论均给出源码或文档出处，行号对应当前工作树。

---

## 1. 三条路径的机制对比

### 1.1 普通最近命中（rtcIntersect，无过滤）

顶层遍历在 `kernels/bvh/bvh_intersector1.cpp:32-114` 的 `BVHNIntersector1::intersect`：

- 节点遍历使用 `traverseClosestHit`（`kernels/bvh/bvh_traverser1.h:58`），命中的子节点**按 tNear 排序、近者优先**，远子节点带距离压栈；
- 弹出栈节点时做距离裁剪：`if (stackPtr->dist > ray.tfar) continue;`（`bvh_intersector1.cpp:79`），随着 `ray.tfar` 不断缩小，远端子树被跳过；
- 叶子内由图元求交器更新命中，离开叶子后把新的 `ray.tfar` 同步回遍历寄存器 `tray.tfar = ray.tfar`（`bvh_intersector1.cpp:104-105`）。

可观察状态：查询结束时 `ray.tfar` 为最近命中距离，`geomID/primID/u/v/Ng/instID` 全部填充；无命中时射线数据**完全不更新**，`geomID` 保持调用方初始化的 `RTC_INVALID_GEOMETRY_ID(-1)`（见 `doc/src/api/rtcIntersect1.md:38-44`）。

### 1.2 过滤后继续遍历（rtcIntersect + intersection filter）

过滤逻辑分两级，实现在 `kernels/geometry/filter.h:15-36` 的 `runIntersectionFilter1Helper`：先执行几何级 `geometry->intersectionFilterN`，再执行查询参数携带的 context filter；任一级把 `valid[0]` 置 0 即拒绝。两级过滤的启用条件与顺序见 `doc/src/api/rtcInitOccludedArguments.md:68-79`（context filter 是第二级，且只有第一级放行的射线在第二级仍 valid）。

单图元 epilog `Intersect1Epilog1::operator()`（`kernels/geometry/intersector_epilog.h:20-72`）的关键序列：

```cpp
const float old_t = ray.tfar;      // intersector_epilog.h:50
ray.tfar = hit.t;                  // :51  回调看到的 tfar 即候选距离
bool found = runIntersectionFilter1(geometry,ray,context,h);
if (!found) ray.tfar = old_t;      // :53  拒绝 → 回滚 tfar，遍历继续
```

SIMD 宽 epilog `Intersect1EpilogM`（`intersector_epilog.h:220-302`）在一个叶子内用 `select_min(valid, hit.vt)` 由近到远逐个处理候选（`:240`、`:250`），每拒绝一个就 `clear(valid,i)` 并用 `valid &= hit.vt <= ray.tfar` 重新裁剪（`:276`，注释明确写道 “intersection filters may modify tfar value”）。拒绝只回滚 `tfar`，不改变 BVH 栈，因此遍历自然继续到下一个更近/同距离候选；全部拒绝时 `foundhit=false`，`ray` 保持进入叶子前的状态。

API 契约（`doc/src/api/rtcSetGeometryIntersectFilterFunction.md:70-77`）：回调通过写 0 到 valid 掩码拒绝命中；接受时**允许修改 hit 并减小（但不应增大）`tfar`**，不应改动其他射线字段。

### 1.3 遮挡查询早停（rtcOccluded + occlusion filter）

顶层遍历在 `bvh_intersector1.cpp:117-197` 的 `occluded`，与 intersect 路径有三点结构性差异：

1. **入口早停**：若射线已经处于遮挡态（`ray.tfar < 0.0f`），立即返回（`bvh_intersector1.cpp:127-129`）；
2. **不排序、不带距离压栈**：栈元素只有 `NodeRef`（`:135`），节点遍历用 `traverseAnyHit`（`bvh_traverser1.h:143-171`），子节点按 BVH 子槽位顺序入栈，**没有 tNear 排序**，弹出时也没有距离裁剪；
3. **接受即终止**：叶子返回 true 时 `ray.tfar = neg_inf; break;`（`bvh_intersector1.cpp:186-189`）。

遮挡 epilog `Occluded1Epilog1`（`intersector_epilog.h:74-116`）与 intersect 版形似而语义不同：

```cpp
const float old_t = ray.tfar;    // :106
ray.tfar = hit.t;                // :107
const bool found = runOcclusionFilter1(geometry,ray,context,h);
if (!found) ray.tfar = old_t;    // :109  拒绝 → 回滚，继续遍历
return found;                    // 接受 → 回到顶层写 -inf 并 break
```

宽 epilog `Occluded1EpilogM`（`intersector_epilog.h:304-369`）在叶子内按 lane 位序（`bsf(m)`）而非距离逐个询问过滤，**第一个被接受的候选立即 `return true`**（`:356`），拒绝则回滚 `tfar` 并 `btc` 该位继续（`:357-359`）。

对外可观察状态由 `doc/src/api/rtcOccluded1.md:33-34` 规定：无命中时射线不更新；有命中时 `tfar` 被置为 `-inf`，且**不返回任何 hit 数据**（遮挡射线结构体没有 hit 部分）。内部判定一律以 `tfar < 0.0f` 为遮挡态（`kernels/common/ray.h:194-197` 的 `RayK<1>::occluded()`；packet 版在 `filter.h:139` 直接对接受 lane 选 `neg_inf`，hybrid 路径见 `bvh_intersector_hybrid.cpp:587`）。

官方文档对这一不对称性有明确表述（`doc/src/api/rtcSetGeometryOccludedFilterFunction.md:39-45`）：

> The `rtcOccluded`-type functions terminate traversal when a hit got committed. As filter functions can only set the `tfar` distance of the ray for a committed hit, the occlusion filter cannot influence the `tfar` value of subsequent traversal, as that directly ends. For that reason `rtcOccluded` and occlusion filters cannot get used to gather the next n-hits, and `rtcIntersect` and intersection filters should get used instead.

代码层面同样可以验证：occlusion filter 对 `tfar` 的任何修改，拒绝时被 `old_t` 回滚（`intersector_epilog.h:109`），接受时被顶层的 `neg_inf` 覆盖（`bvh_intersector1.cpp:187`），没有第三条出路。

### 1.4 深度判定与边界包含关系

三角形 Moeller–Trumbore 求交的深度测试为（`kernels/geometry/triangle_intersector_moeller.h:104`）：

```cpp
valid &= (absDen*vfloat<M>(ray.tnear()) < T) & (T <= absDen*vfloat<M>(ray.tfar));
```

即 **近端严格、远端包含**：`tnear < t <= tfar`。候选距离恰等于当前 `ray.tfar` 时不会被深度测试剔除，会进入 epilog。但 `doc/src/api/rtcOccluded1.md:36-40` 与 `rtcIntersect1.md:57-61` 同时声明：对恰好落在（或极其接近）`tnear/tfar` 上的图元，命中与否**不做保证**，需要确定性应微调端点。

---

## 2. 场景一：相同距离的命中

设两张表面 A、B 与射线的交点距离完全相同（共面重叠、双侧球面、边线上的邻接三角形等）。

### 2.1 普通最近命中

- 节点层：`traverseClosestHit` 仅在 `d0 < d1` 时区分先后，**距离相等时走 else 分支**（`bvh_traverser1.h:87-88`），先后取决于子节点槽位；
- 叶子层：`Intersect1EpilogM` 用 `select_min(valid, hit.vt)` 选最小 t，同 t 时返回最低 lane（`intersector_epilog.h:240`），即最终写回哪张表面由图元在 SIMD 图元块中的排布决定；
- 可观察结果：`ray` 中**恰好保留一个命中**，其 `geomID/primID` 依赖 BVH 构建后的图元顺序，API 不保证跨构建/跨 ISA 稳定。`rtcSetGeometryIntersectFilterFunction.md:49-54` 进一步指出，`RTC_BUILD_QUALITY_HIGH` 下过滤函数可能对同一图元命中被调用多次，恰好打在边线上的射线可能收到同一表面的两份命中报告，建议应用层按 `geomID/primID` 去重。

### 2.2 过滤后继续遍历

- 同距离的每个候选都会进入过滤回调：宽 epilog 处理完一个候选后用 `hit.vt <= ray.tfar` 重裁（`intersector_epilog.h:276`），等号保证同 t 候选不被丢弃；
- 若 A 被拒绝、B 被接受，`tfar` 回滚后 B 仍可在同一叶子循环内被接受并写回 hit；若两者都拒绝，则等价于该距离无命中，继续向更远遍历；
- 可观察结果：最终 hit 字段只反映**某个被接受的同距候选**（仍受 lane 顺序影响），但“至少一个接受即有命中、全部拒绝即无命中”这一布尔结论是确定的。`tutorials/verify/verify.cpp` 的 `IntersectionFilterTest`（`:2958-3033`）正是按此契约断言：被拒 primID 期望 `geomID == RTC_INVALID_GEOMETRY_ID`（`:3023`）。

### 2.3 遮挡查询早停

- `traverseAnyHit` 不排序（`bvh_traverser1.h:143-171`），叶子内 `Occluded1EpilogM` 按 lane 位序询问（`intersector_epilog.h:334-359`），**谁先被访问且回调接受，结果就由谁决定**，另一个同距候选可能根本不会被访问；
- 可观察结果只有二值：任一接受 → `tfar = -inf`；全部拒绝 → `tfar` 保持输入值。应用无法得知是 A 还是 B 遮挡，也不应依赖回调被调用的次序。verify 的同测试对遮挡变体只断言 `(primID & 2) != 0) == (ray.ray.tfar != neg_inf)`（`verify.cpp:3025`），即只检查布尔语义。

---

## 3. 场景二：透明层叠加

设射线上依次有透明层 T1、T2 与不透明体 O（由近到远）。

### 3.1 普通最近命中

无过滤时 T1 即终点，`tfar` 停在 T1，T2 与 O 不可见。普通 intersect 本身没有“穿透”语义，透明只能由回调或多趟查询实现。

### 3.2 过滤后继续遍历（intersection filter）

- 回调对 T1、T2 返回 `valid=0` 后，epilog 回滚 `tfar`（`intersector_epilog.h:53`/`:273`），BVH 栈不变，遍历继续；最终 O 被接受并写回，或全部透明时 `geomID` 保持 -1、`tfar` 保持初始 `inf`；
- 单趟 `rtcIntersect` **只能输出一个最终命中**，无法直接得到层叠序列。仓库内两种标准做法：
  1. **多趟查询**：`tutorials/intersection_filter/intersection_filter_device.cpp:67-122` 的着色循环，每趟把 `tnear` 推进到 `1.001f * tfar`（`:117`）重新发起 intersect，逐层合成；
  2. **回调内收集**：`tutorials/next_hit/next_hit_device.cpp` 的 `gather_all_hits`（`:124-143`）对每个命中都置 `valid=0` 并写入上下文携带的命中表，单趟枚举全部命中；需要结束本趟时再缩小 `tfar` 并重新置 valid（`gather_next_hits`，`:238-242`）。
- 遍历次序保证：intersect 路径近者优先，`SphereFilterMultiHitTest`（`verify.cpp:4828-4931`）在两个沿 z 轴排列的球体上断言回调按 z = -1、1、2、4 的前→后顺序到达（`:4911-4927`，无背面剔除时每个球前后两面各一次，共 4 个命中）。这是“过滤后继续遍历”相对遮挡路径的关键可观察保证。
- 透明透过率等射线私有数据通过 `RTCRayQueryContext` 尾部扩展传递（教程中的 `userRayExt`，`intersection_filter_device.cpp:69`；context 指针原样透传的契约见 `rtcInitOccludedArguments.md:52-59`）。

### 3.3 遮挡查询早停（occlusion filter）

- T1、T2 的回调拒绝 → `tfar` 回滚、遍历继续；O 的回调接受 → 立刻 `neg_inf` 终止。若所有层都拒绝，`tfar` 保持正值，应用据此判定目标可见。教程的判据为 `if (shadow.tfar >= 0.0f)`（`intersection_filter_device.cpp:110`）；
- 层叠量化：教程的 `occlusionFilter` 在回调内累乘透过率 `T *= *transparency`，只要仍有透过率就拒绝（`:204-209`）。这是一种**顺序无关**的聚合（乘法可交换），刻意规避了下一条限制；
- 次序警告：遮挡路径不做距离排序（见 1.3），回调副作用的发生顺序**不保证前→后**。任何依赖次序的合成（带吸收系数的分段积分、K 个最近透明层等）在 `rtcOccluded` 上都不安全，应改用 intersect 路径；
- 能力边界：occlusion filter 无法通过改 `tfar` 截断后续遍历（文档 `rtcSetGeometryOccludedFilterFunction.md:39-45`，代码回滚/覆盖见 1.3），因此“收集前 N 个命中”类需求只能用 `rtcIntersect` + intersection filter（next_hit 教程即官方示范）。

### 3.4 三种路径的可观察状态对照

| 维度 | 普通最近命中 | intersect + 拒绝型过滤 | occluded + 遮挡过滤 |
|---|---|---|---|
| 输出字段 | `tfar`=最近距离；hit 字段全填充 | 同左，但只反映最终接受者；全拒绝时 `geomID=-1`、射线不更新 | 仅 `tfar`：接受→`-inf`，全拒绝→保持输入值；无 hit 字段 |
| 遍历顺序 | 按 tNear 近→远 | 按 tNear 近→远，回调次序可观察 | BVH 槽位顺序，**不保证按距离** |
| 拒绝后的 `tfar` | — | 回滚为 `old_t`（`intersector_epilog.h:53`） | 回滚为 `old_t`（`:109`） |
| 接受后的行为 | 缩小 `tfar`、裁剪远栈 | 同左；回调可进一步缩小 `tfar` | 写 `-inf` 并立即 `break`（`bvh_intersector1.cpp:187`） |
| 透明层叠加 | 不适用 | 单趟仅一个最终命中；多层靠多趟或回调收集 | 可逐层拒绝继续遍历，可在回调内做顺序无关聚合，不能收集 N 命中 |
| 同距离多候选 | 报告一个，身份依赖图元/lane 顺序 | 每个候选都过回调，布尔结论确定、身份不确定 | 先访问到的接受即定结果，余者可能不可见 |
| 回调调用次数 | 0 | 可能重复（HIGH 质量/边线双命中，文档明示） | 同左；且早停后剩余候选不再调用 |

---

## 4. 场景三：实例化几何

实例求交在 `kernels/geometry/instance_intersector.cpp`（实例数组在 `instance_array_intersector.cpp`，结构相同）。

### 4.1 递归与坐标空间

`InstanceIntersector1::intersect`（`:15-38`）/ `occluded`（`:40-67`）：

1. 实例 mask 测试后，把射线原点/方向用 `world2local` 变换到对象空间（`:30-31`、`:57-58`）；
2. 构造新的 `RayQueryContext`，但**复用同一个用户 `RTCRayQueryContext`**（`:32`、`:59`），因此参数级 filter、feature mask、射线扩展载荷随递归跨层传播；几何级 filter 则挂在哪层场景的几何上就在哪层触发；
3. 递归调用内部场景的 `intersectors.intersect/occluded`，返回后恢复世界空间射线并弹出实例 ID 栈（`:34-36`、`:61-64`）。

过滤回调在对象空间被调用，`instID` 栈有效——这是公开契约（`rtcSetGeometryIntersectFilterFunction.md:66-68`：“If the hit geometry is instanced, the `instID` member of the ray is valid, and the ray and the potential hit are in object space”）。因此回调内对 `tfar`、法线、命中点的任何比较或阈值都必须意识到单位随实例变换改变（非均匀缩放尤其如此）。

### 4.2 两条路径的命中状态传播

- **intersect 路径**：内层接受的命中经 `copyHitToRay`（`kernels/common/hit.h:124-136`）带上内层 `geomID/primID` 与完整 `instID` 栈写回同一射线；内层全拒绝则 `tfar` 回滚，外层继续遍历兄弟实例。`InstancingTest`（`verify.cpp:3035-3136`）嵌套 `RTC_MAX_INSTANCE_LEVEL_COUNT` 层 TransformNode，断言过滤回调在最深层仍被计数（`:3123`）且 intersect 变体 `geomID != -1`（`:3126`）；
- **occluded 路径**：内层一旦有接受的命中，内层遍历把 `tfar` 置负，外层实例求交以 `occluded = ray.tfar < 0.0f` 读取并返回 true（`instance_intersector.cpp:63`），顶层 `occluded` 随即 `neg_inf + break`；内层全拒绝时 `tfar` 为正，外层照常继续。同一测试对遮挡变体断言 `ray.ray.tfar == neg_inf`（`verify.cpp:3128`）。入口处的“已遮挡早停”（`bvh_intersector1.cpp:128`）保证嵌套递归不会在已有结论后做多余工作；
- **递归深度保护**：`instance_id_stack::push` 失败（超出 `RTC_MAX_INSTANCE_LEVEL_COUNT`）时整个实例被跳过（`:25`、`:52` 的 if 守卫），表现为该实例对两条查询都“不存在”，回调不会被调用。

### 4.3 实例与透明/同距因素叠加

- 实例内透明表面拒绝后，恢复的是**对象空间 `tfar`**，但该值随共享的射线结构带回外层，外层栈裁剪仍在世界空间一致工作（intersect 路径）；遮挡路径因不做距离压栈，只依赖射线-AABB 测试，语义不变；
- 不同实例中的同距离命中，继承第 2 节的全部不确定性：intersect 报告哪一个取决于外层 BVH 槽位与内层 lane 顺序；occluded 由先访问到且接受回调的实例决定；
- 参数级 filter 的前置条件：场景需带 `RTC_SCENE_FLAG_FILTER_FUNCTION_IN_ARGUMENTS`，且每个实例化的内部场景也要启用（`doc/src/api/rtcInitOccludedArguments.md:64-66`）；几何默认不接受参数 filter，需 `rtcSetGeometryEnableFilterFunctionFromArguments` 显式打开，或用 `RTC_RAY_QUERY_FLAG_INVOKE_ARGUMENT_FILTER` 在查询级强制（`kernels/common/context.h:48-50`，`doc/src/api/rtcSetGeometryEnableFilterFunctionFromArguments.md:24-29`）。intersection_filter 教程同时示范了几何级注册与参数级强制两条用法（`intersection_filter_device.cpp:290-295`、`:76-78`）。

---

## 5. 现有测试与文档材料清单

**源码**

- 顶层遍历：`kernels/bvh/bvh_intersector1.cpp:32-197`；节点次序：`kernels/bvh/bvh_traverser1.h:58`（排序）与 `:143`（不排序）；packet 早停：`kernels/bvh/bvh_intersector_hybrid.cpp:586-589`
- 过滤执行：`kernels/geometry/filter.h:15-84`（单射线两级过滤）、`:122-155`（packet）
- 命中收尾：`kernels/geometry/intersector_epilog.h:20-116`（单图元 epilog）、`:220-369`（SIMD 宽 epilog 的同距循环与首个接受即返回）
- 深度判定：`kernels/geometry/triangle_intersector_moeller.h:104`；遮挡态定义：`kernels/common/ray.h:194-197`
- 实例递归：`kernels/geometry/instance_intersector.cpp:15-67`；上下文门控：`kernels/common/context.h:24-50`；用户几何手工调用过滤：`kernels/common/rtcore.cpp:1522-1534`

**文档契约**

- `doc/src/api/rtcSetGeometryIntersectFilterFunction.md`：接受/拒绝方式、可减小 `tfar`、重复回调与边线双命中警告、实例下对象空间语义
- `doc/src/api/rtcSetGeometryOccludedFilterFunction.md:39-45`：遮挡查询不能用于 N 命中收集的明示限制
- `doc/src/api/rtcOccluded1.md`、`rtcIntersect1.md`：`-inf` 输出、无命中不更新、端点不保证
- `doc/src/api/rtcInitOccludedArguments.md:61-79`：两级过滤的顺序与启用条件

**测试/教程**

- `tutorials/verify/verify.cpp`：`IntersectionFilterTest`（`:2958-3033`，同回调分别断言 intersect 的 geomID 与 occluded 的 tfar 符号）、`InstancingTest`（`:3035-3136`，嵌套实例下两路径契约）、`SphereFilterMultiHitTest`（`:4828-4931`，拒绝型回调枚举全部命中并验证前→后次序）；注册入口在 `:6697` 与 `:6915`
- `tutorials/intersection_filter/intersection_filter_device.cpp`：多趟透明合成（`:67-122`）、遮挡透过率累乘与 `tfar >= 0` 判据（`:110`、`:186-210`）
- `tutorials/next_hit/next_hit_device.cpp`：单趟/多趟 N 命中收集、`(t,instID,geomID,primID)` 全序定义与重复命中去重（`:41-55`、`:166-174`）
- `tests/primitives/*.embree_viewer_occlusion.exr`：遮挡模式的图像回归基线；`tests/` 目录没有针对过滤回调的独立单元测试，过滤行为目前只经 verify 教程与图像回归间接覆盖。

---

## 现状

Embree 的实现与文档对三条路径的核心契约是自洽的：intersect 路径以距离排序、拒绝即回滚 `tfar` 并继续，保证回调按前→后次序观察命中且可通过缩小 `tfar` 影响后续遍历；occluded 路径以 `tfar < 0` 为唯一遮挡态、首个接受即写 `-inf` 终止，过滤回调对 `tfar` 的修改在拒绝时被回滚、接受时被覆盖，因而既不能收集 N 个命中也不保证回调的距离次序。相同距离命中的身份选择依赖 BVH 槽位与 SIMD lane 顺序，API 只承诺布尔层面的确定性，并明确保留端点命中与 HIGH 质量下重复回调的不确定性；实例化通过共享用户上下文递归传播参数级过滤、以对象空间调用回调、以 `tfar` 符号跨层冒泡遮挡结论，verify 中的三个测试已覆盖单层拒绝、嵌套实例计数与全命中枚举。主要的契约空白在于：遮挡路径“回调不按距离次序”这一行为只可由 `traverseAnyHit` 源码推知，文档未直接写明；仓库缺少针对同距离多候选、透明层与实例叠加组合的直接单元测试，现有保证散落在教程断言与图像回归中。

## 建议

第一，需要有序透明合成（分段积分、按层混合、前 N 个命中）时一律使用 `rtcIntersect` + intersection filter，参照 next_hit 的多趟或回调收集模式；`rtcOccluded` 的遮挡过滤只用于布尔可见性与顺序无关的聚合（如透过率累乘），不要在其中依赖回调次序或尝试修改 `tfar`。第二，不要依赖同距离命中报告的具体 `geomID/primID`，需要确定性就在应用层按 `(t, instID, geomID, primID)` 元组裁决或去重（next_hit 已给出实现），需要严格包含/排除端点时按官方建议微调 `tnear/tfar`，并把过滤回调写成幂等、可容忍重复调用的纯判定。第三，实例化场景中回调内的距离与法线比较先做坐标空间换算，参数级过滤记得在每一层场景启用 `RTC_SCENE_FLAG_FILTER_FUNCTION_IN_ARGUMENTS` 并在几何上 opt-in 或查询时带 `RTC_RAY_QUERY_FLAG_INVOKE_ARGUMENT_FILTER`。第四，建议在 `tutorials/verify` 中补充三类直接断言：同距离双表面在两路径下的布尔确定性与身份不稳定性、嵌套实例内透明层拒绝后继续命中兄弟实例、以及遮挡回调调用次序与距离无关的文档化测试，并把“遮挡过滤回调无序”这一限制补写进 `rtcSetGeometryOccludedFilterFunction.md`，使目前只能由源码推知的契约成为显式保证。
