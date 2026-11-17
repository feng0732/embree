# Embree 自定义图元（RTC_GEOMETRY_TYPE_USER）行为分析

> 代码基线：本仓库 `7446fed vendor RenderKit/embree 3b7e4ee`（`RTC_VERSION 4.5.0`，见 `include/embree4/rtcore_config.h:10-14`）。
> 所有 `文件:行号` 引用均指本仓库当前工作树。标注 **[实测]** 的数字来自附录 A 的可复现实验（用本仓库头文件编译），其余均为源码推理。

---

## 0. 结论速览

| 问题 | 结论 |
|---|---|
| 包围范围、求交回调、遮挡回调、命中属性如何共同决定结果 | 四者串行、职责不重叠：**包围范围**只决定"回调会不会被调用"（BVH 节点层裁剪，`kernels/bvh/node_intersector1.h:449-479`）；**求交回调**是唯一写 `tfar` 与 `RTCHit` 的地方；**遮挡回调**只通过 `ray.tfar < 0` 表达"被遮挡"（`kernels/geometry/object_intersector.h:38-49`）；**命中属性**被原样接受，仅做有限性检查（`kernels/common/ray.h:248-263`）。四者之间**没有交叉校验**：包围盒从不用来裁剪回调报告的命中，回调也从不知道包围盒。 |
| 包围范围过紧 | **静默丢失命中**。节点测试失败 → 叶内回调根本不执行；无 `RTCError`；`rtcGetSceneBounds` 还会把被裁小后的盒当作真相返回。动画/实例旋转下表现为"随变换移动的硬边缺口"，主射线与阴影射线同时消失。 |
| 包围范围过松 | 命中结果**仍然正确**，但代价真实：世界 AABB 按旋转显著膨胀（[实测] 45° 旋转使 4×4×4 → 5.657×4×5.657），节点测试与实例转发回调都被外推；`rtcGetSceneBounds` 报告假尺寸；回调里对 `tnear/tfar` 的裁剪会被 `rtcForwardIntersect*` 静默丢弃。 |
| 回调返回多个命中时几何一致性如何体现 | 体现为三条**由回调自己维护**的不变量：(1) 每个被接受的命中必须使 `tfar` 严格收缩且在 `[tnear, tfar)` 内（教程对二次根用更新后的 `tfar` 判据，`tutorials/user_geometry/user_geometry_device.cpp:339`）；(2) 滤波器拒绝候选后必须**把 `tfar` 写回**且**自己分发参数级滤波器**——内置图元这两件事都由内核完成（`kernels/geometry/intersector_epilog.h:50-54`、`kernels/geometry/filter.h:15-36`），用户几何则全由用户代码负责（教程在 `:323-336` 手工保存/恢复、在 `:325-329` 手工二次分发）；(3) `instID` 从上下文深拷贝进命中（`:298`）。 |
| 两个反例 | 反例一（§4.1）：过紧包围盒在实例变换后静默丢命中，且**运动模糊的时间维**同样会过紧，因为用户几何缺少内置实例的非线性线性包围盒校正。反例二（§4.2）：多命中回调在滤波器拒绝候选后未恢复 `tfar`，导致"被拒的最近候选之后的所有真实几何消失"，且结果依赖遍历顺序与实例深度。两者都**没有任何现存测试覆盖**。 |

---

## 1. 四个要素如何共同决定一次射线查询

### 1.1 数据通路

```
rtcNewGeometry(RTC_GEOMETRY_TYPE_USER)
   └─ UserGeometry (kernels/common/scene_user_geometry.h:11-28)
        ├─ rtcSetGeometryUserData        → Geometry::userPtr           ┐ 两者都进入
        ├─ rtcSetGeometryBoundsFunction  → AccelSet::boundsFunc        ┘ geometryUserPtr
        └─ rtcSetGeometryIntersect/OccludedFunction → intersectorN

rtcCommitScene
   └─ createPrimRefArray → buildBounds(i) → boundsFunc(&args)          ← 包围回调（建树期）
        └─ isvalid_non_empty ? PrimRef : 丢弃                          （kernels/common/accelset.h:104-118）
   └─ 生成 BVH 叶，叶内存 Object{geomID, primID}，AABB 只存在于节点

ray query
   └─ 节点 AABB slab 测试（node_intersector1.h:449-479）
        └─ 命中节点 → 叶遍历 → ObjectIntersector1
             └─ AccelSet::intersect/occluded → 用户回调
                  └─ 回调直接读写调用者的 RTCRayHit / RTCRay
```

关键结构事实：`Object` 图元**只保存 `(geomID, primID)`**，不保存包围盒（`kernels/geometry/object.h:32-44, 91-94`）；AABB 只活在 BVH 节点里。这决定了整个架构的两个方向性结论：

- 包围盒**不能**用来裁剪回调报告的命中（回调拿不到它），所以"过紧"只会变成"没命中"，不会变成"命中位置偏了"；
- 回调**看不到**自己被哪个盒包住，所以它无法自我诊断"我的盒写错了"。

### 1.2 包围范围：唯一的"是否被询问"过滤器

`AccelSet::bounds()` 是唯一调用点（`kernels/common/accelset.h:74-85`），它把 `RTCBoundsFunctionArguments` 填好后直接调用 `boundsFunc`。`RTCBounds` 的 6 个字段被当作 `BBox3fa`（`include/embree4/rtcore_common.h:165-169`，结构与 `common/math/bbox.h` 的 `BBox<Vec3fa>` 逐字段兼容）。

建树时唯一的合法性判据是：

```cpp
// kernels/common/accelset.h:104-118
__forceinline bool buildBounds(size_t i, BBox3fa* bbox = nullptr) const {
  const BBox3fa b = bounds(i);
  if (bbox) *bbox = b;
  return isvalid_non_empty(b);            // ← 仅"有限 且 lower<=upper"
}
...
bbox = bounds.bounds0; // use bounding box of first timestep to build BVH
```

```cpp
// common/math/bbox.h:94-102
__forceinline bool isvalid( const BBox<Vec3fa>& v ) {
  return all(gt_mask(v.lower,Vec3fa_t(-FLT_LARGE)) & lt_mask(v.upper,Vec3fa_t(+FLT_LARGE)));
}
__forceinline bool isvalid_non_empty( const BBox<Vec3fa>& v ) {
  return all(gt_mask(v.lower,Vec3fa_t(-FLT_LARGE)) & lt_mask(v.upper,Vec3fa_t(+FLT_LARGE)) & le_mask(v.lower,v.upper));
}
```

即：**回调返回什么，Embree 就信什么**。唯一的"惩罚"是盒为空/含 NaN/含 ±inf 时该图元被静默丢弃（文档也如此表述，`doc/src/api/RTC_GEOMETRY_TYPE_USER.md:48-50`）。

### 1.3 求交回调：唯一写 `tfar` 与 `RTCHit` 的地方

调用链是 `ObjectIntersector1::intersect`（`kernels/geometry/object_intersector.h:25-36`）→ `AccelSet::intersect`（`kernels/common/accelset.h:145-172`）→ 用户的 `RTCIntersectFunctionN`。`args.rayhit` 直接指向遍历器持有的 `RayHit`（`accelset.h:154`），所以**回调对 `ray->tfar` 的任何写入都会立刻生效并对后续遍历可见**。

一个容易被忽略的事实：在这条 CPU 路径上，**回调的 `valid` 掩码不参与任何判定**。`AccelSet::intersect` 把 `mask` 的地址放进 `args.valid`（`:149-151`），调用完返回 `mask != 0`（`:171`），但唯一的调用者 `ObjectIntersector1::intersect` 丢弃了返回值（`object_intersector.h:35`）；包路径 `accelset.h:279-305` 更是 `void`。因此：

- 只写 `valid[0] = -1` 而不写 `hit` 字段 → **没有命中**（调用者只读 rayhit）；
- 只写 `hit` 字段而不动 `valid` → **有命中**。

`valid` 只在 SYCL 转发路径上被消费（`kernels/sycl/rthwif_embree.cpp:143-145` 使用返回的 `ishit`）。教程 `assert(args->N == 1)` + `valid[0] = -1` 的写法（`tutorials/user_geometry/user_geometry_device.cpp:104-105, 331-334`）是正确的工程约定，但在 CPU 上它不是契约，而是"恰好无害"。

### 1.4 遮挡回调：接口只有一个通道 `tfar < 0`

```cpp
// kernels/geometry/object_intersector.h:38-49
static __forceinline bool occluded(..., Ray& ray, ...) {
  AccelSet* accel = (AccelSet*) context->scene->get(prim.geomID());
  ...
  accel->occluded(ray,prim.geomID(),prim.primID(),context);
  return ray.tfar < 0.0f;          // ← 只看 tfar 的符号
}
```

叶遍历拿到 `true` 后立即把 `ray.tfar = neg_inf` 并跳出整条遍历（`kernels/bvh/bvh_intersector1.cpp:181-189`）；下一次 `rtcOccluded1` 也会因 `tfar < 0` 直接早退（`bvh_intersector1.cpp:127-129`）。

推论：遮挡回调若把 `tfar` 设成"命中距离"（一个正值）而非 `-inf`，则 **(a) 遮挡不被承认，(b) 阴影射线的有效区间被意外缩短**，导致更远处真正遮挡物被静默漏掉。文档明确要求写 `-inf`（`doc/src/api/rtcSetGeometryOccludedFunction.md:57-63`），教程也照做（`user_geometry_device.cpp:437-439, 474-476`）。

### 1.5 命中属性：谁写、谁校验

| 字段 | 由谁产生 | 实例下的语义 | 校验 |
|---|---|---|---|
| `tfar` | 用户回调（求交）/ 用户回调写 `-inf`（遮挡） | **不受仿射变换影响**：`rtcForwardIntersect1Ex` 只替换 `org/dir`（`kernels/common/rtcore.cpp:651-654`），而 `xfmPoint(w2l, org+t·dir) = w2l(org)+t·(w2l.l·dir)`，参数 `t` 不变 | `verifyHit` 只查有限性（`kernels/common/ray.h:248-263`），且只在 DEBUG 的 `rtcIntersect1` 上跑（`rtcore.cpp:626-628`），`rtcOccluded1` 不校验 |
| `u`, `v` | 用户回调 | 对象空间 | 同上，仅有限性 |
| `Ng` | 用户回调 | **对象空间**、未归一化（`doc/src/api/rtcIntersect1.md:39-42`）；实例外需逆置变换，教程用 `normal2world = transposed(rcp(local2world.l))`（`tutorials/user_geometry/user_geometry_device.cpp:248, 775-777`） | 仅有限性；把世界空间法线写进去会与教程的变换**双重作用** |
| `instID` | 用户回调**深拷贝**自 `args->context->instID`（`doc/src/api/rtcSetGeometryIntersectFunction.md:69-71`；教程 `:61-68, 298`） | 实例栈 | 无 |
| `geomID`, `primID` | 用户回调（`args->geomID/primID`） | 内层场景的 ID | 无 |

`HitK` 的标准构造会用 `instance_id_stack::copy_UU(context, ...)` 按 `instStackSize` 清零多余层（`kernels/common/hit.h:63-71`，`kernels/common/instance_stack.h:155-174`），而教程的 `copyInstanceIdStack`（`user_geometry_device.cpp:61-68`）只拷贝 `l < instStackSize` 的层、不清零其余层——在 `RTC_MAX_INSTANCE_LEVEL_COUNT > 1` 且命中缓冲被复用时，深层残值可能残留。默认 `RTC_MAX_INSTANCE_LEVEL_COUNT == 1` 时无影响。

### 1.6 一次查询的完整判据

```
结果命中  ⇔  存在某个图元 p：
             ① p 在 BVH 中的 AABB 与射线 slab 相交        （仅由包围回调决定）
             ② 回调被调用且写了 hit.geomID != -1 与合法字段
             ③ 回调把 tfar 收缩到该命中的 t，且之后没有更近的候选
             ④ （若有滤波器）滤波器没有把它置为拒绝
结果遮挡  ⇔  存在某个图元 p：① 成立 且 回调把 tfar 写成 < 0
```

四者串联、每个环节都只做"是否继续"的门控，没有任何一致性交叉验证。下面两节分别展开"① 写错"和"②③ 写错"的后果。

---

## 2. 包围范围过松 / 过紧：实例变换后的可观察行为

### 2.1 契约先明确：包围回调返回的是**世界空间** AABB

用户几何的包围回调没有"本地空间"概念——`boundsFunc` 的返回值被直接当作世界 AABB 使用（`accelset.h:74-85`，无任何变换）。所以"实例变换"完全由**用户自己在回调里完成**，教程正是这么做的：把 `local2world` 作用到 8 个角点再取 min/max（`tutorials/user_geometry/user_geometry_device.cpp:74-96`），与内置 `xfmBounds`（`common/math/affinespace.h:106-118`）语义一致。

由此产生三个"静默"性质，是后面所有异常的根因：

1. **无诊断**：写错盒不产生 `RTCError`（`isvalid_non_empty` 只挡 NaN/±inf/反向盒，`common/math/bbox.h:94-102`）。
2. **NaN 静默丢图元**：`Vec3fa` 的 `empty()` 是 `!all(le_mask(lower,upper))`（`common/math/bbox.h:85-92`），NaN 比较恒 false → 盒被判空 → 该图元从 BVH 消失 [实测：`common/math/bbox.h` `lower.x = NaN` 时 `empty()==1`]。
3. **提交早退会冻结旧盒**：`rtcCommitScene` 在无几何修改计数递增时直接返回（`kernels/common/scene.cpp:828-831` + `kernels/common/scene_verify.cpp:11-25`）。只改 `local2world` 而不 `rtcCommitGeometry`，包围盒就停在上一帧——这正是"手工实例化 + 动画"的默认失败模式（教程靠 `updateInstance` 里的 `rtcCommitGeometry` 规避，`user_geometry_device.cpp:241-250`）。

### 2.2 过紧：静默丢失命中（**反例一**，详见 §4.1）

可观察行为清单：

| 观察点 | 表现 | 原因 |
|---|---|---|
| 图像 | 物体沿世界轴被切掉一块；缺口平面**随实例旋转/平移移动** | 被裁的是变换后的世界 AABB，节点 slab 测试在叶之前（`node_intersector1.h:449-479`），回调根本不执行 |
| 阴影 | 同一区域也丢失遮挡：该暗的地方变亮，且缺口与主射线缺口重合 | `occluded` 走同一套节点裁剪（`bvh_intersector1.cpp:164-189`） |
| API | `rtcGetSceneBounds` 返回"变小后"的盒，不报错 | 该 API 直接回读 accel bounds（`kernels/common/rtcore.cpp:398-416`；`kernels/common/acceln.cpp:174-192`） |
| DEBUG | 无任何异常 | `verifyHit` 只查 `t/u/v/Ng` 有限性（`ray.h:248-263`），从不比对包围盒 |
| 运动模糊 | 即使每个时间步的盒都正确，**步与步之间**仍可能过紧 | 用户几何用通用 `LBBox` 拟合，只做"采样各时间步再外扩"（`accelset.h:121-123` → `common/math/lbbox.h:100-146`），没有内置实例的解析校正（见下） |

最后一行是与实例变换结合时最不易察觉的一种"过紧"：内置 `Instance` 对"线性插值变换 × 线性插值位置"的非线性误差做了显式修正——`boundSegmentLinear` 负责解析推导与 8 角点求极值（`kernels/common/scene_instance.cpp:181-251`），四元数版本用求根（`:253-317`），入口 `nonlinearBounds`（`:336-410`）→ `linearBounds`（`kernels/common/scene_instance.h:83-88`）。用户几何没有这一层：`AccelSet::linearBounds` 只是把回调在整数时间步上的盒喂给 `LBBox`（`accelset.h:120-130`）。所以**同一段旋转动画，用内置 `RTC_GEOMETRY_TYPE_INSTANCE` 是保守的，用手工实例化 + 用户几何则可能在两个时间步之间过度收紧**。文档只要求"每个时间步给一个盒"（`doc/src/api/rtcSetGeometryTimeStepCount.md:36-39`），并未提示这一点。

### 2.3 过松：结果仍正确，但代价真实且会掩盖上一种故障

过松不改变命中（回调仍会被调用、命中仍被接受），但它有可测量的副作用：

1. **世界 AABB 按旋转膨胀** [实测]。局部 4×4×4 的盒（教程 `createInstance(..., Vec3fa(-2,-2,-2), Vec3fa(+2,+2,+2))`，`user_geometry_device.cpp:704-707`）绕 y 轴旋转 45° 后，`xfmBounds` 给出 5.657×4×5.657，体积 64→128（2×）；用 3×3×3 的紧盒则是 4.243×3×4.243，体积 54。也就是说同一物体的世界盒体积相差 **2.37×**——直接放大节点测试与叶访问的假阳性。
2. **实例转发回调被外推**。手工实例化时，回调会为"变换后仍不可能命中"的区域被调用（教程 `instanceIntersectFunc` 无条件把射线变换并转发，`user_geometry_device.cpp:98-150`）；叶内不做逐图元剔除，只有叶 AABB（这是文档对 point query 的同款说明，`README.md:6199-6210` 的"conservative"论述对 ray 查询同样成立）。
3. **回调里对射线段做的裁剪会被静默丢弃**。教程为转发构造了完整 `RTCRay`，包括 `tnear/tfar/time/mask/id/flags`（`user_geometry_device.cpp:135-147`），但 `rtcForwardIntersect1` 的文档写得很清楚：**"Only the ray origin and ray direction members ... are used for forwarding, all additional ray properties are inherited from the initial ray traversal"**（`doc/src/api/rtcForwardIntersect1.md:36-38`），实现也确实只搬 `org/dir`（`rtcore.cpp:651-654`）。所以：**过松的盒把回调暴露给了它原本不该看到的射线区间，而它试图用 `tnear/tfar` 自救的这一手是无效的**。这是"过松"引出正确性问题的唯一常见通道，也是最容易被误诊为"过松导致越界命中"的情形。
4. **`rtcGetSceneBounds` 报告假尺寸**（同 §2.2 的机制），相机拟合、场景级粗剔除、以及基于场景包围盒的加速结构选择都会跟着偏。
5. **与过紧互为掩护**：过松让"某个实例完全消失"看起来像"某个实例被切掉一角"，两者在同一帧里可以同时存在而相互掩盖。

### 2.4 实例化时的三种典型成因

| 成因 | 代码特征 | 后果 |
|---|---|---|
| 本地盒当世界盒 | 回调直接返回 `instance->lower/upper`，漏掉 `local2world` 的 8 角点变换 | 一旦 `local2world` 含旋转/平移，盒与物体错位；错位量随变换增长 → 过紧 |
| 只算一次不随动画更新 | 只在初始化时算盒；帧循环里只改 `local2world`，未 `rtcCommitGeometry` | 提交早退（`scene.cpp:828-831`）→ 盒冻结 → 旋转后必然过紧 |
| 双重变换 | 先把角点变到世界，回调外又乘一次 `local2world`（或把 `rtcSetGeometryTransform` 与手工变换混用） | 盒整体漂移；漂移足够大后物体落在自己的盒外 → 表现为过紧 |

对照：教程采用"每次 `device_render` 重算 `local2world` → `updateInstance` 里 `rtcCommitGeometry` → `rtcCommitScene`"（`user_geometry_device.cpp:902-931`），因此**避免了**这三种错误；但它的盒固定为 ±2，而三个被实例化场景的真实内容范围是 ±1.5（球心 ±1、半径 0.5，`:681-684, 689-692, 697-700`），即它本身是一个**故意过松**的样例。

---

## 3. 回调返回多个命中时的几何一致性

"一个图元与射线有多个交点"在 Embree 里**没有内置语义**：`Object` 的 `max_size()==1`、`blocks(N)==N`（`kernels/geometry/object.h:26-30`），每次叶访问只把一个 `(geomID, primID)` 交给一个回调调用。多命中完全由回调在**一次调用内**依次报告（典型：解析球的 `t0/t1`），或由滤波器把命中"收集起来"（`tutorials/next_hit`）。一致性因此完全落在回调的写法上，具体体现为五条。

### 3.1 不变量一：`t` 单调收缩

每个候选必须满足 `tnear < t < tfar`（用**当前**的 `tfar`），接受后立刻把 `tfar` 收缩到 `t`。教程的球求交把这一点写成了显式判据：

```cpp
// tutorials/user_geometry/user_geometry_device.cpp:302, 339
if ((ray->tnear() < t0) & (t0 < ray->tfar)) { ... ray->tfar = t0; ... }
if ((ray->tnear() < t1) & (t1 < ray->tfar)) { ... ray->tfar = t1; ... }
```

因为 `t0 < t1`，且第一段在内层会把 `ray->tfar` 设为 `t0`（`:324`），第二段的判据在"第一根被接受"时必然为假——**同一调用内不可能报告一个比已接受命中更远的命中**。这是多命中一致性的第一层，也是最容易被"用入参 `tfar` 的旧副本判断"写坏的一层。

### 3.2 不变量二：滤波器拒绝后必须回滚 `tfar`（责任方不同）

内置图元的保护在 epilog 里：

```cpp
// kernels/geometry/intersector_epilog.h:46-54
if (filter) {
  if (unlikely(context->hasContextFilter() || geometry->hasIntersectionFilter())) {
    HitK<1> h(context->user,geomID,primID,hit.u,hit.v,hit.Ng);
    const float old_t = ray.tfar;
    ray.tfar = hit.t;
    bool found = runIntersectionFilter1(geometry,ray,context,h);
    if (!found) ray.tfar = old_t;        // ← 拒绝则回滚
    return found;
  }
}
```

用户几何**没有**这层保护：`AccelSet::intersect` 把 `RayHit&` 原样交出去后就直接返回（`kernels/common/accelset.h:145-172`），既不保存也不恢复 `tfar`（对比 `accelset.h:88-102` 的 `linearBounds` 有 `box[2]` 暂存，可见这里并非"无处可存"）。因此用户必须自己写：

```cpp
// tutorials/user_geometry/user_geometry_device.cpp:323-336
const float old_t = ray->tfar;
ray->tfar = t0;
rtcInvokeIntersectFilterFromGeometry(args,&fargs);
if (imask == -1) { *hit = potentialHit; valid[0] = -1; }
else              ray->tfar = old_t;      // ← 用户负责
```

文档只说"滤波器调用后，只有仍有效的射线才应更新命中"（`doc/src/api/rtcInvokeIntersectFilterFromGeometry.md:24-27`），**没有**说明"必须回滚 `tfar`"，也没有说明内置图元会自动回滚而用户几何不会。这个"半自动"的对称性是 §3.4 反例二的根。

### 3.3 不变量三：`valid` / `hit` 的两阶段协议

滤波器阶段有两道门（`kernels/geometry/filter.h:15-36`）：几何级滤波器先跑，`valid[0]==0` 直接返回 false；随后（在 `RTC_RAY_QUERY_FLAG_INVOKE_ARGUMENT_FILTER` 或几何开启了 argument filter 时）跑参数级滤波器，再次检查 `valid[0]`；两者都放行才 `copyHitToRay`（`:34`，`kernels/common/hit.h:124-137`）。注意 `copyHitToRay` 会把滤波器**改写过的** `u/v/Ng/primID/geomID/instID` 全部写回射线，且**不校验 `t`**（只写 `Ng/u/v/primID/geomID/instID`，t 由 epilog 的 `ray.tfar = hit.t` 负责）。所以滤波器"允许修改命中并减小 `tfar`"（`doc/src/api/rtcSetGeometryIntersectFilterFunction.md:73-78`）在用户几何里意味着：**滤波器改 `args->hit` 与改 `args->ray->tfar` 的效果不同**——前者被 `copyHitToRay` 吸收，后者直接生效但不会被回滚。

`next_hit` 演示了这一协议的正确用法：滤波器永远拒绝（`args->valid[0] = 0`，`tutorials/next_hit/next_hit_device.cpp:132, 206`），只在"收集够 N 个命中"或"遇到不透明面"时把 `ray->tfar` 缩到第 N 个命中并接受（`:237-242`）。这是把"多命中"从"回调内部协议"抬升成"跨多次查询的收集协议"的范式。

### 3.3.1 现状缺口：用户几何上的参数级滤波器不会自动生效

`rtcInvokeIntersectFilterFromGeometry` 的实现**只调用几何级滤波器**：

```cpp
// kernels/common/rtcore.cpp:1522-1527
RTC_API void rtcInvokeIntersectFilterFromGeometry(const struct RTCIntersectFunctionNArguments* const args_i,
                                                  const struct RTCFilterFunctionNArguments* filter_args)
{
  IntersectFunctionNArguments* args = (IntersectFunctionNArguments*) args_i;
  if (args->geometry->intersectionFilterN)
      args->geometry->intersectionFilterN(filter_args);      // ← 没有 params->filter 阶段
}
```

SYCL 版本相同（`kernels/sycl/rtcore_sycl.cpp:135-142`）。而内置图元走的是 `runIntersectionFilter1Helper`，它有两段：几何级滤波器，然后（在 `RTC_RAY_QUERY_FLAG_INVOKE_ARGUMENT_FILTER` 或几何开启了 argument filter 时）参数级滤波器（`kernels/geometry/filter.h:15-36`）。文档也把参数级滤波器描述成"在几何级滤波器之后自动调用的第二阶段"（`doc/src/api/rtcInitIntersectArguments.md:68-79`）。

因此，对 `RTC_GEOMETRY_TYPE_USER`：

- 只设 `iargs.filter`（参数级）而依赖 Embree 自动调用 → **完全不生效**，命中不被过滤。原因是用户几何的上层路径 `AccelSet::intersect` 从不进入 `runIntersectionFilter1Helper`（`kernels/common/accelset.h:145-172`），而唯一可用的入口 `rtcInvoke...FromGeometry` 又只跑几何级那一段；
- 用 `rtcSetGeometryIntersectFilterFunction`（几何级）+ 在回调内调用 `rtcInvokeIntersectFilterFromGeometry` → 几何级那段生效，参数级那段仍不生效。

这是与 §3.2 同源的"责任反转"：内置图元"两段都自动"，用户几何"两段都不自动，只提供一个只管几何级的入口"。教程的应对是**自己再分发一次**：在回调内直接调用上下文滤波器 `contextFilterFunction(&fargs)`（`tutorials/user_geometry/user_geometry_device.cpp:325-329, 362-366, 431-435, 468-472`），并在入口处用 `potential_hit->instID[0] == 0` 决定是否继续（`:523-531`）。也就是说，**多命中一致性里的"滤波器阶段"在用户几何上是由用户代码手工拼出来的**，任何遗漏都会退化为 §3.2 的反例。

### 3.4 不变量四：`instID` 深拷贝与栈

命中里的 `instID[]` 必须反映"当前正在被实例化的是谁"。内置实例由 `instance_id_stack::push/pop` 维护（`kernels/geometry/instance_intersector.cpp:24-37`，`kernels/common/instance_stack.h:22-58`），`HitK` 构造时统一拷贝（`kernels/common/hit.h:63-71`）。用户几何要么自己 push/pop 上下文（教程的 `#if 0` 分支，`:115-132`），要么走 `rtcForwardIntersect*` 让 Embree 替它 push（`rtcore.cpp:660-662`，教程真正启用的分支 `:147, 201`）。**两条路径的栈语义必须一致**，否则同一场景里"内置实例命中"和"用户几何实例命中"会给出不同的 `instID`，下游按 `instID` 取变换（教程 `:775-777`，内置实例教程 `tutorials/instanced_geometry/instanced_geometry_device.cpp:188-193`）就会取错。

### 3.5 不变量五：跨实例时 `t` 不变、`Ng` 需逆转置

这一点决定了"多命中在实例内外是否仍可比"：

- `t` 对任意仿射变换不变（§1.5 的推导；实现上 `rtcore.cpp:651-654` 只换 `org/dir`），所以**实例内报告的两个根与实例外报告的两个根可以直接按 `t` 排序**；
- `Ng` 是对象空间、未归一化（`doc/src/api/rtcIntersect1.md:39-42`；滤波器文档也强调"ray 与 potential hit 都在对象空间"，`doc/src/api/rtcSetGeometryIntersectFilterFunction.md:66-68`）。非一致缩放下必须用逆转置（教程 `:248`），且**同一实例的多个命中共享同一个 `Ng` 变换**——如果回调对每个根各自变换一次（把 `Ng` 在回调内就转成世界），上层会二次变换。

### 3.6 多命中消费者的去重语义（现状中的一个真实弱点）

`next_hit` 用 `(t, instID, geomID, primID)` 的字典序定义命中的全序（`tutorials/next_hit/next_hit_device.cpp:40-53`），并按 `(t,instID,geomID,primID)` 全等去重（`:54-58`）。这个键**无法表达"同一图元的不同交点"**：解析球的两个根 `geomID/primID` 完全相同、只有 `t` 和 `Ng` 不同。于是：

- 两根本身 `t` 不同 → 去重不会误合并（好）；
- 但"去重"的语义与文档建议的"按 `(geomID, primID)` 对"（`doc/src/api/rtcSetGeometryIntersectFilterFunction.md:49-54`）**冲突**：若按文档只记 `(geomID, primID)`，球的第二根会被当作重复命中丢掉。文档这段话是针对细分/网格边重复命中写的，但读者很容易照搬到用户几何，而用户几何恰恰是"同一 ID 多根"最典型的场景。

---

## 4. 两个反例

### 4.1 反例一：过紧的世界包围盒 + 实例变换 → 静默丢命中（含时间维）

**场景**（最小复现，基于教程的球）：

- 图元：单位球，`primID=0`，回调按 `sphereBoundsFunc` 报告 `p ± r`（`user_geometry_device.cpp:256-267`）。
- 实例变换：绕 y 轴旋转 + 平移；包围回调**忘记**对 8 角点做 `local2world`，返回本地盒；或按 §2.4 的"只算一次"冻结了旧盒。
- 射线：`(0.8, 0, -5) + t·(0,0,1)`。真实几何交点 `t = 4.4` 与 `5.6`。

**期望 vs 实际**

| 盒的半边长 | 节点 slab 测试 | 回调是否被调用 | 结果 |
|---|---|---|---|
| `1.0r`（正确） | 命中，`tNear = 4` | 是 | 命中 `t=4.4` |
| `0.75r` | **失败**（`tNear = 4.25 > 4.5 = tFar`） | **否** | 丢失，且无错误 |
| `0.5r` | 失败（`tNear = 4.5 > 4 = tFar`） | 否 | 丢失 |
| `1.5r`（过松） | 命中，`tNear = 3.5` | 是 | 命中 `t=4.4`，仅多一次访问 |

（[实测]，模型逐行对应 `kernels/bvh/node_intersector1.h:449-479` 的 `tNear=max(...)` / `tFar=min(...)` / `mask = tNear<=tFar`，见附录 A.2。）

**机制（源码）**

1. 节点裁剪发生在叶之前：`kernels/bvh/bvh_intersector1.cpp:74-105`（pop → 节点相交 → 选子 → 叶 → `PrimitiveIntersector1::intersect`）。节点不相交 ⇒ 叶永远不被访问 ⇒ `boundsFunc`/`intersectFunc` 都不会跑。
2. 建树期没有任何"盒 vs 几何"的交叉检查，唯一判据是 `isvalid_non_empty`（`kernels/common/accelset.h:104-118`，`common/math/bbox.h:94-102`）——`0.75r` 的盒完全合法。
3. 失败不可见：`rtcIntersect1` 的 DEBUG 校验只查 `t/u/v/Ng` 有限性（`kernels/common/rtcore.cpp:626-628` → `kernels/common/ray.h:248-263`）；`rtcGetSceneBounds` 反而把被裁小的盒当作真相返回（`rtcore.cpp:398-416` ← `kernels/common/acceln.cpp:174-192`）。

**时间维版本（实例旋转 + 运动模糊）**

内置实例对 `lerp(xfm) × lerp(p)` 的非线性误差有解析修正（`kernels/common/scene_instance.cpp:181-251, 319-334, 336-410`；`scene_instance.h:83-88`）。用户几何走的是通用 `LBBox` 拟合（`accelset.h:121-123` → `common/math/lbbox.h:100-146`），它只做"把每个整数时间步的盒采样进来再外扩 `b0/b1`"，**不会**为旋转引入任何补偿。因此即使每个时间步的盒都严格正确，两个时间步之间的真实包络仍可能落在 `lerp(bounds0,bounds1)` 之外 → 就是 §4.1 的失配，只是发生在 `t` 维。文档只要求"每个时间步给一个盒"（`doc/src/api/rtcSetGeometryTimeStepCount.md:36-39`），没有要求"对变换做非线性补偿"。

**为什么现有测试抓不到**

- `tutorials/verify/verify.cpp:366-380` 的 `addUserGeometryEmpty` 是几乎所有随机场景里的用户几何来源，它的 `IntersectFuncN` / `OccludedFuncN` 是**空函数**（`:152-156`），包围盒则严格等于 `Sphere::bounds()`。也就是说整个 verify 套件里的用户几何只被用来"喂一个正确盒给 BVH"，**从不真正命中**——过紧/过松都不会被任何断言观察到。
- `EmptyGeometryTest`（`:1282-1313`）覆盖的是"盒完全为空 → 忽略"，与"盒合法但太小"是两回事。
- `PointQueryAPICallsTest` 的用户几何断言只有 `numCalls == 2`（`:4342-4390`，断言在 `:4383-4387`），而它的 `intersectFunc/occludedFunc` 也是空 lambda（`:4360-4361`）——它证明的是"盒非空 ⇒ point query 回调被调用"，恰好是包围盒驱动回调的正面证据，也反证了"盒过小 ⇒ 回调不被调用"这条路径没有任何反面测试。
- `tutorials/user_geometry/user_geometry.exr` 参考图（由 `tests/models/make_reference_tutorials.py:44` 生成，经 `common/cmake/test.cmake:298-` 的 `ADD_EMBREE_TEST_ECS` 比较，阈值 `referenceImageThreshold = 35.0f`，`tutorials/common/tutorial/tutorial.cpp:107, 646-660`）只证明"教程渲染没炸"；教程本身用的是**过松**的 ±2 盒，因此参考图对过紧完全不敏感。

### 4.2 反例二：多命中回调被滤波器拒绝后未回滚 `tfar` → 被拒候选之后的几何整体消失

**场景**（基于教程的解析球 + 教程自带的"挖洞"滤波器）：

- 一个球，回调依次报告 `t0 < t1` 两个根（`user_geometry_device.cpp:302-375`）。
- 几何上注册一个"按位置挖洞"的滤波器：`T < 0.5f` 时 `valid[0] = 0`（教程 `sphereFilterFunction`，`:499-521`）。
- 令 `t0` 处恰好被挖掉（拒绝），`t1` 处保留（接受）。

**正确写法（教程）**：拒绝时 `ray->tfar = old_t`（`:335-336`），随后 `t1` 段用恢复后的 `tfar` 通过 `(t1 < ray->tfar)` 判据（`:339`）→ `t1` 被接受。

**错误写法（本节反例）**：省略 `else ray->tfar = old_t;`。后果是链式的：

1. 漏洞 1：`ray->tfar` 停在 `t0`。`t1` 段判据 `t1 < tfar` 立即为假 → **同一图元的第二根被自己裁掉**。
2. 漏洞 2（更严重）：`tfar` 的收缩对**整条遍历**可见。叶出栈判定是 `if (*(float*)&stackPtr->dist > ray.tfar) continue;`（`bvh_intersector1.cpp:78-80`），节点 slab 测试也用 `ray.tfar` 作为上界（`node_intersector1.h:449-479` 中 `tFar=min(...,ray.tfar)`）。于是**所有 `t > t0` 的其他图元、其他叶、其他实例的命中全部被裁掉**——一个本应被"alpha 挖洞"忽略的候选，变成了整条射线的一堵墙。
3. 结果依赖**遍历顺序与实例深度**：若接受型图元先被访问，结果正确；若拒绝型先被访问，后面的真实几何消失。多实例嵌套时 `tfar` 的泄漏还会跨 `rtcForwardIntersect1Ex` 的边界传递，因为转发用的是调用者同一个 `rayhit`（`kernels/common/rtcore.cpp:649-665`）。

**为什么这是"成对出现"的缺陷**：内置图元有 `if (!found) ray.tfar = old_t;`（`kernels/geometry/intersector_epilog.h:50-54`），用户几何没有（`kernels/common/accelset.h:145-172`）。同一份"调用滤波器"的代码，在两个路径上的正确性责任不同——而文档 `rtcInvokeIntersectFilterFromGeometry.md:24-27` 只讲了 `valid`，没讲 `tfar` 回滚。

**为什么现有测试抓不到**

- 仓库里唯二的"多命中/滤波器"测试都不是用户几何：
  - `SphereFilterMultiHitTest`（`tutorials/verify/verify.cpp:4828-4931`）用的是 **`RTC_GEOMETRY_TYPE_SPHERE_POINT`**（`:4842`），即内置图元 + 内置 epilog 的回滚；
  - `IntersectionFilterTest`（`:2958-3033`）用的是**三角形**（`:2993`），同样走内置 epilog。
- 唯一带滤波器的用户几何示例是教程 `createAnalyticalSpheres` → `rtcSetGeometryIntersectFilterFunction`（`user_geometry_device.cpp:571-574`），而它恰好是**正确写法**；参考图又只覆盖过松的盒（§4.1），因此这个反例既不会被单元测试抓到，也不会被图像回归抓到。

---

## 5. 现状与建议

### 5.1 现状（事实清单）

| # | 现状 | 证据 |
|---|---|---|
| S1 | 包围盒是"信任输入"：只校验有限性与 `lower<=upper`，不做几何一致性校验；不合格（空/NaN/±inf）即静默丢弃图元 | `accelset.h:104-118`；`common/math/bbox.h:94-102`；`RTC_GEOMETRY_TYPE_USER.md:48-50` |
| S2 | 包围盒只在建树期使用，回调拿不到它，也无法被用来裁剪命中 | `kernels/geometry/object.h:32-44, 91-94`；`accelset.h:74-85` |
| S3 | CPU 路径上，用户求交回调的 `valid` 与返回值不参与判定，唯一通道是 `rayhit` | `accelset.h:145-172`（返回 `mask!=0`）被 `object_intersector.h:35` 丢弃；包路径 `accelset.h:279-305` 为 `void` |
| S4 | 遮挡判定只看 `ray.tfar < 0`；`valid` 不参与；写正值会同时"不遮挡"且缩短后续射线段 | `object_intersector.h:47-48`；`bvh_intersector1.cpp:181-189`；`rtcSetGeometryOccludedFunction.md:57-63` |
| S5 | 用户几何**没有**内置图元那样的"滤波器拒绝即回滚 `tfar`"保护 | 对比 `intersector_epilog.h:50-54` 与 `accelset.h:145-172` |
| S5b | `rtcInvokeIntersectFilterFromGeometry` 只调用几何级滤波器，**参数级滤波器（`args->filter`）在用户几何上不会自动生效** | `kernels/common/rtcore.cpp:1522-1527`；`kernels/sycl/rtcore_sycl.cpp:135-142`；对比 `kernels/geometry/filter.h:15-36`；文档 `doc/src/api/rtcInitIntersectArguments.md:68-79` |
| S6 | 用户几何**没有**内置实例的线性包围盒非线性校正（时间维过紧的来源） | 对比 `scene_instance.cpp:181-334` 与 `accelset.h:120-130` + `common/math/lbbox.h:100-146` |
| S7 | `rtcForwardIntersect*` 只采纳 `org/dir`，其余射线属性（含 `tnear/tfar/mask/time`）一律继承原始射线；回调里对射线段的裁剪无效 | `rtcore.cpp:651-654`；`rtcForwardIntersect1.md:36-38` |
| S8 | `rtcSetGeometryBoundsFunction` 的第三个 `userPtr` 参数被实现丢弃；回调实际收到的是 `rtcSetGeometryUserData` 设的指针 | `kernels/common/scene_user_geometry.cpp:26-28`（参数未使用）；`accelset.h:79`（用 `Geometry::userPtr`）；与 `rtcSetGeometryBoundsFunction.md:32-34` 的"payload (`userPtr`)"表述不一致 |
| S9 | 未设置包围回调/求交回调时，`numPrimitives>0` 会在建树或命中时解引用空函数指针（`assert` 只在 Debug 生效） | `accelset.cpp:10`（`boundsFunc(nullptr)`）；`accelset.h:83`（无空检查）；`accelset.h:168`（`assert(intersectFunc)`）；无对应测试 |
| S10 | 测试覆盖集中"正确盒 + 空回调"场景；用户几何的真实命中、多命中、滤波器交互均无覆盖 | `verify.cpp:152-156, 366-380, 4360-4361`；`SphereFilterMultiHitTest` 用 SPHERE_POINT（`:4842`）；`IntersectionFilterTest` 用三角形（`:2993`） |
| S11 | 命名为 `user_geometry_id` 的测试组内部只创建三角形几何，与用户几何无关 | `verify.cpp:1787-1838`（`:1812, 1819`）；组注册 `:6569-6572` |
| S12 | `RTC_GEOMETRY_TYPE_USER` 的文档示例调用 `rtcSetGeometryBoundsFunction(geometry, boundsFunction)` 只有 2 个实参，与 3 参数原型不符（示例不可编译） | `RTC_GEOMETRY_TYPE_USER.md:40` vs `include/embree4/rtcore_geometry.h:216` |
| S13 | 多命中消费者按 `(t,instID,geomID,primID)` 去重，无法表达"同一 ID 的多个根"；文档给出的 `(geomID,primID)` 去重建议对解析图元是错的 | `next_hit_device.cpp:40-58, 165-174`；`rtcSetGeometryIntersectFilterFunction.md:49-54` |
| S14 | `verifyHit` 只在 DEBUG 的 `rtcIntersect1` 上执行，且只查有限性；`rtcOccluded1` 无校验 | `rtcore.cpp:626-628`；`ray.h:248-263` |
| S15 | 本仓库工作树缺少 `kernels/common/builder.h` 等文件，`make embree` 无法完成；结论因此全部基于源码阅读 + 头文件级实验，未运行集成测试 | 见 §5.3 与附录 A.3 |

### 5.2 建议

**A. API 使用侧（按优先级）**

1. **包围盒只做"宁松勿紧"的保守估计，并且每个时间步都对变换后的 8 角点取 min/max**。理由是 S1/S2/S6：过紧是静默丢命中的单向故障，过松只损失性能。教程的 `instanceBoundsFunc`（`user_geometry_device.cpp:74-96`）是可照抄的范式。
2. **把包围盒当成"随变换派生的缓存"，而不是常量**。任何 `local2world` 变化后必须 `rtcCommitGeometry`（教程 `:249`），否则 `rtcCommitScene` 会因无 modCounter 变化而早退（S1 第 3 点，`scene.cpp:828-831`）。
3. **运动模糊 + 手工实例化时，主动给盒加旋转的保守裕量**（或直接改用内置 `RTC_GEOMETRY_TYPE_INSTANCE`，它自带解析校正，S6）。
4. **在用户求交回调里，围绕每次滤波器调用手工保存/恢复 `tfar`，并自己二次分发参数级滤波器**（S5、S5b）。模板：

   ```cpp
   const float old_t = ray->tfar;
   ray->tfar = candidate_t;
   int imask = -1;
   /* fill fargs（ray/hit/context/N）... */
   rtcInvokeIntersectFilterFromGeometry(args, &fargs);   /* 仅几何级滤波器 */
   if (ctx_args->filter) ctx_args->filter(&fargs);       /* 参数级：需自己调 */
   if (imask == -1) { *hit = candidate; valid[0] = -1; } /* 接受：tfar 已收缩 */
   else             { ray->tfar = old_t; }               /* 拒绝：必须回滚 */
   ```

   `ctx_args` 可取 `((IntersectFunctionNArguments*)args)->args`（内核私有头），或在回调入口把 `args->context` 与外部保存的 `RTCIntersectArguments*` 配对。

5. **遮挡回调只写 `tfar = -inf`，绝不写命中距离**（S4）。需要"半透明遮挡"时用遮挡滤波器（`rtcSetGeometryOccludedFilterFunction`），不要试图让遮挡回调返回 t。
6. **不要指望在转发前裁剪 `xray` 的 `tnear/tfar/mask`**（S7）。要裁剪就在**转发前**修改调用者射线，或把裁剪信息放进 `RTCRayQueryContext` 的扩展区（`doc/src/api/rtcInitIntersectArguments.md:55-59` 明确允许附加数据）。
7. **多命中收集不要用 `(geomID,primID)` 去重**（S13）。用 `(t, Ng, geomID, primID, instID)` 或至少包含 `Ng`/`t`。
8. **别依赖 `valid` 在 CPU 上的门控效果**（S3）；写命中字段才是唯一契约。同时为了让同一份代码在 SYCL 上工作，仍应设置 `valid[0] = -1`。
9. **不要把 `rtcSetGeometryBoundsFunction` 的第三实参当成"包围回调专用 payload"**（S8）：实现丢弃它，回调只会收到 `rtcSetGeometryUserData` 的指针。要么两处设同一个指针，要么干脆传 `nullptr`。

**B. 测试补齐（针对 S10/S11/S13/S14）**

| 建议新增 | 断言方式 | 覆盖的缺口 |
|---|---|---|
| `user_geometry_bounds` 单元测试：同一球用 `1.0r / 0.75r / 0.5r / 1.5r` 四种盒建四个场景，从固定射线打过去 | 断言 `1.0r/1.5r` 命中 `t=4.4`、`0.75r/0.5r` **不命中且 `rtcGetDeviceError==RTC_ERROR_NONE`** | 反例一，S1/S2/S10 |
| `user_geometry_mblur_bounds`：两时间步旋转实例 + 手工用户几何 | 与内置 `RTC_GEOMETRY_TYPE_INSTANCE` 的 `rtcGetSceneLinearBounds` 做包含性比较 | S6 |
| `user_geometry_filter_reject`：球 + 拒绝第一根、接受第二根的滤波器 | 断言命中 `t1`，且同一场景里放一个 `t > t0` 的三角形断言它仍被命中 | 反例二，S5 |
| `user_geometry_argument_filter`：只设 `RTCIntersectArguments::filter`（不设几何级滤波器），对用户几何打一条必然命中的射线 | 断言滤波器被调用（当前：0 次，命中未被过滤）；与同一参数的三角形场景对照 | S5b |
| `user_geometry_missing_callbacks`：`numPrimitives=1`，不设包围/求交回调 | 期望 `RTC_ERROR_INVALID_OPERATION`（当前是崩溃） | S9 |
| 把 `UserGeometryIDTest` 改成真的创建 `RTC_GEOMETRY_TYPE_USER`，或改名 | — | S11 |
| `next_hit` 的球场景（一个解析球产生两个根） | 断言收集到 2 个不同 `t` 的命中，不被去重合并 | S13 |

**C. 实现与文档（可选但值得）**

1. **在 `rtcCommitGeometry` / `preCommit` 时校验 `boundsFunc != nullptr && intersectFunc != nullptr`**，抛 `RTC_ERROR_INVALID_OPERATION` 而不是在 Release 里空指针调用（S9，`kernels/common/geometry.cpp:109-113` 是现成的挂载点）。
2. **让 `UserGeometry::setBoundsFunction` 真正使用第三个 `userPtr`，或删掉该参数**；无论哪种，都应与 `rtcSetGeometryBoundsFunction.md:32-34` 的表述一致（S8）。当前 `AccelSet` 也没有成员可以保存它（`accelset.h:336-338`）。
3. **在 `rtcSetGeometryIntersectFunction` 文档里补一段"滤波器拒绝后必须回滚 `tfar`，内置图元自动回滚、用户几何不会"，并说明 `rtcInvokeIntersectFilterFromGeometry` 只覆盖几何级滤波器、参数级需自行分发**（S5、S5b），同时更新 `rtcInvokeIntersectFilterFromGeometry.md` 的"only rays still valid should update a hit"（`:24-27`）。
4. **在 `rtcSetGeometryTimeStepCount.md` / `RTC_GEOMETRY_TYPE_USER.md` 里说明"用户几何的线性包围盒没有实例的非线性校正，旋转类运动需要更密的时间步或保守裕量"**（S6）。
5. **修正 `RTC_GEOMETRY_TYPE_USER.md:40` 的示例**，补上第三个 `userPtr` 实参（S12）。
6. **考虑在 DEBUG 下扩展 `verifyHit`**：对用户几何记录"命中 `t` 是否落在建树期该 `(geomID,primID)` 的 AABB 内"（需要额外存储），至少能抓住反例一中"盒比命中还小"的明显错配（S14）。
7. **不建议**把"回滚 `tfar`"下沉到 `AccelSet::intersect`：用户回调可能在一次调用内报告多个候选并自带顺序，Embree 无法知道哪一次是"最终接受"。更合适的是把这条写进契约文档 + 提供教程模板。

### 5.3 关于"本仓库未能运行测试"

`kernels/common/builder.h`、`kernels/common/primref.h`、`kernels/geometry/bezier_*_intersector.h`、`kernels/geometry/oriented_curve_intersector.h` 在工作树中缺失且未被 git 跟踪（`kernels/common/builder.h` 被 `.gitignore:12` 的 `build*` 规则误伤），因此 `make embree` 在编译早期即失败（附录 A.3）。这**不影响**本报告的结论——所有结论都来自源码与公开头文件，且 §4 的两个反例都给出了可独立编译的量化实验（附录 A.1/A.2）。但"未运行 verify / 图像回归"这一点必须如实记录，尤其意味着 §5.2-B 的测试建议是**提案**而非已验证有效。

---

## 附录 A 可复现实验

### A.1 包围盒的合法性判据（对应 §2.1、§1.2）

```cpp
// /tmp/bboxcheck/t2.cpp —— 直接 include 仓库的 common/math/bbox.h
BBox3fa(Vec3fa(-1,-1,-1), Vec3fa(1,1,1))      → empty=0 isvalid=1 non_empty=1
BBox3fa(Vec3fa(0,0,0),    Vec3fa(0,0,0))      → empty=0 isvalid=1 non_empty=1   // 零体积合法
BBox3fa(Vec3fa(1,-1,-1),  Vec3fa(-1,1,1))     → empty=1 isvalid=1 non_empty=0   // 反向盒 → 丢弃
BBox3fa(Vec3fa(NaN,-1,-1),Vec3fa(1,1,1))      → empty=1 isvalid=1 non_empty=0   // NaN 静默丢弃
BBox3fa(Vec3fa(-1,-1,-1), Vec3fa(inf,1,1))    → empty=0 isvalid=0 non_empty=0   // inf → 丢弃
第 4 个 SIMD lane（对齐到 RTCBounds.align0/align1）为 NaN/inf → 不影响判定
```

命令：

```bash
g++ -std=c++17 -O2 -msse4.1 -I<srv>/0040-embree-4-f3ddad99 -o t2 t2.cpp && ./t2
```

结论：**过紧的盒（`0.5r`）与正确的盒在合法性上完全等价**——这正是反例一能"静默"的前提。

### A.2 节点 slab 测试的过紧阈值（对应 §4.1）

模型逐行对应 `kernels/bvh/node_intersector1.h:449-479`（非 FMA、AVX512 分支）：

```
tNearX = (lower_x - org_x) * rdir_x  ...   tNear = max(tNearX,tNearY,tNearZ,tnear)
tFarX  = (upper_x - org_x) * rdir_x  ...   tFar  = min(tFarX ,tFarY ,tFarZ ,tfar)
mask   = (tNear <= tFar)
```

球 `r=1` 在原点，射线 `(0.8, 0, -5) + t·(0,0,1)`（真实交点 `t=4.4` 与 `5.6`）：

| 盒半边长 | `nodeHit` | `tNear` | 说明 |
|---|---|---|---|
| `1.0r` | 1 | 4.00 | 正确 |
| `0.75r` | **0** | 4.25 | 被节点测试裁掉（反向：`tFar=4.5`） |
| `0.5r` | **0** | 4.50 | 被裁 |
| `1.5r` | 1 | 3.50 | 过松，仅多一次叶访问 |

### A.3 过松盒在旋转下的膨胀（对应 §2.3）

直接 include 仓库的 `common/math/affinespace.h` 与 `common/math/bbox.h`，用 `xfmBounds`（`affinespace.h:106-118`）计算：

```
loose  (id)                local 4.000^3 -> world 4.000^3   vol  64.0
tight  (id)                local 3.000^3 -> world 3.000^3   vol  27.0
loose  (rot 45 about y)    local 4.000^3 -> world 5.657×4×5.657   vol 128.0
tight  (rot 45 about y)    local 3.000^3 -> world 4.243×3×4.243   vol  54.0
loose  (rot 30 + t(2.2))   local 4.000^3 -> world 5.464×4×5.464   vol 119.4
tight  (rot 30 + t(2.2))   local 3.000^3 -> world 4.098×3×4.098   vol  50.4
```

即：教程的 ±2 盒 vs 真实内容 ±1.5，在 45° 旋转下世界盒体积相差 **2.37×**。

### A.4 构建环境记录

```
cmake -S <repo> -B /tmp/embree40-build -DCMAKE_BUILD_TYPE=Release \
      -DEMBREE_TUTORIALS=OFF -DEMBREE_ISPC_SUPPORT=OFF \
      -DEMBREE_TASKING_SYSTEM=INTERNAL -DEMBREE_STATIC_LIB=ON \
      -DEMBREE_MAX_ISA=AVX2
make -j4 embree
  → fatal error: builder.h: No such file or directory   (kernels/common/scene.h:8)
```

阻断文件（`git ls-files` 与磁盘均无）：`kernels/common/builder.h`（被 `.gitignore:12` 的 `build*` 捕获）、`kernels/common/primref.h`、`kernels/geometry/bezier_curve_intersector.h`、`bezier_hair_intersector.h`、`bezier_ribbon_intersector.h`、`oriented_curve_intersector.h`（被 `kernels/geometry/curve_intersector.h:9-12` 引用）。因此未执行 `verify`、`embree_user_geometry` 与图像回归。

---

## 附录 B 引用索引（按主题）

**公开 API / 结构**
- `include/embree4/rtcore_geometry.h:72-106`（三个回调的参数结构）、`:216-228`（三个 setter + 两个 filter 调用）
- `include/embree4/rtcore_common.h:165-176`（`RTCBounds` / `RTCLinearBounds`）、`:313-344`（`RTCFilterFunctionNArguments`、`RTCRayQueryContext`）
- `include/embree4/rtcore_ray.h:30-52`（`RTCHit` / `RTCRayHit`）
- `include/embree4/rtcore_scene.h:34-58`（`RTCIntersectArguments`）、`:60-72`（`RTCOccludedArguments`）

**内核实现**
- `kernels/common/accelset.h:64-130`（`valid` / `bounds` / `linearBounds` / `buildBounds`）、`:144-202`（单射线 intersect/occluded 到用户回调）、`:279-334`（包版本）、`:336-339`（成员）
- `kernels/common/scene_user_geometry.cpp:11-36`（setter）、`.h:11-97`（`createPrimRefArray*`）
- `kernels/common/accelset.cpp:9-16`（`boundsFunc(nullptr)` 与错误 intersector）
- `kernels/common/geometry.cpp:97-113`（`update`/`commit`/`preCommit`）、`:237-264`（point query）
- `kernels/geometry/object.h:23-44, 86-94`（`Object` 只存 ID）
- `kernels/geometry/object_intersector.h:25-151`（叶内分发；`valid` 返回值被丢弃）
- `kernels/geometry/intersector_iterators.h:22-38`（叶内顺序回调）、`:155-170`（包遮挡 `-inf` 归一）
- `kernels/geometry/intersector_epilog.h:44-58`（内置回滚 `tfar`）、`:101-114`（内置遮挡滤波）
- `kernels/geometry/filter.h:15-36`（两阶段滤波 + `copyHitToRay`）
- `kernels/common/hit.h:14-71, 124-137`（`HitK` / `copyHitToRay`）、`kernels/common/instance_stack.h:22-58, 140-174, 155-174`（`copy_UU` 两个重载）
- `kernels/bvh/node_intersector1.h:449-479`（slab 测试）、`kernels/bvh/bvh_intersector1.cpp:74-110, 127-129, 181-189`
- `kernels/common/scene_instance.cpp:181-334`（非线性包围盒校正）、`kernels/common/scene_instance.h:62-106`
- `kernels/common/acceln.cpp:174-192`（scene bounds 来源）、`kernels/common/rtcore.cpp:398-445`（bounds getter）、`:637-667`（forward intersect）、`:1522-1534`（filter 调用实现）
- `common/math/bbox.h:86-102`（`empty` / `isvalid(_non_empty)`）、`common/math/affinespace.h:106-118`（`xfmBounds`）、`common/math/lbbox.h:62-146`

**文档**
- `doc/src/api/RTC_GEOMETRY_TYPE_USER.md:22-53`（含 `:40` 的 2 参数示例与 `:48-50` 的"空盒即忽略"）
- `doc/src/api/rtcSetGeometryBoundsFunction.md:32-60`（`userPtr` payload 表述）
- `doc/src/api/rtcSetGeometryIntersectFunction.md:57-80`（`tfar`/`hit`/`instID` 深拷贝/多交点须调滤波器）
- `doc/src/api/rtcSetGeometryOccludedFunction.md:57-72`（`tfar = -inf`）
- `doc/src/api/rtcSetGeometryIntersectFilterFunction.md:44-79`（重复命中、允许改 hit、对象空间）
- `doc/src/api/rtcInvokeIntersectFilterFromGeometry.md:20-28`
- `doc/src/api/rtcForwardIntersect1.md:26-56`（只采纳 `org/dir`）
- `doc/src/api/rtcSetGeometryTimeStepCount.md:36-39`（用户几何每时间步一个盒）
- `doc/src/api/rtcIntersect1.md:35-62`（`Ng` 对象空间、`instID` 语义、`verifyHit` 级别的不确定性）
- `README.md:6199-6210`（回调"保守但不精确"的通用论述）

**教程 / 测试材料**
- `tutorials/user_geometry/user_geometry_device.cpp`：`:61-68`（instID 栈拷贝）、`:74-96`（实例包围盒范式）、`:98-204`（转发式实例化）、`:241-250`（`updateInstance`）、`:256-267`（球包围盒）、`:269-375`（球求交：两阶段 + 回滚）、`:377-481`（球遮挡）、`:499-531`（挖洞滤波器）、`:533-578`（注册回调）、`:665-709`（场景装配、±2 盒）、`:758-807`（shading + 阴影）、`:902-931`（每帧重建）
- `tutorials/user_geometry/user_geometry_device.ispc`：`:72-94`（实例盒）、`:96-148`（实例求交转发）、`:150-202`（实例遮挡转发）、`:254-265`（球盒）、`:267-373`（球求交 + 回滚）、`:375-495`（球遮挡）、`:497-519`（挖洞滤波）、`:521-528`（上下文滤波分发）——与 C++ 版逐段同构
- `tutorials/verify/verify.cpp`：`:140-156`（`BoundsFunc` + **空** `IntersectFuncN/OccludedFuncN`）、`:366-380`（`addUserGeometryEmpty`）、`:1282-1313`（空盒）、`:1787-1838` + `:6569-6572`（名不符实的 `user_geometry_id`）、`:2958-3033`（三角 + 滤波器）、`:3035-3136`（内置实例 + 滤波器计次）、`:4342-4390`（用户几何 point query 计次）、`:4828-4931`（SPHERE_POINT 多命中）
- `tutorials/next_hit/next_hit_device.cpp`：`:33-75`（命中全序）、`:123-143`（收集全部）、`:160-178`（排序 + 去重）、`:196-243`（多趟收集与 `tfar` 收缩）、`:260-289`（趟循环，`tnear` 递进）
- `tests/models/make_reference_tutorials.py:44`（`user_geometry.exr` 参考图）、`common/cmake/test.cmake:298-554`（`ADD_EMBREE_TEST_ECS` 的参考图/阈值机制）、`tutorials/common/tutorial/tutorial.cpp:107, 646-660`（阈值 35 与比较）
- `tutorials/user_geometry/CMakeLists.txt:13`（唯一注册的 `user_geometry` 测试）
