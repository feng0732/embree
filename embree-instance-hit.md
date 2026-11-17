# 光线穿过实例几何体后的命中坐标与法线：静态契约分析

> 分析对象：本仓库 vendored 的 Embree（`git log` 只有一条 `vendor RenderKit/embree 3b7e4ee`）。
> 分析方式：**静态契约分析**（源码 + 文档 + 测试断言 + 用仓库自带数学头文件复现内核算式的数值探针），不做性能跑分。
> 约定：本文中的 `M` 表示实例几何体的 **local → world** 仿射变换，`W = M⁻¹ = rcp(M)` 表示 **world → local**。

---

## 0. 证据基础与一个必须先说明的构建障碍

### 0.1 本轮无法链接运行库，原因在仓库本身

`kernels/common/builder.h` **不在仓库里**，但 `kernels/common/scene.h:8` 无条件 `#include "builder.h"`，因此任何 kernel 编译单元都会失败：

```
kernels/geometry/../common/scene.h:8:10: fatal error: builder.h: No such file or directory
```

原因是 `.gitignore` 第 12 行的 `build*` 通配符把 `builder.h` 一起忽略了：

```text
$ git check-ignore -v kernels/common/builder.h
.gitignore:12:build*	kernels/common/builder.h
```

这不是分析失误而是 vendor 时的文件丢失（`git ls-files kernels/common | wc -l` = 55，目录里也只有 55 个文件，说明不是“未提交”，而是从一开始就没被 add）。所以：

* **凡是需要运行 Embree 才能得到的结论，本文一律标注为“静态推导”或“探针复现”**，不冒充实机结果。
* 另有一个次要现象：`kernels/geometry/curve_intersector.h:8-12` 引用了 `../common/primref.h` 与 `bezier_curve_intersector.h` 等同样缺失的头文件，但该头文件**没有被任何编译单元 include**，属于死文件，不阻塞构建。

### 0.2 探针方法（可复现）

`common/math/` 是完整且自包含的（`g++ -I common -I common/math -I common/simd -I common/sys` 即可编译）。下文所有数值都来自一个只依赖这些头文件、并**逐行复刻** `InstanceIntersector1::intersect` 与 Möller–Trumbore 算式的探针。探针源码见附录 A。

---

## 1. 对象空间、实例空间、世界空间

### 1.1 三个空间的严格定义

| 空间 | 定义 | 本文记号 |
|---|---|---|
| 世界空间 world | 顶层 `RTCScene` 中 `rtcIntersect1` 收到的 `ray.org/dir` 所在空间 | `p_w`, `d_w` |
| 实例空间 / 局部空间 local（也叫 instance space） | 某一层实例几何体的 **被实例化场景** 所处空间，即该实例的 `local2world` 的目标空间 | `p_l`, `d_l` |
| 对象空间 object space | **最内层**被命中几何体（`geomID` 指向的那张网格）自身顶点所在空间 | `Ng_obj` |

三者对单层实例是同一组坐标的两个副本；对嵌套实例，**“实例空间”是逐层的**（第 1 层、第 2 层……），而“对象空间”始终是最内层 leaf 几何体的空间。`instID` 栈的层号恰好标出这是第几层实例空间。

### 1.2 内核真正做的事：不是“重算 t”，而是“换坐标、保留 t”

`kernels/geometry/instance_intersector.cpp:15-38`（标量版本；MB 版本 `:103-127`、packet 版本 `:192-217` 结构完全相同）：

```cpp
RTCRayQueryContext* user_context = context->user;
if (likely(instance_id_stack::push(user_context, prim.instID_, 0)))
{
  const AffineSpace3fa world2local = instance->getWorld2Local();
  const Vec3ff ray_org = ray.org;
  const Vec3ff ray_dir = ray.dir;
  ray.org = Vec3ff(xfmPoint(world2local, ray_org), ray.tnear());
  ray.dir = Vec3ff(xfmVector(world2local, ray_dir), ray.time());
  RayQueryContext newcontext((Scene*)instance->object, user_context, context->args);
  instance->object->intersectors.intersect((RTCRayHit&)ray, &newcontext);
  ray.org = ray_org;
  ray.dir = ray_dir;
  instance_id_stack::pop(user_context);
}
```

要点逐条：

1. **只变换 `org` 与 `dir`**，`tnear`、`tfar`、`time`、`mask` 原样带入子场景。
2. `dir` 用 `xfmVector`（只乘 3×3 线性部分，见 `common/math/affinespace.h:103`），**不做归一化**。
3. 子场景返回后**只恢复 `org` 和 `dir`**；`tfar`、`Ng`、`u`、`v`、`geomID`、`primID`、`instID` 全部保留（因为它们已经被写进同一个 `RayHit`）。
4. `push` 在栈满时返回 `false`，**整个实例子树被静默跳过**（`kernels/common/instance_stack.h:28-37`）。

由此得到本文的核心恒等式（对任意一层实例都成立）：

```
p_l(t) = W · p_w(t) = W · (o_w + t d_w) = o_l + t (W d_w)
```

即 **`t` 是跨所有实例层共享的同一个射线参数**。它既不是“世界距离”，也不是“局部距离”，除非 `|dir|` 恰好为 1。

### 1.3 命中记录各字段属于哪个空间

写入点在 `kernels/geometry/intersector_epilog.h:59-68`：

```cpp
ray.tfar = hit.t;
ray.Ng   = hit.Ng;
ray.u    = hit.u;
ray.v    = hit.v;
ray.primID = primID;
ray.geomID = geomID;
instance_id_stack::copy_UU(context->user->instID, ray.instID);
```

| 字段 | 所在空间 | 说明 |
|---|---|---|
| `ray.tfar` | **参数 t（无量纲/长度取决于 dir）** | 世界射线的 `o_w + t·d_w` 就是世界命中点，见 `tutorials/viewer/viewer_device.cpp:249`、`tutorials/instanced_geometry/instanced_geometry_device.cpp:205` |
| `hit.Ng` | **最内层对象空间** | UI 文档：`RTCHit` 存 “unnormalized geometric normal in **object space**”（`doc/src/api/RTCHit.md`；同样文字见 `README.md:7839-7843`（RTCHit 节）与 `README.md:8566-8570`（rtcIntersect1 节））；过滤回调文档更明确：“If the hit geometry is instanced, … the ray and the potential hit are in **object space**”（`README.md:5416-5419`） |
| `hit.u`, `hit.v` | 仿射不变量 | 重心坐标在仿射变换下不变，可直接用于 `rtcInterpolate` |
| `hit.geomID`, `hit.primID` | **最内层被实例化场景**的 ID | `RTC_GEOMETRY_TYPE_INSTANCE.md` / `README.md:4100-4103` |
| `hit.instID[level]` | 每一层实例在**其父场景**中的 geometry ID | 层号 0 = 最外层 |
| `hit.instPrimID[level]` | 仅 instance array：命中的那个变换在该 array 中的下标 | `README.md:4200-4201` |

`Ng` 之所以在对象空间，是因为它由 leaf 三角形在**变换后的局部射线**下算出：`kernels/geometry/triangle_intersector_moeller.h:122` `tri_Ng = cross(tri_e2, tri_e1)`，其中 `e1 = v0-v1, e2 = v2-v0`（同文件 `:145-149`）。整条相交链全程没有把 `Ng` 变回世界。

**结论：Embree 不会给你世界空间的命中点，也不会给你世界空间的法线。** 命中点只能用 `o_w + tfar·d_w` 重建；法线必须由应用自己用 `(M⁻¹)ᵀ` 变换。

### 1.4 单层实例的完整数据流（示意）

```
world:      o_w, d_w,  tnear, tfar
                │  push instID=7;  W = rcp(M)
                ▼
local:      o_l = M⁻¹ o_w        d_l = M⁻¹ d_w  (未归一化!)
            tnear, tfar 原样
                │  三角形求交 (Möller–Trumbore)
                ▼
hit:        t (= tfar)  u, v  Ng_obj = cross(v2-v0, v0-v1)
                │  pop; 只恢复 org/dir
                ▼
world:      ray.org/ray.dir 仍是世界值
            ray.tfar = t,  hit.Ng = Ng_obj,  hit.instID[0] = 7,
            hit.geomID = leaf geomID
```

---

## 2. 嵌套变换与非均匀缩放如何影响距离与法线

### 2.1 嵌套：变换按“外层→内层”逐级左乘，t 依然不变

每进一层，内核都用**该层自己的** `world2local` 变换“当前射线的 org/dir”。设外层 `M_o`、内层 `M_i`，则第 2 层看到的射线是

```
o_2 = W_i (W_o o_w) = (W_i W_o) o_w
d_2 = W_i (W_o d_w) = (W_i W_o) d_w
```

而 `W_i W_o = (M_o M_i)⁻¹`（`rcp(A)rcp(B) = rcp(BA)`，`common/math/affinespace.h:83`）。也就是说，**逐层变换等价于用复合矩阵 `M_comp = M_o M_i …` 一次性变换**。

探针（附录 A，案例 B：`M_o = T(5,0,0)·S(2)`，`M_i = T(0,0,1)·S(0.5,1,3)`）直接比较了逐层累加与复合矩阵：

```
|W_stack - rcp(M_comp)| (max elem) : 0.000e+00     // W_stack = W_i · W_o
|M_comp * leaf_hit - (o_w + t·d_w)| : 0.000e+00     // 逐层命中点确实映射回世界命中点
|t_truth - t_leaf|                 : 4.768e-07     // 与“世界空间三角形直接求交”的 t 一致
```

这同时给出一个**易错点**：如果把逐层 `world2local` 的顺序写成 `W_o W_i`，结果就变成 `rcp(M_o)rcp(M_i) = rcp(M_i M_o) ≠ rcp(M_o M_i)`，只有各层可交换（如纯平移、或同轴缩放）时才看不出来。

**距离**：`t` 仍然是同一个参数。但每层的单位长度不同（本例世界方向取单位向量）：

```
|d_w| = 1.000000, |d_1| = 0.500000, |d_2| = 0.166667
t = 7.900000
leaf hit  = (0.300000, 0.350000, 0.350000)
world hit = o_w + t·d_w = (5.300000, 0.700000, 4.100000) = M_comp · (leaf hit)
距离：世界 t·|d_w| = 7.900 ; layer1 t·|d_1| = 3.950 ; leaf t·|d_2| = 1.317
```

所以“t 是距离”这句话在嵌套场景中**任何一层都不成立**，只有当你显式地用该层的 `|d_level|` 去乘时才是该层的距离；世界距离是 `t·|d_w|`。

嵌套时 `instID` 栈形如 `[3, 9]`：level 0 是外层实例的 geometry ID，level 1 是内层实例的 geometry ID（层级构造方式与 `tutorials/verify/verify.cpp:3083-3086` 相同；但那里的嵌套变换全是 `one`，探针换成了非恒等变换）。**层数上限是编译期常量**，默认 1：

```text
kernels/rtcore_config.h.in:16   #define RTC_MAX_INSTANCE_LEVEL_COUNT @EMBREE_MAX_INSTANCE_LEVEL_COUNT@
CMakeLists.txt:200              SET(EMBREE_MAX_INSTANCE_LEVEL_COUNT 1 CACHE STRING ...)
```

超过深度的实例“release 模式静默消失、debug 模式断言”（`README.md:874-875`、`README.md:4084-4089`），实现上就是 `push` 返回 `false` 后整段跳过（`kernels/common/instance_stack.h:28-37`）。对使用者而言，表现是“某些子树整体不可见”，而不是“命中了但 ID 不对”。

### 2.2 法线：正确算子是 `(M⁻¹)ᵀ`，不是 `M`

仓库自带 `xfmNormal` 的定义是教科书式余切变换：

```cpp
// common/math/linearspace3.h:161
template<typename T> T xfmNormal(const LinearSpace3<T>& s, const T& a) {
  return xfmVector(s.inverse().transposed(), a);   // (M⁻¹)ᵀ · n
}
```

对**非均匀缩放**，`M·n` 与 `(M⁻¹)ᵀ·n` 方向不同：

```
M = S(4,1,1),  Ng_obj = (1,0,-1)
naive M·Ng      = ( 4.000, 0.000, -1.000)
xfmNormal(M,Ng) = ( 0.250, 0.000, -1.000)
angle(naive, 正确方向) = 61.93°
```

而用顶点重建的“地面真值”验证了 `xfmNormal` 是对的：

```
世界空间顶点算出的 Ng = (1.000, 0.000, -4.000)
det(M) · xfmNormal(M,Ng) = (1.000, 0.000, -4.000)   ← 完全一致
```

这个 `det(M)` 因子正是 3D 余子式恒等式 `cof(M) = det(M)·M⁻ᵀ`，也解释了为什么 `xfmNormal` 给出的是**同向但长度被缩放**的向量：它只保证方向，不保证长度（`Ng` 的契约本来就只要求“unnormalized”，见 `kernels/common/ray.h:266` 注释 `not normalized geometry normal`）。

**均匀缩放**时方向恰好不变（只是长度差一个因子）：

```
M = S(4), Ng = (1,0,-1), |Ng| = 1.414214
angle(naive, 正确方向) = 0.03°（浮点级）
xfmNormal 长度 = 0.354 (= 0.25·|Ng|)   naive 长度 = 5.657 (= 4·|Ng|)   两者相差 s² = 16 倍
```

这解释了为什么很多代码“看起来能用”：只要实例是相似变换，方向没错，只错标量。

### 2.3 嵌套 + 非均匀：法线复合顺序必须与变换复合顺序一致

对 `M_comp = M_o M_i`，正确法线矩阵是

```
(M_o M_i)⁻ᵀ = M_o⁻ᵀ M_i⁻ᵀ
```

即**逐层 `(M_layer⁻¹)ᵀ` 按“外层→内层”依次左乘**。仓库的多级实例教程正是这样写的：

```cpp
// tutorials/multi_instanced_geometry/multi_instanced_geometry_device.cpp:28-42
LinearSpace3fa transform = LinearSpace3fa(one);
for (level ...) {
  LinearSpace3fa M = data.g_instanceLevels.normalTransforms[level][instId];
  transform = transform * LinearSpace3fa(M);      // 外层先乘
}
// ... :160-161
Vec3fa Ns = normalize(xfmVector(xfm, Vec3fa(primaryRay.Ng)));
```

其中 `normalTransforms[i] = transposed(rcp(M.l))`（`tutorials/multi_instanced_geometry/scene.cpp:165`）。

探针验证（复合含旋转，避免顺序不可交换的巧合）：

```
外层 M_o = T(5,0,0)·Rot((0.3,1,0.2), 0.7)·S(2)
内层 M_i = T(0,0,1)·RotY(-0.4)·S(0.5,1,3)
N_outer · N_inner · Ng   = ( 0.918486,  0.147988, -0.402816)   angle to (E⁻¹)ᵀ = 0.0000°
N_inner · N_outer · Ng   = ( 0.240731,  0.161912, -0.145528)   angle to (E⁻¹)ᵀ = 22.6404°  ← 顺序反了
```

注意这里有一个**实现细节陷阱**：顺序错时 `xfmNormal` 仍会返回“某个法线”，不会报错、不会 NaN，只是方向偏了 22.6°。这属于静默错误。

### 2.4 `det(M) < 0`（镜像实例）

镜像变换下，用顶点重建的世界法线与 `xfmNormal` 的结果**符号相反**：

```
M = S(-1,1,1), det = -1
Ng_obj              = ( 1.000, 0.000, -1.000)
xfmNormal(M,Ng)     = (-1.000, 0.000, -1.000)
det(M)·xfmNormal    = ( 1.000, 0.000, 1.000)
世界顶点重建        = ( 1.000, 0.000, 1.000)
naive M·Ng          = (-1.000, 0.000, -1.000)
```

因为 `det < 0` 时 `cof = det·M⁻ᵀ` 整体反号。两种做法都自洽（要么统一乘 `sign(det)`，要么统一用“重建顶点”的约定），但**不能混用**——混用会让正面/背面判断翻转，例如 `tutorials/viewer/viewer_device.cpp:198-201` 的 `face_forward` 只按 `dot(dir,Ng)` 定符号，遇到混用就会把背面当成正面。

### 2.5 退化变换：没有校验

`Instance::commit()` 直接求逆，不检查 `det`：

```cpp
// kernels/common/scene_instance.cpp:150-153
if (unlikely(gsubtype == GTY_SUBTYPE_INSTANCE_QUATERNION))
  world2local0 = rcp(quaternionDecompositionToAffineSpace(local2world[0]));
else
  world2local0 = rcp(local2world[0]);
```

探针：

```
M = S(1,1,0), det = 0
world2local.vz = (-nan, -nan, inf)
```

`rcp(LinearSpace3)` 走 `adjoint()/det()`（`common/math/linearspace3.h:51`），除零直接产生 inf/NaN，随后 NaN 会污染整条射线的 BVH 遍历与 `tfar`。文档也没有承诺会报错。**契约上：输入必须是非退化仿射变换，否则行为未定义。**

### 2.6 运动模糊实例：中间时刻的插值矩阵不再是相似变换

多时间步的实例若用 `rtcSetGeometryTransform` 给关键帧，`getLocal2World(t)` 对矩阵做**逐元素线性插值**（`kernels/common/scene_instance.h:135-144`，`lerp` 见 `common/math/affinespace.h:168-176`）：

```cpp
return lerp(local2world[itime+0], local2world[itime+1], ftime);
```

两帧纯旋转 `I` 与 `RotZ(90°)` 的中点：

```
M(0.5) = [[0.500 -0.500 0],[0.500 0.500 0],[0 0 1]]
列长 = (0.707, 0.707, 1.000)   det = 0.500   similarityTransform = false
Ng=(1,0,1) 时：xfmNormal(M,Ng) = (1,1,1)   naive M·Ng = (0.5,0.5,1)   两者夹角 19.47°
```

再取一个带位移 + 均匀缩放的例子（关键帧 `I` 与 `T(5,0,0)·RotZ(90°)·S(2)`，真实刚体中点为 `T(2.5,0,0)·RotZ(45°)·S(2)`）：

```
插值矩阵列长 = (1.118, 1.118, 1.500)   det = 1.875   similarityTransform = false
xfmNormal(插值,Ng) 与真实刚体结果夹角 = 16.17°
每单位 t 的世界距离（沿 -z）：插值 1.500 vs 真实 2.000（相差 25%）
```

即：**中间时刻的实例事实上是“非均匀 + 剪切”变换**，`xfmNormal` 与 `M·n` 都会给出偏离真实刚体结果的向量，`t` ↔ 距离的换算也会偏；同时 `similarityTransform` 会返回 false。若需要旋转正确插值，必须用 `rtcSetGeometryTransformQuaternion` + `RTCQuaternionDecomposition`（`slerp` 路径，`common/math/affinespace.h:201-221`，文档 `README.md:6439-6444`）。

---

## 3. 可观察结果（探针输出，全部可由附录 A 复现）

### 3.1 单层实例（相似变换，`M = T(10,0,0)·S(2)`，三角形在 z=0 平面）

| 观测量 | 值 |
|---|---|
| `|d_w|` / `|d_l|` | `1.000` / `0.500` |
| `tfar` | `3.000` |
| 局部命中点 | `(0.200, 0.300, 0.000)` |
| `o_w + t·d_w`（世界命中点） | `(10.400, 0.600, 0.000)` |
| `M · 局部命中点` | `(10.400, 0.600, 0.000)`（一致） |
| 局部距离 `t·|d_l|` | `1.500` |
| 世界距离 `t·|d_w|` | `3.000` |
| `Ng_obj` | `(0, 0, 1)`，`|Ng| = 1` |
| `xfmNormal(M,Ng)` | `(0, 0, 0.5)`，`det = 8` |
| `det(M)·xfmNormal` | `(0, 0, 4)` = 世界顶点重建的 `Ng` |

**反例：如果你把局部 `dir` 归一化，`t` 会变**

```
kernel 的未归一化局部 dir : t = 3.000
人为归一化后             : t = 1.500
```

而命中点仍然是 `(0.200, 0.300, 0.000)`——说明 **`t` 只有和“同一次变换得到的未归一化 dir”配对才有意义**。用户手动实现实例化（`tutorials/user_geometry/user_geometry_device.cpp:115-123` 的 `#if 0` 分支正是这个写法）时，绝对不能顺手 `normalize`。

### 3.2 嵌套实例（`M_o = T(5,0,0)·S(2)`，`M_i = T(0,0,1)·S(0.5,1,3)`）

| 观测量 | 值 |
|---|---|
| `instID` 栈 | `[3, 9]` |
| `|d_w| / |d_1| / |d_2|` | `1.000000 / 0.500000 / 0.166667` |
| `t` | `7.900000`（世界、layer1、leaf 全部相同） |
| leaf 命中点 | `(0.300000, 0.350000, 0.350000)` |
| 世界命中点 | `(5.300000, 0.700000, 4.100000)` |
| 每单位 `t` 对应的距离 world / level1 / leaf | `1.000000 / 0.500000 / 0.166667`（该层距离 = `t` × 该值） |
| `Ng_obj` | `(0,-1,1)` |
| `det(M_comp)·[(M_o⁻¹)ᵀ(M_i⁻¹)ᵀ]·Ng` | `(0, -6, 2)` |
| 世界顶点重建 `Ng` | `(0, -6, 2)`（一致） |
| naive `M_comp·Ng` | `(0, -2, 6)`（角度差 53.13°） |

### 3.3 非均匀缩放反例（`M = S(4,1,1)`，`Ng_obj = (1,0,-1)`）

| 观测量 | 值 |
|---|---|
| `t` | `4.925` |
| `Ng_obj` | `(1, 0, -1)` |
| `xfmNormal(M,Ng)` | `(0.25, 0, -1)` |
| `det·xfmNormal` | `(1, 0, -4)` |
| 世界顶点重建 `Ng` | `(1, 0, -4)` |
| naive `M·Ng` | `(4, 0, -1)` |
| naive 与真值夹角 | **61.93°** |

方向性差异（同一 `M=S(4,1,1)`，单位世界方向 `d_w`，per-t 世界距离 = `1/|W d_w|`）：

| `d_w` | `|W d_w|` | per-t 世界距离 |
|---|---|---|
| `(0,0,-1)` | 1.000 | 1.000 |
| `(1,0,-1)/√2` | 0.729 | 1.372 |
| `(-1,0,0)` | 0.250 | 4.000 |

即：**非均匀缩放下 `t` 与“世界距离”之间不存在一个只依赖实例的常数换算因子**，必须先知道方向。任何“把 tfar 乘以 scale 得到世界距离”的写法都只对相似变换成立（这正是 `similarityTransform` 存在的理由，`common/math/affinespace.h:318-339`）。

---

## 4. 相关 API 的坐标语义（与射线路径一致/不一致之处）

### 4.1 遮挡查询

`kernels/geometry/instance_intersector.cpp:63`、`:152`：

```cpp
occluded = ray.tfar < 0.0f;
```

遮挡的“命中”约定是 `tfar = neg_inf`（`kernels/bvh/bvh_intersector1.cpp:187`），实例层**只透传**，不做任何缩放。因此 `tfar` 在遮挡查询里不是距离，只是标志位；`Ray::occluded()` 也按 `tfar < 0` 判断（`kernels/common/ray.h:195`）。

### 4.2 点查询：**唯一**显式处理“非相似变换”的路径

`kernels/geometry/instance_intersector.cpp:69-91`：

```cpp
const bool similtude = context->query_type == POINT_QUERY_TYPE_SPHERE
                     && similarityTransform(world2local, &similarityScale);
...
query_inst.p      = xfmPoint(world2local, query->p);
query_inst.radius = query->radius * similarityScale;
...
similtude ? POINT_QUERY_TYPE_SPHERE : POINT_QUERY_TYPE_AABB
```

* 相似变换：把查询点搬到实例空间，半径乘 `similarityScale`（= `D`，见 `common/math/affinespace.h:337`）。
* 非相似变换：`similarityScale = 0`，降级为 AABB 近似（`kernels/common/context.h:142-153`，`:151-152` 用 `xfmBounds` 把球变成轴对齐盒子），回调可能被多余调用（`README.md:6204-6209`）。
* 多级时 `world2inst` 的累加也是**内层左乘外层**：`W_stack = W_new · W_prev`（`kernels/common/instance_stack.h:106-107`），与射线路径的 `W_i W_o` 一致。
* 教程给出了非相似变换的正确做法：把图元搬到世界空间再算距离（`tutorials/closest_point/closest_point_device.cpp:296-341`），并用 `assert(!similarityTransform(...))` 标注了非均匀/剪切的用例（同文件 `:524-526`）。

### 4.3 min-width（曲线/点半径放大）

`enlargeRadiusToMinWidth` 在**当前实例空间**里用 `length(p - ray_org)` 计算放大半径（`kernels/common/context.h:76-85`）：

```cpp
const float d = length(Vec3fa(v) - ray_org);
const float r = clamp(context->getMinWidthDistanceFactor()*d, v.w, geom->maxRadiusScale*v.w);
```

由于 `ray_org` 已被搬到实例空间、`v` 也是实例空间坐标，放大在语义上是正确的；但**像素宽度对应的 `minWidthDistanceFactor` 是应用在世界空间选的数**（`tutorials/viewer/viewer_device.cpp:227` 用屏幕宽高算），进入缩放过的实例后像素尺度会跟着变。契约上这意味着：min-width 与实例缩放叠加时，“半像素宽”不再是全局一致的目标，需要应用自己权衡。文档里 `r' = length(p - ray_org) * minWidthDistanceFactor` 的公式（`README.md:4703-4715`）也隐含了“在哪个空间量 `length` 就在哪个空间定义像素尺度”。

### 4.4 instance array

与普通实例完全同构，只是变换按 `prim.primID_` 取、并额外压入 `instPrimID`：

```cpp
// kernels/geometry/instance_array_intersector.cpp:27-32
if (likely(instance_id_stack::push(user_context, prim.instID_, prim.primID_)))
{
  const AffineSpace3fa world2local = instance->getWorld2Local(prim.primID_);
  ...
  ray.org = Vec3ff(xfmPoint(world2local, ray_org), ray.tnear());
  ray.dir = Vec3ff(xfmVector(world2local, ray_dir), ray.time());
```

`InstanceArray::getWorld2Local(i)` 内部同样是 `rcp(getLocal2World(i))`（`kernels/common/scene_instance_array.h:169-170`），所以 1.2 节的恒等式与所有距离/法线结论对 array 逐条成立。区别仅在“模板变换按索引取”，因此 `instPrimID` 是**定位到具体那一份变换**的唯一途径。

### 4.5 查询变换矩阵的 API 返回的是 local→world，不是 world→local

`rtcGetGeometryTransform` / `rtcGetGeometryTransformEx` / `rtcGetGeometryTransformFromScene` 都返回 `geometry->getTransform(...)`，即 `local2world`（`kernels/common/rtcore.cpp:1487-1515`，文档 `doc/src/api/rtcGetGeometryTransformFromScene.md`）。应用若要拿 `Ng` 的世界方向，需要自己做 `xfmNormal(M, Ng)`，仓库正是这么用的：

```cpp
// tutorials/instanced_geometry/instanced_geometry_device.cpp:191-192
rtcGetGeometryTransformFromTraversable(data.g_traversable, ray.instID[0], 0.0f,
                                       RTC_FORMAT_FLOAT4X4_COLUMN_MAJOR, &xfm);
Ns = xfmNormal(xfm, Ns);
```

---

## 5. 源码与测试证据清单

### 5.1 源码（契约实现）

| 主题 | 位置 |
|---|---|
| 实例求交：压栈、`world2local`、只换 org/dir、只恢复 org/dir | `kernels/geometry/instance_intersector.cpp:15-38`（标量）、`:103-127`（MB）、`:192-217`（packet） |
| 实例数组版本（按 `primID_` 取变换） | `kernels/geometry/instance_array_intersector.cpp:14-40` |
| 栈操作与深度上限 | `kernels/common/instance_stack.h:18-52`（push 返回 false 静默丢弃）、`:79-116`（点查询版累加 `W_new·W_prev`，见 `:106-107`） |
| `local2world` / `world2local` 的获取；`rcp` 在 commit 时做一次 | `kernels/common/scene_instance.h:128-154`、`kernels/common/scene_instance.cpp:145-156` |
| MB 的 `lerp` / `slerp` 分支 | `kernels/common/scene_instance.h:135-144`、`common/math/affinespace.h:168-176`、`:201-221` |
| 命中字段写入（`tfar`/`Ng`/`u`/`v`/`instID`） | `kernels/geometry/intersector_epilog.h:59-68` |
| `Ng` 由局部边叉积产生 | `kernels/geometry/triangle_intersector_moeller.h:84-108`、`:122`、`:145-149` |
| `xfmPoint` / `xfmVector` / `xfmNormal` 定义 | `common/math/affinespace.h:102-104`、`common/math/linearspace3.h:159-161` |
| `rcp(AffineSpace)` = 逆 | `common/math/affinespace.h:83` |
| `similarityTransform` 判定与返回 D | `common/math/affinespace.h:318-339` |
| 遮挡的 `tfar = neg_inf` 约定 | `kernels/bvh/bvh_intersector1.cpp:187`、`kernels/common/ray.h:195` |
| `Ray`/`RayHit` 内存布局（`org.w`=tnear、`dir.w`=time） | `kernels/common/ray.h:200-204`、`:266-273` |
| 对外结构体 | `include/embree4/rtcore_ray.h:11-27`（`RTCRay`）、`:30-45`（`RTCHit`） |
| 查询上下文 | `include/embree4/rtcore_common.h:335-361`（`RTCRayQueryContext`）、`:405-440`（`RTCPointQueryContext`） |

### 5.2 文档（契约声明）

| 声明 | 位置 |
|---|---|
| “unnormalized geometry normal in object space” | `README.md:7839-7843`、`README.md:8566-8570`、`doc/src/api/RTCHit.md` |
| 过滤回调里“the ray and the potential hit are in object space” | `README.md:5416-5419` |
| 射线方向不必归一化 | `README.md:7781-7783` |
| 命中 ID 的层级含义 | `README.md:8573-8582` |
| 实例深度超限静默丢弃 | `README.md:874-875`、`README.md:4084-4089` |
| `instPrimID` 语义 | `README.md:4200-4201` |
| 点查询的 `similarityScale` 语义（非相似⇒必须在世界空间算距离） | `README.md:6179-6197`、`include/embree4/rtcore_common.h:460-471` |
| 非相似变换下点查询域退化为 AABB 近似 | `README.md:6204-6209` |
| min-width 的目标半径公式 | `README.md:4703-4715` |
| 四元数插值用于旋转正确的 MB | `README.md:6439-6444` |

### 5.3 测试与教程（可观察行为）

| 证据 | 说明 |
|---|---|
| `tutorials/verify/verify.cpp:3035-3136`（`InstancingTest`） | 在 `:3083-3086` 用**全 `one` 的变换**堆出 `RTC_MAX_INSTANCE_LEVEL_COUNT` 层实例，然后只断言“有命中 / `geomID` 有效 / 遮挡时 `tfar==neg_inf`”（`:3120-3134`）。**只验证了“深栈能走通”，没有验证任何非恒等嵌套变换，也没有检查 `Ng` 与 `instID` 的内容** |
| `tutorials/verify/verify.cpp:3140-3222`（`InstanceArrayTest`） | 每份变换是纯平移（`:3238`），断言 `geomID` 有效、`instID[0]==0`、`instPrimID[0]==i-1`（`:3204-3212`）；未检查 `Ng` |
| `tutorials/verify/verify.cpp:3317-3325` | 随机变换 = `T · S(0.5..2 各轴独立) · R`，**明确含各向异性缩放** |
| `tutorials/verify/verify.cpp:3354-3640`（`InstanceArrayRandomTest`，`:3353` 起） | 用上述随机非均匀变换对比“instance array”与“逐个 instance”；**只比较 `tfar`，且容许 1% 相对误差**（`:3617-3619`），实例数组 vs 实例的命中数与 `instPrimID↔instID` 等价性断言**被注释掉了**（`:3605-3613`，注释给出的原因是 AVX2 上的精度问题） |
| 全局搜索 | `verify.cpp` 中 `hit.Ng_x` 只出现在 `:2736`（三角形）、`:2813`（四边形）、`:2944`（背面剔除），**没有任何一条断言检查“实例变换后的 `Ng`”** |
| `tutorials/instanced_geometry/instanced_geometry_device.cpp:186-195`、`:334-335` | 单层实例的规范做法：运行时 `xfmNormal(xfm,Ng)`（`:192`），以及 `normal_xfm = transposed(rcp(M.l))`（`:335`）的预计算等价形式 |
| `tutorials/multi_instanced_geometry/multi_instanced_geometry_device.cpp:28-43`、`:160-161`；`scene.cpp:165` | 多层法线的规范做法：外层先乘 |
| `tutorials/viewer/viewer_device.cpp:162-196` | 多层 `local2world` 累加（`local2world = local2world * M`），但 `:192` 用的是 **`xfmVector(local2world, dg.Ng)` 而不是 `xfmNormal`**——**viewer 在非均匀缩放实例下会算错法线**（只在相似变换模型上才正确）。这是仓库内部一处真实的不一致 |
| `tutorials/closest_point/closest_point_device.cpp:296-341`、`:524-526` | 点查询在非相似变换下改为“把图元搬到世界空间”；并显式断言 `S(1,2,3)` 与剪切矩阵“不是相似变换” |
| `tutorials/viewer/viewer_device.cpp:249` | 命中点只能 `ray.org + tfar*ray.dir` 重建 |
| `tutorials/user_geometry/user_geometry_device.cpp:115-123` | 手工实例化的正确写法（`xfmPoint`/`xfmVector`、不归一化、`tnear/tfar` 原样） |
| `tests/models/msmblur/linear_instance_linear_triangle.ecs` + `.embree_viewer.exr` | 单层实例 + MB 的图像级回归（`--instancing geometry`） |
| `tests/models/msmblur/mblur_time_range_instancing.ecs` | MB 实例回归，且 `--compare-threshold 2500  # requires larger threshold # FIXME: why that large ???`（仓库自己承认该用例图像差异大） |
| `tests/models/instance_array/instance_array_mb.ecs` | instance array + 四元数 MB（`--instancing group`），但**没有参考图**，只作为可运行资产 |
| `tests/CMakeLists.txt:5` | 图像测试的 shader 集合包含 `Ng`，即“法线可视化”是正式回归维度——但只覆盖 `tests/primitives/*.xml`（非实例） |
| `tutorials/viewer/CMakeLists.txt:73,77` | 两个实例模型被登记进 viewer 图像测试（`INTENSITY 2`） |

**证据覆盖度总结**：仓库的测试**证明了**单层/多层实例的可见性、`instID`/`instPrimID` 的填法、`tfar` 在 array 与 instance 之间的一致性（1% 容差）；**没有证明**任何“实例变换后的 `Ng` 方向/符号”，也**没有**用非恒等嵌套变换验证过距离。本文 3.x 的数值因此由探针补齐，而非由上游断言背书。

---

## 6. 应用层契约速查（由上述分析直接得出）

1. **命中点**：`P_w = ray.org + ray.tfar * ray.dir`（世界空间）。不要用局部 `org + t*dir` 当世界点。
2. **局部命中点**：若需要，用该层 `world2local` 变换 `P_w`，或直接保留入射射线在每层的副本；**不要**假设 `tfar` 是局部距离。
3. **法线**：`Ns_world_direction = normalize( (M_outer⁻¹)ᵀ · (M_inner⁻¹)ᵀ · … · Ng_obj )`（**连乘，不是求和**），层序**外层→内层**；最后一定 `normalize`（`Ng` 未归一化，且长度同时被 `det` 与缩放污染）。
4. **镜像**：`det(M) < 0` 时决定统一的符号约定（乘 `sign(det)` 或统一用顶点重建约定），并与 `face_forward` 的约定对齐。
5. **`t` 的可比性**：同一射线、同一 `dir` 下 `t` 跨实例、跨层直接可比（这也是 BVH 能全局维护 `tfar` 的前提）；**跨射线不可比**，`t` 也不是距离。
6. **非均匀/剪切实例**：任何“t ↔ 距离”的换算都要带方向（`1/|W d|`）；点查询必须按库的建议退到世界空间（见 4.2）。
7. **不要归一化实例空间里的 `dir`**；若手工实例化，必须原样传递 `tnear/tfar` 并使用 `rcp` 得到的 `W`。
8. **变换必须可逆**（`det ≠ 0`）；库不校验，NaN 会静默扩散。
9. **MB 关键帧矩阵线性插值时，中间时刻不是刚体**；用四元数分解路径。
10. **嵌套深度**受 `RTC_MAX_INSTANCE_LEVEL_COUNT` 限制，超限是“整棵子树消失”而非报错。

---

## 附录 A：数值探针（可复现）

编译（在仓库根目录，只依赖仓库自带的 `common/math`，**不需要**构建 Embree）：

```bash
g++ -std=c++17 -O2 \
    -I common -I common/math -I common/simd -I common/sys \
    probe_report.cpp -o probe_report && ./probe_report
```

探针做两件事：

1. **逐行复刻** `InstanceIntersector1::intersect` 的坐标变换：对每一层执行 `o = xfmPoint(rcp(M), o)`、`d = xfmVector(rcp(M), d)`，**不归一化**，`tnear/tfar` 原样；命中后只恢复 `o/d`。
2. 用与 `kernels/geometry/triangle_intersector_moeller.h` 相同的边约定 `e1 = v0-v1`、`e2 = v2-v0`、`Ng = cross(e2,e1)` 做 Möller–Trumbore 求交，并额外把三角形顶点搬到世界空间求交，作为**独立地面真值**（含 `|t_truth - t_leaf|` 与 `Ng` 的对照）。

```cpp
// Probe for embree-instance-hit.md
// Re-uses ONLY the repository's math headers (common/math) and reproduces the
// exact transform arithmetic of kernels/geometry/instance_intersector.cpp
// plus the triangle edge convention of triangle_intersector_moeller.h.
//
// build: g++ -std=c++17 -O2 -I common -I common/math -I common/simd -I common/sys \
//            probe_report.cpp -o probe_report
#include "math/affinespace.h"
#include <cstdio>
#include <cmath>
#include <vector>
using namespace embree;

struct Tri { Vec3fa v0, v1, v2; };
struct Hit { bool ok=false; float t=0, u=0, v=0; Vec3fa Ng; };

// Moeller-Trumbore with the repository's edge convention:
//   e1 = v0-v1, e2 = v2-v0, Ng = cross(e2, e1)
static Hit isect(const Tri& tr, const Vec3fa& O, const Vec3fa& D, float tn, float tf)
{
  const Vec3fa e1 = tr.v0-tr.v1, e2 = tr.v2-tr.v0, Ng = cross(e2,e1);
  const Vec3fa C = tr.v0-O, R = cross(C,D);
  const float den = dot(Ng,D);
  Hit h; h.Ng = Ng;
  if (den == 0.0f) return h;
  const float sgn = den < 0 ? -1.0f : 1.0f, absDen = std::fabs(den);
  const float U = dot(R,e2)*sgn, V = dot(R,e1)*sgn;
  if (!(U >= 0 && V >= 0 && U+V <= absDen)) return h;
  const float T = dot(Ng,C)*sgn;
  if (!(absDen*tn < T && T <= absDen*tf)) return h;
  h.ok = true; h.t = T/absDen; h.u = U/absDen; h.v = V/absDen;
  return h;
}

// Emulates the chain of InstanceIntersector1::intersect() calls (outer -> inner):
//   push instID; o = xfmPoint(rcp(M), o); d = xfmVector(rcp(M), d)   [no normalize]
// then the leaf is intersected, and only o/d are restored afterwards.
struct Level { AffineSpace3fa M; unsigned geomID; };

static void trace(const char* label, const std::vector<Level>& chain, const Tri& tri,
                  Vec3fa wO, Vec3fa wD, float tnear, float tfar)
{
  Vec3fa O = wO, D = wD;
  printf("\n--- %s ---\n  instID stack : ", label);
  for (size_t i=0;i<chain.size();++i) printf("%u ", chain[i].geomID);
  if (chain.empty()) printf("(empty)");
  printf("\n");
  std::vector<Vec3fa> Os, Ds;                  // per-level copies
  for (const Level& l : chain) {
    const AffineSpace3fa W = rcp(l.M);
    O = xfmPoint(W,O); D = xfmVector(W,D);
    Os.push_back(O); Ds.push_back(D);
  }
  AffineSpace3fa Wstack(one);
  for (const Level& l : chain) Wstack = rcp(l.M) * Wstack;   // W_i * W_o   (inner left-multiplies)
  Hit h = isect(tri, O, D, tnear, tfar);
  if (!h.ok) { printf("  RESULT       : miss\n"); return; }
  printf("  |dir| world/levels : %.6f", length(wD));
  for (size_t i=0;i<Ds.size();++i) printf(" / %.6f", length(Ds[i]));
  printf("\n  tfar         : %.6f\n", h.t);
  printf("  leaf hit     : (% .6f, % .6f, % .6f)\n", (O+h.t*D).x,(O+h.t*D).y,(O+h.t*D).z);
  const Vec3fa pHit = wO + h.t*wD;
  printf("  world hit    : (% .6f, % .6f, % .6f)\n", pHit.x,pHit.y,pHit.z);
  printf("  u,v          : (%.6f, %.6f)\n", h.u, h.v);
  printf("  Ng (object)  : (% .6f, % .6f, % .6f)  |Ng|=%.6f\n", h.Ng.x,h.Ng.y,h.Ng.z,length(h.Ng));
  printf("  per-t distance world/levels : %.6f", length(wD));
  for (size_t i=0;i<Ds.size();++i) printf(" / %.6f", length(Ds[i]));
  printf("\n");

  // ---- independent ground truth: move the triangle to world and intersect there ----
  AffineSpace3fa M(one);
  for (const Level& l : chain) M = M * l.M;
  Tri wt{ xfmPoint(M,tri.v0), xfmPoint(M,tri.v1), xfmPoint(M,tri.v2) };
  Hit g = isect(wt, wO, wD, tnear, tfar);
  printf("  [truth] world triangle: %s t=%.6f\n", g.ok?"hit":"miss", g.ok?g.t:0.0f);
  if (!g.ok) return;
  LinearSpace3fa N(one);
  for (const Level& l : chain) N = N * transposed(rcp(l.M.l));   // outer -> inner
  const Vec3fa nStack = xfmVector(N, h.Ng);
  const Vec3fa nNaive = xfmVector(M, h.Ng);
  printf("  [truth] Ng from world verts        : (% .6f, % .6f, % .6f)\n", g.Ng.x,g.Ng.y,g.Ng.z);
  printf("  xfmNormal stack (outer->inner)     : (% .6f, % .6f, % .6f)\n", nStack.x,nStack.y,nStack.z);
  printf("  det(M)=%.6f ; det(M)*that          : (% .6f, % .6f, % .6f)\n", M.l.det(),
         M.l.det()*nStack.x, M.l.det()*nStack.y, M.l.det()*nStack.z);
  printf("  naive M*Ng                         : (% .6f, % .6f, % .6f)\n", nNaive.x,nNaive.y,nNaive.z);
  float c1 = dot(normalize(nNaive), normalize(g.Ng)); c1 = std::max(-1.f,std::min(1.f,c1));
  printf("  angle(naive, truth)                : %.3f deg\n", std::acos(c1)*180.f/3.14159265f);
  printf("  |t_truth - t_leaf|                 : %.3e\n", std::fabs(g.t - h.t));
  const Vec3fa mapped = xfmPoint(M, O + h.t*D);
  printf("  |M_comp*leaf_hit - (o_w+t*d_w)|    : %.3e\n", length(mapped - pHit));
  printf("  |W_stack - rcp(M_comp)| (max elem) : %.3e\n",
         std::fabs(Wstack.l.vx.x - rcp(M).l.vx.x) + std::fabs(Wstack.l.vy.y - rcp(M).l.vy.y) +
         std::fabs(Wstack.l.vz.z - rcp(M).l.vz.z) + std::fabs(Wstack.p.x - rcp(M).p.x));
}

int main()
{
  const Tri tri   { Vec3fa(0,0,0), Vec3fa(1,0,0), Vec3fa(0,1,0) };  // Ng=(0,0,1)
  const Tri slant { Vec3fa(0,0,0), Vec3fa(1,0,0), Vec3fa(0,1,1) };  // Ng=(0,-1,1)
  const Tri xslant{ Vec3fa(0,0,0), Vec3fa(0,1,0), Vec3fa(1,0,1) };  // Ng=(1,0,-1)

  trace("A) single instance M = T(10,0,0)*S(2)",
        {{AffineSpace3fa::translate(Vec3fa(10,0,0))*AffineSpace3fa::scale(Vec3fa(2)), 7}},
        tri, Vec3fa(10.4f,0.6f,3.0f), Vec3fa(0,0,-1), 0.f, 1e30f);

  {
    const AffineSpace3fa Mo = AffineSpace3fa::translate(Vec3fa(5,0,0))*AffineSpace3fa::scale(Vec3fa(2));
    const AffineSpace3fa Mi = AffineSpace3fa::translate(Vec3fa(0,0,1))*AffineSpace3fa::scale(Vec3fa(0.5f,1,3));
    const Vec3fa leafHit(0.3f,0.35f,0.f), dW(0,0,-1);
    const Vec3fa wHit = xfmPoint(Mo*Mi, leafHit);
    trace("B) nested  outer T(5,0,0)*S(2) [geom 3] / inner T(0,0,1)*S(0.5,1,3) [geom 9]",
          {{Mo,3},{Mi,9}}, slant, wHit - 10.f*dW, dW, 0.f, 1e30f);
  }

  trace("C) non-uniform single instance M = S(4,1,1), triangle Ng=(1,0,-1)",
        {{AffineSpace3fa::scale(Vec3fa(4,1,1)), 1}},
        xslant, Vec3fa(0.3f,0.35f,5.0f), Vec3fa(0,0,-1), 0.f, 1e30f);

  trace("D) mirrored single instance M = S(-1,1,1)",
        {{AffineSpace3fa::scale(Vec3fa(-1,1,1)), 2}},
        xslant, Vec3fa(-0.3f,0.35f,5.0f), Vec3fa(0,0,-1), 0.f, 1e30f);

  // ---- chapter 2.2: conditional coincidence of naive M*n with the correct direction ----
  printf("\n--- C') direction check, M = S(4,1,1) ---\n");
  const AffineSpace3fa M4 = AffineSpace3fa::scale(Vec3fa(4,1,1));
  const Vec3fa ns[3] = { Vec3fa(1,0,-1), Vec3fa(0,-1,1), Vec3fa(1,0,0) };
  for (int i=0;i<3;i++) {
    const Vec3fa a = xfmVector(M4, ns[i]), b = xfmNormal(M4, ns[i]);
    float c = dot(normalize(a),normalize(b)); c = std::max(-1.f,std::min(1.f,c));
    printf("  Ng=(% .1f,% .1f,% .1f)  naive=(% .3f,% .3f,% .3f)  xfmNormal=(% .3f,% .3f,% .3f)  angle=%.2f deg\n",
           ns[i].x,ns[i].y,ns[i].z, a.x,a.y,a.z, b.x,b.y,b.z, std::acos(c)*180.f/3.14159265f);
  }

  // ---- chapter 2.4: order of the per-level normal transforms matters ----
  printf("\n--- 2.3) composite normal order (with rotation) ---\n");
  const AffineSpace3fa Lo = AffineSpace3fa::translate(Vec3fa(5,0,0))*AffineSpace3fa::rotate(Vec3fa(0.3f,1,0.2f),0.7f)*AffineSpace3fa::scale(Vec3fa(2));
  const AffineSpace3fa Li = AffineSpace3fa::translate(Vec3fa(0,0,1))*AffineSpace3fa::rotate(Vec3fa(0,1,0),-0.4f)*AffineSpace3fa::scale(Vec3fa(0.5f,1,3));
  const AffineSpace3fa LE = Lo*Li;
  const LinearSpace3fa No = transposed(rcp(Lo.l)), Ni = transposed(rcp(Li.l));
  const Vec3fa n0(1,0,-1);
  const Vec3fa good = xfmVector(No*Ni, n0), bad = xfmVector(Ni*No, n0), ref = xfmVector(transposed(rcp(LE.l)), n0);
  float cg = dot(normalize(good),normalize(ref)), cb = dot(normalize(bad),normalize(ref));
  cg = std::max(-1.f,std::min(1.f,cg)); cb = std::max(-1.f,std::min(1.f,cb));
  printf("  correct  (No*Ni) = (% .6f, % .6f, % .6f)  angle to (E^-1)^T = %.4f deg\n", good.x,good.y,good.z, std::acos(cg)*180.f/3.14159265f);
  printf("  reversed (Ni*No) = (% .6f, % .6f, % .6f)  angle to (E^-1)^T = %.4f deg\n", bad.x,bad.y,bad.z, std::acos(cb)*180.f/3.14159265f);

  // ---- chapter 2.5: singular transform ----
  printf("\n--- 2.5) singular M = S(1,1,0) ---\n");
  const AffineSpace3fa Ms = AffineSpace3fa::scale(Vec3fa(1,1,0));
  const AffineSpace3fa Ws = rcp(Ms);
  printf("  det=%.3f ; world2local.vz=(%f,%f,%f)\n", Ms.l.det(), Ws.l.vz.x, Ws.l.vz.y, Ws.l.vz.z);

  // ---- chapter 2.6: motion-blur keyframe interpolation ----
  printf("\n--- 2.6a) MB lerp(I, RotZ(90deg), 0.5), pure rotation ---\n");
  {
    const AffineSpace3fa A(one), B = AffineSpace3fa::rotate(Vec3fa(0,0,1), float(M_PI)/2);
    const AffineSpace3fa H = 0.5f*A + 0.5f*B;
    const Vec3fa n(1,0,1);
    printf("  M(0.5)=[[%.3f %.3f 0],[%.3f %.3f 0],[0 0 1]] det=%.3f similarity=%d\n",
           H.l.vx.x,H.l.vy.x,H.l.vx.y,H.l.vy.y,H.l.det(),(int)similarityTransform(H,nullptr));
    printf("  |cols|=(%.3f,%.3f,%.3f) ; xfmNormal=(%.3f,%.3f,%.3f) ; naive M*Ng=(%.3f,%.3f,%.3f)\n",
           length(H.l.vx),length(H.l.vy),length(H.l.vz),
           xfmNormal(H,n).x,xfmNormal(H,n).y,xfmNormal(H,n).z,
           xfmVector(H,n).x,xfmVector(H,n).y,xfmVector(H,n).z);
  }
  printf("\n--- 2.6b) MB lerp(I, T(5,0,0)*RotZ(90)*S(2), 0.5) vs true rigid midpoint T(2.5,0,0)*RotZ(45)*S(2) ---\n");
  {
    const AffineSpace3fa A(one);
    const AffineSpace3fa B = AffineSpace3fa::translate(Vec3fa(5,0,0))*AffineSpace3fa::rotate(Vec3fa(0,0,1), float(M_PI)/2)*AffineSpace3fa::scale(Vec3fa(2));
    const AffineSpace3fa H = 0.5f*A + 0.5f*B;
    const AffineSpace3fa T = AffineSpace3fa::translate(Vec3fa(2.5f,0,0))*AffineSpace3fa::rotate(Vec3fa(0,0,1), float(M_PI)/4)*AffineSpace3fa::scale(Vec3fa(2));
    const Vec3fa n(1,0,1), d(0,0,-1);
    float hi = dot(normalize(xfmNormal(H,n)),normalize(xfmNormal(T,n)));
    hi = std::max(-1.f,std::min(1.f,hi));
    printf("  H cols |vx|=%.3f |vy|=%.3f |vz|=%.3f det=%.3f similarity=%d\n",
           length(H.l.vx),length(H.l.vy),length(H.l.vz),H.l.det(),(int)similarityTransform(H,nullptr));
    printf("  xfmNormal(H,Ng)=(%.4f,%.4f,%.4f) vs true=(%.4f,%.4f,%.4f) -> %.2f deg\n",
           xfmNormal(H,n).x,xfmNormal(H,n).y,xfmNormal(H,n).z,
           xfmNormal(T,n).x,xfmNormal(T,n).y,xfmNormal(T,n).z, std::acos(hi)*180.f/3.14159265f);
    printf("  per-t world distance: interp=%.4f true=%.4f\n",
           1.0f/length(xfmVector(rcp(H),d)), 1.0f/length(xfmVector(rcp(T),d)));
  }
  return 0;
}
```

运行输出（即第 3 章各表的数据来源）：

```

--- A) single instance M = T(10,0,0)*S(2) ---
  instID stack : 7 
  |dir| world/levels : 1.000000 / 0.500000
  tfar         : 3.000000
  leaf hit     : ( 0.200000,  0.300000,  0.000000)
  world hit    : ( 10.400000,  0.600000,  0.000000)
  u,v          : (0.200000, 0.300000)
  Ng (object)  : ( 0.000000, -0.000000,  1.000000)  |Ng|=1.000000
  per-t distance world/levels : 1.000000 / 0.500000
  [truth] world triangle: hit t=3.000000
  [truth] Ng from world verts        : ( 0.000000, -0.000000,  4.000000)
  xfmNormal stack (outer->inner)     : ( 0.000000,  0.000000,  0.500000)
  det(M)=8.000000 ; det(M)*that          : ( 0.000000,  0.000000,  4.000000)
  naive M*Ng                         : ( 0.000000,  0.000000,  2.000000)
  angle(naive, truth)                : 0.040 deg
  |t_truth - t_leaf|                 : 0.000e+00
  |M_comp*leaf_hit - (o_w+t*d_w)|    : 0.000e+00
  |W_stack - rcp(M_comp)| (max elem) : 0.000e+00

--- B) nested  outer T(5,0,0)*S(2) [geom 3] / inner T(0,0,1)*S(0.5,1,3) [geom 9] ---
  instID stack : 3 9 
  |dir| world/levels : 1.000000 / 0.500000 / 0.166667
  tfar         : 7.900000
  leaf hit     : ( 0.300000,  0.350000,  0.350000)
  world hit    : ( 5.300000,  0.700000,  4.100000)
  u,v          : (0.300000, 0.350000)
  Ng (object)  : ( 0.000000, -1.000000,  1.000000)  |Ng|=1.414214
  per-t distance world/levels : 1.000000 / 0.500000 / 0.166667
  [truth] world triangle: hit t=7.900000
  [truth] Ng from world verts        : ( 0.000000, -6.000000,  2.000000)
  xfmNormal stack (outer->inner)     : ( 0.000000, -0.500000,  0.166667)
  det(M)=12.000000 ; det(M)*that          : ( 0.000000, -6.000000,  2.000000)
  naive M*Ng                         : ( 0.000000, -2.000000,  6.000000)
  angle(naive, truth)                : 53.130 deg
  |t_truth - t_leaf|                 : 4.768e-07
  |M_comp*leaf_hit - (o_w+t*d_w)|    : 0.000e+00
  |W_stack - rcp(M_comp)| (max elem) : 0.000e+00

--- C) non-uniform single instance M = S(4,1,1), triangle Ng=(1,0,-1) ---
  instID stack : 1 
  |dir| world/levels : 1.000000 / 1.000000
  tfar         : 4.925000
  leaf hit     : ( 0.075000,  0.350000,  0.075000)
  world hit    : ( 0.300000,  0.350000,  0.075000)
  u,v          : (0.350000, 0.075000)
  Ng (object)  : ( 1.000000,  0.000000, -1.000000)  |Ng|=1.414214
  per-t distance world/levels : 1.000000 / 1.000000
  [truth] world triangle: hit t=4.925000
  [truth] Ng from world verts        : ( 1.000000,  0.000000, -4.000000)
  xfmNormal stack (outer->inner)     : ( 0.250000,  0.000000, -1.000000)
  det(M)=4.000000 ; det(M)*that          : ( 1.000000,  0.000000, -4.000000)
  naive M*Ng                         : ( 4.000000,  0.000000, -1.000000)
  angle(naive, truth)                : 61.928 deg
  |t_truth - t_leaf|                 : 0.000e+00
  |M_comp*leaf_hit - (o_w+t*d_w)|    : 0.000e+00
  |W_stack - rcp(M_comp)| (max elem) : 0.000e+00

--- D) mirrored single instance M = S(-1,1,1) ---
  instID stack : 2 
  |dir| world/levels : 1.000000 / 1.000000
  tfar         : 4.700000
  leaf hit     : ( 0.300000,  0.350000,  0.300000)
  world hit    : (-0.300000,  0.350000,  0.300000)
  u,v          : (0.350000, 0.300000)
  Ng (object)  : ( 1.000000,  0.000000, -1.000000)  |Ng|=1.414214
  per-t distance world/levels : 1.000000 / 1.000000
  [truth] world triangle: hit t=4.700000
  [truth] Ng from world verts        : ( 1.000000,  0.000000,  1.000000)
  xfmNormal stack (outer->inner)     : (-1.000000,  0.000000, -1.000000)
  det(M)=-1.000000 ; det(M)*that          : ( 1.000000, -0.000000,  1.000000)
  naive M*Ng                         : (-1.000000,  0.000000, -1.000000)
  angle(naive, truth)                : 179.966 deg
  |t_truth - t_leaf|                 : 0.000e+00
  |M_comp*leaf_hit - (o_w+t*d_w)|    : 0.000e+00
  |W_stack - rcp(M_comp)| (max elem) : 0.000e+00

--- C') direction check, M = S(4,1,1) ---
  Ng=( 1.0, 0.0,-1.0)  naive=( 4.000, 0.000,-1.000)  xfmNormal=( 0.250, 0.000,-1.000)  angle=61.93 deg
  Ng=( 0.0,-1.0, 1.0)  naive=( 0.000,-1.000, 1.000)  xfmNormal=( 0.000,-1.000, 1.000)  angle=0.03 deg
  Ng=( 1.0, 0.0, 0.0)  naive=( 4.000, 0.000, 0.000)  xfmNormal=( 0.250, 0.000, 0.000)  angle=0.04 deg

--- 2.3) composite normal order (with rotation) ---
  correct  (No*Ni) = ( 0.918486,  0.147988, -0.402816)  angle to (E^-1)^T = 0.0000 deg
  reversed (Ni*No) = ( 0.240731,  0.161912, -0.145528)  angle to (E^-1)^T = 22.6404 deg

--- 2.5) singular M = S(1,1,0) ---
  det=0.000 ; world2local.vz=(-nan,-nan,inf)

--- 2.6a) MB lerp(I, RotZ(90deg), 0.5), pure rotation ---
  M(0.5)=[[0.500 -0.500 0],[0.500 0.500 0],[0 0 1]] det=0.500 similarity=0
  |cols|=(0.707,0.707,1.000) ; xfmNormal=(1.000,1.000,1.000) ; naive M*Ng=(0.500,0.500,1.000)

--- 2.6b) MB lerp(I, T(5,0,0)*RotZ(90)*S(2), 0.5) vs true rigid midpoint T(2.5,0,0)*RotZ(45)*S(2) ---
  H cols |vx|=1.118 |vy|=1.118 |vz|=1.500 det=1.875 similarity=0
  xfmNormal(H,Ng)=(0.4000,0.8000,0.6667) vs true=(0.3536,0.3536,0.5000) -> 16.17 deg
  per-t world distance: interp=1.5000 true=2.0000
```
