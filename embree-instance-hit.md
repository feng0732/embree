# 实例几何体的命中坐标与法线：Embree 4.5.0 静态契约分析

分析对象：本仓库 `main`/`task/16` 分支所 vendor 的 Intel Embree 4.5.0（`README.md:1`，commit `7446fed`）。

本文只做静态契约分析（源码 + 文档 + 测试证据），不做性能跑分。

---

## 0. 结论摘要

| 问题 | 结论 |
|---|---|
| 命中点坐标在哪一空间？ | 没有任何 API 直接返回命中点。`RTCHit` **不包含位置**，只有 `Ng,u,v,primID,geomID,instID[]`。调用方必须用**世界空间**的 `org + tfar*dir` 还原世界坐标。 |
| `tfar` 在哪个空间？ | 是沿**世界空间**射线方向的参数值。实例变换只替换 `org`/`dir`，**不改写** `tfar`，也不改写 `tnear`；因此 `t` 在所有嵌套层级上保持不变。 |
| `Ng` 在哪个空间？ | **最内层 object space**（被实例化场景的局部坐标系），且**未归一化**。见 `doc/src/api/RTCHit.md:28`、`README.md:8567-8568`。 |
| 法线要用什么矩阵变换？ | `(M^-1)^T`（余切/逆转置），即 `xfmNormal()`。只做方向正确性；若要恢复"未归一化"语义，还须乘 `det(M)`。 |
| 非均匀缩放有何影响？ | 不影响 `t`；使 `Ng` 的方向偏离 `M·Ng`，长度被 `det(A)/\|…\|` 扭曲。`Ng` 落在最内层坐标系，所以必须用**累积**法线矩阵变换一次，而不能只套用某一层的矩阵。 |
| 嵌套实例如何合成？ | 逐层用各层 `world2local` 依次变换射线；等价于一次性用 `rcp(M0*M1*…)`。法线累积为 `N0*N1*…`（其中 `Ni = (Mi^-1)^T`），且该乘积**等于** `(M0*M1*…)^-T`。 |
| `instID` 语义？ | `instID[l]` = 第 `l` 层实例**在其父场景**中的 geometry ID（由外向内）。`geomID`/`primID` 属于**最内层**场景。 |
| 深度超限？ | 超过 `RTC_MAX_INSTANCE_LEVEL_COUNT` 的嵌套在 release 下**静默丢弃**（该实例被直接跳过），debug 下断言失败。 |

---

## 1. 三个空间的射线与命中记录

### 1.1 空间定义

| 名称 | 本文含义 | 代码对应 |
|---|---|---|
| **世界空间** (world space) | 顶层场景坐标系；用户传入 `rtcIntersect1` 的 `org`/`dir` 所在空间 | `kernels/common/ray.h:56-59` 的 `Ray::org/dir` |
| **实例空间 / object space** | **被实例化子场景**的局部坐标系。Embree 文档把"命中几何体所处的局部坐标系"一律称为 *object space* | `kernels/geometry/instance_intersector.cpp:30-31` 变换后的 `ray.org/ray.dir` |
| **instance space** | 仅在 point query API 中出现（`RTCPointQueryContext::world2inst`/`inst2world`）。对 ray 查询它没有独立含义，就是"当前层的 object space" | `include/embree4/rtcore_common.h:405-425` |

注意：Embree 的射线查询**没有**一个单独物化的"实例空间"中间坐标系。实例是**逐层压栈**的：每进入一个实例，就把射线变换到该实例所引用子场景的坐标系里，出栈时恢复。因此"嵌套 N 层"意味着有 N 个不同的 object space，但只有"最内层"的那个决定了 `Ng` 与 `u,v` 的含义。

### 1.2 命中记录的结构

`RTCHit`（`include/embree4/rtcore_ray.h:30-45`）：

```c
struct RTC_ALIGN(16) RTCHit {
  float Ng_x, Ng_y, Ng_z;   // geometry normal（object space，未归一化）
  float u, v;               // 重心坐标（最内层 object space）
  unsigned int primID, geomID;
  unsigned int instID[RTC_MAX_INSTANCE_LEVEL_COUNT];      // 实例栈
  unsigned int instPrimID[RTC_MAX_INSTANCE_LEVEL_COUNT];  // 仅 instance array
};
```

关键点：**结构里没有命中点坐标**。命中距离只存在于 `RTCRay::tfar`（`rtcore_ray.h:23`，注释 `set to hit distance`；`doc/src/api/RTCHit.md:32` 明确 "The parametric intersection distance is not stored inside the hit, but stored inside the `tfar` member of the ray"）。因此，"`hit.p`"这个概念在 API 层完全不存在；要得到坐标必须自己算。

### 1.3 坐标关系的完整链条

设某层实例的局部→世界矩阵为 `M = local2world`，则：

```
世界射线:        P_w(t) = org_w + t * dir_w        (t ∈ [tnear, tfar])
进入实例:        org_l  = xfmPoint (M^-1, org_w)
                 dir_l  = xfmVector(M^-1, dir_w)
                 tnear_l = tnear,  tfar_l = tfar   // 原样保留！
物体空间光线:    P_l(t) = org_l + t * dir_l
```

对应源码 `kernels/geometry/instance_intersector.cpp:27-31`（单实例）与 `:29-33`（instance array，`kernels/geometry/instance_array_intersector.cpp:29-33`）：

```cpp
const AffineSpace3fa world2local = instance->getWorld2Local();
const Vec3ff ray_org = ray.org;          // 备份（含 tnear，见下）
const Vec3ff ray_dir = ray.dir;          // 备份（含 time）
ray.org = Vec3ff(xfmPoint (world2local, ray_org), ray.tnear());  // 保留 tnear
ray.dir = Vec3ff(xfmVector(world2local, ray_dir), ray.time());   // 保留 time
```

`Ray` 在标量情况下用 `Vec3ff` 打包：`org.w == tnear`、`dir.w == time`（`kernels/common/ray.h:199-211`）。上面的代码显式把 `tnear`/`time` 重新塞回第四分量，说明**只有前 3 个分量参与变换**。

顶点 `u,v` 与 `primID`：由最内层的最底层 intersector 直接按 object space 三角形/网格计算（例：`kernels/geometry/triangle_intersector_moeller.h:84-108`，`C = tri_v0 - ray.org` 全部在 object space）。它们不需要任何空间变换。

`Ng`：由最内层 intersector 用 object space 边向量叉积得到，`kernels/geometry/triangle_intersector_moeller.h:321-322`：

```cpp
const Vec3vf<K> Ng = cross(e2,e1);   // e1 = v0-v1, e2 = v2-v0（object space）
```

该 `Ng` 经 epilog 原样写入 `RayHit::Ng`，**不做任何实例变换**：`kernels/geometry/intersector_epilog.h:60-66`

```cpp
ray.tfar   = hit.t;
ray.Ng     = hit.Ng;                        // 直接拷贝，仍是 object space
ray.u      = hit.u;
ray.v      = hit.v;
ray.primID = primID;
ray.geomID = geomID;
instance_id_stack::copy_UU(context->user->instID, ray.instID);   // 记录实例栈
```

`hit.t` 是最内层 object space 中 `T/absDen` 的结果（`triangle_intersector_moeller.h:31-34`），但**由于射线方向也被同样变换**，这个 `t` 在数值上就等于世界空间中沿 `dir_w` 的参数值。证明见 §2.1。

### 1.4 出栈与恢复

每条 hit 处理完后立刻恢复世界射线（`instance_intersector.cpp:34-36`）：

```cpp
ray.org = ray_org;
ray.dir = ray_dir;
instance_id_stack::pop(user_context);
```

而 `tfar` 从未被恢复或改写，因此命中后 `tfar` 就是世界空间参数。这解释了教程里的世界坐标还原写法，`tutorials/instanced_geometry/instanced_geometry_device.cpp:205`：

```cpp
Ray shadow(ray.org + ray.tfar*ray.dir, neg(lightDir), 0.001f, inf);
```

以及 `tutorials/multi_instanced_geometry/multi_instanced_geometry_device.cpp:139`：

```cpp
const Vec3fa o = primary.org + primary.tfar * primary.dir;
```

即：**命中点 = 世界空间 `org` + `tfar` × 世界空间 `dir`**，前提是调用方用的是自己传入的那份世界射线。这也与 `doc/src/api/RTCRay.md:34-35`（"The ray direction does not have to be normalized"）自洽——`t` 是参数值，不是欧氏距离。

### 1.5 `instID` 栈的方向与含义

`instance_id_stack::push` 把当前实例的 geometry ID 写在 `instID[instStackSize]` 然后 `++`（`kernels/common/instance_stack.h:27-39`）：

```cpp
context->instID[context->instStackSize] = instanceId;
context->instStackSize++;
```

压栈顺序是"由外向内"，因此 `instID[0]` 是顶层场景中的实例，`instID[1]` 是外层实例所引用场景中的实例，以此类推。文档 `doc/src/api/rtcIntersect1.md:45-52`（及 `README.md:8573-8583`）明确：

> If the instance ID stack has a prefix of values not equal to `RTC_INVALID_GEOMETRY_ID`, the instance ID on each level corresponds to the geometry ID of the hit instance **of the higher-level scene**, the geometry ID corresponds to the hit geometry **inside the hit instanced scene**...

`HitK` 构造时把 context 里的栈拷进 hit（`kernels/common/hit.h:66-69`），并且 `copy_UU` 会按 `instStackSize` 把剩余档位填为 `RTC_INVALID_GEOMETRY_ID`（`instance_stack.h:155-172`），所以"前缀有效、后面全为 INVALID"是可依赖的契约。

对 instance array，`instPrimID[l]` 是实例在该 array 内的索引；`instID[l]` 仍是 array 的 geometry ID（`kernels/geometry/instance_array_intersector.cpp:27`，`prim.instID_`/`prim.primID_`；文档 `doc/src/api/RTC_GEOMETRY_TYPE_INSTANCE_ARRAY.md:56-60`）。

### 1.6 过滤器回调里的空间

这点对写 filter 的人最容易出错。`README.md:5417-5419` 与 `doc/src/api/rtcSetGeometryIntersectFilterFunction.md:68` 明确：

> If the hit geometry is instanced, the `instID` member of the ray is valid, and **the ray and the potential hit are in object space**.

源码印证：filter 在 epilog 内、出栈之前调用（`intersector_epilog.h:46-56`），此时 `ray.org`/`ray.dir` 仍是 object space 版本，`hit.Ng` 也是 object space。filter 里看到的 `ray.tfar` 已经是变换后的 object space 参数——但因为 `t` 不变，它与世界参数在数值上一致（这正是 filter 可以安全地"减小 `tfar` 以拒绝更远命中"的原因，且该修改会被带回外层）。

一旦回到用户代码（`rtcIntersect1` 返回后），`ray.org/dir` 已被恢复成世界空间，而 `hit.Ng` **仍然是 object space**——这是 Embree 契约中唯一的"混合空间"状态，必须由用户自己补齐法线变换。

---

## 2. 嵌套变换与非均匀缩放对距离和法线的影响

### 2.1 距离（`t`）：逐层严格不变

用仓库自带的数学头文件（`common/math/affinespace.h`，可独立编译）验证。复合变换 `M = M0*M1`，`org_l = xfmPoint(M^-1, org_w)`，`dir_l = xfmVector(M^-1, dir_w)`：

```
t=0    world=(1 2 3)            obj=(-1.61087 1.44356 -1.14588)  roundtrip err=4.1e-07
t=0.25 world=(1.075 1.95 3.225) obj=(-1.65861 1.42335 -1.05524)  roundtrip err=5.8e-07
t=1    world=(1.3 1.8 3.9)      obj=(-1.80185 1.36272 -0.783311) roundtrip err=5.5e-07
t=7.5  world=(3.25 0.5 9.75)    obj=(-3.04328 0.837256 1.57341)  roundtrip err=1.1e-06
```

即 `xfmPoint(M, org_l + t*dir_l) == org_w + t*dir_w`（仿射映射对 `t` 线性，误差仅浮点）。同理，`M0`、`M1` 逐层施加（`instance_intersector.cpp:27-31` 每层一次 `xfmPoint`/`xfmVector`）与一次性施加 `rcp(M0*M1)` 完全等价：

```
after level0 (rcp(M0)) org=(0.25 5.125 2.5)
after level1 (rcp(M1)) org=(0.25 0.125 2.5)
one-shot xfmPoint(rcp(M0*M1),org) = (0.25 0.125 2.5)
t still 5 (unchanged at every push)
```

**推论（重要的契约结论）：**

- `t` 不是欧氏距离。当 `dir_w` 未归一化、或被非均匀缩放后，`t * |dir|` 才等于长度。仓库测试就依赖"相对误差 < 1%"而非绝对相等（`tutorials/verify/verify.cpp:3616-3619`）：
  ```cpp
  passed &= (ray0.ray.tfar == float(pos_inf) && ray1.ray.tfar == float(pos_inf))
         || ... || (std::abs(ray0.ray.tfar - ray1.ray.tfar) / std::max(ray0.ray.tfar, ray1.ray.tfar) < 0.01f);
  ```
- **不要**在进入实例前把 `tnear/tfar` 按缩放系数换算。源码不换算（`instance_intersector.cpp:30` 保留 `tnear`），任何用户侧的换算都会引入错误。
- 非均匀缩小 `dir_l` 会让 object space 中的 `tnear/tfar` 与"世界距离"不再对应，但契约上它们是"参数"，所以仍然自洽。
- `occluded` 的 `tfar = -inf` 也是同一套逻辑（`kernels/bvh/bvh_intersector1.cpp:187`，文档 `doc/src/api/rtcOccluded1.md:34`），不受实例影响。

### 2.2 法线：为什么必须用逆转置

object space 的三角面法线是 `Ng_obj = cross(e2, e1)`（`triangle_intersector_moeller.h:321`）。对线性映射 `A`，有恒等式：

```
cross(A u, A v) = det(A) · A^{-T} · cross(u, v)
```

因此世界空间下的对应未归一化法线为：

```
Ng_world = det(A) · A^{-T} · Ng_obj
```

而 `xfmNormal` 只计算 `A^{-T} · Ng_obj`，**不含 `det(A)`**（`common/math/linearspace3.h:161`）：

```cpp
template<typename T> Vec3<T> xfmNormal(const LinearSpace3<T>& s, const T& a) {
  return xfmVector(s.inverse().transposed(), a);
}
```

实地验证（`probe4.cpp`，用仓库头文件，随机三角形 + 多组矩阵），比较"世界空间直接叉积"与 `det(A)·A^{-T}·Ng_obj`：

```
S(2,1,1)        det=  2.000 | world=(0.580 2.080 2.720) | det*A^-T=(0.580 2.080 2.720) | maxerr=0.000e+00
S(2,3,4)        det= 24.000 | world=(6.960 8.320 8.160) | det*A^-T=(6.960 8.320 8.160) | maxerr=1.4e-06
S(-1,1,1) mirror det= -1.000 | world=(0.580 -1.040 -1.360) | det*A^-T=(0.580 -1.040 -1.360) | maxerr=0.000e+00
R*S(1,3,1)*T    det=  3.000 | world=(3.867 -1.087 2.148) | det*A^-T=(3.867 -1.087 2.148) | maxerr=9.5e-07
S(2,2,2) uniform det=  8.000 | world=(2.320 4.160 5.440) | det*A^-T=(2.320 4.160 5.440) | maxerr=0.000e+00
```

所以：

- **方向**：`xfmNormal` 给出的方向与世界叉积法线一致（除镜像情形，见下）。
- **长度**：`xfmNormal` 丢掉了 `det` 因子；`Ng` 本就不保证归一化（`README.md:8567`；`RayHit::Ng` 注释 "not normalized geometry normal"，`kernels/common/ray.h:266`），所以只要调用方做光照前 `normalize`，只关心方向即可。**但若把 `Ng` 当向量参与无归一化的运算（例如某些 LOD / 面积权重），必须补 `det` 或直接在最内层算完。**
- **镜像（`det < 0`）**：`A^{-T}` 会给出**反向**的结果（`S(-1,1,1)` 那行：world 为 `(0.580,-1.040,-1.360)`，而 `A^{-T}` 方向为 `(-0.3209,0.5753,0.7524)`，与世界方向 `(0.3209,-0.5753,-0.7524)` 恰好相反）。这是 Embree 的已知语义：`Ng` 是"未归一化几何法线"，镜像实例会得到朝向翻转的法线；基础 intersector 的朝向由顶点绕序 + 是否 backface culling 决定，不由 `Ng` 的变换修正。用户若要用镜像实例，应自行根据 `det` 修正或统一用 `dot(Ng, dir)` 决定朝向。

### 2.3 非均匀缩放：反例与可观察差异

**反例 1：把法线当普通向量变换（错误做法）。**
`M = S(2,1,1)`，`Ng_obj = normalize(1,1,0)`：

```
object normal      = (0.707107 0.707107 0)
correct  (S^-1)^T  = (0.447214 0.894427 0)
wrong    S         = (0.894427 0.447214 0)
angle(correct,wrong)  = 36.8699 deg
angle(correct,object) = 18.435 deg
```

误差 36.87°，远大于任何数值噪声——这是可肉眼观察的着色差异。这正是为什么 `xfmNormal` 用 `(M^-1)^T` 而不是 `M`。

**反例 2：忘记实例变换、直接在世界空间用 `Ng`。**（即很多初学者的写法 `Ns = ray.Ng`）在世界空间方向偏差 18.44°（上表最后一行），在非均匀缩放下尤其明显；对 uniform scale 则只差一个统一缩放（不影响归一化后方向），所以"只用 uniform scale 时看起来没问题"会掩盖这个 bug。

**反例 3：嵌套时把矩阵乘反（`N1*N0` 而非 `N0*N1`）。** 设 `Ni = (Mi^-1)^T`。正确顺序是 `N0*N1 == (M0*M1)^-T`，即按 `instID` 栈由外层到内层**左乘**。乘反会得到完全不同的法线（`probe3.cpp`）：

```
direct  (M0*M1)^-T      = (0.244959 0.423850 0.226248)
acc N0*N1 (tutorial ord) = (0.244959 0.423850 0.226248)  match=1
acc N1*N0 (other ord)    = (0.101778 0.271487 0.268416)  match=0
```

这是嵌套法线最主要、也最容易犯的错误。`tutorials/multi_instanced_geometry/multi_instanced_geometry_device.cpp:32-42` 的写法是正确顺序：

```cpp
// :32-42  normalTransforms[level] 里存的是 transposed(rcp(M.l))（scene.cpp:165）
transform = transform * LinearSpace3fa(M);   // 依次左乘，外层 → 内层
...
// :161  末尾归一化一次
Vec3fa Ns = normalize(xfmVector(xfm, Vec3fa(primaryRay.Ng)));
```

**关于"逐层归不归一化"**：这一点常被误解为会影响方向，其实**不会**。`normalize()` 只是乘以一个正标量，标量与矩阵乘法可交换，故"每层归一化"与"最后归一化一次"在方向上严格等价（`probe5.cpp` 实测差 2e-2°，纯属 `rcp` 浮点噪声）：

```
single-end normalize = (0.310668 -0.359274 0.880004)
per-level normalize  = (0.310668 -0.359274 0.880004)     angle = 1.98e-02 deg
(M0*M1)^-T direct    = (0.310668 -0.359274 0.880004)     angle = 1.98e-02 deg
```

所以"最后归一化一次"只是**省算力**的写法，不是正确性要求；真正的正确性要求是**矩阵顺序**与**用逆转置而非 `M`**。若为了省算力而在每层都归一化，方向不会错。

**非均匀缩放对长度的影响。** 即使方向对，`Ng` 的长度也编码了面积缩放：`probe.cpp` 中 `S(2,1,1)` 下去变换 `(0,0,1)` 得到 `(0,0,1)`、长度 1；而 `S(2,3,4)` 下 `(0,0,1)` 变为长度 `1/4`（因为 `A^{-T}` 的 z 分量是 `1/4`，而 `det=24` 时世界法线长度应为 `det/4 = 6`）。任何把 `Ng` 长度当作"面积权重"的算法都必须显式乘 `det`，否则在非均匀缩放下会静默错 6 倍。

### 2.4 嵌套实例：法线的累积顺序

用仓库头文件验证：设 `N0 = (M0^-1)^T`、`N1 = (M1^-1)^T`，则 `N0*N1 == (M0*M1)^-T`：

```
direct  (M0*M1)^-T      = (0.244959 0.42385 0.226248)
acc N0*N1 (tutorial ord) = (0.244959 0.42385 0.226248)  match=1
acc N1*N0 (other ord)    = (0.101778 0.271487 0.268416)  match=0
```

所以 `tutorials/multi_instanced_geometry` 的累积顺序 `transform = transform * M`（外层先、内层后；`scene.cpp:165` 存 `transposed(rcp(M.l))`，`device.cpp:32-42` 累乘）是**正确**的；反过来乘会得到完全不同的、错误的法线（上表第三行）。这与 `instID[0]` 为最外层实例一致：按栈顺序从 0 到最深逐层左乘。

一个重要细节：**逐层用各层自己的独立矩阵做变换并不总是等价于"世界空间直接叉积"**——只有当每层都不含镜像时，方向才一致；长度则需要累积 `det` 才能与"世界空间叉积"逐位吻合。若只求方向（最常用于着色），累积 `(M^-1)^T` 再归一化即可。

### 2.5 点查询（point query）中的相似变换契约

point query 是唯一显式区分"相似变换 vs 非相似变换"的 API，可作为理解非均匀缩放影响的补充证据。`similarityTransform`（`common/math/affinespace.h:321-339`）判定正交性 + 各轴等长：

```cpp
if (abs(dot(M.l.vx, M.l.vz)) > 1e-5f) return false;
...
const float D_x = dot(M.l.vx, M.l.vx);  // ...
if (abs(D_x - D_y) > 1e-5f || ...) return false;
if (D) *D = sqrtf(D_x);
```

`InstanceIntersector1::pointQuery`（`kernels/geometry/instance_intersector.cpp:74-97`）：

```cpp
float similarityScale = 0.f;
const bool similtude = context->query_type == POINT_QUERY_TYPE_SPHERE
                     && similarityTransform(world2local, &similarityScale);
...
query_inst.p      = xfmPoint (world2local, query->p);
query_inst.radius = query->radius * similarityScale;     // 相似变换才能这样缩放半径
```

非相似时 `similarityScale = 0`，半径不做缩放，转而用**世界空间的 AABB 近似**（`kernels/common/context.h:142-153`，`updateAABB()` 用 `xfmBounds` 变换包围盒），文档见 `README.md:6203-6212`（"the resulting ellipsoidal query domain (in instance space) is approximated by its axis aligned bounding box internally"）。

用仓库头文件验证 `S(2,1,1)`：

```
similarityTransform(rcp(S)) = 0, scale = 0 -> point query falls back to AABB
```

这正是 §2.3 反例的"官方承认"：非均匀缩放下，半径/距离不能在实例空间里用单一标量缩放，Embree 选择退回保守的 AABB 近似。**射线查询没有这条退路——它对任意仿射变换都做精确的 object space 求交，代价是法线和距离必须在正确的空间里解释。**

---

## 3. 三类场景的可观察结果与证据

### 3.1 单层实例

**代码路径**（`kernels/geometry/instance_intersector.cpp:15-38`）：压栈 → `world2local` → 变换 `org`/`dir` → 递归 `intersect` → 恢复 → 出栈。

**可观察结果**（`probe.cpp`，`M = T(10,0,0)·S(2)`，射线 `org=(10.5,0.25,5)`、`dir=(0,0,-1)`）：

```
world   org=(10.5 0.25 5) dir=(0 0 -1)
object  org=(0.25 0.125 2.5) dir=(0 0 -0.5)
t=5 : world point=(10.5 0.25 0)      -> 命中点 = org_w + 5*dir_w
t=5 : obj   point=(0.25 0.125 0)     -> L2W(obj) = (10.5 0.25 0)  （一致）
world distance |p-org| = 5 ; |dir_w|*t = 5 ; object dist = 2.5
Ng_world = (0 0 0.5) |.|=0.5 -> normalize -> (0 0 1)
similarityTransform(W2L) = 1, scale = 0.5
```

要点：
- 世界命中点由 `org_w + tfar*dir_w` 还原，**不是** `xfmPoint(M, org_l + t*dir_l)` 之外的东西——两者数值一致。
- object space 里走的是"更短"的距离（2.5 而非 5），但 `t` 相同；这直接证明 `t` 是参数而非世界距离。
- `Ng` 长度 0.5 而非 1，说明"未归一化"是真实的、可观察的。

**测试证据**：
- `tutorials/verify/verify.cpp:3116-3126`（`InstancingTest`，`RTC_MAX_INSTANCE_LEVEL_COUNT` 层均为 `AffineSpace3fa(one)`）断言 `numHits > 0`、`geomID != RTC_INVALID_GEOMETRY_ID`；occlusion 变体断言 `tfar == -inf`。
- `tutorials/verify/verify.cpp:3209-3211`（`InstanceArrayTest`）断言 `instID[0] == 0`、`instPrimID[0] == (i-1)`，即"实例数组的 geometry ID + 索引"语义。
- `tutorials/instanced_geometry/instanced_geometry_device.cpp:187-195` / `.ispc:185-193` 展示标准补法线写法：`Ns = ray.Ng; xfmNormal(xfm, Ns); normalize(Ns);`。
- 该测试的构造方式（`verify.cpp:3083-3086`）值得注意：`for (int i = 0; i < RTC_MAX_INSTANCE_LEVEL_COUNT; ++i) parent = new TransformNode(one, child);`——它**按编译期上限**嵌套，依赖 `one` 变换下 `Ng`/`t` 不变来与直接求交对拍。

### 3.2 嵌套实例

**代码路径**：每层 `InstanceIntersector1::intersect` 再递归一次；`instance_id_stack::push` 每次压一层（`instance_stack.h:27-39`）；栈满时 `spaceAvailable=false`，该层直接跳过（`instance_intersector.cpp:25` 的 `if (likely(push(...)))`）。

**可观察结果**（`probe.cpp`，`M0 = T(10,0,0)·S(2)`，`M1 = T(0,5,0)`）：

```
world org=(10.5 10.25 5)
after level0 (rcp(M0)) org=(0.25 5.125 2.5)
after level1 (rcp(M1)) org=(0.25 0.125 2.5)
one-shot xfmPoint(rcp(M0*M1),org) = (0.25 0.125 2.5)   ← 逐层 == 一次性
t still 5 (unchanged at every push)
Ng_world nested = (0 0 0.5) normalize=(0 0 1)
accumulate normal xfm as M0^-T * M1^-T -> (0 0 0.5)     ← 与 (M0*M1)^-T 一致
```

**测试证据与教程证据**：
- `tutorials/verify/verify.cpp:3083-3086, 3116-3126`：以 `RTC_MAX_INSTANCE_LEVEL_COUNT` 层嵌套、全部单位变换，验证多层压栈不破坏命中/遮挡。
- `tutorials/verify/verify.cpp:3484-3633`（`InstanceArrayRandomTest`）：同一批随机变换分别构造"instance array"与"N 个独立 instance 场景"，要求 `rtcGetGeometryTransform` vs `rtcGetGeometryTransformEx` 一致（`check_transform_almost_equal`，`:3397`）、场景包围盒一致（`:3538-3558`），并且 `tfar` 相对误差 < 1%（`:3616-3619`）。这是**多层/多实例距离一致性**最直接的仓库内证据。
- `tutorials/multi_instanced_geometry/scene.cpp:205-226` 真正构造两层（twig → tree → scene，`addInstances(..., tree, ...)` 与 `addInstances(..., scene, ...)`），`scene.cpp:165` 每层存 `transposed(rcp(M.l))`，`device.cpp:32-42` 按 `instID` 栈累乘，`:161` 最后一次归一化。文档 `README.md:10820-10838` 说明"两个层级的实例：树的多个实例嵌套 twig 的实例"，并要求 `EMBREE_MAX_INSTANCE_LEVEL_COUNT > 1`。
- 注意仓库默认 `EMBREE_MAX_INSTANCE_LEVEL_COUNT=1`（`CMakeLists.txt:200`），因此多层行为**只有显式改配置**才生效；文档 `README.md:4086-4090` 与 `doc/src/api/RTC_GEOMETRY_TYPE_INSTANCE.md:21-25` 明确"更深的嵌套在 release 下被静默忽略、debug 下断言"。

### 3.3 非均匀缩放反例（可观察差异的定量结果）

**反例 A：法线用错矩阵**（详见 §2.3 反例 1）
- 观察值：`S(2,1,1)` 下 `xfmNormal` 与 `S·n` 相差 **36.87°**；与世界空间直接叉积法线相差 **18.44°**（`probe.cpp`）。
- 影响面：世界空间光照直接错了 37°，在漫反射着色下表现为明暗边界偏移。

**反例 B：长度语义丢失**
- 观察值：`S(2,3,4)` 下 `(0,0,1)` 经 `xfmNormal` 长度为 `0.25`，而世界空间真实未归一化法线长度应为 `det/4 = 6`，**相差 24 倍**（`probe4.cpp` 的 `maxerr` 对比；`xfmNormal` 只给 `A^-T`，世界叉积给 `det·A^-T`）。
- 影响面：把 `Ng` 长度当面积权重的算法在非均匀缩放下静默错误。

**反例 C：镜像实例（`det < 0`）朝向翻转**
- 观察值：`S(-1,1,1)` 下 `xfmNormal` 方向与世界法线**恰好相反**（`probe4.cpp`：`A^-T` 方向 `(-0.3209,0.5753,0.7524)` vs 世界方向 `(0.3209,-0.5753,-0.7524)`）。
- 影响面：镜像实例的背面/正面判断翻转，需要用户按 `det` 修正或统一以 `dot(Ng,dir)` 定朝向。

**反例 D：点查询半径无法单标量缩放**
- 观察值：`similarityTransform(S(2,1,1)) == false, scale == 0`（`probe.cpp`），point query 退回 `POINT_QUERY_TYPE_AABB`（`instance_intersector.cpp:76-93`）。
- 影响面：回调会被"查询域外"的图元触发（保守但多余），文档 `README.md:6203-6212` 明示这是设计取舍。

**测试证据的边界**：仓库中**没有**专门的"非均匀缩放法线正确性"单元测试。`tutorials/verify/verify.cpp:3320-3326` 的 `getRandomTransform` 确实生成带随机各轴缩放的变换（`scale = Vec3fa(0.5,0.5,0.5) + random*Vec3fa(1.5,1.5,1.5)`，即 `[0.5,2.0]^3` 各轴独立），但这些测试（`InstanceArrayRandomTest`）只对拍**变换/包围盒/`tfar`**，**没有断言 `Ng`**。同一文件的 `Ng` 断言都作用在**非实例化**几何上：`TriangleHitTest`（`:2658`）在 `:2734-2737` 断言 `org + tfar*dir` 与重心坐标还原点一致、`Ng` 恒为 `(0,0,1)`；`QuadHitTest`（`:2745`）在 `:2810-2814` 做同样的事；`BackfaceCullingTest`（`:2890`）在 `:2944-2949` 只断言 `dot(dir,Ng) < 0`。

不过这里有两条**反向**的强证据，说明"命中点必须用世界射线 + `tfar` 还原"不只是理论：上述 `:2733-2735` / `:2810-2812` 的写法正是
```cpp
const Vec3fa ht  = org + rays[i].ray.tfar*dir;      // 世界射线的 org/dir
const Vec3fa huv = vertices[0] + u*(v1-v0) + v*(v2-v0);  // object space 重心插值
if (reduce_max(abs(ht-huv)) > 16.0f*ulp) FAILED;    // 二者必须相等
```
它在**非实例化**场景下把两个空间当成同一个空间来断言相等；一旦几何被实例化，这个等式**只在把 object space 的重心插值再经 `local2world` 变换后才成立**（即 `ht_world == xfmPoint(M, huv_object)`），这恰好是 §1.3 链条的逆运算。也就是说，仓库的测试在这个等式上"沉默地"依赖了"未实例化 ⇒ 两空间重合"这一前提，而没有实例化版本的对拍——这正是缺口所在。

结论：非均匀缩放下 `Ng` 的语义是**文档 + 实现**层面的契约（`doc/src/api/RTCHit.md:28`、`README.md:8567`、`kernels/geometry/intersector_epilog.h:61`），而非仓库内回归测试覆盖的行为。

---

## 4. 契约速查（实践建议）

1. **命中点**：只能用 `org_world + tfar * dir_world` 还原；`RTCHit` 无位置字段。
2. **`t`**：世界参数，跨实例层级不变；不要按缩放换算 `tnear/tfar`。
3. **`Ng`**：最内层 object space、未归一化。用法线矩阵 `(M^-1)^T` 按 `instID` 栈**由外向内左乘**累积（`N0*N1*… == (M0*M1*…)^-T`），最后归一化一次（归一化位置不影响方向，只影响算力）。
4. **`u,v,primID,geomID`**：最内层场景的语义，无需变换。
5. **`instID[l]`**：第 `l` 层实例在其父场景中的 geometry ID，由外向内；`instPrimID[l]` 仅 instance array。
6. **`instID` 为主**：用 `instID[l]` 去 `rtcGetGeometryTransformFromTraversable`/`FromScene` 取该层 `local2world`（`tutorials/instanced_geometry/instanced_geometry_device.cpp:191`），再 `xfmNormal`。
7. **filter 回调**：`ray`/`hit` 都在 object space；出栈后 `Ng` 仍是 object space，`org/dir` 已恢复为 world space。
8. **深度上限**：默认 1；需要嵌套时显式 `-DEMBREE_MAX_INSTANCE_LEVEL_COUNT=N`，否则深层实例在 release 下被静默跳过。
9. **镜像实例**：`det<0` 时 `Ng` 朝向翻转，需自行修正。
10. **只用 uniform scale 的假象**：uniform 缩放下方向正确会掩盖"从未做法线变换"的 bug；要验证请用非均匀缩放。

---

## 5. 关于本环境的可执行性说明

本仓库是 vendored snapshot，**缺少若干被 `#include` 引用的头文件**，因此 `libembree` 无法在本环境编译：

```
$ cmake --build . --target embree        # configure 成功，编译失败
kernels/geometry/../common/scene.h:8:10: fatal error: builder.h: No such file or directory
$ git cat-file -e HEAD:kernels/common/builder.h
fatal: path 'kernels/common/builder.h' does not exist in 'HEAD'
```

缺失清单（对 `kernels/` 做 include 解析所得）：`kernels/common/builder.h`（被 `scene.h`、`accelset.h`、`accelinstance.h`、`builders/bvh_builder_morton.h` 引用）、`kernels/common/primref.h`、`kernels/geometry/bezier_{hair,ribbon,curve}_intersector.h`、`kernels/geometry/oriented_curve_intersector.h`、`kernels/sycl/rtcore.h`。

因此本文所有数值均来自**用仓库自带数学头文件**（`common/math/affinespace.h`、`linearspace3.h`，它们可独立编译）复现 `instance_intersector.cpp:27-31` 与 `intersector_epilog.h:60-66` 的精确运算，而非链接 `libembree` 的端到端跑分/跑测。复现程序位于 `/tmp/embree-probe/{probe,probe2,probe3,probe4}.cpp`；运行输出已逐条引用在正文。源码引用均给出 `file:line`，可直接核对。

---

## 附录：主要引用索引

| 主题 | 引用 |
|---|---|
| 命中结构（无位置、`Ng` 为 object space） | `include/embree4/rtcore_ray.h:30-45`；`doc/src/api/RTCHit.md:26-33`；`README.md:8567-8568` |
| 射线结构（`tfar` = 命中距离） | `include/embree4/rtcore_ray.h:23`；`doc/src/api/RTCRay.md:34-35` |
| 射线变换（保留 `tnear`/`time`） | `kernels/geometry/instance_intersector.cpp:27-36` |
| instance array 射线变换 | `kernels/geometry/instance_array_intersector.cpp:29-38` |
| `world2local` 的来源 | `kernels/common/scene_instance.cpp:145-156`；`scene_instance.h:128-154`；`scene_instance_array.h:169-175` |
| object space 法线计算 | `kernels/geometry/triangle_intersector_moeller.h:31-34, 321-322` |
| `Ng` 直接拷贝进命中、`instID` 拷贝 | `kernels/geometry/intersector_epilog.h:59-69` |
| `(M^-1)^T` 定义 | `common/math/linearspace3.h:161`；`common/math/affinespace.h:103-104` |
| 相似变换判定 | `common/math/affinespace.h:321-339` |
| 栈语义/溢出 | `kernels/common/instance_stack.h:27-39, 155-172`；`include/embree4/rtcore_common.h:334-344` |
| 栈方向文档 | `doc/src/api/rtcIntersect1.md:45-52`；`README.md:8573-8583` |
| filter 的 object space 契约 | `README.md:5417-5419`；`doc/src/api/rtcSetGeometryIntersectFilterFunction.md:68` |
| point query 非相似退回 AABB | `kernels/geometry/instance_intersector.cpp:74-93`；`kernels/common/context.h:142-153`；`README.md:6203-6212` |
| 深度上限默认 1 / 静默丢弃 | `CMakeLists.txt:200`；`README.md:4086-4090` |
| 单层实例测试 | `tutorials/verify/verify.cpp:3035-3131` |
| instance array 语义测试 | `tutorials/verify/verify.cpp:3140-3226` |
| 嵌套/随机变换一致性测试 | `tutorials/verify/verify.cpp:3484-3633` |
| 多层教程（法线累积） | `tutorials/multi_instanced_geometry/scene.cpp:205-226`；`.../multi_instanced_geometry_device.cpp:32-42, 161` |
| 单层教程（`xfmNormal` 用法） | `tutorials/instanced_geometry/instanced_geometry_device.cpp:187-195` |
| 世界坐标还原 | `tutorials/instanced_geometry/instanced_geometry_device.cpp:205`；`tutorials/multi_instanced_geometry/multi_instanced_geometry_device.cpp:139` |
