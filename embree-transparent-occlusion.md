# Embree 透明表面的过滤回调与遮挡查询契约分析

> 分析对象：本仓库 vendored 的 Embree 源码（`include/embree4`，即 Embree 4 API；
> 目录名中的 "embree-3" 为任务编号，非版本号）。
> 主题：当透明表面参与射线遍历时，`rtcIntersect`（最近命中）、
> 交集过滤回调（拒绝后继续遍历）与 `rtcOccluded`（遮挡查询早停）
> 三者在**可观察可见性**与**命中状态**上的契约差异。

## 1. 三条执行路径的源码契约

### 1.1 普通最近命中（无过滤回调）

当几何体未注册过滤回调、且上下文未提供参数级过滤器时，
`Intersect1Epilog1` 直接提交命中（`kernels/geometry/intersector_epilog.h:60` 起）：

```cpp
ray.tfar = hit.t;   // 收缩射线区间
ray.Ng/u/v/primID/geomID = ...;
```

可观察行为：遍历结束后 `RTCRayHit` 中保存的是 `[tnear, tfar]` 区间内的
**最近一个**命中；`tfar` 在遍历过程中单调收缩，后续只接受更近了命中。
若 `hit.geomID == RTC_INVALID_GEOMETRY_ID` 则表示未命中。

### 1.2 交集过滤回调：拒绝后继续遍历

注册 `rtcSetGeometryIntersectFilterFunction` 后，每个候选命中先经过
`runIntersectionFilter1`（`kernels/geometry/filter.h:38`）。关键在
`intersector_epilog.h:49-55` 的临时提交/回滚协议：

```cpp
const float old_t = ray.tfar;
ray.tfar = hit.t;                              // 临时提交：过滤器经 ray->tfar 读到命中距离
bool found = runIntersectionFilter1(geometry,ray,context,h);
if (!found) ray.tfar = old_t;                  // 拒绝：恢复区间，遍历继续
```

契约要点：

- 过滤器通过 `args->valid[i] = 0` 拒绝命中（`include/embree4/rtcore_common.h:313`
  的 `RTCFilterFunctionNArguments`），拒绝后 `tfar` 回滚，**遍历以原区间继续**，
  因此同一射线上更远的表面（透明层之后的层）仍会被报告。
- 过滤器接受命中时允许改写 hit 数据、**减小** `tfar`，但不得修改其他射线数据
  （`doc/src/api/rtcSetGeometryIntersectFilterFunction.md`）。
- 命中距离通过 `ray->tfar` 传给过滤器；实例化场景下射线与候选命中均处于
  **对象空间**（同上文档）。
- 这正是"透明层叠加"的官方惯用法：过滤器把透明命中拒绝掉（或累积透射率后拒绝），
  让遍历穿透到下一层。`tutorials/verify/verify.cpp:4852` 的
  `SphereFilterMultiHitTest::countHits` 即以此收集多重命中——记录命中点后
  无条件 `args->valid[0] = 0`（`verify.cpp:4858`），迫使遍历继续，
  单条 `rtcIntersect1` 即可收集两个球面的全部 2/4 个交点
  （是否 4 个取决于 `RTC_DEVICE_PROPERTY_BACKFACE_CULLING_ENABLED`）。

### 1.3 遮挡查询：接受即早停

`rtcOccluded` 路径使用 `Occluded1Epilog1`（`intersector_epilog.h:74` 起）与
`runOcclusionFilter1`（`filter.h:73`）。契约与交集路径根本不同：

- 过滤器**拒绝**（`valid=0`）→ 遍历继续，与交集路径类似；
- 过滤器**接受**（或几何体无过滤器）→ 候选命中被"提交"，标量遍历立即在
  `kernels/bvh/bvh_intersector1.cpp:187` 执行 `ray.tfar = neg_inf; break;`
  **终止整条遍历**；SIMD 包路径在 `filter.h:139` 对被接受 lane 做
  `ray->tfar = select(valid_o, neg_inf, ray->tfar)`。
- 实例化场景下，内层场景遍历结束后以 `occluded = ray.tfar < 0.0f`
  判定遮挡并向上传播（`kernels/geometry/instance_intersector.cpp:63`，
  同样的模式见 152/244/303 行）。

可观察结果只有一个布尔量：`ray.tfar == -inf` 表示"被遮挡"，否则未被遮挡。
`RTCRay` 不含 hit 字段，调用者拿不到遮挡者的 primID/geomID/距离。

官方文档明确警告（`doc/src/api/rtcSetGeometryOccludedFilterFunction.md`）：
由于提交的命中直接结束遍历，遮挡过滤器对 `tfar` 的修改无法影响后续遍历，
**`rtcOccluded` + 遮挡过滤器不能用来收集前 n 个命中**——那应当用
`rtcIntersect` + 交集过滤器。

### 1.4 透明阴影的参考实现

`tutorials/intersection_filter/intersection_filter_device.cpp` 展示了
两条路径如何配合表达"透明"：

- 主射线（`rtcIntersect` 系）：`intersectionFilter` 计算透明度 `T`，
  全透明（`T >= 1.0`）时 `valid[0] = 0` 拒绝（`intersection_filter_device.cpp:176`），
  否则接受并把 `T` 存入 `context->userRayExt`；着色循环再以
  `tnear = 1.001*tfar` 发射透射射线继续追踪（同文件 116-121 行）。
- 阴影射线（`rtcOccluded` 系）：`occlusionFilter` 把各层透射率**乘性累积**
  到 `userRayExt`，只要还有光透过（`T != 0.0f`）就拒绝命中让遍历继续
  （同文件 208-209 行）；只有完全不透明（`T == 0`）才接受命中、触发早停。
  最终 `shadow.tfar >= 0` 表示"未被完全遮挡"，用累积透射率调制光照
  （同文件 110-113 行）。

## 2. 边界情形分析

### 2.1 相同距离的命中（共面/共边表面）

三角形求交的上界是**闭区间**：`T <= absDen*ray.tfar`
（`kernels/geometry/triangle_intersector_moeller.h:104`，SIMD 版见 292 行）。
由此产生三个可观察后果：

| 路径 | 两个等距命中（距离 t）的行为 | 可观察状态 |
|---|---|---|
| 最近命中（无过滤） | 第一个命中把 `tfar` 收缩为 t 后，第二个仍满足 `t <= tfar`，**两者都会被提交**，后者覆盖前者 | `RTCRayHit` 中是遍历顺序上**最后**报告的那个 primID/geomID；平局时结果依赖 BVH 遍历顺序，调用者不应假设稳定性 |
| 交集过滤器 | 两个命中各自触发一次过滤回调（各自看到 `ray->tfar == t`） | 若过滤器按"拒绝透明层"逻辑工作，两层透明共面时**两层都会被访问**，累积结果与层数一致；若过滤器有副作用（如计数），副作用发生两次 |
| 遮挡查询 | 第一个**被接受**的命中立即 `tfar = -inf` 终止遍历 | 等距的第二层**不会再触发回调**；对二值可见性无影响，但若遮挡过滤器带累积语义（如乘性透射率），共面重复层只会计入一次——与交集路径的"每层都计入"不对称 |

文档还提示（`rtcSetGeometryIntersectFilterFunction.md`）：`RTC_BUILD_QUALITY_HIGH`
下同一图元命中可能被多次报告，恰好命中共享边的射线可能对同一表面报告两次
（geomID/primID 对重复）。需要"每层只计一次"语义的累积型过滤器必须自行按
`(geomID, primID)` 去重。

### 2.2 透明层叠加（多层半透明表面沿同一射线）

- **最近命中**：只返回最前面一层；`tfar` 被收缩到第一层距离，后面的层完全不可见。
  对透明材质这是错误结果——调用者必须像 tutorial 那样手动偏移 `tnear`
  重发射线，或改用过滤器。
- **交集过滤器（拒绝继续）**：每一层都会触发一次回调，遍历持续到区间耗尽或某层被
  接受。`SphereFilterMultiHitTest`（`verify.cpp:4828-4920`）是该语义的回归测试：
  两个球、一条射线，回调被调用 4 次（关闭背面剔除时），命中点按遍历顺序进入
  `userRayExt` 指向的数组。注意：回调触发顺序是 **BVH 遍历顺序**，
  虽然测试中对单光线场景观察到的是由近及远，但 Embree 并未在 API 层面
  承诺排序；需要严格前后序的合成（如非交换的 over 运算）应在回调中按
  `tfar` 自行排序或缓存后处理。乘性透射率累积（tutorial 的做法）对顺序不敏感，
  是安全的。
- **遮挡查询**：只要所有层都半透明（过滤器持续拒绝），遍历同样穿透所有层，
  累积透射率与交集路径一致；一旦某层被判为不透明（过滤器接受），遍历立即
  终止，**其后更远的层不再参与累积**。由于遮挡查询只关心"光是否到达"，
  提前终止是正确且高效的：累积透射率归零后结果不再改变。

### 2.3 实例化几何

实例求交把射线变换到实例对象空间后递归进入内层场景
（`kernels/geometry/instance_intersector.cpp:13-38`），期间
`instID` 被压入实例栈。三种路径的表现：

- **最近命中**：内层提交的命中携带完整实例栈（`HitK` 构造时从
  `context->instID` 拷贝，`kernels/common/hit.h:30-32`），
  `hit.instID[0..]` 可观察；方向向量未归一化地随矩阵变换，因此参数距离
  `t` 在世界空间与对象空间一致，`tfar` 收缩语义跨实例边界无缝成立。
- **交集过滤器**：回调在**对象空间**被调用（`rtcSetGeometryIntersectFilterFunction.md`
  明确说明 "the ray and the potential hit are in object space"），
  此时 `instID` 有效。透明判定若依赖世界空间位置（如 tutorial 的
  3D 程序化透明度 `transparencyFunction(h)`），必须自行用实例变换把
  命中点变换回世界空间，否则同一实例的不同摆放会算出相同的透明度图案——
  这是实例化 + 透明组合下最容易踩的坑。
- **遮挡查询**：内层遍历的早停（`tfar = -inf`）通过
  `instance_intersector.cpp:63` 的 `ray.tfar < 0.0f` 检查翻译为布尔量
  向外传播，外层遍历随即终止。可观察结果仍是单个布尔值；遮挡过滤器的
  累积状态（`userRayExt`）在内层、外层回调间共享同一上下文指针，
  跨实例边界的乘性累积自然成立。多层嵌套实例（`RTC_MAX_INSTANCE_LEVEL_COUNT`）
  下每一层都重复同样的变换/早停传播。
- 回归材料：`verify.cpp` 的 `InstancingTest`（约 3041 行起）在
  `RTC_MAX_INSTANCE_LEVEL_COUNT` 层嵌套实例上注册参数级交集过滤器
  （`args.filter = intersectFilter`，配合
  `RTC_SCENE_FLAG_FILTER_FUNCTION_IN_ARGUMENTS` 与
  `RTC_RAY_QUERY_FLAG_INVOKE_ARGUMENT_FILTER`），验证 16 条射线在深层
  实例栈下每条恰好收到预期次数的回调——过滤器在实例化场景中的触发
  次数契约是有测试保障的。

## 3. 结果对比小结

| 维度 | 最近命中 | 交集过滤器（拒绝继续） | 遮挡查询（接受早停） |
|---|---|---|---|
| 终止条件 | 区间 `[tnear,tfar]` 耗尽 | 同左（拒绝不改区间） | 任一命中被接受即 `tfar=-inf` |
| 命中记录 | 完整 `RTCHit`（最近一个） | 完整 `RTCHit`（最近被接受者）；被拒层仅回调内可见 | 无，仅布尔（`tfar` 符号） |
| 等距命中 | 后者覆盖前者，均触发提交 | 每层各触发一次回调 | 首个被接受层之后不再触发 |
| 透明层叠加 | 只见最前层 | 全部层可见，可累积 | 全部半透明层可见，可累积；遇不透明层即止 |
| 实例化 | `instID` 栈完整 | 回调处于对象空间，`instID` 有效 | 布尔结果跨层传播，累积状态共享 |
| 适用语义 | 不透明最近表面 | 多重命中 / 透明合成 / 轮廓裁剪 | 二值可见性、透射阴影 |

## 现状

当前实现的三条路径各自契约清晰且自洽：交集过滤器通过"临时提交
`tfar`—回调—拒绝则回滚"的协议（`intersector_epilog.h:50-53`）支持穿透式
遍历，遮挡查询通过"接受即 `tfar = -inf`"的协议（`bvh_intersector1.cpp:187`、
`filter.h:139`）换取最早的终止点，实例化边界上两者分别靠实例栈拷贝与
`tfar` 符号传播维持语义（`instance_intersector.cpp:63`）。但契约分散在
epilog 模板、BVH 遍历器、实例求交器与两份 man 页之间，且存在若干
文档才明确的非直觉点：等距命中的闭区间判定（`T <= tfar`）导致平局覆盖
与重复回调、过滤器回调顺序不保证按距离排序、实例化下回调处于对象空间、
遮挡过滤器无法用于收集 n 重命中。测试材料（`SphereFilterMultiHitTest`、
`InstancingTest`）覆盖了"回调触发次数"，未覆盖等距平局、共面透明层
与对象空间透明度求值这些边界组合。

## 建议

1. 把"透明表面"作为一等场景写入集成测试：构造两个共面半透明层 +
   一个实例化透明层 + 一个不透明背板的最小场景，分别断言三条路径的
   回调次数、累积透射率与最终命中/遮挡状态，固化 2.1 节表格中的行为，
   防止遍历器或 epilog 重构时悄悄改变平局语义。
2. 在使用文档（或本文件）层面明确两条使用纪律：需要按距离有序处理
   多层命中时，在交集过滤器内缓存命中并按 `tfar` 排序，不要依赖回调
   顺序；透明度若依赖世界空间坐标，在实例化场景下先变换回世界空间
   再求值。
3. 需要"每层只计一次"的累积型过滤器应按 `(geomID, primID, instID)` 去重，
   以吸收共享边双重报告与 `RTC_BUILD_QUALITY_HIGH` 下的重复回调。
4. 收集前 n 个命中一律走 `rtcIntersect` + 交集过滤器（参照
   `SphereFilterMultiHitTest` 的 `valid[0]=0` 模式），不要用
   `rtcOccluded` 尝试同样效果——后者在首个被接受命中后即终止，
   语义上无法表达该需求。
