# 算法详细推导

## 1. 问题形式化

设栅格地图 $M:\mathbb{Z}^2\to\{0,1\}$，自由空间

$$F = \{x\in\mathbb{Z}^2\mid M(x)=1\}.$$

观测点 $p\in F$ 的有效观测区为

$$S(p) = D(p,r)\cap V(p)$$

其中 $V(p)=\{x\in F:\overline{px}\subset F\}$。求最小 $P\subset F$ 使

$$F\subseteq\bigcup_{p\in P}S(p).$$

## 2. 几何关键洞察 —— 窄通道

设通道局部宽度（中轴点到最近障碍距离的两倍）为 $w$，圆心若位于中轴上，
沿中轴方向单圆盘可覆盖长度

$$L(w)=2\sqrt{r^2-(w/2)^2},\qquad w\le 2r.$$

若两相邻观测点位于中轴上、间距为 $d$，要求其圆盘并集覆盖通道，则

$$d\le L(w)=2\sqrt{r^2-(w/2)^2}.$$

| $w/r$ | $L/r$ | 含义 |
|------|------|------|
| 2.00 | 0.00 | 退化：不可能完整覆盖 |
| 1.80 | 0.87 | 需要密集步长 |
| 1.60 | 1.20 |  |
| 1.40 | 1.43 |  |
| 1.00 | 1.73 | 即六边形铺设的相邻间距 |

**实现策略**：沿中轴行走时根据当前 $w$ 自适应选择步长 $d=\alpha L(w)$，
通常取安全系数 $\alpha\in[0.85,0.95]$。当 $w\to 2r$ 时 $L\to 0$，
此时算法退化为"最大间距 = 最小允许步长 $d_{\min}$"，
再对未覆盖残差用补丁式贪心补点。

## 3. 候选点生成策略（核心）

### 3.1 反射顶点候选（`refl-vertex`）

障碍多边形的每个**凹顶点**（reflex vertex）周围放置候选。
艺术馆问题最优解中，必然存在一组守卫位于这些位置或其附近。
栅格化时近似为：障碍边界上局部曲率为负的像素，沿向内法向偏移 $\epsilon$。

### 3.2 中轴候选（`medial`）

对自由空间做欧氏距离变换 $\mathrm{EDT}(x)=\min_{y\notin F}\|x-y\|$，
取其骨架（medial axis）$\mathcal{M}$。在 $\mathcal{M}$ 上沿连通分量行走，
在弧长参数 $s$ 处取局部宽度 $w(s)=2\,\mathrm{EDT}(\gamma(s))$，按

$$\Delta s_k = \alpha\cdot 2\sqrt{r^2-(w(s_k)/2)^2}$$

自适应步长生成候选。

### 3.3 房间六边形候选（`hex-grid`）

对 $\mathrm{EDT}>r$ 的"宽阔"区域，铺设六边形点阵：

- 行间距 $\Delta y = \tfrac{3}{2}r$
- 列间距 $\Delta x = r\sqrt{3}$
- 偶数行偏移 $\Delta x/2$

仅保留落在自由空间内的点。

## 4. 栅格可见性

对候选点 $c$，计算 $S(c)=D(c,r)\cap V(c)$ 仅需在 $D(c,r)$ 包围盒内：

```
for u in pixels(D(c,r) ∩ F):
    if line_segment(c, u) ⊂ F:    # Bresenham + 障碍命中检测
        S[c].add(u)
```

加速：用极坐标 ray-marching，方向角分辨率 $\Delta\theta = 1/r$，
每条射线沿距离场做球面行进直至撞墙或越界。复杂度 $O(r^2/\bar w)$
（$\bar w$ 为平均自由距离）。

## 5. 贪心 Set Cover（lazy heap）

```
Covered = ∅
H = max-heap of (|S_c \ Covered|, c)
while ⋃ S_c \ Covered ≠ ∅:
    pop (g, c) from H
    g_now = |S_c \ Covered|
    if g_now < g:
        push (g_now, c); continue            # lazy refresh
    select c; Covered ∪= S_c
```

近似比 $\ln|U|+1$（Johnson 1974）；
对几何 disk cover 实测在 1.1–1.3× 最优之间。

## 6. 复杂度汇总

| 模块 | 复杂度 |
|---|---|
| EDT | $O(N)$（$N$=像素数） |
| 中轴 | $O(N)$ |
| 候选点 | $O(\text{骨架长度}/r + \text{房间面积}/r^2)$ |
| 单点可见性 | $O(r^2)$ |
| 覆盖矩阵 | $O(|C|\cdot r^2)$（稀疏） |
| 贪心 | $O((|C|+|U|)\log|C|)$ |

## 7. 已知局限与改进方向

1. **栅格可见性误差**：高障碍贴墙时，沿障碍边的"擦边"可见可能被栅格化误判。
   如需精确，应升级为多边形可见性算法（Shapely / CGAL）。
2. **退化窄段**：$w\to 2r$ 时几何上不可能完整覆盖，
   需引入"覆盖率阈值"（如 ≥99%）才有解。
3. **非并行**：当前为单进程；大地图建议先做区域分解再并行（见 TODO）。
4. **未做后期精修**：贪心解通常含可移除/可合并的冗余点，
   2-opt 与连续微调能再降 5–15% 点数。
