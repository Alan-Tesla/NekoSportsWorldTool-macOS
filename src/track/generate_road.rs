//! 真实道路路由轨迹生成器（模式 B）。
//!
//! 与经典模式（`generator::build`）的分工：
//! - 几何源为路网闭环路线（而非 Catmull-Rom 环）；
//! - 配速为运动学速度剖面：OU 均值回归 + 弯道预判刹车（`kinematics::pace_profile`）；
//! - 步频/步幅为生物力学 OU 耦合（`biomech::ou_cadence_series` + `gait`）；
//! - GPS 抖动为 AR(1) 相关漂移 + 高斯测量噪声 + 建筑 SDF 衰减（`noise::GpsJitter`）。
//!
//! 协议字段/哨兵/断崖/吸附/10s 窗与经典模式完全一致。

use super::generator::{SPEED_CEIL, SPEED_FLOOR};
use super::geom::{
    fmt_gain_time, make_ordered_ring, ring_point_at, round_to, to_bd, wgs84_to_bd09, Rng,
    MET_PER_DEG_LAT, MET_PER_DEG_LNG,
};
use super::model::{GenPoint, Segment, TenWindow, Track};
use super::postfix::apply_post_fixes;
pub use route_planner::CustomCloseMode;
use route_planner::{
    fatigue_from_km, gait, ou_cadence_series, pace_profile, plan_custom_route, plan_route_split,
    point_in_polygon, Coord, GpsJitter, KinParams, RoadGraph, Route, RouteOptions, Sdf,
};

/// 路线算法模式（三选一）。
#[derive(Clone, Copy, PartialEq, Eq, Debug, Default)]
pub enum RouteMode {
    /// 经典打卡点环（Catmull-Rom）。
    #[default]
    Legacy,
    /// 真实道路拓扑路由（自动闭环）。
    Road,
    /// 地图自定义规划路径（按用户放置的途经点路由）。
    Custom,
}

impl RouteMode {
    pub fn from_str(s: &str) -> Self {
        match s {
            "road" => RouteMode::Road,
            "custom" => RouteMode::Custom,
            _ => RouteMode::Legacy,
        }
    }

    pub fn as_str(self) -> &'static str {
        match self {
            RouteMode::Legacy => "legacy",
            RouteMode::Road => "road",
            RouteMode::Custom => "custom",
        }
    }
}

/// 从本地文件加载 OSM 路网。
pub fn load_network_path(path: &str) -> Result<RoadGraph, String> {
    let bytes = std::fs::read(path).map_err(|e| format!("读取 {path} 失败: {e}"))?;
    route_planner::load_osm(&bytes)
}

/// 路网预览计划（供地图显示）：对齐后的路网/建筑/水绿地/操场/标签 + 生成的路线（BD 系）。
#[derive(Clone, Default)]
pub struct RoadPlan {
    pub route: Vec<(f64, f64)>,
    pub length_m: f64,
    pub edges: Vec<Vec<(f64, f64)>>,
    pub buildings: Vec<Vec<(f64, f64)>>,
    pub waters: Vec<Vec<(f64, f64)>>,
    pub greens: Vec<Vec<(f64, f64)>>,
    pub pitches: Vec<Vec<(f64, f64)>>,
    /// `(lat, lon, name, kind)`
    pub labels: Vec<(f64, f64, String, String)>,
    pub checkpoints: Vec<(f64, f64)>,
    pub fences: Vec<Vec<(f64, f64)>>,
    pub route_error: Option<String>,
}

fn extract_map_features(
    g: &RoadGraph,
) -> (
    Vec<Vec<(f64, f64)>>,
    Vec<Vec<(f64, f64)>>,
    Vec<Vec<(f64, f64)>>,
    Vec<Vec<(f64, f64)>>,
    Vec<Vec<(f64, f64)>>,
    Vec<(f64, f64, String, String)>,
) {
    let to_rings = |rings: &[Vec<Coord>]| -> Vec<Vec<(f64, f64)>> {
        rings
            .iter()
            .map(|r| r.iter().map(|c| (c.lat, c.lon)).collect())
            .collect()
    };
    let edges = to_rings(&g.edges_deg());
    let buildings = to_rings(&g.buildings);
    let waters = to_rings(&g.waters);
    let greens = to_rings(&g.greens);
    let pitches = to_rings(&g.pitches);
    let labels = g
        .labels
        .iter()
        .map(|l| (l.coord.lat, l.coord.lon, l.name.clone(), l.kind.clone()))
        .collect();
    (edges, buildings, waters, greens, pitches, labels)
}

/// 用围栏（BD 系，与打卡点同系）裁剪路网：仅保留围栏内道路；裁剪后无路则退回原网。
pub fn apply_fences(net: &RoadGraph, fences: &[Vec<(f64, f64)>]) -> RoadGraph {
    if fences.is_empty() {
        return net.clone();
    }
    let mut g = net.clone();
    let polys: Vec<Vec<Coord>> = fences
        .iter()
        .map(|r| r.iter().map(|p| Coord::new(p.1, p.0)).collect())
        .collect();
    g.retain_inside_any(&polys);
    // 进一步裁剪：移除「穿越围栏边界」的边（端点在内但中段跑出围栏，如跨校区道路）。
    g.graph.retain_edges(|graph, e| {
        let (a, b) = graph.edge_endpoints(e).unwrap();
        edge_inside(&polys, graph[a].coord, graph[b].coord)
    });
    g.rebuild_index();
    if g.graph.edge_count() == 0 {
        return net.clone();
    }
    g
}

/// 边是否整体落在任一围栏内：沿边等距采样，所有采样点（含端点）必须在围栏内。
fn edge_inside(polys: &[Vec<Coord>], a: Coord, b: Coord) -> bool {
    const SEGS: usize = 8;
    for k in 0..=SEGS {
        let t = k as f64 / SEGS as f64;
        let p = Coord::new(a.lon + (b.lon - a.lon) * t, a.lat + (b.lat - a.lat) * t);
        if !polys.iter().any(|poly| point_in_polygon(poly, p)) {
            return false;
        }
    }
    true
}

/// 生成路网预览（对齐 + 围栏裁剪 + 闭环路由），不产生协议字段。
///
/// 预览阶段拿不到 policy 必经点，故仅做「软引导 + 吸附」，不强制经过任何打卡点。
pub fn plan_road_view(
    net: &RoadGraph,
    points_bd: &[(f64, f64)],
    dist: f64,
    seed: u64,
    fences: &[Vec<(f64, f64)>],
) -> Result<RoadPlan, String> {
    let mut g = net.clone();
    align_network(&mut g);
    g = apply_fences(&g, fences);
    let ordered = radial_order(points_bd);
    let waypoints: Vec<Coord> = ordered.iter().map(|p| Coord::new(p.1, p.0)).collect();
    let opts = RouteOptions::default();
    let route = plan_route_split(&g, &waypoints, &[], dist, seed, &opts)?;
    let route_pts: Vec<(f64, f64)> = route.points.iter().map(|p| (p.lat, p.lon)).collect();
    let (edges, buildings, waters, greens, pitches, labels) = extract_map_features(&g);
    Ok(RoadPlan {
        route: route_pts,
        length_m: route.length_m,
        edges,
        buildings,
        waters,
        greens,
        pitches,
        labels,
        checkpoints: ordered,
        fences: fences.to_vec(),
        route_error: None,
    })
}

/// 生成自定义路径规划预览：
/// - 若 `snap_road = true` 且已加载 OSM 路网，则对齐到 BD-09（可选围栏裁剪）并沿真实道路按序连接 `waypoints_bd`；
/// - 若 `snap_road = false`（自由手绘 / 操场跑圈模式）或未加载 OSM 路网，则严格按用户点击的 `waypoints_bd` 原始位置拟合平滑闭合环或折返路线（仍保留底图道路/建筑/操场显示，不强制吸附到外部道路）；
/// - 即使途经点不足或寻路报错，仍返回完整的底图要素供交互点击。
#[allow(clippy::too_many_arguments)]
pub fn plan_custom_view(
    net: Option<&RoadGraph>,
    waypoints_bd: &[(f64, f64)],
    checkpoints_bd: &[(f64, f64)],
    fences: &[Vec<(f64, f64)>],
    clip_fence: bool,
    snap_road: bool,
    close_mode: CustomCloseMode,
    dist_m: f64,
    seed: u64,
) -> RoadPlan {
    let ordered_cp = radial_order(checkpoints_bd);
    let (edges, buildings, waters, greens, pitches, labels, aligned_net) =
        if let Some(raw_net) = net {
            let mut g = raw_net.clone();
            align_network(&mut g);
            if clip_fence {
                g = apply_fences(&g, fences);
            }
            let (e, b, w, gr, p, l) = extract_map_features(&g);
            (e, b, w, gr, p, l, Some(g))
        } else {
            (
                Vec::new(),
                Vec::new(),
                Vec::new(),
                Vec::new(),
                Vec::new(),
                Vec::new(),
                None,
            )
        };

    // 模式 1：开启了「沿道路寻路」且已加载 OSM 路网
    if snap_road {
        if let Some(g) = aligned_net {
            if waypoints_bd.is_empty() {
                return RoadPlan {
                    route: Vec::new(),
                    length_m: 0.0,
                    edges,
                    buildings,
                    waters,
                    greens,
                    pitches,
                    labels,
                    checkpoints: ordered_cp,
                    fences: fences.to_vec(),
                    route_error: Some(
                        "左键点击地图添加起点与途经点（当前为「沿道路寻路」，若进操场可切换为「自由手绘/操场」）"
                            .into(),
                    ),
                };
            }
            let wp_coords: Vec<Coord> = waypoints_bd
                .iter()
                .map(|&(lat, lon)| Coord::new(lon, lat))
                .collect();
            let opts = RouteOptions::default();
            return match plan_custom_route(&g, &wp_coords, close_mode, dist_m, seed, &opts) {
                Ok(route) => {
                    let route_pts: Vec<(f64, f64)> =
                        route.points.iter().map(|p| (p.lat, p.lon)).collect();
                    RoadPlan {
                        route: route_pts,
                        length_m: route.length_m,
                        edges,
                        buildings,
                        waters,
                        greens,
                        pitches,
                        labels,
                        checkpoints: ordered_cp,
                        fences: fences.to_vec(),
                        route_error: None,
                    }
                }
                Err(e) => RoadPlan {
                    route: waypoints_bd.to_vec(),
                    length_m: 0.0,
                    edges,
                    buildings,
                    waters,
                    greens,
                    pitches,
                    labels,
                    checkpoints: ordered_cp,
                    fences: fences.to_vec(),
                    route_error: Some(e),
                },
            };
        }
    }

    // 模式 2：自由手绘 / 操场跑圈模式（不强制吸附道路，严格經過用户点击的每个点）
    if waypoints_bd.len() < 2 {
        return RoadPlan {
            route: waypoints_bd.to_vec(),
            length_m: 0.0,
            edges,
            buildings,
            waters,
            greens,
            pitches,
            labels,
            checkpoints: ordered_cp,
            fences: fences.to_vec(),
            route_error: Some(
                "自由手绘/操场模式：左键点击任意位置（如操场跑道）添加至少 2 个点，或点「生成操场跑圈」"
                    .into(),
            ),
        };
    }
    let close_loop = close_mode != CustomCloseMode::OutAndBack;
    let (dense, arcs, (cx, cy)) = make_ordered_ring(waypoints_bd, close_loop);
    let mut route_pts: Vec<(f64, f64)> = dense
        .iter()
        .map(|&(x, y)| to_bd(x, y, cx, cy))
        .collect();
    if let Some(&first) = route_pts.first() {
        route_pts.push(first);
    }
    let length_m = *arcs.last().unwrap_or(&0.0);
    RoadPlan {
        route: route_pts,
        length_m,
        edges,
        buildings,
        waters,
        greens,
        pitches,
        labels,
        checkpoints: ordered_cp,
        fences: fences.to_vec(),
        route_error: None,
    }
}

/// 生成标准 400m 田径操场跑道椭圆点集（BD-09 `(lat, lon)`，共 12 个有序控制点）：
/// - 若 `existing_wps` 有 1 个点：以该点为操场中心，默认南北向；
/// - 若 `existing_wps` 有 2 个点：以两点中点为操场中心，两点连线方向为操场长轴方向；
/// - 其他情况：以 `fallback_center` 为操场中心，默认南北向。
pub fn make_playground_oval(
    existing_wps: &[(f64, f64)],
    fallback_center: (f64, f64),
) -> Vec<(f64, f64)> {
    let (center, axis_angle) = match existing_wps.len() {
        1 => (existing_wps[0], std::f64::consts::FRAC_PI_2),
        2 => {
            let a = existing_wps[0];
            let b = existing_wps[1];
            let mid = ((a.0 + b.0) * 0.5, (a.1 + b.1) * 0.5);
            let dx = (b.1 - a.1) * MET_PER_DEG_LNG;
            let dy = (b.0 - a.0) * MET_PER_DEG_LAT;
            let ang = if dx.hypot(dy) > 5.0 {
                dy.atan2(dx)
            } else {
                std::f64::consts::FRAC_PI_2
            };
            (mid, ang)
        }
        _ => (fallback_center, std::f64::consts::FRAC_PI_2),
    };

    // 标准 400m 跑道：两条直道各 84.39m（半长 42.2m），两端半圆半径 36.5m
    let half_straight = 42.2f64;
    let radius = 36.5f64;
    let local_pts: [(f64, f64); 12] = [
        // 东侧/右侧直道（南→北）
        (-half_straight, -radius),
        (0.0, -radius),
        (half_straight, -radius),
        // 北侧弯道（3 个采样点）
        (
            half_straight + radius * 0.707,
            -radius * 0.707,
        ),
        (half_straight + radius, 0.0),
        (
            half_straight + radius * 0.707,
            radius * 0.707,
        ),
        // 西侧/左侧直道（北→南）
        (half_straight, radius),
        (0.0, radius),
        (-half_straight, radius),
        // 南侧弯道（3 个采样点）
        (
            -half_straight - radius * 0.707,
            radius * 0.707,
        ),
        (-half_straight - radius, 0.0),
        (
            -half_straight - radius * 0.707,
            -radius * 0.707,
        ),
    ];

    let (sin_a, cos_a) = axis_angle.sin_cos();
    local_pts
        .iter()
        .map(|&(u, v)| {
            // u 沿长轴方向，v 垂直于长轴
            let x = u * cos_a - v * sin_a;
            let y = u * sin_a + v * cos_a;
            to_bd(x, y, center.0, center.1)
        })
        .collect()
}

/// 将坐标点吸附到对齐后的 BD-09 路网最近道路上（距离阈值内生效）。
pub fn snap_point_to_aligned_road(
    net: &RoadGraph,
    fences: &[Vec<(f64, f64)>],
    clip_fence: bool,
    lat: f64,
    lon: f64,
) -> Option<((f64, f64), f64)> {
    let mut g = net.clone();
    align_network(&mut g);
    if clip_fence {
        g = apply_fences(&g, fences);
    }
    let (snapped, dist_m) = g.snap_to_road(Coord::new(lon, lat))?;
    Some(((snapped.lat, snapped.lon), dist_m))
}

/// 将 WGS84 路网逐节点转换为 BD-09（与打卡点同系）。
///
/// OSM 为 WGS84，打卡点为 BD-09。两者不是常数平移关系（境内 GCJ/BD 偏移随位置
/// 变化），故用标准 WGS84→GCJ-02→BD-09 变换逐节点转换，校园尺度下与真实道路
/// 对齐（优于旧版质心常数平移的米级~数十米残差）。
pub fn align_network(net: &mut RoadGraph) {
    if net.graph.node_count() == 0 {
        return;
    }
    for node in net.graph.node_weights_mut() {
        let (lat, lon) = wgs84_to_bd09(node.coord.lat, node.coord.lon);
        node.coord.lat = lat;
        node.coord.lon = lon;
    }
    for ring in net.buildings.iter_mut() {
        for c in ring.iter_mut() {
            let (lat, lon) = wgs84_to_bd09(c.lat, c.lon);
            c.lat = lat;
            c.lon = lon;
        }
    }
    for ring in net.waters.iter_mut() {
        for c in ring.iter_mut() {
            let (lat, lon) = wgs84_to_bd09(c.lat, c.lon);
            c.lat = lat;
            c.lon = lon;
        }
    }
    for ring in net.greens.iter_mut() {
        for c in ring.iter_mut() {
            let (lat, lon) = wgs84_to_bd09(c.lat, c.lon);
            c.lat = lat;
            c.lon = lon;
        }
    }
    for ring in net.pitches.iter_mut() {
        for c in ring.iter_mut() {
            let (lat, lon) = wgs84_to_bd09(c.lat, c.lon);
            c.lat = lat;
            c.lon = lon;
        }
    }
    for lb in net.labels.iter_mut() {
        let (lat, lon) = wgs84_to_bd09(lb.coord.lat, lb.coord.lon);
        lb.coord.lat = lat;
        lb.coord.lon = lon;
    }
    net.compute_anchor();
    net.rebuild_index();
}

/// 按质心角度排序打卡点，形成自然环序。
pub fn radial_order(points_bd: &[(f64, f64)]) -> Vec<(f64, f64)> {
    let n = points_bd.len();
    if n == 0 {
        return Vec::new();
    }
    let (cx, cy) = (
        points_bd.iter().map(|q| q.0).sum::<f64>() / n as f64,
        points_bd.iter().map(|q| q.1).sum::<f64>() / n as f64,
    );
    let mut ordered = points_bd.to_vec();
    ordered.sort_by(|a, b| {
        (a.0 - cx)
            .atan2(a.1 - cy)
            .partial_cmp(&(b.0 - cx).atan2(b.1 - cy))
            .unwrap()
    });
    ordered
}

/// 路线 → 平面折线 + 弧长表。
///
/// `arcs[k]` 必须与 `dense[k]` 一一对应（`ring_point_at` 按同下标取弧长），
/// 因此直接取每个采样点的累计弧长 `p.s`（`dense` 末点已闭合回起点）。
fn route_ring(route: &Route, c_lat: f64, c_lng: f64) -> (Vec<(f64, f64)>, Vec<f64>) {
    let dense: Vec<(f64, f64)> = route
        .points
        .iter()
        .map(|p| {
            (
                (p.lon - c_lng) * MET_PER_DEG_LNG,
                (p.lat - c_lat) * MET_PER_DEG_LAT,
            )
        })
        .collect();
    let arcs: Vec<f64> = route.points.iter().map(|p| p.s).collect();
    (dense, arcs)
}

/// 模式 B：真实道路拓扑路由轨迹生成。
///
/// `route_bd[0]` 为起点，`route_bd[1..]` 为软引导点（用于方向锚点，不必全经过）；
/// `must_bd` 为强制必经点（按序，可为空，最终必达）；吸附在 `route_bd` 全量上做。
pub fn build_road(
    dist: f64,
    dur: i64,
    seed: u64,
    start_ms: i64,
    route_bd: &[(f64, f64)],
    must_bd: &[(f64, f64)],
    net: &RoadGraph,
) -> Result<Track, String> {
    let dur_f = dur as f64;
    let avg_v = dist / dur_f;
    let a_c_max = 2.5;
    let radius = (avg_v * avg_v / a_c_max).max(6.0);
    let opts = RouteOptions {
        a_c_max,
        min_radius_m: radius,
        ..Default::default()
    };

    let waypoints: Vec<Coord> = route_bd.iter().map(|p| Coord::new(p.1, p.0)).collect();
    let must: Vec<Coord> = must_bd.iter().map(|p| Coord::new(p.1, p.0)).collect();
    let route = plan_route_split(net, &waypoints, &must, dist, seed, &opts)?;
    build_track_from_route(dist, dur, seed, start_ms, &route, route_bd, route_bd, net)
}

/// 模式 C（有 OSM 路网）：按用户自定义途经点在真实路网上规划路线并生成完整协议轨迹。
#[allow(clippy::too_many_arguments)]
pub fn build_custom_road(
    dist: f64,
    dur: i64,
    seed: u64,
    start_ms: i64,
    waypoints_bd: &[(f64, f64)],
    close_mode: CustomCloseMode,
    snap_bd: &[(f64, f64)],
    net: &RoadGraph,
) -> Result<Track, String> {
    let dur_f = dur as f64;
    let avg_v = dist / dur_f;
    let a_c_max = 2.5;
    let radius = (avg_v * avg_v / a_c_max).max(6.0);
    let opts = RouteOptions {
        a_c_max,
        min_radius_m: radius,
        ..Default::default()
    };
    let waypoints: Vec<Coord> = waypoints_bd.iter().map(|p| Coord::new(p.1, p.0)).collect();
    let route = plan_custom_route(net, &waypoints, close_mode, dist, seed, &opts)?;
    build_track_from_route(
        dist,
        dur,
        seed,
        start_ms,
        &route,
        waypoints_bd,
        snap_bd,
        net,
    )
}

#[allow(clippy::too_many_arguments)]
fn build_track_from_route(
    dist: f64,
    dur: i64,
    seed: u64,
    start_ms: i64,
    route: &Route,
    ref_bd: &[(f64, f64)],
    snap_bd: &[(f64, f64)],
    net: &RoadGraph,
) -> Result<Track, String> {
    let mut rng = Rng::new(seed);
    let dur_f = dur as f64;
    let avg_v = dist / dur_f;
    let a_c_max = 2.5;

    let n_pts = ref_bd.len().max(1);
    let (c_lat, c_lng) = (
        ref_bd.iter().map(|q| q.0).sum::<f64>() / n_pts as f64,
        ref_bd.iter().map(|q| q.1).sum::<f64>() / n_pts as f64,
    );
    let (dense, arcs) = route_ring(route, c_lat, c_lng);

    // 时间网格（同经典模式）
    let mut times = Vec::new();
    let mut t = 0.0;
    while t < dur_f {
        times.push(t);
        t += if rng.random() < 0.80 {
            5.0
        } else {
            rng.choice(&[1.0, 2.0, 3.0, 4.0, 6.0, 7.0, 8.0])
        };
    }
    let n = times.len();
    let mut dts: Vec<f64> = (0..n - 1).map(|i| times[i + 1] - times[i]).collect();
    dts.push(1.0f64.max(dur_f - times[n - 1]));

    // 运动学速度剖面：OU 配速 + 弯道预判刹车 + 精确命中目标距离
    let kin = KinParams {
        a_c_max,
        ..Default::default()
    };
    let speed_seed = seed.wrapping_add(0x9E37_79B9_7F4A_7C15);
    let w = pace_profile(
        route,
        avg_v,
        &dts,
        dist,
        speed_seed,
        &kin,
        SPEED_FLOOR,
        SPEED_CEIL,
    );
    let speeds: Vec<f64> = w.clone();
    let seg_dist: Vec<f64> = (0..n).map(|i| w[i] * dts[i]).collect();

    // 点位类型（同经典模式）
    let tl_w = [
        ((-1i64, 4i64), 54u32),
        ((-1, 1), 27),
        ((-1, 12), 13),
        ((-1, 5), 3),
        ((-1, 6), 2),
    ];
    let mut kinds: Vec<(i64, i64)> = Vec::with_capacity(n);
    for _ in 0..n {
        let u = rng.random();
        if u < 0.39 {
            kinds.push((3, 1));
        } else if u < 0.93 {
            kinds.push((0, 1));
        } else if u < 0.96 {
            kinds.push(rng.weighted(&tl_w));
        } else {
            kinds.push((rng.choice(&[1, 1, 1, 1, 2, 2]), 1));
        }
    }
    for i in 1..kinds.len() {
        let prev_drift = (-1 == kinds[i - 1].0) || kinds[i - 1].0 == 5 || kinds[i - 1].0 == 6;
        if kinds[i].0 != -1 && prev_drift && rng.random() < 0.08 {
            kinds[i] = (if rng.random() < 0.75 { 7 } else { 8 }, 1);
        }
    }
    for i in 1..n {
        if kinds[i].0 == -1 && kinds[i - 1].0 == -1 {
            kinds[i] = (rng.choice(&[3, 0]), 1);
        }
    }
    let normal_idx: Vec<usize> = (0..n).filter(|&i| kinds[i].0 != -1 && i > 1).collect();
    let share: f64 = normal_idx.iter().map(|&i| seg_dist[i]).sum();
    let share = if share == 0.0 { 1.0 } else { share };
    let mut disp_of = vec![0.0f64; n];
    for &i in &normal_idx {
        disp_of[i] = seg_dist[i] * (dist / share);
    }

    // 建筑 SDF（自适应放大漂移）
    let sdf = Sdf::from_buildings(
        &net.buildings,
        |c| {
            [
                (c.lon - c_lng) * MET_PER_DEG_LNG,
                (c.lat - c_lat) * MET_PER_DEG_LAT,
            ]
        },
        2.0,
        30.0,
    );

    // 生物力学：步频在目标值附近 OU 波动，步幅反解严格满足 v=(cad/60)·stride
    let targets: Vec<f64> = (0..n)
        .map(|i| {
            let km_done = (times[i] / dur_f) * dist / 1000.0;
            gait(speeds[i], fatigue_from_km(km_done)).cadence
        })
        .collect();
    let cadence_seed = seed.wrapping_add(0xC6A4_A793_5BD1_E995);
    let cadences = ou_cadence_series(&targets, &dts, cadence_seed, 0.25, 0.5, 110.0, 220.0);

    // GPS 抖动：AR(1) 相关漂移 + 高斯测量噪声（SDF 逐点放大）
    let mut jitter = GpsJitter::new(0.72, 0.75, 0.3);

    let mut locs: Vec<GenPoint> = Vec::with_capacity(n);
    let mut s = 0.0f64;
    let mut t_acc = 0.0f64;
    let mut dist_acc = 0.0f64;
    let mut steps_acc = 0.0f64;
    let mut alt = 82.0 + rng.uniform(-1.0, 1.0);
    let n_est = 1.max((dur_f / 5.0) as i64);
    let alt_sigma = rng.uniform(3.8, 6.2) / (0.40 * n_est as f64);
    let mut ten_t = 0.0f64;
    let mut ten_d = 0.0f64;
    let mut ten_st = 0.0f64;
    let mut ten_speed: Vec<TenWindow> = Vec::new();
    let mut ten_steps: Vec<TenWindow> = Vec::new();

    for i in 0..n {
        let dt = dts[i];
        let (typ, lt) = kinds[i];
        t_acc += dt;
        let pos = |ss: f64| ring_point_at(&dense, &arcs, ss);
        let mut d_step = 0.0f64;
        let px;
        let py;
        let x;
        let y;
        let rad;
        let state;
        if typ != -1 {
            d_step = disp_of[i];
            s += d_step;
            let (bx, by) = pos(s);
            x = bx;
            y = by;
            // AR(1) 相关漂移 + 高斯测量噪声，SDF 自适应放大
            let sc = sdf.sigma_scale([bx, by]);
            let z = [
                rng.gauss(0.0, 1.0),
                rng.gauss(0.0, 1.0),
                rng.gauss(0.0, 1.0),
                rng.gauss(0.0, 1.0),
            ];
            let (jxd, jyd) = jitter.step(sc, z);
            px = bx + jxd;
            py = by + jyd;
            rad = round_to(
                if typ == 3 {
                    rng.uniform(1.4, 5.1)
                } else {
                    rng.uniform(1.4, 2.4)
                },
                2,
            );
            state = if typ == 0 {
                rng.weighted(&[(1, 145), (2, 45), (3, 164)])
            } else {
                rng.weighted(&[(1, 145), (2, 256), (3, 151)])
            };
        } else {
            match lt {
                4 => {
                    if rng.random() >= 0.68 {
                        d_step = if rng.random() < 0.95 {
                            rng.uniform(2.0, 60.0)
                        } else {
                            rng.uniform(60.0, 250.0)
                        };
                    }
                }
                1 => {
                    if rng.random() >= 0.83 {
                        d_step = rng.uniform(0.5, 36.0);
                    }
                }
                12 => {
                    d_step = if rng.random() < 0.9 {
                        rng.uniform(5.0, 80.0)
                    } else {
                        rng.uniform(80.0, 220.0)
                    };
                }
                5 => d_step = rng.uniform(5.0, 60.0),
                _ => d_step = rng.uniform(100.0, 300.0),
            }
            let (bx, by) = pos(s);
            x = bx;
            y = by;
            if d_step > 0.0 {
                let ang = rng.uniform(0.0, std::f64::consts::TAU);
                px = x + d_step * ang.sin();
                py = y + d_step * ang.cos();
            } else if let Some(last) = locs.last() {
                py = (last.gLat - c_lat) * MET_PER_DEG_LAT;
                px = (last.gLng - c_lng) * MET_PER_DEG_LNG;
            } else {
                let (jx, jy) = jitter.state();
                px = x + jx;
                py = y + jy;
            }
            rad = if lt == 4 {
                round_to(
                    if rng.random() < 0.75 {
                        rng.uniform(30.0, 100.0)
                    } else {
                        rng.uniform(100.0, 550.0)
                    },
                    2,
                )
            } else if lt == 1 {
                round_to(
                    if rng.random() < 0.75 {
                        rng.uniform(1.6, 12.0)
                    } else {
                        rng.uniform(12.0, 95.0)
                    },
                    2,
                )
            } else if lt == 12 {
                round_to(rng.uniform(30.0, 125.0), 2)
            } else if lt == 5 {
                round_to(rng.uniform(25.0, 300.0), 2)
            } else {
                550.0
            };
            state = rng.weighted(&[(1, 102), (2, 124), (3, 136)]);
        }
        if typ != -1 {
            dist_acc += d_step;
        }
        let (lat, lng) = to_bd(px, py, c_lat, c_lng);
        alt += 0.04 * (82.0 - alt) + rng.gauss(0.0, alt_sigma);
        // 生物力学联动：v=(cad/60)·stride
        let v_now = speeds[i];
        let mut stride = v_now * 60.0 / cadences[i];
        if stride > 1.7 {
            stride = 1.7;
        } else if stride < 0.5 {
            stride = 0.5;
        }
        let cad = v_now * 60.0 / stride;
        steps_acc += cad / 60.0 * dt;
        let nxt = pos(s + 2.0);
        let brg =
            ((nxt.0 - x).atan2(nxt.1 - y).to_degrees() + rng.gauss(0.0, 35.0)).rem_euclid(360.0);
        let (avg_sp, gps_speed) = if typ == -1 {
            let avg = round_to(dist_acc / t_acc.max(1.0), 4);
            let gps = if rng.random() < 0.12 {
                rng.uniform(15.0, 46.0)
            } else {
                rng.uniform(0.5, 6.0)
            };
            (avg, round_to(gps, 4))
        } else {
            let avg = round_to(d_step / dt, 4);
            let kmh = avg * 3.6;
            let sigma = (kmh * 0.08).max(0.05);
            let gps = if rng.random() < 0.20 {
                0.0
            } else {
                round_to((kmh + rng.gauss(0.0, sigma)).max(0.0), 4)
            };
            (avg, gps)
        };

        ten_t += dt;
        ten_d += speeds[i] * dt;
        ten_st += cad / 60.0 * dt;
        while ten_t >= 10.0 {
            let k = 10.0 / ten_t;
            let (out_d, out_st) = (ten_d * k, ten_st * k);
            ten_speed.push(TenWindow {
                time: 10,
                value: round_to(out_d, 2),
            });
            ten_steps.push(TenWindow {
                time: 10,
                value: round_to(out_st, 0),
            });
            ten_t -= 10.0;
            ten_d -= out_d;
            ten_st -= out_st;
        }
        locs.push(GenPoint {
            id: i as i64 + 1,
            flag: start_ms,
            lat: -1.0,
            lng: -1.0,
            gLat: round_to(lat, 7),
            gLng: round_to(lng, 7),
            speed: round_to(gps_speed, 4),
            avgSpeed: avg_sp,
            radius: rad,
            accuracy: rad,
            ptype: typ,
            locType: lt,
            hasAltitude: true,
            totalTime: round_to(t_acc, 0) as i64,
            totalDis: round_to(dist_acc, 4),
            validDis: round_to(dist_acc, 4),
            validTime: round_to(t_acc, 0) as i64,
            steps: steps_acc as i64,
            stepDistance: 0.0,
            gainTime: fmt_gain_time(start_ms + (t_acc * 1000.0) as i64),
            gainTimeMs: start_ms + (t_acc * 1000.0) as i64,
            queueNum: 0,
            coorType: "gcj02".into(),
            bdA: round_to(alt, 2),
            bdD: round_to(brg, 2),
            bdS: round_to((avg_sp * rng.uniform(0.6, 0.95)).max(0.0), 3),
            bdG: rng.choice(&[1, 1, 1, -1]),
            count: rng.randint(20, 88),
            dtr: 0.0,
            state,
            locationId: String::new(),
        });
    }
    if ten_t > 1.0 {
        let k = (10.0 / ten_t).min(1.4);
        ten_speed.push(TenWindow {
            time: 10,
            value: round_to(ten_d * k, 2),
        });
        ten_steps.push(TenWindow {
            time: 10,
            value: round_to(ten_st * k, 0),
        });
    }

    let mut segments: Vec<Segment> = Vec::new();
    let (mut seg_t, mut seg_d) = (0.0f64, 0.0f64);
    let mut seg_v: Vec<f64> = Vec::new();
    for i in 0..n {
        seg_t += dts[i];
        seg_d += seg_dist[i];
        seg_v.push(speeds[i]);
        if seg_t >= 60.0 || i == n - 1 {
            segments.push(Segment {
                totalTime: round_to(seg_t, 0) as i64,
                distance: round_to(seg_d, 0) as i64,
                startTime: round_to((times[i] - seg_t) * 1000.0, 0) as i64,
                endTime: round_to(times[i] * 1000.0, 0) as i64,
                avgSpeed: round_to(seg_v.iter().sum::<f64>() / seg_v.len() as f64, 3),
                avgStep: round_to(steps_acc / 1.0f64.max(t_acc) * 60.0, 0) as i64,
                state: 0,
            });
            seg_t = 0.0;
            seg_d = 0.0;
            seg_v.clear();
        }
    }

    apply_post_fixes(&mut locs, &mut rng, start_ms);

    // 点位吸附：<40m 精确落位（全量打卡点，含必经点与普通打卡点）
    for pl in snap_bd {
        let mut best_i = None;
        let mut best_d = 1e18f64;
        for (i, q) in locs.iter().enumerate() {
            let dd = ((q.gLat - pl.0) * MET_PER_DEG_LAT).powi(2)
                + ((q.gLng - pl.1) * MET_PER_DEG_LNG).powi(2);
            if dd < best_d {
                best_d = dd;
                best_i = Some(i);
            }
        }
        if let Some(i) = best_i {
            if best_d < 40.0 * 40.0 {
                locs[i].gLat = round_to(pl.0, 7);
                locs[i].gLng = round_to(pl.1, 7);
            }
        }
    }

    let total_dis = round_to(dist, 3);
    Ok(Track {
        totalTime: round_to(t_acc, 0) as i64,
        totalDistance: total_dis,
        validDistance: total_dis,
        validTime: round_to(t_acc, 0) as i64,
        startTime: start_ms,
        startLatitude: locs[0].gLat,
        startLongitude: locs[0].gLng,
        totalSteps: steps_acc as i64,
        locations: locs,
        speedPerTenSec: ten_speed,
        stepsPerTenSec: ten_steps,
        segments,
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn custom_view_and_ordered_track_work_with_and_without_osm() {
        let wps = vec![
            (38.889000, 121.531000),
            (38.890500, 121.531000),
            (38.890500, 121.533000),
        ];
        let view = plan_custom_view(
            None,
            &wps,
            &wps,
            &[],
            false,
            false,
            CustomCloseMode::Loop,
            2000.0,
            7,
        );
        assert!(view.route_error.is_none());
        assert!(view.length_m > 200.0);
        assert!(view.route.len() > 10);

        // 测试生成 400m 标准操场跑道椭圆
        let oval = make_playground_oval(&[(38.889000, 121.531000)], (38.889000, 121.531000));
        assert_eq!(oval.len(), 12);
        let oval_view = plan_custom_view(
            None,
            &oval,
            &[],
            &[],
            false,
            false,
            CustomCloseMode::Loop,
            2000.0,
            7,
        );
        assert!(
            (oval_view.length_m - 400.0).abs() < 45.0,
            "操场跑道单圈长度应接近 400m，实际: {}",
            oval_view.length_m
        );

        let track = crate::track::generator::build_ordered(
            1500.0,
            540,
            7,
            1_700_000_000_000,
            &wps,
            true,
            &wps,
        );
        assert!((track.totalDistance - 1500.0).abs() < 1.0);
        assert!(!track.locations.is_empty());
    }
}
