//! 合成轨迹生成：按路线米制弧长前进，热身后稳定巡航，统计由最终采样点统一派生。
#![allow(non_snake_case)]

use super::geom::{
    fmt_gain_time, make_ordered_path, make_point_ring, open_point_at, ring_point_at, round_to,
    to_bd, Rng, meters_per_degree_lat, meters_per_degree_lng,
};
use super::model::{GenPoint, Track};
use super::postfix::apply_post_fixes;

/// 有效配速窗口：4'00"-10'00"/km（即 240-600 秒/km ≈ 1.667-4.167 m/s），留安全余量（约 4'04"-9'41"/km）。
pub const SPEED_FLOOR: f64 = 1.72;
pub const SPEED_CEIL: f64 = 4.10;

/// 等比缩放逐点速度至目标总距：越界点钳在 `[floor, ceil]` 边界，剩余差量由未饱和点分摊（迭代收敛）。
pub fn fit_speeds_bounded(w: &mut [f64], dts: &[f64], target: f64, floor: f64, ceil: f64) {
    let floor = floor.clamp(SPEED_FLOOR, SPEED_CEIL);
    let ceil = ceil.clamp(floor, SPEED_CEIL);
    for _ in 0..32 {
        let cur: f64 = w.iter().zip(dts).map(|(x, dt)| x * dt).sum();
        if (cur - target).abs() <= 0.5 || cur <= 1e-9 {
            break;
        }
        let k = target / cur;
        for x in w.iter_mut() {
            if *x > 0.0 {
                *x = (*x * k).clamp(floor, ceil);
            }
        }
    }
    for x in w.iter_mut() {
        if *x > 0.0 {
            *x = x.clamp(floor, ceil);
        }
    }
}

/// 轨迹生成主入口。points_bd 为 BD 系打卡点。
pub fn build(
    dist: f64,
    dur: i64,
    seed: u64,
    _center: (f64, f64),
    start_ms: i64,
    points_bd: &[(f64, f64)],
) -> Track {
    let (dense, arcs, pc) = make_point_ring(points_bd);
    build_from_ring(
        dist, dur, seed, start_ms, &dense, &arcs, pc, false, true, points_bd,
    )
}

/// 无路网自定义途经点轨迹生成（兼容布尔闭合标志）。
#[allow(dead_code)]
pub fn build_ordered(
    dist: f64,
    dur: i64,
    seed: u64,
    start_ms: i64,
    ordered_bd: &[(f64, f64)],
    close_loop: bool,
    snap_bd: &[(f64, f64)],
) -> Track {
    let mode = if close_loop {
        route_planner::CustomCloseMode::Loop
    } else {
        route_planner::CustomCloseMode::OutAndBack
    };
    build_ordered_mode(dist, dur, seed, start_ms, ordered_bd, mode, snap_bd)
}

/// 按自定义闭合模式（含 `Open` 开环单程、`Loop` 闭合环线、`OutAndBack` 原路折返）生成轨迹。
pub fn build_ordered_mode(
    dist: f64,
    dur: i64,
    seed: u64,
    start_ms: i64,
    ordered_bd: &[(f64, f64)],
    mode: route_planner::CustomCloseMode,
    snap_bd: &[(f64, f64)],
) -> Track {
    let (dense, arcs, pc) = make_ordered_path(ordered_bd, mode);
    build_from_ring(
        dist,
        dur,
        seed,
        start_ms,
        &dense,
        &arcs,
        pc,
        true,
        mode.is_closed(),
        snap_bd,
    )
}

#[allow(clippy::too_many_arguments)]
fn build_from_ring(
    dist: f64,
    dur: i64,
    seed: u64,
    start_ms: i64,
    dense: &[(f64, f64)],
    arcs: &[f64],
    pc: (f64, f64),
    pin_start: bool,
    closed: bool,
    points_bd: &[(f64, f64)],
) -> Track {
    let mut rng = Rng::new(seed);
    let dur_f = dur as f64;
    let (c_lat, c_lng) = pc;
    let direction: f64 = if pin_start {
        1.0
    } else {
        rng.choice(&[1.0, -1.0])
    };
    let s0 = if pin_start {
        0.0
    } else {
        rng.uniform(0.0, *arcs.last().unwrap_or(&400.0))
    };
    let _phase_v = rng.uniform(0.0, std::f64::consts::TAU);
    let _phase_l = rng.uniform(0.0, std::f64::consts::TAU);

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

    let base = (dist / dur_f).clamp(SPEED_FLOOR + 0.05, SPEED_CEIL - 0.05);
    let local_floor = (base * 0.80).max(SPEED_FLOOR);
    let local_ceil = (base * 1.20).min(SPEED_CEIL).max(local_floor + 0.05);

    // Stable cruising after a short warmup; use the same profile as road routing.
    let mut w = Vec::with_capacity(n);
    let mut v = base*route_planner::kinematics::steady_pacing_factor(0.0,dur_f,30.0);
    for i in 0..n {
        let dt = dts[i];
        let factor = route_planner::kinematics::steady_pacing_factor(times[i],dur_f,30.0);
        let target_v = (base*factor).clamp(local_floor,local_ceil);
        let decay = (-0.04*dt).exp();
        let noise_sd = 0.008*((1.0-decay*decay)/0.08).sqrt();
        v = (target_v+(v-target_v)*decay+rng.gauss(0.0,noise_sd)).clamp(local_floor,local_ceil);
        w.push(v);
    }

    let mut kinds: Vec<(i64, i64)> = Vec::with_capacity(n);
    for _ in 0..n {
        let u = rng.random();
        // 全程纯绿有效定位点：type=0（62%）与 type=3（38%），绝不注入 type=1/2（暂停）或 type=-1（断点）
        if u < 0.38 {
            kinds.push((3, 1));
        } else {
            kinds.push((0, 1));
        }
    }

    // 仅哨兵点 i<=1 不计位移，所有跑步过程中的正常点均连续向前推进
    for i in 0..n {
        if i <= 1 {
            w[i] = 0.0;
        }
    }
    fit_speeds_bounded(&mut w, &dts, dist, local_floor, local_ceil);
    // 若温和窗口 [local_floor, local_ceil] 已饱和仍差少量距离，放宽到 [SPEED_FLOOR, SPEED_CEIL] 收敛
    let cur_sum: f64 = w.iter().zip(&dts).map(|(x, dt)| x * dt).sum();
    if (cur_sum - dist).abs() > 2.0 {
        fit_speeds_bounded(&mut w, &dts, dist, SPEED_FLOOR, SPEED_CEIL);
    }
    let disp_of: Vec<f64> = (0..n).map(|i| w[i] * dts[i]).collect();
    let speeds: Vec<f64> = (0..n)
        .map(|i| if w[i] > 0.0 { w[i] } else { base })
        .collect();

    let mut locs: Vec<GenPoint> = Vec::with_capacity(n);
    let mut s = s0;
    let mut t_acc = 0.0f64;
    let mut dist_acc = 0.0f64;
    let mut steps_acc = 0.0f64;
    let mut jx = 0.0f64;
    let mut jy = 0.0f64;
    let mut alt = 82.0 + rng.uniform(-1.0, 1.0);
    let n_est = 1.max((dur_f / 5.0) as i64);
    let alt_sigma = rng.uniform(3.8, 6.2) / (0.40 * n_est as f64);
    for i in 0..n {
        let dt = dts[i];
        let (typ, lt) = kinds[i];
        t_acc += dt;
        let pos = |ss: f64| {
            if closed {
                ring_point_at(dense, arcs, ss)
            } else {
                // 1:1 真实米制步进，避免坐标被拉伸导致 GPS 实际移动配速失真
                open_point_at(dense, arcs, ss)
            }
        };
        let d_step = disp_of[i];
        s += direction * d_step;
        let (bx, by) = pos(s);
        let x = bx;
        let y = by;
        jx = 0.72 * jx + rng.gauss(0.0, 0.75);
        jy = 0.72 * jy + rng.gauss(0.0, 0.75);
        let px = bx + jx;
        let py = by + jy;
        let rad = round_to(
            if typ == 3 {
                rng.uniform(1.4, 4.2)
            } else {
                rng.uniform(1.4, 2.4)
            },
            2,
        );
        let state = if typ == 0 {
            rng.weighted(&[(1, 145), (2, 45), (3, 164)])
        } else {
            rng.weighted(&[(1, 145), (2, 256), (3, 151)])
        };
        dist_acc += d_step;
        let (lat, lng) = to_bd(px, py, c_lat, c_lng);
        alt += 0.04 * (82.0 - alt) + rng.gauss(0.0, alt_sigma);
        // 生物力学高步频小步幅联动（平均步频约 185 spm，平均步幅约 74 cm，严格满足 v = cad/60 * stride）
        let v_now = speeds[i];
        let km_done = (t_acc / dur_f) * dist / 1000.0;
        let fat = route_planner::fatigue_from_km(km_done);
        let gait = route_planner::gait(v_now, fat);
        let cad = (gait.cadence + rng.gauss(0.0, 2.5)).clamp(160.0, 205.0);
        let stride = (v_now * 60.0 / cad).clamp(0.32, 1.25);
        let cad = v_now * 60.0 / stride;
        if d_step > 0.0 { steps_acc += cad / 60.0 * dt; }
        let nxt = pos(s + direction * 2.0);
        let brg =
            ((nxt.0 - x).atan2(nxt.1 - y).to_degrees() + rng.gauss(0.0, 35.0)).rem_euclid(360.0);
        let avg_sp = round_to(d_step / dt.max(0.1), 4);
        let kmh = avg_sp * 3.6;
        let sigma = (kmh * 0.015).max(0.02);
        let gps_speed = if i <= 1 {
            0.0
        } else {
            round_to(
                (kmh + rng.gauss(0.0, sigma)).clamp(SPEED_FLOOR * 3.6, SPEED_CEIL * 3.6),
                4,
            )
        };

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
    apply_post_fixes(&mut locs, &mut rng, start_ms);

    // 点位吸附：经典模式 <40m 吸附；自定义规划模式（pin_start=true）仅在 <=15m 时吸附，
    // 防止把用户未经过、距离路线二三十米外的打卡点强行拉过来算作已打卡。
    let snap_limit = if pin_start { 15.0 * 15.0 } else { 40.0 * 40.0 };
    for pl in points_bd {
        let mut best_i = None;
        let mut best_d = 1e18f64;
        for (i, q) in locs.iter().enumerate() {
            let dd = ((q.gLat - pl.0) * meters_per_degree_lat(pl.0)).powi(2)
                + ((q.gLng - pl.1) * meters_per_degree_lng(pl.0)).powi(2);
            if dd < best_d {
                best_d = dd;
                best_i = Some(i);
            }
        }
        if let Some(i) = best_i {
            if best_d < snap_limit {
                locs[i].gLat = round_to(pl.0, 7);
                locs[i].gLng = round_to(pl.1, 7);
            }
        }
    }

    let total_dis = round_to(dist, 3);
    let mut track = Track {
        totalTime: round_to(t_acc, 0) as i64,
        totalDistance: total_dis,
        validDistance: total_dis,
        validTime: round_to(t_acc, 0) as i64,
        startTime: start_ms,
        startLatitude: locs[0].gLat,
        startLongitude: locs[0].gLng,
        totalSteps: steps_acc as i64,
        locations: locs,
        speedPerTenSec: Vec::new(),
        stepsPerTenSec: Vec::new(),
        segments: Vec::new(),
    };
    track.rebuild_statistics();
    track
}
