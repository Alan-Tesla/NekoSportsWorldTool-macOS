//! 自然轨迹生成器。
//!
//! 画像驱动：打卡点拟合闭合环（Catmull-Rom，每段 18 采样）→ 弧长表；
//! 采样间隔主 5s（80%）；速度曲线 = ramp × 疲劳 × 三正弦 × 余弦凹陷 × 微噪；
//! 正常点按速度曲线分配位移并归一到精确总距离；异常点（-1）零位移/跳变；
//! 哨兵点（首 type∈{0,7}、索引1 type=5、末 type=6）；结尾断崖；点位吸附。
#![allow(non_snake_case)]

use super::geom::{
    fmt_gain_time, make_ordered_path, make_point_ring, open_point_at, ring_point_at, round_to,
    to_bd, Rng, MET_PER_DEG_LAT, MET_PER_DEG_LNG,
};
use super::model::{GenPoint, Segment, TenWindow, Track};
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
    let phase_v = rng.uniform(0.0, std::f64::consts::TAU);
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

    let base = (dist / dur_f).clamp(SPEED_FLOOR + 0.05, SPEED_CEIL - 0.05);
    let local_floor = (base * 0.78).max(SPEED_FLOOR);
    let local_ceil = (base * 1.20).min(SPEED_CEIL).max(local_floor + 0.05);

    // 真实学生体能分布曲线：
    // 首公里轻快充沛（比均速快 15~20%，如实测 5'48" vs 平均 7'17"），中程平稳（6'45"），后程疲劳减速（8'05"~9'02"）
    let n_dips = rng.choice(&[2, 3]);
    let mut dips = Vec::new();
    for _ in 0..n_dips {
        dips.push((
            rng.uniform(0.45, 0.92) * dur_f,
            rng.uniform(18.0, 36.0),
            rng.uniform(0.08, 0.18),
        ));
    }
    let dip_factor = |t: f64| -> f64 {
        let mut f = 1.0;
        for &(c, hw, d) in &dips {
            if (t - c).abs() < hw {
                f *= 1.0 - d * 0.5 * (1.0 + (std::f64::consts::PI * (t - c) / hw).cos());
            }
        }
        f
    };

    let mut w = Vec::with_capacity(n);
    for &tt in &times {
        let frac = (tt / dur_f).clamp(0.0, 1.0);
        let pacing_curve = if frac < 0.28 {
            1.19 - 0.11 * (frac / 0.28)
        } else if frac < 0.60 {
            1.08 - 0.10 * ((frac - 0.28) / 0.32)
        } else {
            0.98 - 0.20 * ((frac - 0.60) / 0.40)
        };
        let wave = 1.0
            + 0.008 * (std::f64::consts::TAU * tt / 115.0 + phase_v).sin()
            + 0.018 * (std::f64::consts::TAU * tt / 47.0 + phase_v * 2.3).sin();
        let noise = 1.0 + rng.gauss(0.0, 0.006);
        let target_v = base * pacing_curve * wave * noise * dip_factor(tt);
        w.push(target_v.clamp(local_floor, local_ceil));
    }
    let mut dts: Vec<f64> = (0..n - 1).map(|i| times[i + 1] - times[i]).collect();
    dts.push(1.0f64.max(dur_f - times[n - 1]));

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

    // 先将非正常移动点（哨兵 i<=1 及异常点 -1）的速度置 0，再直接在正常点上做带界速度拟合，
    // 确保每个正常点的实际位移速度 d_step / dt 严格落在 4~10 分配速窗口内，不因分摊而被二次放大。
    for i in 0..n {
        if i <= 1 || kinds[i].0 == -1 {
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
    let seg_dist: Vec<f64> = disp_of.clone();
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
    let mut ten_t = 0.0f64;
    let mut ten_d = 0.0f64;
    let mut ten_st = 0.0f64;
    let mut ten_speed: Vec<TenWindow> = Vec::new();
    let mut ten_steps: Vec<TenWindow> = Vec::new();

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
        let mut d_step = 0.0f64;
        let px;
        let py;
        let x;
        let y;
        let rad;
        let state;
        if typ != -1 {
            d_step = disp_of[i];
            s += direction * d_step;
            let (bx, by) = pos(s);
            x = bx;
            y = by;
            jx = 0.72 * jx + rng.gauss(0.0, 0.75);
            jy = 0.72 * jy + rng.gauss(0.0, 0.75);
            px = bx + jx;
            py = by + jy;
            rad = round_to(
                if typ == 3 { rng.uniform(1.4, 5.1) } else { rng.uniform(1.4, 2.4) },
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
                        d_step = if rng.random() < 0.95 { rng.uniform(2.0, 60.0) } else { rng.uniform(60.0, 250.0) };
                    }
                }
                1 => {
                    if rng.random() >= 0.83 {
                        d_step = rng.uniform(0.5, 36.0);
                    }
                }
                12 => {
                    d_step = if rng.random() < 0.9 { rng.uniform(5.0, 80.0) } else { rng.uniform(80.0, 220.0) };
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
                // 零位移：精确复制上一点坐标（BD→平面）
                py = (last.gLat - c_lat) * MET_PER_DEG_LAT;
                px = (last.gLng - c_lng) * MET_PER_DEG_LNG;
            } else {
                px = x + jx;
                py = y + jy;
            }
            rad = if lt == 4 {
                round_to(if rng.random() < 0.75 { rng.uniform(30.0, 100.0) } else { rng.uniform(100.0, 550.0) }, 2)
            } else if lt == 1 {
                round_to(if rng.random() < 0.75 { rng.uniform(1.6, 12.0) } else { rng.uniform(12.0, 95.0) }, 2)
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
            dist_acc += d_step; // 异常点漂移不计入累计距离
        }
        let (lat, lng) = to_bd(px, py, c_lat, c_lng);
        alt += 0.04 * (82.0 - alt) + rng.gauss(0.0, alt_sigma);
        // 生物力学高步频小步幅联动（平均步频约 185 spm，平均步幅约 74 cm，严格满足 v = cad/60 * stride）
        let v_now = if typ != -1 { speeds[i] } else { base };
        let km_done = (t_acc / dur_f) * dist / 1000.0;
        let fat = route_planner::fatigue_from_km(km_done);
        let gait = route_planner::gait(v_now, fat);
        let u_artifact = rng.random();
        let cad = if u_artifact < 0.015 {
            rng.uniform(255.0, 320.0) // 模拟口袋晃动瞬时峰值（实测最大 320）
        } else if u_artifact < 0.030 {
            rng.uniform(72.0, 120.0)  // 模拟起步/减速低点（实测最小 72）
        } else {
            (gait.cadence + rng.gauss(0.0, 2.5)).clamp(160.0, 205.0)
        };
        let stride = (v_now * 60.0 / cad).clamp(0.32, 1.25);
        let cad = v_now * 60.0 / stride;
        steps_acc += cad / 60.0 * dt;
        let nxt = pos(s + direction * 2.0);
        let brg = ((nxt.0 - x).atan2(nxt.1 - y).to_degrees() + rng.gauss(0.0, 35.0)).rem_euclid(360.0);
        // 异常点（-1）：avgSpeed 为累计均值（真人与此一致，不为 0）；
        // GPS 瞬时速度多为低速，偶发 15-46 km/h 漂移尖峰
        let (avg_sp, gps_speed) = if typ == -1 {
            let avg = round_to((dist_acc / t_acc.max(1.0)).clamp(SPEED_FLOOR, SPEED_CEIL), 4);
            let gps = rng.uniform(SPEED_FLOOR * 3.6, SPEED_CEIL * 3.6);
            (avg, round_to(gps, 4))
        } else {
            let avg = round_to((d_step / dt).clamp(SPEED_FLOOR, SPEED_CEIL), 4);
            let kmh = avg * 3.6;
            let sigma = (kmh * 0.06).max(0.05);
            let gps = if rng.random() < 0.20 {
                0.0
            } else {
                round_to(
                    (kmh + rng.gauss(0.0, sigma)).clamp(SPEED_FLOOR * 3.6, SPEED_CEIL * 3.6),
                    4,
                )
            };
            (avg, gps)
        };

        ten_t += dt;
        ten_d += speeds[i] * dt;
        ten_st += cad / 60.0 * dt;
        while ten_t >= 10.0 {
            let k = 10.0 / ten_t; // 只取前 10s 的量，余量留给下一窗
            let (out_d, out_st) = (ten_d * k, ten_st * k);
            ten_speed.push(TenWindow { time: 10, value: round_to(out_d, 2) });
            ten_steps.push(TenWindow { time: 10, value: round_to(out_st, 0) });
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
        // 尾窗：限制放大倍数 ≤1.4
        let k = (10.0 / ten_t).min(1.4);
        ten_speed.push(TenWindow { time: 10, value: round_to(ten_d * k, 2) });
        ten_steps.push(TenWindow { time: 10, value: round_to(ten_st * k, 0) });
    }

    let mut segments: Vec<Segment> = Vec::new();
    let (mut seg_t, mut seg_d, mut seg_n) = (0.0f64, 0.0f64, 0i64);
    let mut seg_v: Vec<f64> = Vec::new();
    for i in 0..n {
        seg_t += dts[i];
        seg_d += seg_dist[i];
        seg_v.push(speeds[i]);
        seg_n += 1;
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
            seg_n = 0;
        }
    }
    let _ = seg_n;

    apply_post_fixes(&mut locs, &mut rng, start_ms);

    // 点位吸附：经典模式 <40m 吸附；自定义规划模式（pin_start=true）仅在 <=15m 时吸附，
    // 防止把用户未经过、距离路线二三十米外的打卡点强行拉过来算作已打卡。
    let snap_limit = if pin_start { 15.0 * 15.0 } else { 40.0 * 40.0 };
    for pl in points_bd {
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
            if best_d < snap_limit {
                locs[i].gLat = round_to(pl.0, 7);
                locs[i].gLng = round_to(pl.1, 7);
            }
        }
    }

    let total_dis = round_to(dist, 3);
    Track {
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
    }
}
