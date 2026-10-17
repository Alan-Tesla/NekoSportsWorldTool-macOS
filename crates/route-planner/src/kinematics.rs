//! 运动学速度剖面：OU 配速 + 弯道预判刹车（有限减速度 lookahead）。
//!
//! 与逐点硬限速的区别：真人不会在入弯瞬间才减速，而是在进入弯道前一段距离
//! 就以有限制动减速度（约 1.5 m/s²）开始收速。这里对每个弧长 s 向后看
//! `lookahead_m` 范围内的曲率限速，并施加 `v_now² ≤ v_corner² + 2·b·d` 的
//! 可刹停约束，得到平滑且带预判的速度剖面。

use rand::rngs::StdRng;
use rand::SeedableRng;

use crate::smooth::turn_speed_limit;
use crate::Route;

#[derive(Clone, Copy, Debug)]
pub struct KinParams {
    /// 最大向心加速度 m/s²（弯道限速上限）。
    pub a_c_max: f64,
    /// 最大制动减速度 m/s²（人体可接受的收速斜率）。
    pub brake_max: f64,
    /// 弯道预判前瞻距离（米）。
    pub lookahead_m: f64,
    /// 热身时长（秒），起点从 0.90× 匀速 ramp 到 1.0×。
    pub warmup_s: f64,
    /// OU 均值回归速率 θ（越小越持久）。
    pub theta: f64,
    /// OU 波动率 σ（m/s / √s）。
    pub sigma: f64,
}

impl Default for KinParams {
    fn default() -> Self {
        KinParams {
            a_c_max: 2.5,
            brake_max: 1.5,
            lookahead_m: 40.0,
            warmup_s: 30.0,
            theta: 1.0 / 75.0,
            sigma: 0.0046,
        }
    }
}

/// Illustrative pacing profile, not an observation of any particular runner.
/// Cruise steadily after warmup; introduce gradual fatigue only after 60% elapsed.
/// The final target speed is 7% below cruising speed, without abrupt phase changes.
pub fn steady_pacing_factor(elapsed: f64, duration: f64, warmup: f64) -> f64 {
    let u = (elapsed / duration.max(1.0)).clamp(0.0, 1.0);
    let warm = if warmup > 0.0 {
        let t = (elapsed / warmup).clamp(0.0, 1.0);
        0.90 + 0.10 * t * t * (3.0 - 2.0 * t)
    } else { 1.0 };
    let t = ((u - 0.60) / 0.40).clamp(0.0, 1.0);
    let fatigue = t * t * (3.0 - 2.0 * t);
    warm * (1.0 - 0.07 * fatigue)
}

/// Correlated variation with a 75s OU timescale and a 25s low-pass response.
/// Filtering the variation (not the fatigue trend) removes per-sample jitter.
/// The timescales describe a stochastic process, not fixed oscillation periods.
pub fn slow_pace_noise(latent: &mut f64, smoothed: &mut f64, dt: f64,
    standard_normal: f64, theta: f64, sigma: f64) -> f64 {
    let theta = theta.max(1e-6);
    let decay = (-theta * dt).exp();
    let sd = sigma * ((1.0 - decay * decay) / (2.0 * theta)).sqrt();
    *latent = *latent * decay + sd * standard_normal;
    let response = 1.0 - (-dt / 25.0).exp();
    *smoothed += response * (*latent - *smoothed);
    *smoothed
}

/// 从弧长 s 出发，在 lookahead 范围内取最小的「可刹停速度」。
///
/// 对每个前方曲率限速 v_corner(q)（距离 d=q.s-s），当前速度最多允许
/// `sqrt(v_corner² + 2·b·d)`，取所有候选的最小值。
pub fn speed_limit_ahead(route: &Route, s: f64, p: &KinParams) -> f64 {
    let mut v_max = f64::INFINITY;
    let total = route.length_m;
    let s_mod = if total > 1.0 {
        s.rem_euclid(total)
    } else {
        s
    };
    for pt in &route.points {
        let mut d = pt.s - s_mod;
        if d <= 0.0 && total > 1.0 {
            d += total;
        }
        if d <= 0.0 || d > p.lookahead_m {
            continue;
        }
        let vc = turn_speed_limit(p.a_c_max, pt.curvature.abs());
        if vc.is_finite() {
            let ok = (vc * vc + 2.0 * p.brake_max * d).sqrt();
            if ok < v_max {
                v_max = ok;
            }
        }
    }
    v_max
}

/// 生成速度序列：OU 均值回归 + 热身 ramp + 弯道预判限速 + 精确命中目标距离。
///
/// 最终逐点速度被约束在 `[floor, min(ceil, 弯道限速)]` 内，并等比分摊使总距离
/// 精确等于 `target_dist`（越界点钳在 cap，剩余差量由未饱和点承担）。
pub fn pace_profile(
    route: &Route,
    v0: f64,
    dts: &[f64],
    target_dist: f64,
    seed: u64,
    p: &KinParams,
    floor: f64,
    ceil: f64,
) -> Vec<f64> {
    use rand_distr::{Distribution, Normal};

    let n = dts.len();
    let mut rng = StdRng::seed_from_u64(seed);
    let normal = Normal::<f64>::new(0.0, 1.0).unwrap();
    let total_time_est: f64 = dts.iter().sum::<f64>().max(1.0);

    let mut w = Vec::with_capacity(n);
    let mut latent = 0.0;
    let mut smoothed = 0.0;
    let mut t = 0.0;
    for &dt in dts {
        let variation = slow_pace_noise(&mut latent, &mut smoothed, dt,
            normal.sample(&mut rng), p.theta, p.sigma);
        let target_v = v0 * steady_pacing_factor(t, total_time_est, p.warmup_s);
        w.push((target_v + variation).clamp(floor, ceil));
        t += dt;
    }

    // ② 弧长近似（未限速前）
    let mut s_of = Vec::with_capacity(n);
    let mut s = 0.0;
    for i in 0..n {
        s_of.push(s);
        s += w[i] * dts[i];
    }

    // ③ 弯道预判限速 cap（钳到有效窗口内）
    let mut caps = Vec::with_capacity(n);
    for i in 0..n {
        let cap = speed_limit_ahead(route, s_of[i], p).clamp(floor, ceil);
        w[i] = w[i].min(cap);
        caps.push(cap);
    }

    // ④ 精确命中目标距离（尊重逐点 cap）
    fit_capped(&mut w, dts, target_dist, floor, &caps);
    w
}

fn fit_capped(w: &mut [f64], dts: &[f64], target: f64, floor: f64, caps: &[f64]) {
    for _ in 0..64 {
        let cur: f64 = w.iter().zip(dts).map(|(x, dt)| x * dt).sum();
        if (cur - target).abs() <= 0.5 {
            break;
        }
        let k = target / cur;
        for i in 0..w.len() {
            w[i] = (w[i] * k).clamp(floor, caps[i]);
        }
    }
    for i in 0..w.len() {
        w[i] = w[i].clamp(floor, caps[i]);
    }
}
