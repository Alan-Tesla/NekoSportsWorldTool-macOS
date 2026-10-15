//! 生物力学联动：配速 v、步频 f_cadence、步幅 L_stride 的非线性耦合。
//!
//!   硬约束：v = (f_cadence / 60) · L_stride   （v: m/s，f: 步/分，L: 米）
//!
//! 由速度与疲劳度确定 (步频, 步幅)，**始终严格满足等式**。

/// 步态结果。
#[derive(Clone, Copy, Debug)]
pub struct Gait {
    /// 步频（步/分）。
    pub cadence: f64,
    /// 步幅（米）。
    pub stride: f64,
}

/// 由速度与疲劳度计算步态，严格满足 `v = cadence/60 * stride`。
///
/// 大学校园真实跑步特征（参考实测数据）：
/// - 平均步频约 180~190 spm（基准约 185 spm）；
/// - 平均步幅约 65~85 cm（基准约 74 cm）；
/// - 后程体能疲劳时，步频保持相对稳定（约 175~185 spm），主要通过收紧步幅（降至 55~70 cm）来降低速度；
/// - 步幅包络严格落在 [0.32, 1.25] 米（对应实测最小 32cm、最大 116cm）。
pub fn gait(v: f64, fatigue: f64) -> Gait {
    // 基础步频：速度越高步频微升，疲劳时步频仅极轻微下降（真人主要靠收步幅减速）
    let target_cad = (176.0 + 5.2 * v) * (1.0 - 0.025 * fatigue);
    let mut cadence = target_cad.clamp(160.0, 205.0);
    let mut stride = v * 60.0 / cadence;
    // 步幅硬包络：实测最小 0.32m（走步/极慢起跑），最大 1.25m（冲刺快跑）
    if stride > 1.25 {
        stride = 1.25;
        cadence = v * 60.0 / stride;
    } else if stride < 0.32 {
        stride = 0.32;
        cadence = v * 60.0 / stride;
    }
    Gait { cadence, stride }
}

/// 由速度与疲劳度取步幅（米），包络 [0.55, 1.35]。
pub fn stride_for(v: f64, fatigue: f64) -> f64 {
    gait(v, fatigue).stride
}

/// 由速度反解步频（步/分）。
pub fn cadence_for(v: f64, stride: f64) -> f64 {
    (60.0 * v / stride.max(1e-6)).clamp(110.0, 220.0)
}

/// 由步频与步幅计算速度（m/s）。
pub fn speed_from(cadence: f64, stride: f64) -> f64 {
    (cadence / 60.0) * stride
}

/// 疲劳度（0..0.85）：随已跑里程（公里）与已用时间比例自然抬升。
pub fn fatigue_from_km(km: f64) -> f64 {
    if km <= 0.6 {
        0.0
    } else {
        ((km - 0.6) * 0.30).min(0.85)
    }
}

/// 步频 OU 耦合：围绕可配置目标连续波动，不人为注入手机传感器误检尖峰。
pub fn ou_cadence_series(
    targets: &[f64],
    dts: &[f64],
    seed: u64,
    theta: f64,
    sigma: f64,
    floor: f64,
    ceil: f64,
) -> Vec<f64> {
    use rand::rngs::StdRng;
    use rand::SeedableRng;
    use rand_distr::{Distribution, Normal};

    let mut rng = StdRng::seed_from_u64(seed);
    let normal = Normal::<f64>::new(0.0, 1.0).unwrap();
    let mut out = Vec::with_capacity(targets.len());
    let mut c = targets.first().copied().unwrap_or(185.0);
    for i in 0..targets.len() {
        let dt = dts[i];
        let rate = theta.max(1e-6);
        let decay = (-rate*dt).exp();
        let noise_sd = sigma*((1.0-decay*decay)/(2.0*rate)).sqrt();
        c = targets[i]+(c-targets[i])*decay+noise_sd*normal.sample(&mut rng);
        out.push(c.clamp(floor, ceil));
    }
    out
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn gait_coupling_equation_holds() {
        for v in [1.7f64, 2.3, 2.9, 3.5, 4.1] {
            for fat in [0.0f64, 0.3, 0.6] {
                let g = gait(v, fat);
                let back = speed_from(g.cadence, g.stride);
                assert!((back - v).abs() < 1e-6, "v={v} back={back}");
                assert!((0.32..=1.25).contains(&g.stride), "stride={}", g.stride);
                assert!((155.0..=215.0).contains(&g.cadence), "cad={}", g.cadence);
            }
        }
    }

    #[test]
    fn cadence_ou_stays_in_envelope() {
        let n = 200;
        let targets: Vec<f64> = (0..n).map(|i| 180.0 + 0.05 * i as f64).collect();
        let dts = vec![5.0; n];
        let cad = ou_cadence_series(&targets, &dts, 42, 0.25, 1.8, 160.0, 205.0);
        assert_eq!(cad.len(), n);
        assert!(cad.iter().all(|c| (160.0..=205.0).contains(c)));
        let avg_cad = cad.iter().sum::<f64>() / n as f64;
        assert!(
            (180.0..=190.0).contains(&avg_cad),
            "平均步频应稳定在 180~190 之间，实际: {avg_cad}"
        );
    }

    #[test]
    fn fatigue_monotonic_capped() {
        assert_eq!(fatigue_from_km(0.4), 0.0);
        assert!(fatigue_from_km(1.0) > 0.0);
        assert_eq!(fatigue_from_km(5.0), 0.85);
    }
}
