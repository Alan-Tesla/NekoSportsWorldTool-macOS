//! 起终点协议标记与时间戳一致化；不改变累计距离、步数或终点用时。

use super::geom::{fmt_gain_time, round_to, Rng};
use super::model::GenPoint;

pub fn apply_post_fixes(locs: &mut [GenPoint], rng: &mut Rng, start_ms: i64) {
    if locs.len() > 2 {
        // 起点哨兵（type=5→setStartPoint）：全零累计 + state=1
        let sp = &mut locs[1];
        sp.ptype = 5;
        sp.state = 1;
        sp.locType = 1;
        sp.radius = round_to(rng.uniform(2.0, 3.5), 2);
        sp.speed = 0.0;
        sp.avgSpeed = 0.0;
        sp.bdS = 0.0;
        sp.totalDis = 0.0;
        sp.validDis = 0.0;
        sp.totalTime = 0;
        sp.validTime = 0;
        sp.steps = 0;
        // 终点哨兵
        let last = locs.len() - 1;
        let e = &mut locs[last];
        e.ptype = 6;
        e.locType = 1;
        e.radius = round_to(rng.uniform(1.5, 3.0), 2);
    }
    if let Some(first) = locs.first_mut() {
        first.totalTime = 0;
        first.validTime = 0;
        first.totalDis = 0.0;
        first.validDis = 0.0;
        first.steps = 0;
        first.speed = 0.0;
        first.avgSpeed = 0.0;
        first.bdS = 0.0;
        first.state = 1;
        first.ptype = 0;
        first.locType = 1;
    }
    for point in locs {
        point.gainTimeMs = start_ms + point.totalTime * 1000;
        point.gainTime = fmt_gain_time(point.gainTimeMs);
    }
}
