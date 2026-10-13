//! 地图画布绘制原语与交互式路径规划器：等距投影 + 正逆坐标变换 + 光标中心缩放 + 途经点编辑。
//!
//! 供「路网」页与「跑步」页复用，纯离线绘制折线/多边形/打卡点/途经点徽章/方向箭头/比例尺。

use eframe::egui;
use egui::{Align2, Color32, FontId, Pos2, Rect, Stroke, Vec2};

use crate::track::geom::{MET_PER_DEG_LAT, MET_PER_DEG_LNG};

/// 地图路径规划交互结果。
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum PlannerEdit {
    None,
    Added(usize),
    Moved(usize),
    Removed(usize),
}

impl PlannerEdit {
    pub fn changed(self) -> bool {
        !matches!(self, PlannerEdit::None)
    }
}

/// 地图视图状态（世界坐标为度，BD-09 工作系）。
pub struct MapState {
    /// 参考中心（lat, lon）。
    pub center: (f64, f64),
    /// 屏幕每米像素（缩放）。
    pub scale: f32,
    /// 平移偏移（像素）。
    pub offset: Vec2,
    /// 最近是否被拖拽过（供外部判断）。
    pub dragged: bool,
    /// 当前正在拖拽的途经点下标。
    pub dragging_wp: Option<usize>,
    /// 当前鼠标悬停的途经点下标。
    pub hover_wp: Option<usize>,
    /// 当前鼠标所在的地图经纬度 `(lat, lon)`。
    pub hover_coord: Option<(f64, f64)>,
}

impl Default for MapState {
    fn default() -> Self {
        MapState {
            center: (38.9, 121.5),
            scale: 0.3,
            offset: Vec2::ZERO,
            dragged: false,
            dragging_wp: None,
            hover_wp: None,
            hover_coord: None,
        }
    }
}

impl MapState {
    /// 依据一组点（lat, lon）自动适配视野（支持单点或多点）。
    pub fn fit(&mut self, rect: Rect, pts: &[(f64, f64)]) {
        if pts.is_empty() {
            return;
        }
        let (mut slat, mut slon) = (0.0f64, 0.0f64);
        for (la, lo) in pts {
            slat += *la;
            slon += *lo;
        }
        let n = pts.len() as f64;
        self.center = (slat / n, slon / n);
        if pts.len() == 1 {
            self.scale = 0.5;
            self.offset = Vec2::ZERO;
            return;
        }
        let (mut minx, mut maxx, mut miny, mut maxy) = (
            f64::INFINITY,
            f64::NEG_INFINITY,
            f64::INFINITY,
            f64::NEG_INFINITY,
        );
        for (la, lo) in pts {
            let x = (lo - self.center.1) * MET_PER_DEG_LNG;
            let y = (la - self.center.0) * MET_PER_DEG_LAT;
            minx = minx.min(x);
            maxx = maxx.max(x);
            miny = miny.min(y);
            maxy = maxy.max(y);
        }
        let w = (maxx - minx).max(60.0);
        let h = (maxy - miny).max(60.0);
        let sx = rect.width().max(100.0) as f64 / w;
        let sy = rect.height().max(100.0) as f64 / h;
        self.scale = ((sx.min(sy)) * 0.88).clamp(0.002, 200.0) as f32;
        self.offset = Vec2::ZERO;
    }

    /// 度坐标 → 屏幕坐标。
    pub fn to_screen(&self, rect: Rect, lat: f64, lon: f64) -> Pos2 {
        let x = (lon - self.center.1) * MET_PER_DEG_LNG * self.scale as f64;
        let y = -(lat - self.center.0) * MET_PER_DEG_LAT * self.scale as f64;
        let c = rect.center();
        Pos2::new(
            (c.x as f64 + x + self.offset.x as f64) as f32,
            (c.y as f64 + y + self.offset.y as f64) as f32,
        )
    }

    /// 屏幕坐标 → 度坐标 `(lat, lon)`（`to_screen` 的精确逆变换）。
    pub fn from_screen(&self, rect: Rect, pos: Pos2) -> (f64, f64) {
        let c = rect.center();
        let scale = (self.scale as f64).max(1e-6);
        let x = (pos.x as f64 - c.x as f64 - self.offset.x as f64) / scale;
        let y = (pos.y as f64 - c.y as f64 - self.offset.y as f64) / scale;
        let lon = self.center.1 + x / MET_PER_DEG_LNG;
        let lat = self.center.0 - y / MET_PER_DEG_LAT;
        (lat, lon)
    }

    /// 以屏幕指定支点 `pivot` 为中心按倍率缩放，保持支点下方的地理坐标不漂移。
    pub fn zoom_at(&mut self, rect: Rect, pivot: Pos2, factor: f32) {
        let old_scale = self.scale.max(0.002);
        let new_scale = (old_scale * factor).clamp(0.002, 200.0);
        let ratio = new_scale / old_scale;
        let rel = pivot - rect.center();
        self.offset = rel - (rel - self.offset) * ratio;
        self.scale = new_scale;
    }

    /// 处理纯浏览模式的拖拽平移与光标中心滚轮缩放。
    pub fn interact(&mut self, ui: &egui::Ui, rect: Rect) {
        self.dragged = false;
        let (hover_pos, drag, scroll) = ui.input(|i| {
            let pos = i.pointer.hover_pos().or_else(|| i.pointer.interact_pos());
            let hover = pos.map(|p| rect.contains(p)).unwrap_or(false);
            let dragging = hover && i.pointer.primary_down() && i.pointer.is_decidedly_dragging();
            (
                pos.filter(|p| rect.contains(*p)),
                dragging,
                if hover { i.raw_scroll_delta.y } else { 0.0 },
            )
        });
        self.hover_coord = hover_pos.map(|p| self.from_screen(rect, p));
        if drag {
            let d = ui.input(|i| i.pointer.delta());
            self.offset += d;
            self.dragged = true;
        }
        if scroll != 0.0 {
            let factor = (scroll * 0.0015).exp();
            let pivot = hover_pos.unwrap_or_else(|| rect.center());
            self.zoom_at(rect, pivot, factor);
        }
    }

    /// 处理路径规划交互：
    /// - 左键点击地图空白处：新增途经点（靠近打卡点自动吸附，启用道路吸附时自动落路）；
    /// - 左键按住已有途经点拖拽：实时移动该途经点；
    /// - 左键按住空白处拖拽：平移地图；
    /// - 右键点击途经点：删除该途经点（在空白处右键则撤销最后一个途经点）；
    /// - 滚轮：以鼠标指针为中心缩放地图。
    pub fn interact_planner(
        &mut self,
        ui: &egui::Ui,
        rect: Rect,
        response: &egui::Response,
        waypoints: &mut Vec<(f64, f64)>,
        checkpoints: &[(f64, f64)],
        snap_road: Option<&dyn Fn(f64, f64) -> Option<((f64, f64), f64)>>,
    ) -> PlannerEdit {
        let mut edit = PlannerEdit::None;
        self.dragged = false;

        let (pointer_pos, primary_pressed, primary_down, is_dragging, delta, scroll) =
            ui.input(|i| {
                let pos = i.pointer.hover_pos().or_else(|| i.pointer.interact_pos());
                let in_rect = pos.map(|p| rect.contains(p)).unwrap_or(false);
                (
                    pos.filter(|p| rect.contains(*p)),
                    in_rect && i.pointer.primary_pressed(),
                    i.pointer.primary_down(),
                    i.pointer.is_decidedly_dragging(),
                    i.pointer.delta(),
                    if in_rect { i.raw_scroll_delta.y } else { 0.0 },
                )
            });

        self.hover_coord = pointer_pos.map(|p| self.from_screen(rect, p));

        // 命中检测：查找距离光标 14px 内的途经点
        if self.dragging_wp.is_none() {
            self.hover_wp = pointer_pos.and_then(|pos| {
                let mut best: Option<(usize, f32)> = None;
                for (i, &(la, lo)) in waypoints.iter().enumerate() {
                    let sp = self.to_screen(rect, la, lo);
                    let d = sp.distance(pos);
                    if d <= 14.0 && best.map_or(true, |(_, bd)| d < bd) {
                        best = Some((i, d));
                    }
                }
                best.map(|(i, _)| i)
            });
        }

        // 按下左键瞬间：若命中已有途经点，进入途经点拖拽态
        if primary_pressed {
            if let Some(idx) = self.hover_wp {
                self.dragging_wp = Some(idx);
            }
        }

        if primary_down {
            if let Some(idx) = self.dragging_wp {
                if idx < waypoints.len() {
                    if let Some(pos) = pointer_pos {
                        if is_dragging || delta.length_sq() > 0.0 {
                            let pt = self.resolve_point(rect, pos, checkpoints, snap_road);
                            if (waypoints[idx].0 - pt.0).abs() > 1e-8
                                || (waypoints[idx].1 - pt.1).abs() > 1e-8
                            {
                                waypoints[idx] = pt;
                                edit = PlannerEdit::Moved(idx);
                            }
                            self.dragged = true;
                        }
                    }
                } else {
                    self.dragging_wp = None;
                }
            } else if pointer_pos.is_some() && is_dragging {
                self.offset += delta;
                self.dragged = true;
            }
        } else {
            self.dragging_wp = None;
        }

        // 滚轮缩放
        if scroll != 0.0 {
            let factor = (scroll * 0.0015).exp();
            let pivot = pointer_pos.unwrap_or_else(|| rect.center());
            self.zoom_at(rect, pivot, factor);
        }

        // 右键删除：优先删除悬停的途经点，否则撤销末尾途经点
        if response.secondary_clicked() {
            if let Some(idx) = self.hover_wp.filter(|&i| i < waypoints.len()) {
                waypoints.remove(idx);
                self.hover_wp = None;
                self.dragging_wp = None;
                return PlannerEdit::Removed(idx);
            } else if !waypoints.is_empty() {
                let idx = waypoints.len() - 1;
                waypoints.pop();
                return PlannerEdit::Removed(idx);
            }
        }

        // 左键点击空白或打卡点：新增途经点
        if response.clicked() && !self.dragged && self.dragging_wp.is_none() {
            if let Some(pos) = response.interact_pointer_pos().filter(|p| rect.contains(*p)) {
                if self.hover_wp.is_none() {
                    let pt = self.resolve_point(rect, pos, checkpoints, snap_road);
                    waypoints.push(pt);
                    edit = PlannerEdit::Added(waypoints.len() - 1);
                }
            }
        }

        edit
    }

    /// 将屏幕像素位置解析为经纬度：优先吸附 14px 内的打卡点，其次吸附 150m 内的道路。
    fn resolve_point(
        &self,
        rect: Rect,
        pos: Pos2,
        checkpoints: &[(f64, f64)],
        snap_road: Option<&dyn Fn(f64, f64) -> Option<((f64, f64), f64)>>,
    ) -> (f64, f64) {
        // 1. 屏幕 14px 内若有打卡点，优先精确吸附到打卡点
        let mut best_cp: Option<((f64, f64), f32)> = None;
        for &(la, lo) in checkpoints {
            let sp = self.to_screen(rect, la, lo);
            let d = sp.distance(pos);
            if d <= 14.0 && best_cp.map_or(true, |(_, bd)| d < bd) {
                best_cp = Some(((la, lo), d));
            }
        }
        if let Some((cp, _)) = best_cp {
            return cp;
        }
        // 2. 逆投影为度坐标，若提供道路吸附器且在 150m 内则吸附到最近道路
        let (lat, lon) = self.from_screen(rect, pos);
        if let Some(snap_fn) = snap_road {
            if let Some((snapped, dist_m)) = snap_fn(lat, lon) {
                if dist_m <= 150.0 {
                    return snapped;
                }
            }
        }
        (lat, lon)
    }

    pub fn draw_polyline(
        &self,
        painter: &egui::Painter,
        rect: Rect,
        pts: &[(f64, f64)],
        color: Color32,
        width: f32,
    ) {
        if pts.len() < 2 {
            return;
        }
        let ps: Vec<Pos2> = pts
            .iter()
            .map(|(la, lo)| self.to_screen(rect, *la, *lo))
            .collect();
        painter.add(egui::Shape::line(ps, Stroke::new(width, color)));
    }

    /// 绘制带行进方向箭头的路线折线。
    pub fn draw_route_with_arrows(
        &self,
        painter: &egui::Painter,
        rect: Rect,
        pts: &[(f64, f64)],
        color: Color32,
        width: f32,
    ) {
        if pts.len() < 2 {
            return;
        }
        let ps: Vec<Pos2> = pts
            .iter()
            .map(|(la, lo)| self.to_screen(rect, *la, *lo))
            .collect();
        // 浅色描边衬底，提升在道路网上的辨识度
        painter.add(egui::Shape::line(
            ps.clone(),
            Stroke::new(width + 2.0, Color32::from_rgba_unmultiplied(255, 255, 255, 190)),
        ));
        painter.add(egui::Shape::line(ps.clone(), Stroke::new(width, color)));

        // 每隔约 75 像素绘制一个方向箭头
        let mut acc = 35.0f32;
        for w in ps.windows(2) {
            let (a, b) = (w[0], w[1]);
            let seg = a.distance(b);
            if seg < 1.0 {
                continue;
            }
            let dir = (b - a) / seg;
            let normal = Vec2::new(-dir.y, dir.x);
            let mut d = acc;
            while d < seg {
                let tip = a + dir * d;
                if rect.contains(tip) {
                    let wing1 = tip - dir * 6.5 + normal * 4.0;
                    let wing2 = tip - dir * 6.5 - normal * 4.0;
                    painter.add(egui::Shape::line(
                        vec![wing1, tip, wing2],
                        Stroke::new(1.8_f32, Color32::WHITE),
                    ));
                    painter.add(egui::Shape::line(
                        vec![wing1, tip, wing2],
                        Stroke::new(1.3_f32, color),
                    ));
                }
                d += 75.0;
            }
            acc = d - seg;
        }
    }

    pub fn draw_polygon(
        &self,
        painter: &egui::Painter,
        rect: Rect,
        pts: &[(f64, f64)],
        color: Color32,
        width: f32,
    ) {
        if pts.len() < 3 {
            return;
        }
        let mut ps: Vec<Pos2> = pts
            .iter()
            .map(|(la, lo)| self.to_screen(rect, *la, *lo))
            .collect();
        ps.push(ps[0]);
        painter.add(egui::Shape::closed_line(ps, Stroke::new(width, color)));
    }

    pub fn draw_point(
        &self,
        painter: &egui::Painter,
        rect: Rect,
        lat: f64,
        lon: f64,
        color: Color32,
        radius: f32,
    ) {
        let c = self.to_screen(rect, lat, lon);
        painter.circle_filled(c, radius, color);
    }

    /// 绘制带序号与起终点配色的途经点徽章。
    pub fn draw_waypoints(
        &self,
        painter: &egui::Painter,
        rect: Rect,
        waypoints: &[(f64, f64)],
    ) {
        let total = waypoints.len();
        for (i, &(lat, lon)) in waypoints.iter().enumerate() {
            let pos = self.to_screen(rect, lat, lon);
            if !rect.expand(20.0).contains(pos) {
                continue;
            }
            let is_active = self.dragging_wp == Some(i) || self.hover_wp == Some(i);
            let fill = if i == 0 {
                Color32::from_rgb(22, 160, 90) // 起点绿
            } else if i + 1 == total {
                Color32::from_rgb(220, 50, 47) // 末点红
            } else {
                Color32::from_rgb(30, 111, 216) // 途经点蓝
            };
            let radius = if is_active { 10.5 } else { 8.5 };
            if is_active {
                painter.circle_stroke(
                    pos,
                    radius + 3.0,
                    Stroke::new(
                        2.0_f32,
                        Color32::from_rgba_unmultiplied(fill.r(), fill.g(), fill.b(), 140),
                    ),
                );
            }
            painter.circle_filled(pos, radius, fill);
            painter.circle_stroke(pos, radius, Stroke::new(1.8_f32, Color32::WHITE));
            let text = if i == 0 {
                "起".to_string()
            } else {
                format!("{}", i + 1)
            };
            painter.text(
                pos,
                Align2::CENTER_CENTER,
                text,
                FontId::proportional(10.5),
                Color32::WHITE,
            );
        }
    }

    /// 在地图左下角绘制自适应米制比例尺。
    pub fn draw_scale_bar(&self, painter: &egui::Painter, rect: Rect) {
        let px_per_m = self.scale.max(1e-4);
        // 目标像素宽度约 60~120px，从标准档位中挑选最接近的一档
        let candidates = [10.0f32, 20.0, 50.0, 100.0, 200.0, 500.0, 1000.0, 2000.0, 5000.0];
        let mut chosen_m = 100.0f32;
        for &m in &candidates {
            let px = m * px_per_m;
            chosen_m = m;
            if px >= 55.0 {
                break;
            }
        }
        let bar_px = (chosen_m * px_per_m).clamp(32.0, 160.0);
        let x0 = rect.left() + 10.0;
        let y0 = rect.bottom() - 10.0;
        let x1 = x0 + bar_px;
        let color = Color32::from_rgb(70, 82, 78);
        painter.line_segment(
            [Pos2::new(x0, y0), Pos2::new(x1, y0)],
            Stroke::new(2.0_f32, color),
        );
        painter.line_segment(
            [Pos2::new(x0, y0 - 4.0), Pos2::new(x0, y0 + 1.0)],
            Stroke::new(2.0_f32, color),
        );
        painter.line_segment(
            [Pos2::new(x1, y0 - 4.0), Pos2::new(x1, y0 + 1.0)],
            Stroke::new(2.0_f32, color),
        );
        let label = if chosen_m >= 1000.0 {
            format!("{:.0} km", chosen_m / 1000.0)
        } else {
            format!("{:.0} m", chosen_m)
        };
        painter.text(
            Pos2::new((x0 + x1) * 0.5, y0 - 4.0),
            Align2::CENTER_BOTTOM,
            label,
            FontId::proportional(11.0),
            color,
        );
    }

    /// 在地图右下角绘制悬停坐标或操作提示。
    pub fn draw_hud_footer(&self, painter: &egui::Painter, rect: Rect, interactive: bool) {
        let text = match (self.hover_coord, interactive) {
            (Some((lat, lon)), true) => {
                format!("BD ({lat:.6}, {lon:.6}) · 左键加点 / 拖拽调点 / 右键删点 / 滚轮缩放")
            }
            (Some((lat, lon)), false) => {
                format!("BD ({lat:.6}, {lon:.6}) · 拖拽平移 / 滚轮缩放")
            }
            (None, true) => "左键点击加点 · 拖拽途经点移动 · 右键删除途经点 · 滚轮缩放".into(),
            (None, false) => "拖拽平移 · 滚轮缩放".into(),
        };
        painter.text(
            Pos2::new(rect.right() - 8.0, rect.bottom() - 8.0),
            Align2::RIGHT_BOTTOM,
            text,
            FontId::proportional(11.0),
            Color32::from_rgb(105, 118, 112),
        );
    }
}

/// 收集一组折线/环的所有点，用于 fit。
pub fn collect_bounds(edges: &[Vec<(f64, f64)>]) -> Vec<(f64, f64)> {
    let mut pts: Vec<(f64, f64)> = Vec::new();
    for e in edges.iter().take(400) {
        for p in e {
            pts.push(*p);
        }
    }
    pts
}

/// 综合电子围栏、自定义途经点、规划路线、打卡点和道路计算视野边界点。
pub fn collect_scene_bounds(
    fences: &[Vec<(f64, f64)>],
    waypoints: &[(f64, f64)],
    route: &[(f64, f64)],
    checkpoints: &[(f64, f64)],
    edges: &[Vec<(f64, f64)>],
) -> Vec<(f64, f64)> {
    let mut bounds: Vec<(f64, f64)> = Vec::new();
    for f in fences {
        bounds.extend(f.iter().copied());
    }
    bounds.extend(waypoints.iter().copied());
    bounds.extend(route.iter().copied());
    bounds.extend(checkpoints.iter().copied());
    if bounds.is_empty() {
        bounds = collect_bounds(edges);
    }
    bounds
}

/// 地图图例：颜色方块 + 说明文字。
pub fn legend(ui: &mut egui::Ui, items: &[(&str, Color32)]) {
    ui.horizontal_wrapped(|ui| {
        ui.label("图例：");
        for (label, color) in items {
            let (rect, _) = ui.allocate_exact_size(Vec2::new(10.0, 10.0), egui::Sense::hover());
            ui.painter()
                .rect_filled(rect, egui::Rounding::same(2.0), *color);
            ui.label(*label);
        }
    });
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn screen_and_world_coordinates_roundtrip_accurately() {
        let mut map = MapState {
            center: (38.889123, 121.531456),
            scale: 0.65,
            offset: Vec2::new(42.0, -18.5),
            ..Default::default()
        };
        let rect = Rect::from_min_size(Pos2::new(20.0, 40.0), Vec2::new(640.0, 420.0));
        for &(lat, lon) in &[
            (38.889123, 121.531456),
            (38.892500, 121.536200),
            (38.885000, 121.528000),
        ] {
            let screen = map.to_screen(rect, lat, lon);
            let (rt_lat, rt_lon) = map.from_screen(rect, screen);
            assert!(
                (rt_lat - lat).abs() < 1e-6 && (rt_lon - lon).abs() < 1e-6,
                "roundtrip mismatch: ({lat}, {lon}) -> ({rt_lat}, {rt_lon})"
            );
        }

        // 光标中心缩放后，支点处的世界坐标在屏幕上的位置保持不变
        let pivot = Pos2::new(210.0, 190.0);
        let (plat, plon) = map.from_screen(rect, pivot);
        map.zoom_at(rect, pivot, 1.8);
        let after_screen = map.to_screen(rect, plat, plon);
        assert!(
            after_screen.distance(pivot) < 0.05,
            "zoom_at shifted pivot: {after_screen:?} vs {pivot:?}"
        );
    }
}
