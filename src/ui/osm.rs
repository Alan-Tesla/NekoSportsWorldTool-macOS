//! 「路网」与路径规划页：选择 OSM 文件 → 后台解析 → 交互式地图规划路径。
//!
//! 统一在 BD-09 工作坐标系下展示道路、建筑、电子围栏、学校打卡点与自定义途经点，
//! 支持左键点击加点、拖拽移动途经点、右键删除途经点、打卡点/道路自动吸附，以及一键应用到跑步页。

use eframe::egui;
use std::sync::Arc;

use route_planner::RoadGraph;

use super::map::{collect_scene_bounds, MapState};
use super::{mobile, theme, App};
use crate::track::generate_road::{
    plan_custom_view, snap_point_to_aligned_road, CustomCloseMode, RoadPlan, RouteMode,
};

pub struct OsmPage {
    pub path: String,
    pub busy: bool,
    pub status: String,
    pub map: MapState,
    pub fitted: bool,
    pub snap_road: bool,
    pub preview: Option<RoadPlan>,
    pub preview_stale: bool,
}

impl Default for OsmPage {
    fn default() -> Self {
        OsmPage {
            path: String::new(),
            busy: false,
            status: String::new(),
            map: MapState::default(),
            fitted: false,
            snap_road: true,
            preview: None,
            preview_stale: true,
        }
    }
}

impl App {
    pub fn draw_osm_page(&mut self, ui: &mut egui::Ui) {
        let mut load_now = false;

        // ── 第 1 栏：OSM 路网加载与数据拉取 ─────────────────────────
        mobile::row(ui, |ui| {
            ui.label("OSM 文件：");
            ui.add_sized(
                [280.0f32.min(ui.available_width().max(120.0)), 26.0],
                egui::TextEdit::singleline(&mut self.osm_page.path)
                    .hint_text("拖入 .osm 文件或填写路径"),
            );
            #[cfg(not(target_os = "android"))]
            if ui.button("浏览…").clicked() {
                if let Some(p) = rfd::FileDialog::new()
                    .add_filter("OSM", &["osm", "xml", "pbf"])
                    .pick_file()
                {
                    self.osm_page.path = p.display().to_string();
                }
            }
            let can_load = !self.osm_page.path.is_empty() && !self.osm_page.busy;
            if ui
                .add_enabled(can_load, theme::primary_btn("加载路网"))
                .clicked()
            {
                load_now = true;
            }
            if ui
                .add_enabled(self.session.is_some(), egui::Button::new("刷新打卡点/围栏"))
                .on_hover_text("从服务端拉取最新校园打卡点与电子围栏")
                .clicked()
            {
                self.refresh_fence();
                self.refresh_points();
            }
        });

        // 拖拽导入
        let dropped: Vec<String> = ui.ctx().input(|i| {
            i.raw
                .dropped_files
                .iter()
                .filter_map(|f| f.path.as_ref().map(|p| p.display().to_string()))
                .collect()
        });
        if let Some(p) = dropped.into_iter().find(|p| !p.is_empty()) {
            self.osm_page.path = p;
            if !self.osm_page.busy {
                load_now = true;
            }
        }

        if !self.osm_page.status.is_empty() {
            ui.colored_label(theme::plain(), &self.osm_page.status);
        }

        // ── 第 2 栏：路径规划工具栏 ─────────────────────────────────
        let mut close_mode = CustomCloseMode::from_str(&self.config.custom_close_mode);
        let prev_close = close_mode;
        let prev_clip = self.config.custom_clip_fence;
        let mut need_sync = false;
        let mut fit_requested = false;
        let mut zoom_delta: Option<f32> = None;
        let mut apply_to_run = false;
        let mut chain_checkpoints = false;

        ui.add_space(2.0);
        mobile::row(ui, |ui| {
            ui.label("规划模式：");
            egui::ComboBox::from_id_salt("osm_custom_close_mode")
                .width(185.0)
                .selected_text(close_mode.label())
                .show_ui(ui, |ui| {
                    for m in [
                        CustomCloseMode::Loop,
                        CustomCloseMode::OutAndBack,
                        CustomCloseMode::AutoPadLoop,
                    ] {
                        ui.selectable_value(&mut close_mode, m, m.label());
                    }
                });
            ui.checkbox(&mut self.osm_page.snap_road, "道路吸附");
            ui.checkbox(&mut self.config.custom_clip_fence, "围栏裁剪");
            ui.separator();
            if ui
                .button("串联打卡点")
                .on_hover_text("按环序将当前学校的打卡点填入途经点，可在此基础上拖拽微调")
                .clicked()
            {
                chain_checkpoints = true;
            }
            if ui
                .add_enabled(
                    !self.config.custom_waypoints.is_empty(),
                    egui::Button::new("撤销上一点"),
                )
                .clicked()
            {
                self.config.custom_waypoints.pop();
                need_sync = true;
            }
            if ui
                .add_enabled(
                    self.config.custom_waypoints.len() >= 2,
                    egui::Button::new("反转方向"),
                )
                .clicked()
            {
                self.config.custom_waypoints.reverse();
                need_sync = true;
            }
            if ui
                .add_enabled(
                    !self.config.custom_waypoints.is_empty(),
                    egui::Button::new("清空"),
                )
                .clicked()
            {
                self.config.custom_waypoints.clear();
                need_sync = true;
            }
            ui.separator();
            if ui.button("居中适配").clicked() {
                fit_requested = true;
            }
            if ui.small_button("＋").on_hover_text("放大地图").clicked() {
                zoom_delta = Some(1.3);
            }
            if ui.small_button("－").on_hover_text("缩小地图").clicked() {
                zoom_delta = Some(1.0 / 1.3);
            }
            if ui
                .add(theme::primary_btn("应用到跑步页 →"))
                .on_hover_text("切换跑步页为「自定义规划路径」并跳转")
                .clicked()
            {
                apply_to_run = true;
            }
        });

        if close_mode != prev_close {
            self.config.custom_close_mode = close_mode.as_str().into();
            need_sync = true;
        }
        if self.config.custom_clip_fence != prev_clip {
            need_sync = true;
        }

        let (pts_bd, fences) = self.cached_points_and_fences();
        if chain_checkpoints {
            if !pts_bd.is_empty() {
                self.config.custom_waypoints =
                    crate::track::generate_road::radial_order(&pts_bd);
                need_sync = true;
                fit_requested = true;
            } else {
                self.osm_page.status =
                    "暂无打卡点缓存，请先登录并点击「刷新打卡点/围栏」或直接在地图上点击加点"
                        .into();
            }
        }
        if need_sync {
            self.sync_custom_route_config();
        }
        if apply_to_run {
            self.run_page.route_mode = RouteMode::Custom;
            self.config.route_mode = RouteMode::Custom.as_str().into();
            self.sync_custom_route_config();
            self.tab = 0;
        }

        // ── 重算预览 ─────────────────────────────────────────────────
        let target_dist_m = self
            .run_page
            .plan
            .as_ref()
            .map(|p| p.dist * 1000.0)
            .unwrap_or((self.run_page.dist_min.max(1.0) as f64) * 1000.0);
        let seed = self.run_page.plan.as_ref().map(|p| p.seed).unwrap_or(42);

        if self.osm_page.preview_stale || self.osm_page.preview.is_none() {
            self.osm_page.preview_stale = false;
            let plan = plan_custom_view(
                self.network.as_deref(),
                &self.config.custom_waypoints,
                &pts_bd,
                &fences,
                self.config.custom_clip_fence,
                close_mode,
                target_dist_m,
                seed,
            );
            self.osm_page.preview = Some(plan);
        }

        // ── 统计与图例栏 ─────────────────────────────────────────────
        ui.add_space(2.0);
        ui.horizontal_wrapped(|ui| {
            if let Some(net) = &self.network {
                ui.label(format!(
                    "路网：{} 节点 / {} 边 / {} 建筑",
                    net.graph.node_count(),
                    net.graph.edge_count(),
                    net.buildings.len()
                ));
                ui.separator();
            }
            ui.label(format!(
                "打卡点：{} · 围栏：{} · 途经点：{}",
                pts_bd.len(),
                fences.len(),
                self.config.custom_waypoints.len()
            ));
            if let Some(p) = &self.osm_page.preview {
                if p.length_m > 0.0 {
                    let loops = target_dist_m / p.length_m;
                    ui.separator();
                    ui.colored_label(
                        theme::ok(),
                        format!(
                            "单圈路线 {:.0} m（目标 {:.2} km ≈ {:.1} 圈）",
                            p.length_m,
                            target_dist_m / 1000.0,
                            loops
                        ),
                    );
                } else if let Some(err) = &p.route_error {
                    ui.separator();
                    ui.colored_label(theme::warn(), err);
                }
            }
        });

        super::map::legend(
            ui,
            &[
                ("道路", egui::Color32::from_rgb(190, 200, 196)),
                ("建筑", egui::Color32::from_rgb(214, 182, 158)),
                ("电子围栏", egui::Color32::from_rgb(180, 118, 0)),
                ("打卡点", egui::Color32::from_rgb(240, 180, 0)),
                ("规划路线", egui::Color32::from_rgb(30, 111, 216)),
                ("起点(起)", egui::Color32::from_rgb(22, 160, 90)),
                ("途经点", egui::Color32::from_rgb(30, 111, 216)),
                ("末点", egui::Color32::from_rgb(220, 50, 47)),
            ],
        );

        // ── 交互式地图画布 ───────────────────────────────────────────
        ui.add_space(4.0);
        let rect = ui.available_rect_before_wrap();
        let (response, painter) = ui.allocate_painter(
            egui::Vec2::new(rect.width().max(200.0), rect.height().max(240.0)),
            egui::Sense::click_and_drag(),
        );
        let canvas = response.rect;

        if fit_requested {
            self.osm_page.fitted = false;
        }
        if !self.osm_page.fitted {
            if let Some(p) = &self.osm_page.preview {
                let mut bounds = collect_scene_bounds(
                    &p.fences,
                    &self.config.custom_waypoints,
                    &p.route,
                    &p.checkpoints,
                    &p.edges,
                );
                if bounds.is_empty() {
                    for b in &p.buildings {
                        bounds.extend(b.iter().copied());
                    }
                }
                if bounds.is_empty() && !self.identity.has_unconfigured_default_location() {
                    bounds.push((self.identity.anchor_lat, self.identity.anchor_lon));
                }
                if !bounds.is_empty() {
                    self.osm_page.map.fit(canvas, &bounds);
                    self.osm_page.fitted = true;
                }
            }
        }
        if let Some(factor) = zoom_delta {
            self.osm_page.map.zoom_at(canvas, canvas.center(), factor);
        }

        painter.rect_filled(
            canvas,
            egui::Rounding::same(4.0),
            egui::Color32::from_rgb(248, 251, 249),
        );

        if let Some(p) = &self.osm_page.preview {
            let road_color = egui::Color32::from_rgb(196, 206, 202);
            for e in &p.edges {
                self.osm_page
                    .map
                    .draw_polyline(&painter, canvas, e, road_color, 1.2);
            }
            let bld_color = egui::Color32::from_rgb(220, 190, 166);
            for b in &p.buildings {
                self.osm_page
                    .map
                    .draw_polygon(&painter, canvas, b, bld_color, 1.0);
            }
            let fence_color = egui::Color32::from_rgb(180, 118, 0);
            for f in &p.fences {
                self.osm_page
                    .map
                    .draw_polygon(&painter, canvas, f, fence_color, 2.0);
            }
            // 学校打卡点（金色圆点 + 白边）
            let cp_color = egui::Color32::from_rgb(240, 180, 0);
            for &(la, lo) in &p.checkpoints {
                let sp = self.osm_page.map.to_screen(canvas, la, lo);
                if canvas.contains(sp) {
                    painter.circle_filled(sp, 4.5, cp_color);
                    painter.circle_stroke(
                        sp,
                        4.5,
                        egui::Stroke::new(1.2_f32, egui::Color32::WHITE),
                    );
                }
            }
            // 规划路线（带方向箭头）
            if p.route.len() >= 2 {
                let route_color = if p.length_m > 0.0 {
                    egui::Color32::from_rgb(30, 111, 216)
                } else {
                    egui::Color32::from_rgb(180, 100, 100)
                };
                self.osm_page.map.draw_route_with_arrows(
                    &painter,
                    canvas,
                    &p.route,
                    route_color,
                    2.8,
                );
            }
            // 用户途经点序号徽章
            self.osm_page
                .map
                .draw_waypoints(&painter, canvas, &self.config.custom_waypoints);

            // 空画布引导文案
            if p.edges.is_empty()
                && p.checkpoints.is_empty()
                && p.fences.is_empty()
                && self.config.custom_waypoints.is_empty()
            {
                painter.text(
                    canvas.center(),
                    egui::Align2::CENTER_CENTER,
                    "左键点击地图即可添加途经点规划路径\n（导入 OSM 路网或登录后刷新打卡点可显示校园底图）",
                    egui::FontId::proportional(14.0),
                    egui::Color32::from_rgb(140, 152, 148),
                );
            }
        }

        self.osm_page.map.draw_scale_bar(&painter, canvas);
        self.osm_page.map.draw_hud_footer(&painter, canvas, true);

        // ── 处理交互点击/拖拽/删除途经点 ─────────────────────────────
        let net_ref = self.network.clone();
        let clip_fence = self.config.custom_clip_fence;
        let snap_road_enabled = self.osm_page.snap_road && net_ref.is_some();
        let snap_closure = |lat: f64, lon: f64| -> Option<((f64, f64), f64)> {
            let net = net_ref.as_deref()?;
            snap_point_to_aligned_road(net, &fences, clip_fence, lat, lon)
        };
        let snap_fn: Option<&dyn Fn(f64, f64) -> Option<((f64, f64), f64)>> =
            if snap_road_enabled {
                Some(&snap_closure)
            } else {
                None
            };
        let edit = self.osm_page.map.interact_planner(
            ui,
            canvas,
            &response,
            &mut self.config.custom_waypoints,
            &pts_bd,
            snap_fn,
        );
        if edit.changed() {
            self.sync_custom_route_config();
        }

        if load_now {
            let path = self.osm_page.path.clone();
            self.load_osm_async(path);
        }
    }

    /// 后台线程加载 OSM 路网，成功后经 `net_rx` 回传。
    pub fn load_osm_async(&mut self, path: String) {
        self.osm_page.path = path.clone();
        self.osm_page.busy = true;
        self.osm_page.status = format!("加载中：{path} …");
        self.config.osm_path = path.clone();
        let _ = crate::api::model::save_config(&self.config);
        let tx = self.net_tx.clone();
        std::thread::spawn(move || {
            let res = (|| -> Result<Arc<RoadGraph>, String> {
                let bytes = std::fs::read(&path).map_err(|e| format!("读取失败: {e}"))?;
                let net = route_planner::load_osm(&bytes)?;
                Ok(Arc::new(net))
            })();
            let _ = tx.send(res);
        });
    }
}
