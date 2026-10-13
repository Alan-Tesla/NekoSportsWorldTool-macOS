//! 「路网」与路径规划页：选择 OSM 文件 → 后台解析 → 交互式地图规划路径。
//!
//! 统一在 BD-09 工作坐标系下展示道路、建筑、电子围栏、学校打卡点与自定义途经点，
//! 支持左键点击加点、拖拽移动途经点、右键删除途经点、打卡点/道路自动吸附，以及一键应用到跑步页。

use eframe::egui;
use std::sync::Arc;

use route_planner::RoadGraph;

use super::map::{collect_scene_bounds, MapState, TileSource};
use super::{mobile, theme, App};
use crate::track::generate_road::{
    append_custom_loop_track, append_playground_laps, plan_custom_view, repeat_last_loop,
    snap_point_to_aligned_road, CustomCloseMode, LoopEntryMode, LoopExitMode, LoopTrackSpec,
    RoadPlan, RouteMode,
};

pub struct OsmPage {
    pub path: String,
    pub busy: bool,
    pub status: String,
    pub map: MapState,
    pub fitted: bool,
    pub snap_road: bool,
    pub show_road_overlay: bool,
    pub show_labels: bool,
    pub playground_laps: i64,
    pub show_loop_designer: bool,
    pub special_badges: Vec<(usize, String, egui::Color32)>,
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
            snap_road: false,
            show_road_overlay: true,
            show_labels: true,
            playground_laps: 2,
            show_loop_designer: false,
            special_badges: Vec::new(),
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

        // ── 第 2 栏：路径规划与底图工具栏 ───────────────────────────
        let mut close_mode = CustomCloseMode::from_str(&self.config.custom_close_mode);
        let prev_close = close_mode;
        let mut tile_source = TileSource::from_str(&self.config.tile_source);
        let prev_tile = tile_source;
        let prev_clip = self.config.custom_clip_fence;
        let prev_snap = self.config.custom_snap_road;
        let mut need_sync = false;
        let mut fit_requested = false;
        let mut zoom_delta: Option<f32> = None;
        let mut apply_to_run = false;
        let mut chain_checkpoints = false;
        let mut gen_playground = false;
        let mut repeat_loop = false;
        let mut sync_dist_m: Option<f64> = None;

        ui.add_space(2.0);
        mobile::row(ui, |ui| {
            ui.label("底图：");
            egui::ComboBox::from_id_salt("osm_tile_source")
                .width(145.0)
                .selected_text(tile_source.label())
                .show_ui(ui, |ui| {
                    for s in [
                        TileSource::GaodeStreet,
                        TileSource::OsmStandard,
                        TileSource::VectorOnly,
                    ] {
                        ui.selectable_value(&mut tile_source, s, s.label());
                    }
                });
            ui.checkbox(&mut self.osm_page.show_road_overlay, "路网图层");
            ui.checkbox(&mut self.osm_page.show_labels, "名称标注");
            ui.separator();
            ui.label("寻路：");
            ui.selectable_value(
                &mut self.config.custom_snap_road,
                false,
                "自由手绘/操场",
            )
            .on_hover_text("不强制吸附外部道路，可在操场或任意位置自由点选路径点");
            ui.selectable_value(&mut self.config.custom_snap_road, true, "沿道路寻路")
                .on_hover_text("将途经点吸附到 OSM 道路并自动计算最短道路连线");
            if self.config.custom_snap_road {
                ui.checkbox(&mut self.config.custom_clip_fence, "围栏裁剪");
            }
        });

        mobile::row(ui, |ui| {
            ui.label("模式：");
            egui::ComboBox::from_id_salt("osm_custom_close_mode")
                .width(175.0)
                .selected_text(close_mode.label())
                .show_ui(ui, |ui| {
                    for m in [
                        CustomCloseMode::Open,
                        CustomCloseMode::Loop,
                        CustomCloseMode::OutAndBack,
                        CustomCloseMode::AutoPadLoop,
                    ] {
                        ui.selectable_value(&mut close_mode, m, m.label());
                    }
                });
            ui.separator();
            ui.toggle_value(&mut self.osm_page.show_loop_designer, "⚙ 跑圈设计器")
                .on_hover_text("展开/收起跑圈参数面板（单圈周长、圈数、椭圆/正圆、旋转走向、入点与出点）");
            if ui
                .button("＋标准400m操场圈")
                .on_hover_text(
                    "在末点自动追加标准 400m 田径跑道（保留前面路径），之后可继续点选出操场后的下一个点位",
                )
                .clicked()
            {
                gen_playground = true;
            }
            if ui
                .add_enabled(
                    self.config.custom_waypoints.len() >= 3,
                    egui::Button::new("＋手绘再绕一圈"),
                )
                .on_hover_text("若你刚在操场上手动点了一圈点位，点击即可原地再复制绕一圈，之后可继续点选出操场后的终点")
                .clicked()
            {
                repeat_loop = true;
            }
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
                    egui::Button::new("撤销"),
                )
                .clicked()
            {
                self.config.custom_waypoints.pop();
                need_sync = true;
            }
            if ui
                .add_enabled(
                    self.config.custom_waypoints.len() >= 2,
                    egui::Button::new("反转"),
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
                self.osm_page.special_badges.clear();
                need_sync = true;
            }
            ui.separator();
            if ui.button("居中").clicked() {
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

        // 跑圈参数设计器展开面板
        if self.osm_page.show_loop_designer {
            egui::Frame::group(ui.style()).show(ui, |ui| {
                ui.horizontal_wrapped(|ui| {
                    ui.strong("跑圈参数：");
                    ui.label("单圈周长：");
                    for &preset in &[400.0, 300.0, 250.0, 200.0] {
                        if ui
                            .selectable_label(
                                (self.config.loop_perimeter - preset).abs() < 1.0,
                                format!("{preset:.0}m"),
                            )
                            .clicked()
                        {
                            self.config.loop_perimeter = preset;
                            need_sync = true;
                        }
                    }
                    mobile::drag_f64(
                        ui,
                        "osm_loop_perim_drag",
                        &mut self.config.loop_perimeter,
                        100.0..=1500.0,
                        5.0,
                        0,
                    );
                    ui.label("m");
                    ui.separator();
                    ui.label("圈数：");
                    mobile::drag_i64(
                        ui,
                        "osm_loop_laps_drag",
                        &mut self.config.loop_laps,
                        1..=20,
                        0.2,
                        "",
                        " 圈",
                    );
                    ui.separator();
                    ui.label("形状：");
                    ui.selectable_value(&mut self.config.loop_is_oval, true, "标准操场椭圆");
                    ui.selectable_value(&mut self.config.loop_is_oval, false, "正圆跑道");
                });

                ui.horizontal_wrapped(|ui| {
                    ui.label("旋转走向：");
                    ui.selectable_value(&mut self.config.loop_rotation_deg, 0.0, "南北 0°");
                    ui.selectable_value(&mut self.config.loop_rotation_deg, 90.0, "东西 90°");
                    mobile::drag_f64(
                        ui,
                        "osm_loop_rot_drag",
                        &mut self.config.loop_rotation_deg,
                        -180.0..=180.0,
                        1.0,
                        0,
                    );
                    ui.label("°");
                    ui.separator();
                    ui.label("转向：");
                    ui.selectable_value(&mut self.config.loop_ccw, true, "逆时针左转 (标准)");
                    ui.selectable_value(&mut self.config.loop_ccw, false, "顺时针右转");
                    ui.separator();
                    let mut entry_mode = LoopEntryMode::from_str(&self.config.loop_entry_mode);
                    let prev_entry = entry_mode;
                    ui.label("入点：");
                    egui::ComboBox::from_id_salt("osm_entry_mode")
                        .width(115.0)
                        .selected_text(entry_mode.label())
                        .show_ui(ui, |ui| {
                            for m in [
                                LoopEntryMode::AutoClosest,
                                LoopEntryMode::South,
                                LoopEntryMode::North,
                                LoopEntryMode::East,
                                LoopEntryMode::West,
                            ] {
                                ui.selectable_value(&mut entry_mode, m, m.label());
                            }
                        });
                    if entry_mode != prev_entry {
                        self.config.loop_entry_mode = entry_mode.as_str().into();
                        need_sync = true;
                    }

                    let mut exit_mode = LoopExitMode::from_str(&self.config.loop_exit_mode);
                    let prev_exit = exit_mode;
                    ui.label("出点：");
                    egui::ComboBox::from_id_salt("osm_exit_mode")
                        .width(135.0)
                        .selected_text(exit_mode.label())
                        .show_ui(ui, |ui| {
                            for m in [
                                LoopExitMode::SameAsEntry,
                                LoopExitMode::Opposite,
                                LoopExitMode::North,
                                LoopExitMode::South,
                                LoopExitMode::East,
                                LoopExitMode::West,
                            ] {
                                ui.selectable_value(&mut exit_mode, m, m.label());
                            }
                        });
                    if exit_mode != prev_exit {
                        self.config.loop_exit_mode = exit_mode.as_str().into();
                        need_sync = true;
                    }

                    if ui
                        .add(theme::primary_btn("＋在末点插入自定义跑圈"))
                        .on_hover_text("在当前路径末尾无缝插入指定周长与出入点的跑圈（在地图上以「入」和「出」特殊标记）")
                        .clicked()
                    {
                        let spec = LoopTrackSpec {
                            perimeter_m: self.config.loop_perimeter,
                            laps: self.config.loop_laps.clamp(1, 20) as usize,
                            is_oval: self.config.loop_is_oval,
                            rotation_deg: self.config.loop_rotation_deg,
                            counter_clockwise: self.config.loop_ccw,
                            entry_mode,
                            exit_mode,
                        };
                        let fallback = if self.osm_page.map.center != (0.0, 0.0) {
                            self.osm_page.map.center
                        } else {
                            (self.identity.anchor_lat, self.identity.anchor_lon)
                        };
                        let (entry_idx, exit_idx) = append_custom_loop_track(
                            &mut self.config.custom_waypoints,
                            fallback,
                            &spec,
                        );
                        self.osm_page.special_badges = vec![
                            (entry_idx, "入".into(), egui::Color32::from_rgb(16, 185, 129)),
                            (exit_idx, "出".into(), egui::Color32::from_rgb(249, 115, 22)),
                        ];
                        self.run_page.special_badges = self.osm_page.special_badges.clone();
                        self.config.custom_snap_road = false;
                        self.osm_page.snap_road = false;
                        need_sync = true;
                    }
                });
            });
        }

        if close_mode != prev_close {
            self.config.custom_close_mode = close_mode.as_str().into();
            need_sync = true;
        }
        if self.config.custom_snap_road != prev_snap {
            self.osm_page.snap_road = self.config.custom_snap_road;
            need_sync = true;
        }
        if tile_source != prev_tile {
            self.config.tile_source = tile_source.as_str().into();
            let _ = crate::api::model::save_config(&self.config);
        }
        if self.config.custom_clip_fence != prev_clip {
            need_sync = true;
        }

        let (pts_bd, fences) = self.cached_points_and_fences();
        if gen_playground {
            let fallback = if self.osm_page.map.center != (0.0, 0.0) {
                self.osm_page.map.center
            } else {
                (self.identity.anchor_lat, self.identity.anchor_lon)
            };
            append_playground_laps(
                &mut self.config.custom_waypoints,
                fallback,
                self.osm_page.playground_laps.clamp(1, 15) as usize,
            );
            self.config.custom_snap_road = false;
            self.osm_page.snap_road = false;
            need_sync = true;
        }
        if repeat_loop && repeat_last_loop(&mut self.config.custom_waypoints) {
            need_sync = true;
        }
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
                self.config.custom_snap_road,
                close_mode,
                target_dist_m,
                seed,
            );
            self.run_page.sync_planned_route_len(plan.length_m);
            self.osm_page.preview = Some(plan);
        }

        // 统计打卡点实际覆盖情况
        let (passed_cp_cnt, total_cp_cnt, missing_cps) = if let Some(p) = &self.osm_page.preview {
            let total = p.checkpoints.len();
            let mut passed = 0usize;
            let mut missing = Vec::new();
            for &(cla, clo) in &p.checkpoints {
                let hit = p.route.iter().any(|&(rla, rlo)| {
                    let dy = (rla - cla) * crate::track::geom::MET_PER_DEG_LAT;
                    let dx = (rlo - clo) * crate::track::geom::MET_PER_DEG_LNG;
                    dx.hypot(dy) <= 20.0
                });
                if hit {
                    passed += 1;
                } else {
                    missing.push((cla, clo));
                }
            }
            (passed, total, missing)
        } else {
            (0, 0, Vec::new())
        };

        // ── 统计与图例栏 ─────────────────────────────────────────────
        ui.add_space(2.0);
        let mut do_complete_cps = false;
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
            if total_cp_cnt > 0 {
                if passed_cp_cnt >= total_cp_cnt {
                    ui.colored_label(
                        theme::ok(),
                        format!("打卡点：已覆盖 {}/{} 个 √（全部达标）", passed_cp_cnt, total_cp_cnt),
                    );
                } else {
                    ui.colored_label(
                        theme::warn(),
                        format!(
                            "打卡点：已覆盖 {}/{} 个 ⚠（缺 {} 个可能未达标）",
                            passed_cp_cnt,
                            total_cp_cnt,
                            total_cp_cnt - passed_cp_cnt
                        ),
                    );
                    if ui
                        .small_button("＋一键补齐未途经点")
                        .on_hover_text("自动将尚未经过的打卡点插入路线末尾前，确保全部打卡达标")
                        .clicked()
                    {
                        do_complete_cps = true;
                    }
                }
                ui.separator();
            } else {
                ui.label(format!("围栏：{} · 途经点：{}", fences.len(), self.config.custom_waypoints.len()));
            }
            if let Some(p) = &self.osm_page.preview {
                if p.length_m > 0.0 {
                    ui.separator();
                    if close_mode == CustomCloseMode::Open {
                        ui.colored_label(
                            theme::ok(),
                            format!(
                                "开环路线总长 {:.0} m（{:.2} km，目标跑步 {:.2} km）",
                                p.length_m,
                                p.length_m / 1000.0,
                                target_dist_m / 1000.0,
                            ),
                        );
                    } else {
                        let loops = target_dist_m / p.length_m;
                        ui.colored_label(
                            theme::ok(),
                            format!(
                                "单圈路线 {:.0} m（目标 {:.2} km ≈ {:.1} 圈）",
                                p.length_m,
                                target_dist_m / 1000.0,
                                loops
                            ),
                        );
                    }
                    if p.length_m >= 500.0
                        && ui
                            .small_button("设为跑步距离")
                            .on_hover_text("将跑步页的距离范围同步为当前规划路线的长度")
                            .clicked()
                    {
                        sync_dist_m = Some(p.length_m);
                    }
                } else if let Some(err) = &p.route_error {
                    ui.separator();
                    ui.colored_label(theme::warn(), err);
                }
            }
        });
        if let Some(len_m) = sync_dist_m {
            let km = (len_m / 1000.0).clamp(0.5, 19.8) as f32;
            let lo = (km * 100.0).floor() / 100.0;
            let hi = ((lo + 0.06).min(20.0) * 100.0).round() / 100.0;
            self.run_page.dist_min = lo;
            self.run_page.dist_max = hi;
            self.run_page.regen_plan();
            self.config.dist_min = lo;
            self.config.dist_max = hi;
            self.sync_custom_route_config();
            self.status = format!("已将跑步目标距离同步为 {:.2}~{:.2} km", lo, hi);
        }
        if do_complete_cps && !missing_cps.is_empty() {
            if self.config.custom_waypoints.len() >= 2 {
                let last = self.config.custom_waypoints.pop().unwrap();
                self.config.custom_waypoints.extend(missing_cps);
                self.config.custom_waypoints.push(last);
            } else {
                self.config.custom_waypoints.extend(missing_cps);
            }
            self.sync_custom_route_config();
            self.status = "已自动补齐尚未途经的打卡点".into();
        }

        super::map::legend(
            ui,
            &[
                ("可跑道路", egui::Color32::from_rgb(16, 135, 115)),
                ("操场跑道", egui::Color32::from_rgb(116, 186, 132)),
                ("建筑", egui::Color32::from_rgb(214, 182, 158)),
                ("电子围栏", egui::Color32::from_rgb(180, 118, 0)),
                ("已途经打卡点(绿√)", egui::Color32::from_rgb(16, 185, 129)),
                ("未途经打卡点(橙标)", egui::Color32::from_rgb(245, 158, 11)),
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
                if bounds.is_empty() {
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

        // 底色填充 + 在线瓦片底图渲染
        painter.rect_filled(
            canvas,
            egui::Rounding::same(4.0),
            egui::Color32::from_rgb(242, 239, 233),
        );
        self.osm_page
            .map
            .draw_tile_basemap(ui.ctx(), &painter, canvas, tile_source);

        if let Some(p) = &self.osm_page.preview {
            let has_tiles = tile_source != TileSource::VectorOnly;
            self.osm_page.map.draw_rich_osm_layer(
                &painter,
                canvas,
                p,
                has_tiles,
                self.osm_page.show_road_overlay,
                self.osm_page.show_labels,
            );

            let clipped = painter.with_clip_rect(canvas);
            let fence_color = egui::Color32::from_rgb(180, 118, 0);
            for f in &p.fences {
                self.osm_page
                    .map
                    .draw_polygon(&clipped, canvas, f, fence_color, 2.2);
            }
            // 学校打卡点（已途经绿底带「√」标 / 未途经橙底带编号，悬停显示剩余米数）
            self.osm_page
                .map
                .draw_checkpoints(&painter, canvas, &p.checkpoints, &p.route);

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
                    3.0,
                );
            }
            // 用户途经点序号徽章（含特殊「入」「出」标记）
            let badges: Vec<(usize, &str, egui::Color32)> = self
                .osm_page
                .special_badges
                .iter()
                .map(|(idx, s, c)| (*idx, s.as_str(), *c))
                .collect();
            self.osm_page
                .map
                .draw_waypoints_ext(&painter, canvas, &self.config.custom_waypoints, &badges);
        }

        self.osm_page.map.draw_scale_bar(&painter, canvas);
        self.osm_page.map.draw_hud_footer(&painter, canvas, true);

        // ── 处理交互点击/拖拽/删除途经点 ─────────────────────────────
        let net_ref = self.network.clone();
        let clip_fence = self.config.custom_clip_fence;
        let snap_road_enabled = self.config.custom_snap_road && net_ref.is_some();
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
        let route_slice: Vec<(f64, f64)> = self
            .osm_page
            .preview
            .as_ref()
            .map(|p| p.route.clone())
            .unwrap_or_default();
        let edit = self.osm_page.map.interact_planner(
            ui,
            canvas,
            &response,
            &mut self.config.custom_waypoints,
            &route_slice,
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
