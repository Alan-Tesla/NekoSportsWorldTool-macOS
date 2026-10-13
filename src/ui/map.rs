//! 地图画布绘制原语、在线瓦片底图引擎与交互式路径规划器。
//!
//! 供「路网」页与「跑步」页复用：
//! - 支持在线瓦片底图（高德街道地图 GCJ-02 / OpenStreetMap 标准图 WGS-84，自动配准到 BD-09 工作系，带内存+磁盘双缓存与父瓦片过渡）；
//! - 支持增强版 OSM 彩色矢量渲染（绿地、水域、操场跑道、实心建筑、双线道路及碰撞避让的中文名称标注）；
//! - 支持左键点击加点、拖拽移动途经点、右键删除途经点、打卡点/道路自动吸附。

use std::collections::{HashMap, HashSet};
use std::sync::{Mutex, OnceLock};
use std::time::{Duration, Instant};

use eframe::egui;
use egui::{Align2, Color32, FontId, Mesh, Pos2, Rect, Stroke, TextureHandle, Vec2};

use crate::track::generate_road::RoadPlan;
use crate::track::geom::{
    gcj02_to_bd09, wgs84_to_bd09, wgs84_to_gcj02, MET_PER_DEG_LAT, MET_PER_DEG_LNG,
};
use crate::track::wire::bd09_to_gcj02;

/// 底图数据源。
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, Default, serde::Serialize, serde::Deserialize)]
pub enum TileSource {
    /// 高德街道地图（国内直连，带完整校园楼宇/道路/操场中文标注）。
    #[default]
    GaodeStreet,
    /// OpenStreetMap 标准瓦片地图。
    OsmStandard,
    /// 纯离线彩色矢量图（仅用本地 .osm 绘制绿地/水系/操场/建筑/路网）。
    VectorOnly,
}

impl TileSource {
    pub fn from_str(s: &str) -> Self {
        match s {
            "osm" => TileSource::OsmStandard,
            "vector" => TileSource::VectorOnly,
            _ => TileSource::GaodeStreet,
        }
    }

    pub fn as_str(self) -> &'static str {
        match self {
            TileSource::GaodeStreet => "gaode",
            TileSource::OsmStandard => "osm",
            TileSource::VectorOnly => "vector",
        }
    }

    pub fn label(self) -> &'static str {
        match self {
            TileSource::GaodeStreet => "高德街道地图（在线）",
            TileSource::OsmStandard => "OSM 标准地图（在线）",
            TileSource::VectorOnly => "离线彩色矢量图",
        }
    }
}

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

// ── 在线瓦片缓存与配准（BD-09 ↔ GCJ-02 / WGS-84 ↔ Web Mercator）─────

#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
struct TileKey {
    source: TileSource,
    z: u8,
    x: i32,
    y: i32,
}

struct DecodedTile {
    key: TileKey,
    width: usize,
    height: usize,
    rgba: Vec<u8>,
}

#[derive(Default)]
struct TileCache {
    textures: HashMap<TileKey, TextureHandle>,
    pending: HashSet<TileKey>,
    failed: HashMap<TileKey, Instant>,
    ready: Vec<DecodedTile>,
}

static TILE_CACHE: OnceLock<Mutex<TileCache>> = OnceLock::new();

fn tile_cache() -> &'static Mutex<TileCache> {
    TILE_CACHE.get_or_init(|| Mutex::new(TileCache::default()))
}

/// BD-09 → WGS-84（先转 GCJ-02，再用一步不动点反演 WGS-84，精度 <0.1m）。
fn bd09_to_wgs84(bd_lat: f64, bd_lon: f64) -> (f64, f64) {
    let (glat, glon) = bd09_to_gcj02(bd_lat, bd_lon);
    let (glat2, glon2) = wgs84_to_gcj02(glat, glon);
    (2.0 * glat - glat2, 2.0 * glon - glon2)
}

/// 经纬度 → Web Mercator 浮点瓦片坐标 `(tx, ty)`。
fn latlon_to_tile_frac(lat: f64, lon: f64, z: u8) -> (f64, f64) {
    let n = (1u32 << z) as f64;
    let lat_clamped = lat.clamp(-85.0511, 85.0511);
    let lat_rad = lat_clamped.to_radians();
    let tx = (lon + 180.0) / 360.0 * n;
    let ty = (1.0 - (lat_rad.tan() + 1.0 / lat_rad.cos()).ln() / std::f64::consts::PI) * 0.5 * n;
    (tx, ty)
}

/// Web Mercator 瓦片左上角 `(x, y)` → 经纬度 `(lat, lon)`。
fn tile_to_latlon(x: i32, y: i32, z: u8) -> (f64, f64) {
    let n = (1u32 << z) as f64;
    let lon = x as f64 / n * 360.0 - 180.0;
    let arg = std::f64::consts::PI * (1.0 - 2.0 * y as f64 / n);
    let lat = arg.sinh().atan().to_degrees();
    (lat, lon)
}

/// 瓦片四个角转换回 BD-09 后的包络 `(nw_lat, nw_lon, se_lat, se_lon)`。
fn tile_bounds_bd09(source: TileSource, z: u8, x: i32, y: i32) -> ((f64, f64), (f64, f64)) {
    let (lat_nw, lon_nw) = tile_to_latlon(x, y, z);
    let (lat_se, lon_se) = tile_to_latlon(x + 1, y + 1, z);
    match source {
        TileSource::GaodeStreet => (
            gcj02_to_bd09(lat_nw, lon_nw),
            gcj02_to_bd09(lat_se, lon_se),
        ),
        TileSource::OsmStandard | TileSource::VectorOnly => (
            wgs84_to_bd09(lat_nw, lon_nw),
            wgs84_to_bd09(lat_se, lon_se),
        ),
    }
}

fn tile_disk_path(key: TileKey) -> std::path::PathBuf {
    crate::platform::data_dir()
        .join("tiles_cache")
        .join(key.source.as_str())
        .join(key.z.to_string())
        .join(format!("{}_{}.png", key.x, key.y))
}

fn spawn_tile_fetch(key: TileKey, ctx: egui::Context) {
    std::thread::spawn(move || {
        let disk_path = tile_disk_path(key);
        let bytes = if let Ok(cached) = std::fs::read(&disk_path) {
            Some(cached)
        } else {
            let url = match key.source {
                TileSource::GaodeStreet => {
                    let sub = ((key.x.abs() + key.y.abs()) & 3) + 1;
                    format!(
                        "https://webrd0{sub}.is.autonavi.com/appmaptile?lang=zh_cn&size=1&scale=1&style=8&x={}&y={}&z={}",
                        key.x, key.y, key.z
                    )
                }
                TileSource::OsmStandard => {
                    format!("https://tile.openstreetmap.org/{}/{}/{}.png", key.z, key.x, key.y)
                }
                TileSource::VectorOnly => return,
            };
            let agent = ureq::AgentBuilder::new()
                .timeout_connect(Duration::from_secs(6))
                .timeout_read(Duration::from_secs(10))
                .build();
            match agent
                .get(&url)
                .set(
                    "User-Agent",
                    "Mozilla/5.0 (Windows NT 10.0; Win64; x64) NekoSportsWorldTool/0.3",
                )
                .call()
            {
                Ok(resp) => {
                    let mut buf = Vec::new();
                    use std::io::Read;
                    if resp.into_reader().take(2 * 1024 * 1024).read_to_end(&mut buf).is_ok()
                        && !buf.is_empty()
                    {
                        if let Some(parent) = disk_path.parent() {
                            let _ = std::fs::create_dir_all(parent);
                        }
                        let _ = std::fs::write(&disk_path, &buf);
                        Some(buf)
                    } else {
                        None
                    }
                }
                Err(_) => None,
            }
        };

        let decoded = bytes.and_then(|raw| {
            let img = image::load_from_memory(&raw).ok()?.to_rgba8();
            let (w, h) = img.dimensions();
            Some(DecodedTile {
                key,
                width: w as usize,
                height: h as usize,
                rgba: img.into_raw(),
            })
        });

        if let Ok(mut cache) = tile_cache().lock() {
            cache.pending.remove(&key);
            match decoded {
                Some(tile) => cache.ready.push(tile),
                None => {
                    cache.failed.insert(key, Instant::now());
                }
            }
        }
        ctx.request_repaint();
    });
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

    /// 绘制在线栅格瓦片底图（自动换算并配准到 BD-09 屏幕视野）。
    pub fn draw_tile_basemap(
        &self,
        ctx: &egui::Context,
        painter: &egui::Painter,
        rect: Rect,
        source: TileSource,
    ) {
        if source == TileSource::VectorOnly || self.center == (0.0, 0.0) {
            return;
        }
        let clipped = painter.with_clip_rect(rect);
        let (bd_nw_lat, bd_nw_lon) = self.from_screen(rect, rect.left_top());
        let (bd_se_lat, bd_se_lon) = self.from_screen(rect, rect.right_bottom());
        if !bd_nw_lat.is_finite() || !bd_se_lat.is_finite() {
            return;
        }

        let (nw_lat, nw_lon, se_lat, se_lon) = match source {
            TileSource::GaodeStreet => {
                let (a, b) = bd09_to_gcj02(bd_nw_lat, bd_nw_lon);
                let (c, d) = bd09_to_gcj02(bd_se_lat, bd_se_lon);
                (a, b, c, d)
            }
            TileSource::OsmStandard | TileSource::VectorOnly => {
                let (a, b) = bd09_to_wgs84(bd_nw_lat, bd_nw_lon);
                let (c, d) = bd09_to_wgs84(bd_se_lat, bd_se_lon);
                (a, b, c, d)
            }
        };

        let mid_lat = ((nw_lat + se_lat) * 0.5).clamp(-80.0, 80.0);
        let cos_lat = mid_lat.to_radians().cos().abs().max(0.2);
        let raw_z = ((self.scale as f64) * 156543.03 * cos_lat)
            .max(1.0)
            .log2()
            .round() as i32;
        let mut z = raw_z.clamp(4, 18) as u8;

        let max_idx = (1i32 << z) - 1;
        let (mut x0, mut y0, mut x1, mut y1);
        loop {
            let m = (1i32 << z) - 1;
            let (fx0, fy0) = latlon_to_tile_frac(nw_lat, nw_lon, z);
            let (fx1, fy1) = latlon_to_tile_frac(se_lat, se_lon, z);
            x0 = (fx0.min(fx1).floor() as i32).clamp(0, m);
            x1 = (fx0.max(fx1).floor() as i32).clamp(0, m);
            y0 = (fy0.min(fy1).floor() as i32).clamp(0, m);
            y1 = (fy0.max(fy1).floor() as i32).clamp(0, m);
            let count = (x1 - x0 + 1).max(1) * (y1 - y0 + 1).max(1);
            if count <= 36 || z <= 4 {
                break;
            }
            z -= 1;
        }
        let _ = max_idx;

        let Ok(mut cache) = tile_cache().lock() else {
            return;
        };

        // 将后台解码完成的瓦片上传为 GPU 纹理
        let ready = std::mem::take(&mut cache.ready);
        for item in ready {
            let image =
                egui::ColorImage::from_rgba_unmultiplied([item.width, item.height], &item.rgba);
            let tex = ctx.load_texture(
                format!(
                    "tile_{}_{}_{}_{}",
                    item.key.source.as_str(),
                    item.key.z,
                    item.key.x,
                    item.key.y
                ),
                image,
                egui::TextureOptions::LINEAR,
            );
            cache.textures.insert(item.key, tex);
        }

        let now = Instant::now();
        let full_uv = Rect::from_min_max(Pos2::ZERO, Pos2::new(1.0, 1.0));

        for ty in y0..=y1 {
            for tx in x0..=x1 {
                let key = TileKey {
                    source,
                    z,
                    x: tx,
                    y: ty,
                };
                let (bd_nw, bd_se) = tile_bounds_bd09(source, z, tx, ty);
                let p_min = self.to_screen(rect, bd_nw.0, bd_nw.1);
                let p_max = self.to_screen(rect, bd_se.0, bd_se.1);
                // 外扩 0.5px 消除瓦片接缝白线
                let tile_rect =
                    Rect::from_min_max(p_min - Vec2::splat(0.4), p_max + Vec2::splat(0.4));

                if let Some(tex) = cache.textures.get(&key) {
                    clipped.image(
                        tex.id(),
                        tile_rect,
                        full_uv,
                        Color32::from_rgba_unmultiplied(255, 255, 255, 245),
                    );
                } else {
                    // 若当前层级瓦片尚未就绪，尝试用上一级父瓦片对应象限先行过渡渲染
                    if z > 4 {
                        let parent_key = TileKey {
                            source,
                            z: z - 1,
                            x: tx >> 1,
                            y: ty >> 1,
                        };
                        if let Some(ptex) = cache.textures.get(&parent_key) {
                            let u0 = if (tx & 1) == 0 { 0.0 } else { 0.5 };
                            let v0 = if (ty & 1) == 0 { 0.0 } else { 0.5 };
                            let sub_uv = Rect::from_min_max(
                                Pos2::new(u0, v0),
                                Pos2::new(u0 + 0.5, v0 + 0.5),
                            );
                            clipped.image(
                                ptex.id(),
                                tile_rect,
                                sub_uv,
                                Color32::from_rgba_unmultiplied(255, 255, 255, 230),
                            );
                        }
                    }
                    // 触发后台下载（并发上限 8，失败冷却 12 秒）
                    let on_cooldown = cache
                        .failed
                        .get(&key)
                        .map_or(false, |t| now.duration_since(*t) < Duration::from_secs(12));
                    if !cache.pending.contains(&key) && !on_cooldown && cache.pending.len() < 8 {
                        cache.pending.insert(key);
                        spawn_tile_fetch(key, ctx.clone());
                    }
                }
            }
        }
    }

    /// 绘制增强版 OSM 矢量底图要素（绿地、水系、运动场、实心建筑、道路网及名称标注）。
    ///
    /// `has_tiles` 为真（叠加在在线瓦片上）时自动降低填充不透明度并仅高亮可跑道路拓扑与操场，
    /// 为假（离线彩色矢量模式）时按完整地图配色渲染全部要素与名称标签。
    pub fn draw_rich_osm_layer(
        &self,
        painter: &egui::Painter,
        rect: Rect,
        plan: &RoadPlan,
        has_tiles: bool,
        show_roads: bool,
        show_labels: bool,
    ) {
        let clipped = painter.with_clip_rect(rect);

        if !has_tiles {
            // 1. 绿地 / 公园 / 草坪
            let green_fill = Color32::from_rgb(212, 236, 206);
            let green_stroke = Color32::from_rgb(186, 218, 178);
            for ring in &plan.greens {
                self.draw_filled_polygon(&clipped, rect, ring, green_fill, Some((1.0, green_stroke)));
            }

            // 2. 水系 / 湖泊 / 河道
            let water_fill = Color32::from_rgb(170, 211, 223);
            let water_stroke = Color32::from_rgb(138, 188, 206);
            for ring in &plan.waters {
                self.draw_filled_polygon(&clipped, rect, ring, water_fill, Some((1.0, water_stroke)));
            }

            // 3. 操场 / 田径场 / 球场
            let pitch_fill = Color32::from_rgb(184, 228, 192);
            let pitch_stroke = Color32::from_rgb(116, 186, 132);
            for ring in &plan.pitches {
                self.draw_filled_polygon(&clipped, rect, ring, pitch_fill, Some((1.4, pitch_stroke)));
            }

            // 4. 实心建筑物（暖灰杏色填充 + 立体描边）
            let bld_fill = Color32::from_rgb(232, 218, 204);
            let bld_stroke = Color32::from_rgb(204, 182, 162);
            for ring in &plan.buildings {
                self.draw_filled_polygon(&clipped, rect, ring, bld_fill, Some((1.0, bld_stroke)));
            }
        } else if show_roads {
            // 瓦片底图模式下，建筑与操场仅用轻量半透明轮廓辅助参考
            let bld_stroke = Color32::from_rgba_unmultiplied(180, 130, 95, 95);
            for ring in &plan.buildings {
                self.draw_polygon(&clipped, rect, ring, bld_stroke, 1.0);
            }
            let pitch_stroke = Color32::from_rgba_unmultiplied(34, 150, 80, 140);
            for ring in &plan.pitches {
                self.draw_polygon(&clipped, rect, ring, pitch_stroke, 1.4);
            }
        }

        // 5. 路网拓扑线
        if show_roads {
            let road_w = (self.scale * 2.2).clamp(1.2, 4.2);
            if has_tiles {
                // 在线瓦片上方：绘制半透明青蓝色可跑路网拓扑线，清晰指示哪些路在 OSM 路网中可寻路
                let topo_color = Color32::from_rgba_unmultiplied(16, 135, 115, 155);
                for e in &plan.edges {
                    self.draw_polyline(&clipped, rect, e, topo_color, road_w.min(2.2));
                }
            } else {
                // 离线矢量图：先画道路灰色路肩，再画白色/浅黄路面（真实地图双线风格）
                let casing = Color32::from_rgb(184, 194, 190);
                let fill = Color32::from_rgb(255, 255, 255);
                for e in &plan.edges {
                    self.draw_polyline(&clipped, rect, e, casing, road_w + 1.6);
                }
                for e in &plan.edges {
                    self.draw_polyline(&clipped, rect, e, fill, road_w);
                }
            }
        }

        // 6. 建筑 / 操场 / 道路 / 地标 中文名称标注（碰撞避让）
        if show_labels && (!has_tiles || self.scale >= 0.45) {
            self.draw_labels(&clipped, rect, &plan.labels, has_tiles);
        }
    }

    /// 使用耳切法（Ear-Clipping）将任意凹凸多边形三角剖分并填充绘制。
    pub fn draw_filled_polygon(
        &self,
        painter: &egui::Painter,
        rect: Rect,
        pts: &[(f64, f64)],
        fill: Color32,
        stroke: Option<(f32, Color32)>,
    ) {
        if pts.len() < 3 {
            return;
        }
        let mut screen_pts: Vec<Pos2> = pts
            .iter()
            .map(|(la, lo)| self.to_screen(rect, *la, *lo))
            .collect();
        // 去除首尾重合点
        if screen_pts.len() >= 3
            && screen_pts.first().unwrap().distance(*screen_pts.last().unwrap()) < 0.5
        {
            screen_pts.pop();
        }
        if screen_pts.len() < 3 {
            return;
        }
        // 视锥裁剪：整块多边形在画布外则跳过
        let poly_bbox = Rect::from_points(&screen_pts);
        if !poly_bbox.intersects(rect) {
            return;
        }

        let tris = triangulate_earclip(&screen_pts);
        if !tris.is_empty() {
            let mut mesh = Mesh::default();
            for &p in &screen_pts {
                mesh.colored_vertex(p, fill);
            }
            mesh.indices = tris;
            painter.add(egui::Shape::mesh(mesh));
        }

        if let Some((w, c)) = stroke {
            let mut closed = screen_pts;
            closed.push(closed[0]);
            painter.add(egui::Shape::closed_line(closed, Stroke::new(w, c)));
        }
    }

    /// 绘制带屏幕碰撞检测避让的地图文字标签。
    fn draw_labels(
        &self,
        painter: &egui::Painter,
        rect: Rect,
        labels: &[(f64, f64, String, String)],
        has_tiles: bool,
    ) {
        // 缩放太小时只显示操场/地标，放大后再显示建筑与路名
        let min_scale_for_bld = 0.22f32;
        let mut placed: Vec<Rect> = Vec::new();
        let inner_rect = rect.shrink(14.0);

        for (lat, lon, name, kind) in labels {
            if has_tiles && kind == "road" {
                continue; // 在线瓦片自带路名
            }
            if self.scale < min_scale_for_bld && (kind == "building" || kind == "road") {
                continue;
            }
            let pos = self.to_screen(rect, *lat, *lon);
            if !inner_rect.contains(pos) {
                continue;
            }
            let char_count = name.chars().count().max(1) as f32;
            let approx_w = (char_count * 11.5 + 8.0).min(160.0);
            let approx_h = 16.0;
            let box_rect = Rect::from_center_size(pos, Vec2::new(approx_w, approx_h));
            if placed.iter().any(|r| r. expand(4.0).intersects(box_rect)) {
                continue;
            }
            placed.push(box_rect);

            let text_color = match kind.as_str() {
                "sport" => Color32::from_rgb(26, 118, 62),
                "road" => Color32::from_rgb(82, 96, 108),
                _ => Color32::from_rgb(92, 74, 60),
            };
            // 半透明白底胶囊衬底，保证文字清晰可读
            painter.rect_filled(
                box_rect,
                egui::Rounding::same(3.0),
                Color32::from_rgba_unmultiplied(255, 255, 255, 205),
            );
            painter.text(
                pos,
                Align2::CENTER_CENTER,
                name,
                FontId::proportional(11.0),
                text_color,
            );
        }
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
        let clipped = painter.with_clip_rect(rect);
        let ps: Vec<Pos2> = pts
            .iter()
            .map(|(la, lo)| self.to_screen(rect, *la, *lo))
            .collect();
        clipped.add(egui::Shape::line(
            ps.clone(),
            Stroke::new(
                width + 2.4,
                Color32::from_rgba_unmultiplied(255, 255, 255, 215),
            ),
        ));
        clipped.add(egui::Shape::line(ps.clone(), Stroke::new(width, color)));

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
                    clipped.add(egui::Shape::line(
                        vec![wing1, tip, wing2],
                        Stroke::new(1.8_f32, Color32::WHITE),
                    ));
                    clipped.add(egui::Shape::line(
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
        if rect.contains(c) {
            painter.circle_filled(c, radius, color);
        }
    }

    /// 绘制带序号与起终点配色的途经点徽章。
    pub fn draw_waypoints(&self, painter: &egui::Painter, rect: Rect, waypoints: &[(f64, f64)]) {
        let clipped = painter.with_clip_rect(rect);
        let total = waypoints.len();
        for (i, &(lat, lon)) in waypoints.iter().enumerate() {
            let pos = self.to_screen(rect, lat, lon);
            if !rect.expand(20.0).contains(pos) {
                continue;
            }
            let is_active = self.dragging_wp == Some(i) || self.hover_wp == Some(i);
            let fill = if i == 0 {
                Color32::from_rgb(22, 160, 90)
            } else if i + 1 == total {
                Color32::from_rgb(220, 50, 47)
            } else {
                Color32::from_rgb(30, 111, 216)
            };
            let radius = if is_active { 10.5 } else { 8.5 };
            if is_active {
                clipped.circle_stroke(
                    pos,
                    radius + 3.0,
                    Stroke::new(
                        2.0_f32,
                        Color32::from_rgba_unmultiplied(fill.r(), fill.g(), fill.b(), 140),
                    ),
                );
            }
            clipped.circle_filled(pos, radius, fill);
            clipped.circle_stroke(pos, radius, Stroke::new(1.8_f32, Color32::WHITE));
            let text = if i == 0 {
                "起".to_string()
            } else {
                format!("{}", i + 1)
            };
            clipped.text(
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
        let candidates = [
            10.0f32, 20.0, 50.0, 100.0, 200.0, 500.0, 1000.0, 2000.0, 5000.0,
        ];
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
        let bg = Rect::from_min_max(Pos2::new(x0 - 4.0, y0 - 18.0), Pos2::new(x1 + 4.0, y0 + 4.0));
        painter.rect_filled(
            bg,
            egui::Rounding::same(3.0),
            Color32::from_rgba_unmultiplied(255, 255, 255, 195),
        );
        let color = Color32::from_rgb(55, 68, 64);
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
        let pos = Pos2::new(rect.right() - 8.0, rect.bottom() - 8.0);
        painter.text(
            pos,
            Align2::RIGHT_BOTTOM,
            text,
            FontId::proportional(11.0),
            Color32::from_rgb(75, 88, 84),
        );
    }
}

/// 2D 任意多边形耳切三角剖分（支持凹多边形建筑/水域轮廓）。
fn triangulate_earclip(pts: &[Pos2]) -> Vec<u32> {
    let n = pts.len();
    if n < 3 || n > 500 {
        return Vec::new();
    }
    if n == 3 {
        return vec![0, 1, 2];
    }
    // 计算有向面积判断顶点环绕方向
    let mut area2 = 0.0f32;
    for i in 0..n {
        let a = pts[i];
        let b = pts[(i + 1) % n];
        area2 += a.x * b.y - b.x * a.y;
    }
    let sign = if area2 >= 0.0 { 1.0f32 } else { -1.0f32 };
    let mut verts: Vec<usize> = (0..n).collect();
    let mut out: Vec<u32> = Vec::with_capacity((n - 2) * 3);
    let mut guard = n * 3;

    let cross2 = |a: Pos2, b: Pos2, c: Pos2| -> f32 {
        (b.x - a.x) * (c.y - a.y) - (b.y - a.y) * (c.x - a.x)
    };
    let in_tri = |p: Pos2, a: Pos2, b: Pos2, c: Pos2| -> bool {
        let c1 = cross2(a, b, p) * sign;
        let c2 = cross2(b, c, p) * sign;
        let c3 = cross2(c, a, p) * sign;
        c1 >= -1e-4 && c2 >= -1e-4 && c3 >= -1e-4
    };

    let mut idx = 0usize;
    while verts.len() > 3 && guard > 0 {
        guard -= 1;
        let m = verts.len();
        let i_prev = verts[(idx + m - 1) % m];
        let i_curr = verts[idx % m];
        let i_next = verts[(idx + 1) % m];
        let (a, b, c) = (pts[i_prev], pts[i_curr], pts[i_next]);

        if cross2(a, b, c) * sign > 1e-4 {
            let mut ear = true;
            for &vi in &verts {
                if vi == i_prev || vi == i_curr || vi == i_next {
                    continue;
                }
                if in_tri(pts[vi], a, b, c) {
                    ear = false;
                    break;
                }
            }
            if ear {
                out.push(i_prev as u32);
                out.push(i_curr as u32);
                out.push(i_next as u32);
                verts.remove(idx % m);
                idx = 0;
                continue;
            }
        }
        idx += 1;
    }
    if verts.len() == 3 {
        out.push(verts[0] as u32);
        out.push(verts[1] as u32);
        out.push(verts[2] as u32);
    }
    out
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

        let pivot = Pos2::new(210.0, 190.0);
        let (plat, plon) = map.from_screen(rect, pivot);
        map.zoom_at(rect, pivot, 1.8);
        let after_screen = map.to_screen(rect, plat, plon);
        assert!(
            after_screen.distance(pivot) < 0.05,
            "zoom_at shifted pivot: {after_screen:?} vs {pivot:?}"
        );
    }

    #[test]
    fn tile_coordinate_math_and_triangulation_work() {
        let (tx, ty) = latlon_to_tile_frac(30.2741, 120.1551, 16);
        let (lat, lon) = tile_to_latlon(tx.floor() as i32, ty.floor() as i32, 16);
        assert!((lat - 30.2741).abs() < 0.01 && (lon - 120.1551).abs() < 0.01);

        let poly = vec![
            Pos2::new(0.0, 0.0),
            Pos2::new(100.0, 0.0),
            Pos2::new(100.0, 50.0),
            Pos2::new(50.0, 50.0),
            Pos2::new(50.0, 100.0),
            Pos2::new(0.0, 100.0),
        ];
        let tris = triangulate_earclip(&poly);
        assert_eq!(tris.len(), (poly.len() - 2) * 3);
    }
}
