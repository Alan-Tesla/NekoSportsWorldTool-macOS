//! 数据模型与本地持久化。
//!
//! 持久化文件位于桌面 exe 目录或 Android 应用私有目录：identity.json（设备身份，device_id 固定复用
//! ——严禁每次随机，会触发 10121 风控）、session.json（登录态）、config.json
//! （账号/参数）、points_cache.json（点位缓存）。

pub use crate::crypto::header::HeaderIdentity;

pub const HOST: &str = "https://run.gxapp.iydsj.com";
/// 排行榜 / 违规名单域名（信封链与 RUN 相同）。
pub const DISCOVERY: &str = "https://discovery.gxapp.iydsj.com";

/// 登录态（session.json）。
#[derive(Debug, Clone, Default, serde::Serialize, serde::Deserialize)]
pub struct Session {
    #[serde(default)]
    pub uid: i64,
    #[serde(default)]
    pub token: String,
    #[serde(default)]
    pub unid: String,
    #[serde(default)]
    pub name: String,
    #[serde(default = "default_weight")]
    pub weight: f64,
    #[serde(default)]
    pub username: String,
    #[serde(default)]
    pub device_id: String,
    /// 登录响应完整业务数据（我的页展示）
    #[serde(default)]
    pub profile: serde_json::Value,
}

fn default_weight() -> f64 {
    68.0
}

impl Session {
    pub fn is_logged_in(&self) -> bool {
        self.uid >= 1 && !self.token.is_empty()
    }
}

/// A local named route; personal coordinates stay in config.json.
#[derive(Debug, Clone, serde::Serialize, serde::Deserialize)]
pub struct CustomRoutePreset {
    pub id: String,
    pub waypoints: Vec<(f64, f64)>,
    pub close_mode: String,
    pub clip_fence: bool,
    pub snap_road: bool,
    /// Approved edge indices in this saved base; empty after manual replacement.
    #[serde(default)]
    pub random_main_road_edges: Vec<usize>,
    /// Optional internal base routes for a single combined UI preset.
    #[serde(default)]
    pub random_source_ids: Vec<String>,
}

/// UI 参数（config.json）。
#[derive(Debug, Clone, serde::Serialize, serde::Deserialize)]
pub struct Config {
    #[serde(default)]
    pub username: String,
    #[serde(default)]
    pub password: String,
    #[serde(default)]
    pub remember: bool,
    #[serde(default = "default_f32")]
    pub dist_min: f32,
    #[serde(default = "default_f32")]
    pub dist_max: f32,
    #[serde(default = "default_pace_min")]
    pub pace_min: f32,
    #[serde(default = "default_pace_max")]
    pub pace_max: f32,
    #[serde(default)]
    pub face_check: bool,
    /// 跑步时覆盖轨迹点 bdA；为空表示使用生成器的海拔曲线。
    #[serde(default)]
    pub manual_altitude: Option<f64>,
    /// 跑步时将生成器海拔曲线映射到此范围；为空表示不使用范围覆盖。
    #[serde(default)]
    pub manual_altitude_range: Option<crate::track::altitude::AltitudeRange>,
    #[serde(default = "default_ai_minutes")]
    pub ai_minutes: i64,
    #[serde(default = "default_ai_reps")]
    pub ai_reps: i64,
    /// 启动时检查更新：silent（静默，发现新版才弹窗）/ ask（每次询问）/ off
    #[serde(default = "default_update_check")]
    pub update_check: String,
    /// OSM 路网文件路径（真实道路路由用）。
    #[serde(default)]
    pub osm_path: String,
    /// 路线算法：legacy | road | custom。
    #[serde(default = "default_route_mode")]
    pub route_mode: String,
    /// 用户在地图上自定义规划的途经点列表（BD-09，(lat, lon)）。
    #[serde(default)]
    pub custom_waypoints: Vec<(f64, f64)>,
    #[serde(default)]
    pub custom_route_presets: Vec<CustomRoutePreset>,
    #[serde(default)]
    pub custom_generated_preset: String,
    #[serde(default)]
    pub custom_generated_route_hash: u64,
    /// 自定义路径闭合方式：loop | out_and_back | auto_pad。
    #[serde(default = "default_custom_close_mode")]
    pub custom_close_mode: String,
    /// 自定义路径是否受电子围栏裁剪。
    #[serde(default = "default_true")]
    pub custom_clip_fence: bool,
    /// 自定义路径是否启用道路吸附与沿路网寻路（false = 自由手绘/操场跑圈模式，点哪走哪）。
    #[serde(default)]
    pub custom_snap_road: bool,
    /// 跑圈单圈周长（米，默认 400.0）。
    #[serde(default = "default_loop_perimeter")]
    pub loop_perimeter: f64,
    /// 跑圈圈数（默认 2 圈）。
    #[serde(default = "default_loop_laps")]
    pub loop_laps: i64,
    /// 跑圈形状：true=标准操场椭圆，false=正圆。
    #[serde(default = "default_true")]
    pub loop_is_oval: bool,
    /// 跑圈走向旋转角（度，0=南北走向）。
    #[serde(default)]
    pub loop_rotation_deg: f64,
    /// 跑圈转向：true=逆时针左转（标准跑道），false=顺时针。
    #[serde(default = "default_true")]
    pub loop_ccw: bool,
    /// 跑圈入点模式：auto | south | north | east | west。
    #[serde(default = "default_loop_entry")]
    pub loop_entry_mode: String,
    /// 跑圈出点模式：same | opposite | north | south | east | west。
    #[serde(default = "default_loop_exit")]
    pub loop_exit_mode: String,
    /// 允许未经过的普通打卡点数量（学校 5 选 3 规则允许未过 2 个普通点，默认 2）。
    #[serde(default = "default_allowed_skip")]
    pub allowed_skip_ordinary: i64,
    /// 地图底图源：gaode | osm | vector。
    #[serde(default = "default_tile_source")]
    pub tile_source: String,
}

fn default_allowed_skip() -> i64 {
    2
}

fn default_loop_perimeter() -> f64 {
    400.0
}

fn default_loop_laps() -> i64 {
    2
}

fn default_loop_entry() -> String {
    "auto".into()
}

fn default_loop_exit() -> String {
    "same".into()
}

fn default_route_mode() -> String {
    "legacy".into()
}

fn default_custom_close_mode() -> String {
    "open".into()
}

fn default_true() -> bool {
    true
}

fn default_tile_source() -> String {
    "gaode".into()
}

fn default_f32() -> f32 {
    1.0
}
fn default_pace_min() -> f32 {
    360.0
}
fn default_pace_max() -> f32 {
    480.0
}
fn default_ai_minutes() -> i64 {
    1
}
fn default_ai_reps() -> i64 {
    5
}
fn default_update_check() -> String {
    if cfg!(target_os = "macos") { "off".into() } else { "silent".into() }
}

impl Default for Config {
    fn default() -> Self {
        Self {
            username: String::new(),
            password: String::new(),
            remember: false,
            dist_min: 1.0,
            dist_max: 1.5,
            pace_min: default_pace_min(),
            pace_max: default_pace_max(),
            face_check: true,
            manual_altitude: None,
            manual_altitude_range: None,
            ai_minutes: default_ai_minutes(),
            ai_reps: default_ai_reps(),
            update_check: default_update_check(),
            osm_path: String::new(),
            route_mode: default_route_mode(),
            custom_waypoints: Vec::new(),
            custom_route_presets: Vec::new(),
            custom_generated_preset: String::new(),
            custom_generated_route_hash: 0,
            custom_close_mode: default_custom_close_mode(),
            custom_clip_fence: true,
            custom_snap_road: false,
            loop_perimeter: default_loop_perimeter(),
            loop_laps: default_loop_laps(),
            loop_is_oval: true,
            loop_rotation_deg: 0.0,
            loop_ccw: true,
            loop_entry_mode: default_loop_entry(),
            loop_exit_mode: default_loop_exit(),
            allowed_skip_ordinary: default_allowed_skip(),
            tile_source: default_tile_source(),
        }
    }
}

// ── 持久化（平台数据目录）──────────────────────────────────────

fn exe_dir() -> std::path::PathBuf {
    crate::platform::data_dir()
}

impl Config {
    // Stable non-security fingerprint; manual geometry/mode edits invalidate
    // provenance without retaining an extra copy of location coordinates.
    fn custom_route_hash(&self) -> u64 {
        let mut hash=14695981039346656037u64;
        let mut feed=|bytes: &[u8]| {for b in bytes {hash^=*b as u64;hash=hash.wrapping_mul(1099511628211);}};
        for (lat,lon) in &self.custom_waypoints {feed(&lat.to_bits().to_le_bytes());feed(&lon.to_bits().to_le_bytes());}
        feed(self.custom_close_mode.as_bytes());feed(&[self.custom_clip_fence as u8,self.custom_snap_road as u8]);hash
    }
    pub fn custom_preset_random_selected(&self, id: &str) -> bool {
        self.route_mode=="custom" && self.custom_generated_preset==id
            && self.custom_generated_route_hash==self.custom_route_hash()
            && self.custom_route_presets.iter().any(|p|p.id==id)
    }

    pub fn save_custom_preset(&mut self, id: &str) -> bool {
        if self.custom_waypoints.len() < 2 { return false; }
        let Some(preset) = self.custom_route_presets.iter_mut().find(|p| p.id == id) else { return false; };
        if preset.waypoints != self.custom_waypoints {
            preset.random_main_road_edges.clear();
            preset.random_source_ids.clear();
        }
        preset.waypoints = self.custom_waypoints.clone();
        preset.close_mode = self.custom_close_mode.clone();
        preset.clip_fence = self.custom_clip_fence;
        preset.snap_road = self.custom_snap_road;
        self.custom_generated_preset.clear();
        true
    }

    pub fn custom_preset_selected(&self, id: &str) -> bool {
        self.custom_preset_random_selected(id) || self.custom_route_presets.iter().any(|p| p.id == id
            && p.waypoints == self.custom_waypoints
            && p.close_mode == self.custom_close_mode
            && p.clip_fence == self.custom_clip_fence
            && p.snap_road == self.custom_snap_road)
    }

    fn random_route_sources(&self, id: &str) -> Result<Vec<CustomRoutePreset>,String> {
        let preset=self.custom_route_presets.iter().find(|p|p.id==id)
            .ok_or("未找到基础方案")?;
        if preset.random_source_ids.is_empty() {return Ok(vec![preset.clone()]);}
        let mut sources=Vec::new();
        for source_id in &preset.random_source_ids {
            let source=self.custom_route_presets.iter()
                .find(|p|p.id==*source_id && p.id!=id && p.random_source_ids.is_empty())
                .ok_or("合并方案的基础路线配置无效")?;
            if !sources.iter().any(|p: &CustomRoutePreset|p.id==source.id) {sources.push(source.clone());}
        }
        Ok(sources)
    }

    pub fn randomize_main_road_preset(&mut self, id: &str, checkpoints: &[(f64,f64)]) -> Result<(),String> {
        use rand::seq::SliceRandom;
        let mut sources=self.random_route_sources(id)?;
        let mut rng=rand::thread_rng();sources.shuffle(&mut rng);
        let mut reason="没有可用的基础路线".to_string();
        for preset in sources {
            let route=match super::route_variants::vary_main_roads(&preset.waypoints,
                &self.custom_waypoints,&preset.random_main_road_edges,&preset.close_mode,
                checkpoints,&mut rng) {
                Ok(route)=>route,Err(e)=>{reason=e;continue;}
            };
            self.custom_waypoints=route;
            self.custom_close_mode=preset.close_mode;
            self.custom_clip_fence=preset.clip_fence;
            self.custom_snap_road=preset.snap_road;
            self.route_mode="custom".into();self.allowed_skip_ordinary=0;
            self.custom_generated_preset=id.into();
            self.custom_generated_route_hash=self.custom_route_hash();
            return Ok(());
        }
        Err(reason)
    }

    /// Generate from the saved base, never from the previous random result.
    /// Preserves the base preset and its road segment inventory.
    #[cfg(test)]
    pub fn randomize_custom_preset(&mut self, id: &str) -> bool {
        let Some(preset) = self.custom_route_presets.iter().find(|p| p.id == id).cloned()
            else { return false; };
        let Some(route) = super::route_variants::random_route(
            &preset.waypoints, &self.custom_waypoints, &mut rand::thread_rng())
            else { return false; };
        self.custom_waypoints = route;
        self.custom_close_mode = preset.close_mode;
        self.custom_clip_fence = preset.clip_fence;
        self.custom_snap_road = preset.snap_road;
        self.route_mode = "custom".into();
        self.allowed_skip_ordinary = 0;
        true
    }

    pub fn apply_custom_preset(&mut self, id: &str) -> bool {
        let Some(preset) = self.custom_route_presets.iter()
            .find(|p| p.id == id && p.waypoints.len() >= 2).cloned() else { return false };
        self.custom_generated_preset.clear();
        self.custom_waypoints = preset.waypoints;
        self.custom_close_mode = preset.close_mode;
        self.custom_clip_fence = preset.clip_fence;
        self.custom_snap_road = preset.snap_road;
        self.route_mode = "custom".into();
        self.allowed_skip_ordinary = 0;
        true
    }
}

fn read_json<T: serde::de::DeserializeOwned>(name: &str) -> Option<T> {
    let path = exe_dir().join(name);
    let text = std::fs::read_to_string(path).ok()?;
    serde_json::from_str(&text).ok()
}

fn write_json<T: serde::Serialize>(name: &str, value: &T) -> Result<(), String> {
    let directory = exe_dir();
    std::fs::create_dir_all(&directory).map_err(|e| format!("创建数据目录失败: {e}"))?;
    let path = directory.join(name);
    let json =
        serde_json::to_string_pretty(value).map_err(|e| format!("序列化 {name} 失败: {e}"))?;
    std::fs::write(&path, json).map_err(|e| format!("写入 {} 失败: {e}", path.display()))
}

/// 加载设备身份；device_id / app_install_time 缺失时生成一次并立即落盘，
/// 此后同一设备全生命周期复用（逐请求漂移会影响设备一致性）。
pub fn load_identity() -> HeaderIdentity {
    load_identity_for_platform(if cfg!(target_os = "android") {
        "android"
    } else {
        "ios"
    })
}

fn load_identity_for_platform(platform: &str) -> HeaderIdentity {
    let mut id: HeaderIdentity = read_json("identity.json").unwrap_or_else(|| {
        let mut identity = HeaderIdentity::default();
        if platform == "android" {
            identity.platform = "android".into();
            // Generic fallback until the user opts in or fills the fields manually.
            identity.device_name = "Android".into();
            identity.os_version = "16".into();
        }
        identity
    });
    let mut dirty = false;
    if id.device_id.is_empty() {
        id.device_id = uuid::Uuid::new_v4().to_string().to_uppercase();
        dirty = true;
    }
    if id.app_install_time <= 0 {
        id.app_install_time =
            crate::crypto::header::HeaderIdentity::fresh_install_time(&id.platform);
        dirty = true;
    }
    if id.mac_address.is_empty() {
        id.mac_address = crate::crypto::header::random_mac();
        dirty = true;
    }
    if dirty {
        let _ = save_identity(&id);
    }
    id
}

#[cfg(any(target_os = "android", test))]
#[derive(Debug, serde::Deserialize)]
pub struct DeviceInfo {
    pub manufacturer: String,
    pub model: String,
    pub os_version: String,
}

#[cfg(any(target_os = "android", test))]
pub fn identity_with_device_info(
    identity: &HeaderIdentity,
    info: &DeviceInfo,
) -> Result<HeaderIdentity, String> {
    if info.model.trim().is_empty() || info.os_version.trim().is_empty() {
        return Err("未能读取完整的机型和系统版本，请手动填写".into());
    }
    let mut updated = identity.clone();
    updated.platform = "android".into();
    updated.device_name = info.model.trim().into();
    updated.manufacturer = info.manufacturer.trim().into();
    updated.os_version = info.os_version.trim().into();
    Ok(updated)
}

pub fn save_identity(id: &HeaderIdentity) -> Result<(), String> {
    write_json("identity.json", id)
}

/// AI 项目列表永久缓存：拉取成功即落盘，网络异常时兜底展示。
pub fn load_ai_sports() -> Option<Vec<crate::api::ai::AiSport>> {
    read_json("ai_sports.json")
}

pub fn save_ai_sports(list: &[crate::api::ai::AiSport]) -> Result<(), String> {
    write_json("ai_sports.json", &list.to_vec())
}

pub fn load_session() -> Session {
    read_json("session.json").unwrap_or_default()
}

pub fn save_session(s: &Session) -> Result<(), String> {
    write_json("session.json", s)
}

pub fn clear_session() {
    let _ = std::fs::remove_file(exe_dir().join("session.json"));
}

pub fn load_config() -> Config {
    read_json("config.json").unwrap_or_default()
}

pub fn save_config(c: &Config) -> Result<(), String> {
    write_json("config.json", c)
}

#[cfg(test)]
mod storage_tests {
    use super::*;

    #[test]
    fn android_identity_import_persists_brand_and_reuses_uuid() {
        let directory =
            std::env::temp_dir().join(format!("neko-identity-{}", uuid::Uuid::new_v4()));
        std::fs::create_dir_all(&directory).unwrap();
        crate::platform::TEST_DATA_DIR.with(|p| *p.borrow_mut() = Some(directory.clone()));
        struct Cleanup(std::path::PathBuf);
        impl Drop for Cleanup {
            fn drop(&mut self) {
                crate::platform::TEST_DATA_DIR.with(|p| *p.borrow_mut() = None);
                let _ = std::fs::remove_dir_all(&self.0);
            }
        }
        let _cleanup = Cleanup(directory);
        let first = load_identity_for_platform("android");
        assert_eq!(first.platform, "android");
        assert_ne!(first.device_name, "iPhone");
        assert!(uuid::Uuid::parse_str(&first.device_id).is_ok());
        let again = load_identity_for_platform("android");
        assert_eq!(
            serde_json::to_value(&first).unwrap(),
            serde_json::to_value(&again).unwrap()
        );

        let mut existing = first.clone();
        existing.platform = "ios".into();
        existing.device_name = "Existing manual name".into();
        existing.idfa = "manual-value".into();
        let mut legacy = serde_json::to_value(&existing).unwrap();
        legacy.as_object_mut().unwrap().remove("manufacturer");
        write_json("identity.json", &legacy).unwrap();
        let loaded = load_identity_for_platform("android");
        assert_eq!(
            serde_json::to_value(&loaded).unwrap(),
            serde_json::to_value(&existing).unwrap(),
            "opening an existing install must not overwrite a manual identity"
        );

        let info: DeviceInfo = serde_json::from_str(
            r#"{"manufacturer":"Example","model":"Phone 16","os_version":"16"}"#,
        )
        .unwrap();
        assert_eq!(info.manufacturer, "Example");
        let imported = identity_with_device_info(&loaded, &info).unwrap();
        assert_eq!(imported.platform, "android");
        assert_eq!(imported.device_name, "Phone 16");
        assert_eq!(imported.os_version, "16");
        let mut expected = serde_json::to_value(&existing).unwrap();
        expected["platform"] = "android".into();
        expected["device_name"] = "Phone 16".into();
        expected["manufacturer"] = "Example".into();
        expected["os_version"] = "16".into();
        assert_eq!(
            serde_json::to_value(&imported).unwrap(),
            expected,
            "import must preserve UUID, MAC, manual IMEI/IDFA, install time and location"
        );
        assert_eq!(
            load_identity().device_name,
            existing.device_name,
            "preview must not save implicitly"
        );
        save_identity(&imported).unwrap();
        let stored: serde_json::Value = read_json("identity.json").unwrap();
        assert_eq!(
            stored.get("manufacturer").and_then(|value| value.as_str()),
            Some("Example"),
            "the consented brand must be written to identity.json"
        );
        let reloaded = load_identity();
        assert_eq!(
            serde_json::to_value(&reloaded).unwrap(),
            serde_json::to_value(&imported).unwrap(),
            "brand and identity must survive a fresh load from disk"
        );
        assert_eq!(reloaded.device_id, first.device_id);
        let (header, _) =
            crate::crypto::header::build_header_for(&reloaded, 0, "", Some(1_700_000_000_000));
        let header: serde_json::Value = serde_json::from_str(&header).unwrap();
        assert_eq!(header["DeviceId"], first.device_id);
        assert_eq!(
            header["deviceName"], "Phone 16",
            "brand must not be prepended to the protocol model field"
        );
        assert_eq!(header["osVersion"], "16");
        assert!(
            header.get("manufacturer").is_none() && header.get("brand").is_none(),
            "persisting local brand metadata must not invent new protocol fields"
        );
    }

    #[test]
    fn incomplete_native_device_info_cannot_replace_identity() {
        let identity = HeaderIdentity::default();
        for (model, os_version) in [("", "16"), ("Phone", " ")] {
            let info = DeviceInfo {
                manufacturer: String::new(),
                model: model.into(),
                os_version: os_version.into(),
            };
            assert!(identity_with_device_info(&identity, &info).is_err());
        }
    }

    #[test]
    fn session_roundtrip_and_logout_use_private_data_directory() {
        let directory = std::env::temp_dir().join(format!("neko-storage-{}", uuid::Uuid::new_v4()));
        std::fs::create_dir_all(&directory).unwrap();
        crate::platform::TEST_DATA_DIR.with(|p| *p.borrow_mut() = Some(directory.clone()));
        struct Cleanup(std::path::PathBuf);
        impl Drop for Cleanup {
            fn drop(&mut self) {
                crate::platform::TEST_DATA_DIR.with(|p| *p.borrow_mut() = None);
                let _ = std::fs::remove_dir_all(&self.0);
            }
        }
        let _cleanup = Cleanup(directory.clone());
        assert_eq!(
            exe_dir(),
            directory,
            "storage must use the injected app directory"
        );
        let session = Session {
            uid: 123,
            token: "test-only-token".into(),
            ..Default::default()
        };
        save_session(&session).unwrap();
        assert!(directory.join("session.json").is_file());
        assert_eq!(load_session().uid, 123);
        assert_eq!(load_session().token, session.token);
        clear_session();
        assert!(!directory.join("session.json").exists());
        assert_eq!(load_session().uid, 0);
    }
}

/// 点位缓存：{ts_ms, points}，TTL 300s（服务端限流 10603：5 分钟 3 次）。
pub const POINTS_TTL_MS: i64 = 300_000;

#[derive(Debug, serde::Serialize, serde::Deserialize)]
struct PointsCache {
    ts: i64,
    anchor: crate::location::Coordinate,
    points: Vec<serde_json::Value>,
}

/// 只返回与本次锚点相同的缓存。旧版未记录锚点的缓存会自然失效，避免串城市。
pub fn load_points_cache_for(
    anchor: crate::location::Coordinate,
) -> Option<(i64, Vec<serde_json::Value>)> {
    let v: serde_json::Value = read_json("points_cache.json")?;
    let cache: PointsCache = serde_json::from_value(v).ok()?;
    if !cache.anchor.is_near(anchor, 0.0001) {
        return None;
    }
    let ts = cache.ts;
    let pts = cache.points;
    Some((ts, pts))
}

pub fn save_points_cache(
    anchor: crate::location::Coordinate,
    points: &[serde_json::Value],
) -> Result<(), String> {
    let doc = PointsCache {
        ts: crate::crypto::envelope::now_ms(),
        anchor,
        points: points.to_vec(),
    };
    write_json("points_cache.json", &doc)
}

#[cfg(test)]
mod points_cache_tests {
    use super::*;
    #[test]
    fn cache_is_scoped_to_anchor() {
        let cache = PointsCache {
            ts: 1,
            anchor: crate::location::Coordinate::new(39.9, 116.4, 0.0).unwrap(),
            points: vec![],
        };
        let decoded: PointsCache =
            serde_json::from_value(serde_json::to_value(cache).unwrap()).unwrap();
        assert!(decoded.anchor.is_near(
            crate::location::Coordinate::new(39.9, 116.4, 0.0).unwrap(),
            0.0001
        ));
        assert!(!decoded.anchor.is_near(
            crate::location::Coordinate::new(38.9, 121.5, 0.0).unwrap(),
            0.0001
        ));
    }
}

/// 围栏缓存：拉取成功后落盘，UI 预览无网时兜底。
pub fn load_fence_cache() -> Option<Vec<Vec<(f64, f64)>>> {
    let v: serde_json::Value = read_json("fence_cache.json")?;
    serde_json::from_value(v.get("fences")?.clone()).ok()
}

pub fn save_fence_cache(fences: &[Vec<(f64, f64)>]) -> Result<(), String> {
    let doc = serde_json::json!({
        "ts": crate::crypto::envelope::now_ms(),
        "fences": fences,
    });
    write_json("fence_cache.json", &doc)
}

/// 必经打卡点缓存：拉取策略成功后落盘，供地图与路径规划识别必经点。
pub fn load_must_points_cache() -> Vec<(f64, f64)> {
    read_json("must_points_cache.json").unwrap_or_default()
}

pub fn save_must_points_cache(must_points: &[(f64, f64)]) -> Result<(), String> {
    write_json("must_points_cache.json", &must_points.to_vec())
}

#[cfg(test)]
mod custom_preset_tests {
    use super::*;
    #[test]
    fn switching_presets_roundtrips_routes_and_preserves_pace() {
        let mut c=Config::default();
        let pace=c.pace_min;
        for (id,lat) in [("stadium",1.0),("campus",2.0)] {
            c.custom_route_presets.push(CustomRoutePreset { id:id.into(),
                waypoints:vec![(lat,0.0),(lat,0.001)], close_mode:"open".into(),
                clip_fence:false,snap_road:false,random_main_road_edges:Vec::new(),random_source_ids:Vec::new() });
        }
        assert!(c.apply_custom_preset("stadium"));
        let original=c.custom_waypoints.clone();
        assert!(c.apply_custom_preset("campus"));
        assert!(c.custom_preset_selected("campus"));
        assert!(!c.custom_preset_selected("stadium"));
        let mut c:Config=serde_json::from_str(&serde_json::to_string(&c).unwrap()).unwrap();
        assert!(c.apply_custom_preset("stadium"));
        assert_eq!(c.custom_waypoints,original);
        assert_eq!(c.pace_min,pace);
        assert_eq!(c.allowed_skip_ordinary,0);
        c.custom_waypoints.reverse();
        assert!(!c.custom_preset_selected("stadium"));
        assert!(c.save_custom_preset("stadium"));
        assert!(c.custom_preset_selected("stadium"));
        let edited=c.custom_waypoints.clone();
        c.apply_custom_preset("campus");
        c.apply_custom_preset("stadium");
        assert_eq!(c.custom_waypoints,edited);
    }
    #[test]
    fn random_generation_keeps_both_bases_and_pace_and_can_restore() {
        let mut c=Config::default();
        for (id,lat) in [("stadium",1.0),("campus",2.0)] {
            c.custom_route_presets.push(CustomRoutePreset { id:id.into(),
                waypoints:vec![(lat,0.0),(lat,0.001),(lat+0.001,0.001),(lat,0.0)],
                close_mode:"rounded".into(),clip_fence:false,snap_road:false,random_main_road_edges:Vec::new(),random_source_ids:Vec::new() });
        }
        let originals=serde_json::to_string(&c.custom_route_presets).unwrap();
        let pace=(c.pace_min,c.pace_max);
        for id in ["stadium","campus"] {
            c.apply_custom_preset(id);
            let base=c.custom_waypoints.clone();
            assert!(c.randomize_custom_preset(id));
            assert_ne!(c.custom_waypoints,base);
            assert_eq!(c.custom_close_mode,"rounded");
            assert_eq!(c.allowed_skip_ordinary,0);
            assert_eq!((c.pace_min,c.pace_max),pace);
            assert_eq!(serde_json::to_string(&c.custom_route_presets).unwrap(), originals);
            assert!(c.apply_custom_preset(id));
            assert_eq!(c.custom_waypoints,base);
        }
        let before=c.custom_waypoints.clone();
        assert!(!c.randomize_custom_preset("missing"));
        assert_eq!(c.custom_waypoints,before);
    }
    #[test]
    fn unified_preset_resolves_both_sources_and_manual_save_invalidates_pool() {
        let mut c=Config::default();
        for id in ["stadium","campus","unified"] {
            c.custom_route_presets.push(CustomRoutePreset { id:id.into(),
                waypoints:vec![(29.0,119.0),(29.001,119.0),(29.0,119.0)],
                close_mode:"rounded".into(),clip_fence:false,snap_road:false,
                random_main_road_edges:vec![0,1],random_source_ids:Vec::new() });
        }
        c.custom_route_presets[2].random_source_ids=vec!["stadium".into(),"campus".into(),"campus".into()];
        assert_eq!(c.random_route_sources("unified").unwrap().iter().map(|p|p.id.as_str()).collect::<Vec<_>>(),vec!["stadium","campus"]);
        c.apply_custom_preset("unified");
        c.custom_waypoints.insert(1,(29.0005,119.0));
        assert!(c.save_custom_preset("unified"));
        assert!(c.custom_route_presets[2].random_source_ids.is_empty());
        assert!(c.custom_route_presets[2].random_main_road_edges.is_empty());
        assert_eq!(c.custom_route_presets[0].waypoints.len(),3);
        c.custom_route_presets[2].random_source_ids=vec!["unified".into()];
        assert!(c.random_route_sources("unified").is_err());
        c.custom_route_presets[2].random_source_ids=vec!["missing".into()];
        let before=c.custom_waypoints.clone();
        assert!(c.randomize_main_road_preset("unified",&[]).is_err());
        assert_eq!(before,c.custom_waypoints);
    }
    #[test]
    #[ignore = "local combined-route audit; requires explicit private config and point paths"]
    fn audit_local_unified_generation() {
        let path=std::env::var("NEKO_ROUTE_CHECK_CONFIG").unwrap();
        let mut c:Config=serde_json::from_str(&std::fs::read_to_string(path).unwrap()).unwrap();
        let path=std::env::var("NEKO_ROUTE_CHECK_POINTS").unwrap();
        let pts:serde_json::Value=serde_json::from_str(&std::fs::read_to_string(path).unwrap()).unwrap();
        let points:Vec<_>=pts["points"].as_array().unwrap().iter().map(|p|(p["lat"].as_f64().unwrap(),p["lon"].as_f64().unwrap())).collect();
        let bases=serde_json::to_string(&c.custom_route_presets).unwrap();
        let mut changed=std::collections::BTreeSet::new();
        for _ in 0..40 {
            let before=c.custom_waypoints.clone();
            c.randomize_main_road_preset("unified",&points).unwrap();
            assert_ne!(before,c.custom_waypoints);
            assert_eq!(bases,serde_json::to_string(&c.custom_route_presets).unwrap());
            assert_eq!(c.custom_close_mode,"rounded");
            assert_eq!(c.allowed_skip_ordinary,0);
            changed.insert(serde_json::to_string(&c.custom_waypoints).unwrap());
        }
        assert!(changed.len()>5);
    }
    #[test]
    fn generated_selection_survives_restart_and_detects_manual_edits() {
        let mut c=Config::default();
        c.custom_route_presets.push(CustomRoutePreset{id:"unified".into(),waypoints:vec![(29.0,119.0),(29.1,119.1)],close_mode:"rounded".into(),clip_fence:false,snap_road:false,random_main_road_edges:vec![],random_source_ids:vec![]});
        c.apply_custom_preset("unified");c.custom_waypoints.reverse();
        c.custom_generated_preset="unified".into();c.custom_generated_route_hash=c.custom_route_hash();
        let mut c:Config=serde_json::from_str(&serde_json::to_string(&c).unwrap()).unwrap();
        assert!(c.custom_preset_random_selected("unified"));assert!(c.custom_preset_selected("unified"));
        c.custom_waypoints[0].0+=0.001;assert!(!c.custom_preset_selected("unified"));
        c.apply_custom_preset("unified");assert!(!c.custom_preset_random_selected("unified"));assert!(c.custom_preset_selected("unified"));
    }
    #[test]
    fn old_config_without_presets_still_loads_and_missing_preset_is_safe() {
        let mut c:Config=serde_json::from_str("{}").unwrap();
        c.custom_waypoints=vec![(0.0,0.0),(0.0,0.001)];
        let points=c.custom_waypoints.clone();
        assert!(!c.apply_custom_preset("missing"));
        assert_eq!(c.custom_waypoints,points);
    }
}
