//! OSM `.osm`（XML）解析：节点 / 道路 way / 建筑 way+multipolygon relation。
//!
//! 输入为 openstreetmap.org 导出的标准 OSM XML（`<node>`/`<way>`/`<relation>`）。
//! 只抽取：`highway` 路径（剔除机动车道）、`building` 轮廓。

use std::collections::HashMap;

use quick_xml::events::Event;
use quick_xml::Reader;

use crate::graph::{Coord, EdgeData, MapLabel, RoadGraph};

/// 一条道路 way。
pub struct OsmRoad {
    pub id: u64,
    pub node_refs: Vec<u64>,
    pub oneway: bool,
    pub oneway_reversed: bool,
    pub highway: String,
    pub maxspeed_kmh: Option<f64>,
    pub name: Option<String>,
}

/// 一个建筑或区域多边形：外环为节点 id 序列。
pub struct OsmBuilding {
    pub outer: Vec<Vec<u64>>,
    pub name: Option<String>,
}

pub struct ParsedOsm {
    pub nodes: HashMap<u64, Coord>,
    pub roads: Vec<OsmRoad>,
    pub buildings: Vec<OsmBuilding>,
    pub waters: Vec<Vec<u64>>,
    pub greens: Vec<Vec<u64>>,
    pub pitches: Vec<OsmBuilding>,
    pub poi_labels: Vec<(u64, String)>,
}

/// 不应被当作可跑步道路的 highway 值。
fn is_motorized(h: &str) -> bool {
    matches!(
        h,
        "motorway"
            | "motorway_link"
            | "trunk"
            | "trunk_link"
            | "primary"
            | "primary_link"
            | "secondary"
            | "secondary_link"
            | "construction"
            | "proposed"
    )
}

fn parse_maxspeed(v: &str) -> Option<f64> {
    let digits: String = v.chars().filter(|c| c.is_ascii_digit()).collect();
    if digits.is_empty() {
        return None;
    }
    let n: f64 = digits.parse().ok()?;
    if v.to_ascii_lowercase().contains("mph") {
        Some(n * 1.60934)
    } else {
        Some(n)
    }
}

struct Attrs {
    map: HashMap<String, String>,
}

impl Attrs {
    fn new() -> Self {
        Attrs {
            map: HashMap::new(),
        }
    }
    fn insert(&mut self, k: &[u8], v: &[u8]) {
        self.map.insert(
            String::from_utf8_lossy(k).into_owned(),
            String::from_utf8_lossy(v).into_owned(),
        );
    }
    fn get(&self, k: &str) -> Option<&str> {
        self.map.get(k).map(|s| s.as_str())
    }
    fn id(&self) -> Option<u64> {
        self.get("id").and_then(|v| v.parse().ok())
    }
}

/// 解析 OSM XML 字节流。
pub fn parse_osm(bytes: &[u8]) -> Result<ParsedOsm, String> {
    let mut reader = Reader::from_reader(bytes);
    reader.config_mut().trim_text(true);

    let mut nodes: HashMap<u64, Coord> = HashMap::new();
    let mut roads: Vec<OsmRoad> = Vec::new();
    let mut buildings: Vec<OsmBuilding> = Vec::new();
    let mut waters: Vec<Vec<u64>> = Vec::new();
    let mut greens: Vec<Vec<u64>> = Vec::new();
    let mut pitches: Vec<OsmBuilding> = Vec::new();
    let mut poi_labels: Vec<(u64, String)> = Vec::new();

    // 所有 way 的节点序列（供 relation 建筑外环回填）。
    let mut way_nodes: HashMap<u64, Vec<u64>> = HashMap::new();
    // relation 建筑：收集 (outer way ids, name)。
    let mut rel_buildings: Vec<(Vec<u64>, Option<String>)> = Vec::new();

    let mut cur_node: Option<(u64, Vec<(String, String)>)> = None;
    let mut cur_way: Option<(u64, Vec<u64>, Vec<(String, String)>)> = None;
    let mut cur_rel: Option<Vec<(String, String, String)>> = None;
    let mut cur_rel_tags: Vec<(String, String)> = Vec::new();

    let mut buf = Vec::new();
    loop {
        buf.clear();
        let ev = reader.read_event_into(&mut buf);
        let is_empty_tag = matches!(ev, Ok(Event::Empty(_)));
        match ev {
            Ok(Event::Start(e)) | Ok(Event::Empty(e)) => {
                let local = String::from_utf8_lossy(e.name().as_ref()).into_owned();
                let mut attrs = Attrs::new();
                for a in e.attributes().with_checks(false) {
                    let a = a.map_err(|e| format!("属性解析失败: {e}"))?;
                    attrs.insert(a.key.as_ref(), a.value.as_ref());
                }
                match local.as_str() {
                    "node" => {
                        if let (Some(id), Some(lat), Some(lon)) = (
                            attrs.id(),
                            attrs.get("lat").and_then(|v| v.parse().ok()),
                            attrs.get("lon").and_then(|v| v.parse().ok()),
                        ) {
                            nodes.insert(id, Coord::new(lon, lat));
                            if !is_empty_tag {
                                cur_node = Some((id, Vec::new()));
                            }
                        }
                    }
                    "way" => {
                        cur_way = Some((attrs.id().unwrap_or(0), Vec::new(), Vec::new()));
                    }
                    "relation" => {
                        cur_rel = Some(Vec::new());
                        cur_rel_tags = Vec::new();
                    }
                    "nd" => {
                        if let Some((_, refs, _)) = cur_way.as_mut() {
                            if let Some(r) = attrs.get("ref").and_then(|v| v.parse().ok()) {
                                refs.push(r);
                            }
                        }
                    }
                    "tag" => {
                        let k = attrs.get("k").unwrap_or("").to_string();
                        let v = attrs.get("v").unwrap_or("").to_string();
                        if let Some((_, tags)) = cur_node.as_mut() {
                            tags.push((k.clone(), v.clone()));
                        }
                        if let Some((_, _, tags)) = cur_way.as_mut() {
                            tags.push((k.clone(), v.clone()));
                        }
                        if cur_rel.is_some() {
                            cur_rel_tags.push((k, v));
                        }
                    }
                    "member" => {
                        if let Some(members) = cur_rel.as_mut() {
                            let mtype = attrs.get("type").unwrap_or("").to_string();
                            let mref = attrs.get("ref").unwrap_or("").to_string();
                            let role = attrs.get("role").unwrap_or("").to_string();
                            members.push((mtype, mref, role));
                        }
                    }
                    _ => {}
                }
            }
            Ok(Event::End(e)) => {
                let local = String::from_utf8_lossy(e.name().as_ref()).into_owned();
                match local.as_str() {
                    "node" => {
                        if let Some((id, tags)) = cur_node.take() {
                            let name = tags
                                .iter()
                                .find(|(k, _)| k == "name")
                                .map(|(_, v)| v.trim().to_string())
                                .filter(|s| !s.is_empty());
                            let is_poi = tags.iter().any(|(k, _)| {
                                matches!(
                                    k.as_str(),
                                    "amenity" | "barrier" | "entrance" | "leisure" | "tourism" | "building"
                                )
                            });
                            if let (Some(n), true) = (name, is_poi) {
                                poi_labels.push((id, n));
                            }
                        }
                    }
                    "way" => {
                        if let Some((id, refs, tags)) = cur_way.take() {
                            way_nodes.insert(id, refs.clone());
                            let get_tag = |key: &str| -> Option<&str> {
                                tags.iter()
                                    .find(|(k, _)| k == key)
                                    .map(|(_, v)| v.as_str())
                            };
                            let name = get_tag("name")
                                .map(|s| s.trim().to_string())
                                .filter(|s| !s.is_empty());
                            let is_building =
                                tags.iter().any(|(k, v)| k == "building" && v != "no");
                            let leisure = get_tag("leisure").unwrap_or("");
                            let natural = get_tag("natural").unwrap_or("");
                            let landuse = get_tag("landuse").unwrap_or("");
                            let waterway = get_tag("waterway").unwrap_or("");

                            if is_building && refs.len() >= 3 {
                                buildings.push(OsmBuilding {
                                    outer: vec![refs.clone()],
                                    name: name.clone(),
                                });
                            }
                            if matches!(
                                leisure,
                                "pitch" | "track" | "stadium" | "sports_centre"
                            ) && refs.len() >= 3
                            {
                                pitches.push(OsmBuilding {
                                    outer: vec![refs.clone()],
                                    name: name.clone(),
                                });
                            }
                            if (natural == "water"
                                || matches!(waterway, "riverbank" | "pond" | "canal")
                                || matches!(landuse, "reservoir" | "basin"))
                                && refs.len() >= 3
                            {
                                waters.push(refs.clone());
                            }
                            if (matches!(
                                landuse,
                                "grass" | "meadow" | "forest" | "village_green" | "recreation_ground"
                            ) || matches!(leisure, "park" | "garden" | "nature_reserve")
                                || matches!(natural, "wood" | "grassland" | "scrub"))
                                && refs.len() >= 3
                            {
                                greens.push(refs.clone());
                            }
                            let highway_tag = get_tag("highway")
                                .map(|s| s.to_string())
                                .or_else(|| (leisure == "track").then(|| "track".to_string()));
                            if let Some(h) = highway_tag {
                                if !is_motorized(&h) && refs.len() >= 2 {
                                    let oneway = tags.iter().any(|(k, v)| {
                                        k == "oneway" && matches!(v.as_str(), "yes" | "true" | "1")
                                    });
                                    let oneway_reversed =
                                        tags.iter().any(|(k, v)| k == "oneway" && v == "-1");
                                    let maxspeed = get_tag("maxspeed").and_then(parse_maxspeed);
                                    roads.push(OsmRoad {
                                        id,
                                        node_refs: refs,
                                        oneway,
                                        oneway_reversed,
                                        highway: h,
                                        maxspeed_kmh: maxspeed,
                                        name,
                                    });
                                }
                            }
                        }
                    }
                    "relation" => {
                        if let Some(members) = cur_rel.take() {
                            let is_building = cur_rel_tags
                                .iter()
                                .any(|(k, v)| k == "building" && v != "no");
                            let name = cur_rel_tags
                                .iter()
                                .find(|(k, _)| k == "name")
                                .map(|(_, v)| v.trim().to_string())
                                .filter(|s| !s.is_empty());
                            if is_building {
                                let outer: Vec<u64> = members
                                    .iter()
                                    .filter(|(mtype, _r, role)| mtype == "way" && role == "outer")
                                    .filter_map(|(_, r, _)| r.parse().ok())
                                    .collect();
                                if !outer.is_empty() {
                                    rel_buildings.push((outer, name));
                                }
                            }
                        }
                        cur_rel_tags = Vec::new();
                    }
                    _ => {}
                }
            }
            Ok(Event::Eof) => break,
            Err(e) => return Err(format!("OSM 解析失败: {e}")),
            _ => {}
        }
    }

    // 回填 relation 建筑外环（outer way 的节点序列）。
    for (outer_ways, name) in rel_buildings {
        let mut rings: Vec<Vec<u64>> = Vec::new();
        for wid in outer_ways {
            if let Some(refs) = way_nodes.get(&wid) {
                if refs.len() >= 3 {
                    rings.push(refs.clone());
                }
            }
        }
        if !rings.is_empty() {
            buildings.push(OsmBuilding { outer: rings, name });
        }
    }

    Ok(ParsedOsm {
        nodes,
        roads,
        buildings,
        waters,
        greens,
        pitches,
        poi_labels,
    })
}

/// 用道路 way 构建路网图。
pub fn build_graph(parsed: &ParsedOsm) -> RoadGraph {
    let mut g = RoadGraph::new();
    let mut node_map: HashMap<u64, petgraph::graph::NodeIndex> = HashMap::new();

    for r in &parsed.roads {
        // 先解析节点 id → NodeIndex（避免闭包跨迭代借用）。
        let idxs: Vec<petgraph::graph::NodeIndex> = r
            .node_refs
            .iter()
            .filter_map(|id| {
                let c = *parsed.nodes.get(id)?;
                if let Some(&n) = node_map.get(id) {
                    Some(n)
                } else {
                    let n = g.add_node(c);
                    node_map.insert(*id, n);
                    Some(n)
                }
            })
            .collect();
        for w in idxs.windows(2) {
            let (p, next) = (w[0], w[1]);
            let a = g.node_coord(p);
            let b = g.node_coord(next);
            let len = geo_haversine(a, b);
            if !r.oneway_reversed {
                g.add_edge(
                    p,
                    next,
                    EdgeData {
                        way_id: r.id,
                        oneway: r.oneway,
                        highway: r.highway.clone(),
                        maxspeed_kmh: r.maxspeed_kmh,
                        length_m: len,
                    },
                );
            }
            if !r.oneway {
                g.add_edge(
                    next,
                    p,
                    EdgeData {
                        way_id: r.id,
                        oneway: r.oneway,
                        highway: r.highway.clone(),
                        maxspeed_kmh: r.maxspeed_kmh,
                        length_m: len,
                    },
                );
            }
        }
    }

    let mut bld: Vec<Vec<Coord>> = Vec::new();
    let mut labels: Vec<MapLabel> = Vec::new();
    let ring_centroid = |coords: &[Coord]| -> Coord {
        let n = coords.len().max(1) as f64;
        let lat = coords.iter().map(|c| c.lat).sum::<f64>() / n;
        let lon = coords.iter().map(|c| c.lon).sum::<f64>() / n;
        Coord::new(lon, lat)
    };

    for b in &parsed.buildings {
        let mut first_centroid = None;
        for ring in &b.outer {
            let coords: Vec<Coord> = ring
                .iter()
                .filter_map(|id| parsed.nodes.get(id).copied())
                .collect();
            if coords.len() >= 3 {
                if first_centroid.is_none() {
                    first_centroid = Some(ring_centroid(&coords));
                }
                bld.push(coords);
            }
        }
        if let (Some(name), Some(coord)) = (&b.name, first_centroid) {
            labels.push(MapLabel {
                coord,
                name: name.clone(),
                kind: "building".into(),
            });
        }
    }
    g.buildings = bld;

    let mut pitches: Vec<Vec<Coord>> = Vec::new();
    for p in &parsed.pitches {
        let mut first_centroid = None;
        for ring in &p.outer {
            let coords: Vec<Coord> = ring
                .iter()
                .filter_map(|id| parsed.nodes.get(id).copied())
                .collect();
            if coords.len() >= 3 {
                if first_centroid.is_none() {
                    first_centroid = Some(ring_centroid(&coords));
                }
                pitches.push(coords);
            }
        }
        if let (Some(name), Some(coord)) = (&p.name, first_centroid) {
            labels.push(MapLabel {
                coord,
                name: name.clone(),
                kind: "sport".into(),
            });
        }
    }
    g.pitches = pitches;

    g.waters = parsed
        .waters
        .iter()
        .filter_map(|ring| {
            let coords: Vec<Coord> = ring
                .iter()
                .filter_map(|id| parsed.nodes.get(id).copied())
                .collect();
            (coords.len() >= 3).then_some(coords)
        })
        .collect();

    g.greens = parsed
        .greens
        .iter()
        .filter_map(|ring| {
            let coords: Vec<Coord> = ring
                .iter()
                .filter_map(|id| parsed.nodes.get(id).copied())
                .collect();
            (coords.len() >= 3).then_some(coords)
        })
        .collect();

    // 道路名称标注（同名道路按约 120m 去重，避免满屏重复）
    let mut road_names_seen: Vec<(String, Coord)> = Vec::new();
    for r in &parsed.roads {
        let Some(name) = &r.name else { continue };
        let mid_ref = r.node_refs.get(r.node_refs.len() / 2);
        let Some(&mid_coord) = mid_ref.and_then(|id| parsed.nodes.get(id)) else {
            continue;
        };
        let duplicate = road_names_seen.iter().any(|(n, c)| {
            n == name && geo_haversine(*c, mid_coord) < 150.0
        });
        if !duplicate {
            road_names_seen.push((name.clone(), mid_coord));
            labels.push(MapLabel {
                coord: mid_coord,
                name: name.clone(),
                kind: "road".into(),
            });
        }
    }

    for (nid, name) in &parsed.poi_labels {
        if let Some(&coord) = parsed.nodes.get(nid) {
            if !labels.iter().any(|l| l.name == *name) {
                labels.push(MapLabel {
                    coord,
                    name: name.clone(),
                    kind: "poi".into(),
                });
            }
        }
    }
    g.labels = labels;

    g.compute_anchor();
    g.rebuild_index();
    g
}

/// 完整流程：解析 OSM → 构建路网。
pub fn load_osm(bytes: &[u8]) -> Result<RoadGraph, String> {
    let parsed = parse_osm(bytes)?;
    if parsed.roads.is_empty() {
        return Err("OSM 中未找到可用的 highway 道路".into());
    }
    Ok(build_graph(&parsed))
}

/// 球面近似距离（米）。
fn geo_haversine(a: Coord, b: Coord) -> f64 {
    use geo::HaversineDistance;
    let pa = geo::Point::new(a.lon, a.lat);
    let pb = geo::Point::new(b.lon, b.lat);
    pa.haversine_distance(&pb)
}
