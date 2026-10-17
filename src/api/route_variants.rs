//! Random traversal of a preset's existing roads. No coordinate noise or new links.
use rand::Rng;
#[cfg(test)]
use rand::seq::SliceRandom;
#[cfg(test)]
use std::collections::HashMap;
type Point = (f64, f64);

/// Closed presets form a directed multigraph. Shuffle an Euler circuit so every
/// existing segment is retained exactly once, including intentional repeat laps.
/// Open routes retain their endpoints and only support reversal.
#[cfg(test)]
pub(super) fn random_route<R: Rng + ?Sized>(
    base: &[Point], current: &[Point], rng: &mut R,
) -> Option<Vec<Point>> {
    if base.len() < 2 || base.iter().any(|p| !p.0.is_finite() || !p.1.is_finite()) {
        return None;
    }
    if base.first() != base.last() {
        let mut out = base.to_vec();
        if current == base || rng.gen_bool(0.5) { out.reverse(); }
        return (out != current).then_some(out);
    }
    let mut lookup = HashMap::new();
    let mut nodes = Vec::new();
    let mut ids = Vec::new();
    for &p in base {
        let key = (if p.0 == 0.0 { 0 } else { p.0.to_bits() },
                   if p.1 == 0.0 { 0 } else { p.1.to_bits() });
        let id = *lookup.entry(key).or_insert_with(|| { nodes.push(p); nodes.len()-1 });
        ids.push(id);
    }
    let mut edges = vec![Vec::new(); nodes.len()];
    for pair in ids.windows(2) { edges[pair[0]].push(pair[1]); }
    for _ in 0..64 {
        let mut remaining = edges.clone();
        for outgoing in &mut remaining { outgoing.shuffle(rng); }
        let start = ids[rng.gen_range(0..ids.len()-1)];
        let mut stack = vec![start];
        let mut tour = Vec::with_capacity(base.len());
        while let Some(&v) = stack.last() {
            if let Some(next) = remaining[v].pop() { stack.push(next); }
            else { tour.push(nodes[stack.pop().unwrap()]); }
        }
        tour.reverse();
        if rng.gen_bool(0.5) { tour.reverse(); }
        if tour.len() == base.len() && tour != current { return Some(tour); }
    }
    None
}

#[cfg(test)]
mod tests {
    use super::*;
    use rand::{rngs::StdRng, SeedableRng};
    use std::collections::{BTreeMap, BTreeSet};
    fn roads(points: &[Point]) -> BTreeMap<((u64,u64),(u64,u64)), usize> {
        let mut result = BTreeMap::new();
        for w in points.windows(2) {
            let a = (w[0].0.to_bits(), w[0].1.to_bits());
            let b = (w[1].0.to_bits(), w[1].1.to_bits());
            *result.entry(if a < b { (a,b) } else { (b,a) }).or_default() += 1;
        }
        result
    }
    #[test]
    fn variants_preserve_every_road_and_repeat_without_shortcuts() {
        let a=(29.0,119.0); let b=(29.001,119.0); let c=(29.001,119.001);
        let d=(29.002,119.0); let e=(29.002,119.001);
        let base=vec![a,b,c,a,d,e,a,b,c,a];
        let expected=roads(&base); let mut previous=base.clone(); let mut distinct=BTreeSet::new();
        let mut rng=StdRng::seed_from_u64(17);
        for _ in 0..500 {
            let next=random_route(&base,&previous,&mut rng).unwrap();
            assert_ne!(next,previous); assert_eq!(next.first(),next.last());
            assert_eq!(roads(&next),expected);
            distinct.insert(next.iter().map(|p|(p.0.to_bits(),p.1.to_bits())).collect::<Vec<_>>());
            previous=next;
        }
        assert!(distinct.len()>10);
    }
    #[test]
    fn invalid_and_open_routes_do_not_create_links() {
        let mut rng=StdRng::seed_from_u64(1);
        assert!(random_route(&[],&[],&mut rng).is_none());
        assert!(random_route(&[(f64::NAN,1.0),(2.0,3.0)],&[],&mut rng).is_none());
        let base=vec![(1.0,2.0),(2.0,3.0),(3.0,4.0)];
        let next=random_route(&base,&base,&mut rng).unwrap();
        assert_eq!(roads(&base),roads(&next)); assert_ne!(base,next);
        assert!(random_route(&[(1.0,1.0);3],&[(1.0,1.0);3],&mut rng).is_none());
    }
}

fn key(p: Point) -> (u64,u64) { (p.0.to_bits(),p.1.to_bits()) }
fn edge(a: Point,b: Point) -> ((u64,u64),(u64,u64)) {
    let a=key(a);let b=key(b);if a<b {(a,b)} else {(b,a)}
}
fn distance(a: Point,b: Point) -> f64 {
    ((a.0-b.0)*111195.0).hypot((a.1-b.1)*111195.0*((a.0+b.0)*0.5).to_radians().cos())
}
fn inventory(p: &[Point]) -> std::collections::BTreeMap<((u64,u64),(u64,u64)),usize> {
    let mut out=std::collections::BTreeMap::new();
    for w in p.windows(2) { *out.entry(edge(w[0],w[1])).or_default()+=1; }out
}
// Compare cyclic A-B-A turns, including the closing seam. Existing necessary
// reversals are retained, but replacements must not add new ones at junctions.
fn reversals(points: &[Point]) -> std::collections::BTreeMap<((u64,u64),(u64,u64)),usize> {
    let mut out=std::collections::BTreeMap::new();
    if points.len()<4 || points.first()!=points.last() {return out;}
    let n=points.len()-1;
    for i in 0..n {
        let a=points[(i+n-1)%n];let b=points[i];let c=points[(i+1)%n];
        if a==c && a!=b {*out.entry((key(a),key(b))).or_default()+=1;}
    }
    out
}

fn rotate_closed<R: Rng + ?Sized>(points: &[Point], rng: &mut R) -> Vec<Point> {
    let n=points.len()-1;let start=rng.gen_range(0..n);
    let mut out:Vec<_>=(0..n).map(|i|points[(i+start)%n]).collect();
    out.push(out[0]);if rng.gen_bool(0.5){out.reverse();}out
}

fn geometry(points: &[Point], mode: &str, checkpoints: &[Point]) -> (f64,bool) {
    use crate::track::geom::{make_ordered_path,meters_per_degree_lat,meters_per_degree_lng};
    let (plane,arcs,center)=make_ordered_path(points,route_planner::CustomCloseMode::from_str(mode));
    let sx=meters_per_degree_lng(center.0);let sy=meters_per_degree_lat(center.0);
    let covered=checkpoints.iter().all(|p| {
        let p=((p.1-center.1)*sx,(p.0-center.0)*sy);
        plane.windows(2).any(|w| {
            let a=(w[0].0-p.0,w[0].1-p.1);let d=(w[1].0-w[0].0,w[1].1-w[0].1);
            let n=d.0*d.0+d.1*d.1;let t=if n>0.0 { (-(a.0*d.0+a.1*d.1)/n).clamp(0.0,1.0) }else{0.0};
            (a.0+t*d.0).hypot(a.1+t*d.1)<=20.0
        })
    });
    (*arcs.last().unwrap_or(&0.0),covered)
}

/// Replace only approved main-road subpaths with connected alternatives on the
/// same approved graph. Track laps and field-confirmed access passages are locked.
pub(super) fn vary_main_roads<R: Rng + ?Sized>(
    base: &[Point], current: &[Point], main_edges: &[usize], mode: &str,
    checkpoints: &[Point], rng: &mut R,
) -> Result<Vec<Point>,String> {
    use std::collections::{BTreeMap,BTreeSet};
    if base.len()<3 || base.first()!=base.last() || base.iter().any(|p|!p.0.is_finite()||!p.1.is_finite()) {
        return Err("此功能需要有效的闭环基础方案".into());
    }
    if main_edges.is_empty() || main_edges.iter().any(|i|*i+1>=base.len()) {
        return Err("此方案尚未配置主干道路段，请先确认主干道白名单".into());
    }
    let (base_len,covered)=geometry(base,mode,checkpoints);
    if !covered { return Err("基础方案尚未覆盖当前全部点位，已保留当前路线；需先补齐可走通道".into()); }
    let allowed:BTreeSet<_>=main_edges.iter().map(|i|edge(base[*i],base[*i+1])).collect();
    let mut nodes=Vec::new();let mut ids=BTreeMap::new();
    for i in main_edges { for p in [base[*i],base[*i+1]] {
        ids.entry(key(p)).or_insert_with(||{nodes.push(p);nodes.len()-1});
    }}
    let mut graph=vec![Vec::new();nodes.len()];
    for (a,b) in &allowed {
        let u=ids[a];let v=ids[b];if u!=v {graph[u].push(v);graph[v].push(u);}
    }
    let before=inventory(base);let current_inv=inventory(current);
    let base_reversals=reversals(base);
    let lo=(base_len*0.84).max(4100.0);let hi=(base_len*1.16).min(5300.0);
    for _ in 0..20000 {
        let i=rng.gen_range(0..base.len()-2);
        let mut end=i;
        while end+1<base.len() && allowed.contains(&edge(base[end],base[end+1])) { end+=1; }
        if end<i+2 {continue;}
        let j=rng.gen_range(i+2..=end);
        if base[i]==base[j] {continue;}
        let start=ids[&key(base[i])];let target=ids[&key(base[j])];
        // One stable random cost per undirected edge keeps shortest paths simple.
        let costs:BTreeMap<_,_>=allowed.iter().map(|e|(*e,rng.gen_range(0.55..1.8))).collect();
        let shortest = |source: usize, target: usize, penalty: &std::collections::BTreeSet<_>| -> Option<Vec<usize>> {
            let mut dist=vec![f64::INFINITY;nodes.len()];let mut prev=vec![usize::MAX;nodes.len()];let mut done=vec![false;nodes.len()];dist[source]=0.0;
            for _ in 0..nodes.len() {
                let u=(0..nodes.len()).filter(|u|!done[*u]).min_by(|a,b|dist[*a].total_cmp(&dist[*b]))?;
                if !dist[u].is_finite() || u==target {break;}done[u]=true;
                for &v in &graph[u] {
                    let e=edge(nodes[u],nodes[v]);
                    let d=dist[u]+distance(nodes[u],nodes[v])*costs[&e]*if penalty.contains(&e){4.0}else{1.0};
                    if d<dist[v] {dist[v]=d;prev[v]=u;}
                }
            }
            if !dist[target].is_finite(){return None;}
            let mut path=vec![target];let mut u=target;
            while u!=source {u=prev[u];path.push(u);}path.reverse();Some(path)
        };
        let mut route=match shortest(start,target,&BTreeSet::new()) {Some(p)=>p,None=>continue};
        if rng.gen_bool(0.6) {
            let via=rng.gen_range(0..nodes.len());
            let Some(mut first)=shortest(start,via,&BTreeSet::new()) else{continue;};
            let penalty=first.windows(2).map(|w|edge(nodes[w[0]],nodes[w[1]])).collect();
            let Some(second)=shortest(via,target,&penalty) else{continue;};
            first.extend_from_slice(&second[1..]);
            if first.windows(3).any(|w|w[0]==w[2]) {continue;}
            route=first;
        }
        let replacement:Vec<_>=route.into_iter().map(|u|nodes[u]).collect();
        let mut candidate=base[..i].to_vec();candidate.extend(replacement);candidate.extend_from_slice(&base[j+1..]);
        if reversals(&candidate).iter().any(|(turn,n)|*n>base_reversals.get(turn).copied().unwrap_or(0)){continue;}
        let inv=inventory(&candidate);
        if inv==before || inv==current_inv {continue;}
        let delta:f64=before.keys().chain(inv.keys()).collect::<BTreeSet<_>>().iter().map(|e|{
            let n=before.get(*e).copied().unwrap_or(0).abs_diff(inv.get(*e).copied().unwrap_or(0));
            let a=(f64::from_bits(e.0.0),f64::from_bits(e.0.1));let b=(f64::from_bits(e.1.0),f64::from_bits(e.1.1));distance(a,b)*n as f64
        }).sum();
        if delta<80.0 || delta>base_len*0.25 {continue;}
        // Keep the overall outline: at least 82% of unique road length remains.
        let unique_length: f64=base.windows(2).filter(|w| before.contains_key(&edge(w[0],w[1])))
            .map(|w|distance(w[0],w[1]) / before[&edge(w[0],w[1])] as f64).sum();
        let omitted: f64=base.windows(2).filter(|w|!inv.contains_key(&edge(w[0],w[1])))
            .map(|w|distance(w[0],w[1]) / before[&edge(w[0],w[1])] as f64).sum();
        if omitted>unique_length*0.18 {continue;}
        let rough:f64=candidate.windows(2).map(|w|distance(w[0],w[1])).sum();
        if rough<lo || rough>hi+100.0 {continue;}
        // Preserve the connected traversal order. Validate the exact final
        // geometry after selecting its start/direction, including its seam.
        let candidate=rotate_closed(&candidate,rng);
        let (length,covered)=geometry(&candidate,mode,checkpoints);
        if !covered || length<lo || length>hi {continue;}
        return Ok(candidate);
    }
    Err("暂无同时满足主干道、4公里以上及全部点位要求的不同路线，已保留当前路线；可再次尝试".into())
}

#[cfg(test)]
mod mainroad_tests {
    use super::*;
    use rand::{rngs::StdRng,SeedableRng};
    #[test]
    fn rotation_keeps_turns_and_every_cyclic_transition() {
        let points=vec![(1.0,2.0),(2.0,3.0),(1.0,2.0),(3.0,4.0),(4.0,5.0),(1.0,2.0)];
        let mut rng=StdRng::seed_from_u64(41);
        assert_eq!(reversals(&points).values().sum::<usize>(),1);
        for _ in 0..100 {
            let next=rotate_closed(&points,&mut rng);
            assert_eq!(inventory(&next),inventory(&points));
            assert_eq!(reversals(&next),reversals(&points));
        }
    }
    #[test]
    fn mainroad_changes_preserve_locked_access_and_reject_impossible_points() {
        // A square road with a diagonal and a repeated triangle, plus a locked spur.
        let a=(29.0,119.0);let b=(29.0,119.010);let c=(29.010,119.010);
        let d=(29.010,119.0);let gate=(29.0,118.999);
        let e=(29.001,119.010);let f=(29.001,119.011);
        let base=vec![gate,a,b,e,f,b,c,d,a,gate];
        let approved=vec![1,2,3,4,5,6,7];let mut rng=StdRng::seed_from_u64(22);
        let variant=vary_main_roads(&base,&base,&approved,"rounded",&[b,d,gate],&mut rng).unwrap();
        let orig=inventory(&base);let next=inventory(&variant);
        assert_ne!(orig,next);
        assert_eq!(orig[&edge(gate,a)],next[&edge(gate,a)]);
        let allowed:std::collections::BTreeSet<_>=base.windows(2).map(|w|edge(w[0],w[1])).collect();
        assert!(variant.windows(2).all(|w|allowed.contains(&edge(w[0],w[1]))));
        let (length,covered)=geometry(&variant,"rounded",&[b,d,gate]);
        assert!(covered && (4100.0..=5300.0).contains(&length));
        assert!(vary_main_roads(&base,&base,&[],"rounded",&[],&mut rng).is_err());
        assert!(vary_main_roads(&base,&base,&[99],"rounded",&[],&mut rng).is_err());
        assert!(vary_main_roads(&base,&base,&approved,"rounded",&[(30.0,120.0)],&mut rng).is_err());
    }
    #[test]
    #[ignore = "local route and checkpoint audit; requires explicit file paths"]
    fn audit_local_mainroad_variants() {
        use std::path::Path;
        let config=std::env::var("NEKO_ROUTE_CHECK_CONFIG").unwrap();
        let points=std::env::var("NEKO_ROUTE_CHECK_POINTS").unwrap();
        let output=std::env::var("NEKO_ROUTE_CHECK_OUTPUT").unwrap();
        let config:crate::api::model::Config=serde_json::from_str(&std::fs::read_to_string(config).unwrap()).unwrap();
        let cache:serde_json::Value=serde_json::from_str(&std::fs::read_to_string(points).unwrap()).unwrap();
        let checkpoints:Vec<_>=cache["points"].as_array().unwrap().iter().map(|p|(p["lat"].as_f64().unwrap(),p["lon"].as_f64().unwrap())).collect();
        let mut report=Vec::new();
        for preset in &config.custom_route_presets {
            let before=inventory(&preset.waypoints);
            let allowed:std::collections::BTreeSet<_>=preset.random_main_road_edges.iter().map(|i|edge(preset.waypoints[*i],preset.waypoints[*i+1])).collect();
            let mut current=preset.waypoints.clone();
            for seed in 0..30 {
                let mut rng=StdRng::seed_from_u64(seed);
                let next=vary_main_roads(&preset.waypoints,&current,&preset.random_main_road_edges,&preset.close_mode,&checkpoints,&mut rng).unwrap_or_else(|e|panic!("{} seed {}: {}",preset.id,seed,e));
                let after=inventory(&next);assert_ne!(before,after);assert_ne!(inventory(&current),after);
                for (e,n) in &before {if !allowed.contains(e) {assert_eq!(after.get(e),Some(n));}}
                for e in after.keys() {assert!(before.contains_key(e));}
                let (length,covered)=geometry(&next,&preset.close_mode,&checkpoints);assert!(covered);
                let turns=reversals(&preset.waypoints);
                assert!(reversals(&next).iter().all(|(k,n)|*n<=turns.get(k).copied().unwrap_or(0)));
                let (base_len,_)=geometry(&preset.waypoints,&preset.close_mode,&checkpoints);
                assert!(length>=(base_len*0.84).max(4100.0) && length<=(base_len*1.16).min(5300.0));
                report.push(serde_json::json!({"id":preset.id,"seed":seed,"length_m":length,"waypoints":next}));current=next;
            }
        }
        std::fs::write(Path::new(&output),serde_json::to_string_pretty(&report).unwrap()).unwrap();
    }
}
