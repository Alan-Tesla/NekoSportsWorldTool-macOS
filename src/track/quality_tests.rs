use super::{generator, geom, model::Track};

fn assert_conservation(t: &Track) {
    t.validate_consistency().expect("cumulative samples and summary must agree");
    let window_distance: f64 = t.speedPerTenSec.iter().map(|w| w.value).sum();
    let window_steps: f64 = t.stepsPerTenSec.iter().map(|w| w.value).sum();
    assert!((window_distance-t.totalDistance).abs()<1e-6);
    assert!((window_steps-t.totalSteps as f64).abs()<1e-6);
    assert_eq!(t.speedPerTenSec.iter().map(|w| w.time).sum::<i64>(),t.totalTime);
    assert_eq!(t.segments.iter().map(|s| s.totalTime).sum::<i64>(),t.totalTime);
    assert!((t.segments.iter().map(|s| s.distance).sum::<i64>() as f64-t.totalDistance).abs()<=0.51);
    assert!(t.segments.iter().all(|s| s.startTime>=0 && s.endTime-s.startTime==s.totalTime*1000));
    for p in t.locations.windows(2) {
        let dt=p[1].totalTime-p[0].totalTime;
        if dt>0 {
            let measured=(p[1].totalDis-p[0].totalDis)/dt as f64;
            assert!((measured-p[1].avgSpeed).abs()<1e-6);
            assert!((measured-p[1].bdS).abs()<1e-6);
            let pace=if measured>0.0 {1000.0/(60.0*measured)} else {0.0};
            assert!((pace-p[1].speed).abs()<1e-6);
        }
    }
}

#[test]
fn synthetic_route_quality_for_96_pace_and_seed_combinations() {
    let points: Vec<(f64,f64)>=serde_json::from_str(include_str!("fixtures/synthetic_loop.json")).unwrap();
    let (_, arcs, _)=geom::make_ordered_path(&points,route_planner::CustomCloseMode::Open);
    let distance=*arcs.last().unwrap();
    assert!((4200.0..4400.0).contains(&distance),"Synthetic route distance: {distance}");
    let mut min_cad=f64::INFINITY;
    let mut max_cad=0.0f64;
    let mut window_min=f64::INFINITY;
    let mut window_max=0.0f64;
    for seed in 0..32 {
        for pace in [360.0,390.0,420.0] {
            let duration=(distance/1000.0*pace).round() as i64;
            let track=generator::build_ordered_mode(distance,duration,seed,1_800_000_000_000,&points,route_planner::CustomCloseMode::Open,&[]);
            assert_conservation(&track);
            assert_eq!(track.totalTime,duration,"postprocessing must not extend the run");
            assert!((track.totalDistance-distance).abs()<0.51);
            for window in track.stepsPerTenSec.iter().skip(2).filter(|w| w.time==10) {
                let cadence=window.value*6.0;
                window_min=window_min.min(cadence);window_max=window_max.max(cadence);
            }
            for pair in track.locations.windows(2) {
                let dt=pair[1].totalTime-pair[0].totalTime;
                if pair[0].totalTime>=20 && dt>0 {
                    let cad=(pair[1].steps-pair[0].steps) as f64*60.0/dt as f64;
                    min_cad=min_cad.min(cad);max_cad=max_cad.max(cad);
                    // Integer step counts allow up to 1 step of rounding per interval.
                    assert!((100.0..=240.0).contains(&cad),"unexpected injected cadence {cad}");
                }
            }
        }
    }
    println!("Synthetic route audit: 96 local simulations, distance {distance:.2} m, moving interval cadence {min_cad:.1}..{max_cad:.1} spm; 10-second cadence {window_min:.1}..{window_max:.1} spm; no network submissions.");
}

#[test]
fn short_tail_window_is_not_inflated_to_ten_seconds() {
    let pts=vec![(0.0,0.0),(0.002,0.0),(0.002,0.003),(0.0,0.003)];
    let track=generator::build(1000.0,391,42,(0.0,0.0),1_800_000_000_000,&pts);
    assert_conservation(&track);
    assert_eq!(track.speedPerTenSec.last().unwrap().time,1);
    assert!(track.speedPerTenSec.last().unwrap().value<5.0);
}

#[test]
fn longitude_scale_follows_latitude() {
    // WGS84 east-west length of 0.001 degree at 29° is about 97.439 m.
    let east=geom::meters_per_degree_lng(29.0)*0.001;
    assert!((east-97.439).abs()<0.1,"distance {east}");
    assert!(geom::meters_per_degree_lng(29.0)>geom::meters_per_degree_lng(39.0));
}

#[test]
fn validation_rejects_nonfinite_distance_and_nonmonotonic_time() {
    let pts=vec![(0.0,0.0),(0.002,0.0),(0.002,0.003)];
    let mut track=generator::build(1000.0,390,42,(0.0,0.0),1_800_000_000_000,&pts);
    track.totalDistance=f64::NAN;
    assert!(track.validate_consistency().is_err());
    track.rebuild_statistics();
    track.locations[3].totalTime=-1;
    assert!(track.validate_consistency().is_err());
}

#[test]
fn pace_smoothness_and_late_fatigue_for_synthetic_route() {
    let pts: Vec<(f64,f64)>=serde_json::from_str(include_str!("fixtures/synthetic_long_loop.json")).unwrap();
    let (_,arcs,_)=geom::make_ordered_path(&pts,route_planner::CustomCloseMode::Open);
    let distance=*arcs.last().unwrap();
    let mut deltas=Vec::new();
    let mut comparisons=Vec::new();
    for seed in 0..32 {
        for pace in [360.0,390.0,420.0] {
            let duration=(distance/1000.0*pace).round() as i64;
            let t=generator::build_ordered_mode(distance,duration,seed,1_800_000_000_000,&pts,route_planner::CustomCloseMode::Open,&[]);
            assert_conservation(&t);
            let windows: Vec<(f64,f64)>=t.speedPerTenSec.iter().enumerate()
                .filter(|(i,w)| *i>=6 && w.time==10 && w.value>0.0)
                .map(|(i,w)|(i as f64*10.0,1000.0*w.time as f64/w.value)).collect();
            let mean=|a:f64,b:f64| {
                let vals:Vec<_>=windows.iter().filter(|(s,_)| *s>=duration as f64*a && *s<duration as f64*b).map(|(_,p)| *p).collect();
                vals.iter().sum::<f64>()/vals.len() as f64
            };
            let early=mean(0.10,0.40);let late=mean(0.75,0.95);
            comparisons.push(serde_json::json!({"seed":seed,"target_sec_per_km":pace,"early_sec_per_km":early,"late_sec_per_km":late}));
            for pair in windows.windows(2) { deltas.push((pair[1].1-pair[0].1).abs()); }
            if seed==0 && pace==390.0 {
                println!("PACE_EXAMPLE={}",serde_json::to_string(&windows).unwrap());
            }
        }
    }
    deltas.sort_by(|a,b|a.total_cmp(b));
    let p95=deltas[(deltas.len() as f64*0.95) as usize];
    let max=deltas.last().unwrap();
    println!("PACE_METRICS={}",serde_json::json!({"route_m":distance,"simulations":96,"adjacent_10s_pace_delta_p95":p95,"adjacent_10s_pace_delta_max":max,"runs":comparisons}));
    assert!(p95 < 2.5, "10-second pace variation is too large: {p95}");
    assert!(*max < 5.0, "abrupt pace change: {max}");
    for run in &comparisons {
        let early=run["early_sec_per_km"].as_f64().unwrap();
        let late=run["late_sec_per_km"].as_f64().unwrap();
        assert!(late > early*1.025 && late < early*1.10,"fatigue should be gradual: {run}");
    }
}

#[test]
fn low_pass_reduces_short_interval_direction_reversals() {
    let mut latent=0.0;
    let mut smoothed=0.0;
    let mut raw=Vec::new();
    let mut filtered=Vec::new();
    for i in 0..400 {
        let z=if i%2==0 { 1.0 } else { -1.0 };
        route_planner::kinematics::slow_pace_noise(&mut latent,&mut smoothed,5.0,z,1.0/75.0,0.0046);
        raw.push(latent);filtered.push(smoothed);
    }
    let variation=|v:&[f64]|v.windows(2).map(|p|(p[1]-p[0]).abs()).sum::<f64>();
    assert!(variation(&filtered)<variation(&raw)*0.20);
}

#[test]
fn road_polyline_keeps_corners_without_extra_control_points() {
    let points=vec![(0.0,0.0),(0.0,0.002),(0.002,0.002)];
    let (dense,arcs,(lat,lon))=geom::make_ordered_path(&points,route_planner::CustomCloseMode::OpenPolyline);
    let corner=(0.002*geom::meters_per_degree_lng(lat)-lon*geom::meters_per_degree_lng(lat),-lat*geom::meters_per_degree_lat(lat));
    assert!(dense.iter().any(|p|(p.0-corner.0).abs()<1e-6 && (p.1-corner.1).abs()<1e-6));
    assert!(dense.windows(2).all(|p|(p[1].0-p[0].0).hypot(p[1].1-p[0].1)<=4.001));
    assert!(arcs.windows(2).all(|p|p[1]>=p[0]));
    for (x,y) in dense { assert!((y-corner.1).abs()<1e-6 || (x-corner.0).abs()<1e-6); }
    assert_eq!(route_planner::CustomCloseMode::from_str("polyline").as_str(),"polyline");
    assert!(!route_planner::CustomCloseMode::OpenPolyline.is_closed());
}

#[test]
fn rounded_road_has_curved_corners_without_wide_cutting() {
    let points=vec![(0.0,0.0),(0.0,0.002),(0.002,0.002)];
    let (path,arcs,(lat,lon))=geom::make_ordered_path(&points,route_planner::CustomCloseMode::OpenRounded);
    let corner=((0.002-lon)*geom::meters_per_degree_lng(lat),-lat*geom::meters_per_degree_lat(lat));
    assert!(path.iter().any(|p|p.0<corner.0-0.1 && p.1>corner.1+0.1));
    assert!(path.iter().all(|p|(p.0-corner.0).abs().min((p.1-corner.1).abs())<=1.501));
    assert!(arcs.windows(2).all(|p|p[1]>=p[0]));
    assert!(path.windows(2).all(|p|(p[1].0-p[0].0).hypot(p[1].1-p[0].1)<=4.001));
    for p in path.windows(3) {
        let a=(p[1].0-p[0].0,p[1].1-p[0].1);let b=(p[2].0-p[1].0,p[2].1-p[1].1);
        let n=a.0.hypot(a.1)*b.0.hypot(b.1);
        if n>1e-9 { assert!((a.0*b.0+a.1*b.1)/n>0.97); }
    }
    assert_eq!(route_planner::CustomCloseMode::from_str("rounded").as_str(),"rounded");
}

#[test]
fn rounded_road_handles_duplicate_points_and_retains_endpoints() {
    let points=vec![(0.0,0.0),(0.0,0.0),(0.0,0.001),(0.0,0.001),(0.0,0.0)];
    let (path,arcs,_)=geom::make_ordered_path(&points,route_planner::CustomCloseMode::OpenRounded);
    assert_eq!(path.first(),path.last());
    assert!(path.iter().all(|p|p.0.is_finite() && p.1.is_finite()));
    assert!(arcs.iter().all(|x|x.is_finite()));
    assert!(path.iter().all(|p|p.1.abs()<1e-8));
}

/// Optional local geometry diagnostic. Exports only route coordinates, not config or credentials.
#[test]
#[ignore]
fn export_local_rounded_previews() {
    let input=std::env::var("NEKO_ROUTE_CHECK_CONFIG").expect("set config path");
    let output=std::env::var("NEKO_ROUTE_CHECK_OUTPUT").expect("set geometry output path");
    let value:serde_json::Value=serde_json::from_slice(&std::fs::read(input).unwrap()).unwrap();
    let mut routes=Vec::new();
    for preset in value["custom_route_presets"].as_array().unwrap() {
        let points:Vec<(f64,f64)>=serde_json::from_value(preset["waypoints"].clone()).unwrap();
        let (path,arcs,(lat,lon))=geom::make_ordered_path(&points,route_planner::CustomCloseMode::OpenRounded);
        let geometry:Vec<_>=path.iter().map(|&(x,y)|geom::to_bd(x,y,lat,lon)).collect();
        routes.push(serde_json::json!({"id":preset["id"],"length_m":arcs.last(),"route_bd09":geometry}));
    }
    std::fs::write(output,serde_json::to_vec_pretty(&routes).unwrap()).unwrap();
}

#[test]
fn startup_marker_preserves_time_and_chart_uses_pace() {
    let pts=vec![(0.0,0.0),(0.002,0.0),(0.002,0.003),(0.0,0.003)];
    for seed in 0..32 {
        let track=generator::build(1000.0,390,seed,(0.0,0.0),1_800_000_000_000,&pts);
        let marker=&track.locations[1];
        assert!(marker.totalTime>0);
        assert_eq!(marker.totalDis,0.0);
        let first=&track.locations[2];
        let dt=(first.totalTime-marker.totalTime) as f64;
        let expected_pace=dt*1000.0/(60.0*first.totalDis);
        assert!((first.speed-expected_pace).abs()<1e-6);
        assert!(first.speed>4.0 && first.speed<10.0,"startup pace {}",first.speed);
        assert_conservation(&track);
    }
}
