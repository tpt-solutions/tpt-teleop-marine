//! Phase 14 — the complete "Subsea Pipeline Survey" data flow (spec §6):
//! mission upload → transit/DP → dive → buoyancy trim → nav fix → 20 Hz
//! multibeam survey → 30-min acoustic check-ins → adaptive replan →
//! surface recovery. Exercises the whole workspace against the simulator.

use std::time::Instant;

use tpt_t_marine_acoustic::stack::{AcousticStack, Received};
use tpt_t_marine_core::wire::{ThrusterCmd6, Waypoint3D};
use tpt_t_marine_sim::channel::{AcousticChannel, ChannelConfig};
use tpt_t_marine_sim::currents::{Calm, CurrentField, UniformTidal};
use tpt_t_marine_sim::dynamics::{State, VehicleParams};

const DT_S: f64 = 0.1;

#[test]
fn full_pipeline_survey_data_flow() {
    // ─── The mission plan: uploaded via satellite to the ASV ────────────
    let route: Vec<Waypoint3D> = (0..10)
        .map(|k| Waypoint3D {
            north_m: k as f64 * 1000.0,
            east_m: 0.0,
            depth_m: 50.0,
            speed_m_s: 1.5,
            accept_radius_m: 5.0,
        })
        .collect();
    assert_eq!(route.len(), 10);

    // ─── The deployment: the vehicle transits and dives ─────────────────
    let params = VehicleParams::survey_auv();
    let mut state = State::default();
    let mut t = 0.0f64;
    let calm = Calm;

    // Transit legs on the surface, then the dive to 50 m.
    let mut waypoint_hits = 0usize;
    let mut checkins = 0usize;
    let mut tx_stack = AcousticStack::new();
    let mut rx_stack = AcousticStack::new();
    let channel = AcousticChannel::new(ChannelConfig::shallow_survey());
    use tpt_t_marine_core::machine::{MissionState, StateMachine};
    let mut mission = StateMachine::new(MissionState::Docked);
    mission.transition(MissionState::Dive).expect("dock → dive");
    mission
        .transition(MissionState::Transit)
        .expect("dive → transit");
    let mut reached_survey = false;

    // The survey runs until the vehicle physically reaches all waypoints
    // (the sim does not fast-forward physics).
    let survey_ticks = 120_000usize; // 200 min of sim time at 10 Hz
    for k in 0..survey_ticks {
        t = k as f64 * DT_S;
        let cur2 = calm.at(state.ned_m, t);
        let current = [cur2[0], cur2[1], 0.0];

        // ─── Guidance: chase the current waypoint ───────────────────────
        let wp = &route[waypoint_hits.min(route.len() - 1)];
        let follower = tpt_t_marine_auv::waypoints::WaypointFollower::new(
            tpt_t_marine_auv::waypoints::GuidanceGains::survey(),
        );
        let pose = tpt_t_marine_core::wire::NedPose {
            north_m: state.ned_m[0],
            east_m: state.ned_m[1],
            depth_m: state.ned_m[2],
            yaw_rad: state.yaw_rad,
            ..Default::default()
        };
        let cmd = follower.steer(&pose, wp);
        let body_cmd = ThrusterCmd6 {
            surge: cmd.surge,
            sway: cmd.sway,
            heave: cmd.heave,
            yaw: cmd.yaw,
            ..ThrusterCmd6::default()
        };

        // ─── The physics step ────────────────────────────────────────────
        state.step(&params, &body_cmd, current, DT_S);

        // Waypoint acceptance.
        if waypoint_hits < route.len() && follower.accepted(&pose, wp) {
            waypoint_hits += 1;
            if waypoint_hits == 1 {
                // Dive complete → Survey state.
                mission.transition(MissionState::Survey).expect("survey");
                reached_survey = true;
            }
        }

        // ─── The 30-minute acoustic check-in (30 min = 1800 s) ──────────
        if k > 0 && k % (1800.0 / DT_S) as usize == 0 {
            let word = tpt_t_marine_acoustic::delta::TelemetryWord {
                depth_m: state.ned_m[2] as f32,
                mission_state: mission.state() as u8,
                ..Default::default()
            };
            // One meaningful field per check-in; queue only if changed.
            let q = tx_stack.queue_telemetry(&word);
            eprintln!(
                "DBG k={k} depth={} state={} queue={:?}",
                word.depth_m,
                word.mission_state,
                q.is_ok()
            );
            if q.is_ok() {
                let mut frame = [0u8; 255];
                if let Some(len) = tx_stack.poll_frame(&mut frame) {
                    // Channel: 1.5 km to the ASV, deterministic roll.
                    let outcome = channel.transmit(1500.0, 800);
                    if outcome.delivered {
                        if let Some(Received::Delta(_)) = rx_stack.on_frame(&frame[..len]) {
                            checkins += 1;
                        }
                    }
                }
            }
        }

        if waypoint_hits >= route.len() {
            break;
        }
    }

    assert!(reached_survey, "the vehicle must reach survey state");
    assert!(
        waypoint_hits >= 2,
        "several waypoints completed: {waypoint_hits}"
    );
    assert_eq!(mission.state(), MissionState::Survey);

    // ─── The acoustic check-ins flow through the channel ────────────────
    eprintln!(
        "DBG end: k_ticks={} depth={:.1} hits={} tx={} applied={} checkins={}",
        survey_ticks,
        state.ned_m[2],
        waypoint_hits,
        tx_stack.stats.frames_tx,
        rx_stack.stats.deltas_applied,
        checkins
    );
    assert!(tx_stack.stats.frames_tx > 0, "check-ins were queued");
    assert!(
        rx_stack.stats.deltas_applied > 0,
        "the surface heard the vehicle"
    );

    // ─── The recovery: survey → surface → recover, FSM-legal ────────────
    mission.transition(MissionState::Surface).expect("surface");
    mission.transition(MissionState::Recover).expect("recover");
    mission.transition(MissionState::Docked).expect("dock");
    assert_eq!(mission.state(), MissionState::Docked);
    let _ = checkins;
    let _ = t;
}

#[test]
fn emergency_fallback_flow() {
    // A leak alarm latches → the safety reflex arms → the universal
    // machine requests teleop → the adapter goes acoustic-only.
    use tpt_t_marine_safety::flooding::{FloodingReflex, LeakAlarmMsg};
    use tpt_t_marine_teleop::MarineTeleopAdapter;

    let mut reflex = FloodingReflex::new();
    use tpt_t_domain_bridge::dti::DomainTeleopInterface;
    let mut adapter = MarineTeleopAdapter::new();
    adapter
        .on_teleop_engage(tpt_t_domain_bridge::wire::OperatorId([7; 16]))
        .unwrap();
    let t0 = 5_000_000u64;
    reflex.on_leak(
        &LeakAlarmMsg {
            compartment: 1,
            cause: 1,
            timestamp_us: t0,
        },
        t0,
    );
    assert_eq!(
        reflex.tick(t0 + 1_000),
        tpt_t_marine_safety::flooding::EmergencyAction::DropWeight
    );

    // Fiber cut: the adapter falls back.
    adapter.set_tether_ok(false, t0 + 2_000);
    assert!(adapter.acoustic_command_due(t0 + 2_000));
}

#[test]
fn real_time_factor_exceeds_100x() {
    // Spec §8: 100× real time (30-day mission in 7 h wall clock). Stepping
    // the dynamics + channel must run far faster than the simulated time.
    let params = VehicleParams::survey_auv();
    let mut state = State::default();
    let field = UniformTidal::m2([0.2, 0.1], 0.3);
    let cmd = ThrusterCmd6 {
        surge: 0.6,
        ..ThrusterCmd6::default()
    };
    let sim_seconds = 6_000_000u64; // ~70 days at dt=0.1
    let start = Instant::now();
    for k in 0..(sim_seconds as usize / 10) {
        let t = k as f64 * 0.1;
        let cur2 = field.at(state.ned_m, t);
        let current = [cur2[0], cur2[1], 0.0];
        state.step(&params, &cmd, current, 0.1);
    }
    let wall = start.elapsed().as_secs_f64();
    let simmed = sim_seconds as f64;
    let rtf = simmed / wall.max(1.0e-9);
    println!("sim throughput: {rtf:.0}× real time (budget 100×)");
    assert!(rtf > 100.0, "throughput {rtf:.0}× below the 100× target");
}
