//! 6DOF thruster coordination: mix a normalized [`ThrusterCmd6`] into
//! per-thruster outputs for a standard 8-thruster ROV (4 horizontal in an
//! X pattern, 4 vertical).

use tpt_t_marine_core::wire::ThrusterCmd6;

/// Number of thrusters.
pub const N_THRUSTERS: usize = 8;

/// Thruster identifiers.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(usize)]
pub enum Thruster {
    /// Horizontal, fore-starboard, +45°.
    Hfs = 0,
    /// Horizontal, fore-port, −45°.
    Hfp = 1,
    /// Horizontal, aft-starboard, −45°.
    Has = 2,
    /// Horizontal, aft-port, +45°.
    Hap = 3,
    /// Vertical, fore-starboard.
    Vfs = 4,
    /// Vertical, fore-port.
    Vfp = 5,
    /// Vertical, aft-starboard.
    Vas = 6,
    /// Vertical, aft-port.
    Vap = 7,
}

/// The 8×6 mixing matrix (rows = thrusters, columns = surge, sway, heave,
/// roll, pitch, yaw). X-configuration horizontals at ±45° contribute
/// √½/2 to surge/sway; verticals contribute heave; lever arms give
/// roll/pitch/yaw authority.
pub const MIX: [[f32; 6]; N_THRUSTERS] = [
    //           surge   sway   heave  roll   pitch  yaw
    /* Hfs */
    [0.354, 0.354, 0.0, 0.0, 0.0, 0.354],
    /* Hfp */ [0.354, -0.354, 0.0, 0.0, 0.0, -0.354],
    /* Has */ [0.354, -0.354, 0.0, 0.0, 0.0, 0.354],
    /* Hap */ [0.354, 0.354, 0.0, 0.0, 0.0, -0.354],
    /* Vfs */ [0.0, 0.0, 0.25, 0.25, 0.25, 0.0],
    /* Vfp */ [0.0, 0.0, 0.25, -0.25, 0.25, 0.0],
    /* Vas */ [0.0, 0.0, 0.25, 0.25, -0.25, 0.0],
    /* Vap */ [0.0, 0.0, 0.25, -0.25, -0.25, 0.0],
];

/// The mixer output: per-thruster normalized values `[-1, 1]`.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ThrusterOutputs {
    /// Per-thruster outputs indexed by [`Thruster`].
    pub out: [f32; N_THRUSTERS],
}

/// Mix a body command. Saturates gracefully: if any thruster exceeds 1,
/// the whole set scales down proportionally (preserving the direction).
pub fn mix(cmd: &ThrusterCmd6) -> ThrusterOutputs {
    let axes = [cmd.surge, cmd.sway, cmd.heave, cmd.roll, cmd.pitch, cmd.yaw];
    let mut out = [0f32; N_THRUSTERS];
    let mut max = 0f32;
    for (t, row) in MIX.iter().enumerate() {
        let mut v = 0f32;
        for (a, &axis) in axes.iter().enumerate() {
            v += row[a] * axis;
        }
        out[t] = v;
        max = max.max(v.abs());
    }
    if max > 1.0 {
        for v in &mut out {
            *v /= max;
        }
    }
    ThrusterOutputs { out }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn pure_surge_drives_all_horizontals_forward() {
        let o = mix(&ThrusterCmd6 {
            surge: 1.0,
            ..ThrusterCmd6::default()
        });
        for t in [Thruster::Hfs, Thruster::Hfp, Thruster::Has, Thruster::Hap] {
            assert!(o.out[t as usize] > 0.3, "horizontal {t:?} forward");
        }
        for t in [Thruster::Vfs, Thruster::Vfp, Thruster::Vas, Thruster::Vap] {
            assert_eq!(o.out[t as usize], 0.0, "vertical {t:?} idle");
        }
    }

    #[test]
    fn pure_yaw_spins_closer_horizontals() {
        let o = mix(&ThrusterCmd6 {
            yaw: 1.0,
            ..ThrusterCmd6::default()
        });
        // Starboard-bow thruster reverses, port-bow drives (turn right).
        assert!(o.out[Thruster::Hfs as usize] > 0.0);
        assert!(o.out[Thruster::Hfp as usize] < 0.0);
    }

    #[test]
    fn saturation_scales_proportionally() {
        let extreme = ThrusterCmd6 {
            surge: 1.0,
            sway: 1.0,
            heave: 1.0,
            roll: 1.0,
            pitch: 1.0,
            yaw: 1.0,
        };
        let o = mix(&extreme);
        for v in o.out {
            assert!(v.abs() <= 1.0, "saturated output {v}");
        }
        // Direction preserved: doubling a small command doubles outputs.
        let small = ThrusterCmd6 {
            surge: 0.1,
            sway: 0.1,
            ..ThrusterCmd6::default()
        };
        let a = mix(&small);
        let b = mix(&ThrusterCmd6 {
            surge: 0.2,
            sway: 0.2,
            ..ThrusterCmd6::default()
        });
        assert!((b.out[0] - 2.0 * a.out[0]).abs() < 1e-6);
    }

    #[test]
    fn zero_command_is_all_zero() {
        let o = mix(&ThrusterCmd6::default());
        assert!(o.out.iter().all(|&v| v == 0.0));
    }
}
