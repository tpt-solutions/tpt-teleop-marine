//! Variable ballast tank control: hold a target net buoyancy at the current
//! depth with rate-limited pumps.
//!
//! The controller tracks tank fill (kg of water) against the neutral-ballast
//! demand computed from the [`physics`] model (or an operator setpoint), and
//! emits a pump command within the hardware's flow limits. Depth compensation
//! is feed-forward: as the vehicle descends and soft buoyancy compresses, the
//! demand shrinks and the controller dumps water *before* the vehicle starts
//! sinking, so depth keeping never fights a buoyancy transient.
//!
//! Payload changes (tool deployed, sample collected) shift the mass budget:
//! [`BallastController::apply_payload_change`] records the offset so the
//! demand moves by the same amount and the loop re-trims automatically —
//! this is the hook `tpt-t-marine-teleop` drives for ROV tool handling.

use crate::physics::VehicleBuoyancyModel;

/// Pump command: normalized pump rate `[-1.0, 1.0]` (negative = pump out,
/// positive = pump in), with diagnostics.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct PumpCmd {
    /// Normalized pump rate, `[-1.0, 1.0]`.
    pub rate: f64,
    /// Fill target this tick's command is chasing, kg.
    pub target_kg: f64,
    /// Current fill, kg.
    pub fill_kg: f64,
}

/// Ballast controller state (`Copy`, no allocation).
#[derive(Debug, Clone, Copy)]
pub struct BallastController {
    model: VehicleBuoyancyModel,
    /// Max tank capacity, kg of water.
    capacity_kg: f64,
    /// Max pump flow, kg/s at full rate.
    flow_kg_per_s: f64,
    /// Current fill.
    fill_kg: f64,
    /// Net payload offset, kg (positive = vehicle carries extra mass the
    /// ballast schedule must cancel by dumping water).
    payload_offset_kg: f64,
    /// Proportional gain, pump fraction per kg of error.
    kp: f64,
}

impl BallastController {
    /// Create a controller. `capacity_kg` bounds the tank; `flow_kg_per_s`
    /// is the pump's full-rate flow.
    pub fn new(model: VehicleBuoyancyModel, capacity_kg: f64, flow_kg_per_s: f64) -> Self {
        Self {
            model,
            capacity_kg,
            flow_kg_per_s,
            fill_kg: 0.0,
            payload_offset_kg: 0.0,
            kp: 0.05,
        }
    }

    /// Current water fill, kg.
    pub fn fill_kg(&self) -> f64 {
        self.fill_kg
    }

    /// The neutral-buoyancy demand at `depth_m` before payload offsets.
    pub fn neutral_demand_kg(&self, depth_m: f64) -> f64 {
        self.model.neutral_ballast_kg(depth_m)
    }

    /// Advance one control tick: integrate the previous command's effect,
    /// compute the demand (operator `target_kg` when provided, else the
    /// neutral schedule, both less the payload offset), and emit the next
    /// rate-limited pump command.
    pub fn update(&mut self, depth_m: f64, dt_s: f64, target_kg: Option<f64>) -> PumpCmd {
        let base = target_kg.unwrap_or_else(|| self.neutral_demand_kg(depth_m));
        let demand = base - self.payload_offset_kg;
        let error = demand - self.fill_kg;
        let fraction = (self.kp * error).clamp(-1.0, 1.0);
        let cmd = PumpCmd {
            rate: fraction,
            target_kg: demand,
            fill_kg: self.fill_kg,
        };
        // Integrate the commanded flow against capacity limits.
        self.fill_kg =
            (self.fill_kg + fraction * self.flow_kg_per_s * dt_s).clamp(0.0, self.capacity_kg);
        cmd
    }

    /// A payload change shifts the mass budget: `delta_kg` positive = mass
    /// taken aboard (vehicle heavier → demand drops by the same amount, the
    /// loop then dumps ballast to re-trim); negative = mass dropped.
    pub fn apply_payload_change(&mut self, delta_kg: f64) {
        self.payload_offset_kg += delta_kg;
    }

    /// Current payload offset, kg.
    pub fn payload_offset_kg(&self) -> f64 {
        self.payload_offset_kg
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::physics::VehicleBuoyancyModel;

    fn model() -> VehicleBuoyancyModel {
        VehicleBuoyancyModel {
            mass_kg: 500.0,
            volume_surface_m3: 0.55,
            soft_fraction: 0.4,
            soft_compress_per_m: 1.0e-4,
            air_volume_m3: 0.0,
            water_density_kg_m3: 1029.0,
        }
    }

    fn run_to_trim(c: &mut BallastController, depth: f64, ticks: usize) {
        for _ in 0..ticks {
            let cmd = c.update(depth, 0.1, None);
            if (cmd.fill_kg - cmd.target_kg).abs() < 0.05 {
                break;
            }
        }
    }

    #[test]
    fn converges_to_neutral_at_depth() {
        let mut c = BallastController::new(model(), 80.0, 2.0);
        let depth = 500.0;
        run_to_trim(&mut c, depth, 1500);
        let residual = c.model.net_force_n(depth, c.fill_kg());
        assert!(
            residual.abs() < 1.0,
            "must reach neutral, residual {residual} N"
        );
    }

    #[test]
    fn pump_respects_flow_limits() {
        let mut c = BallastController::new(model(), 80.0, 2.0);
        // Huge error: command saturates at full in-flow.
        let cmd = c.update(0.0, 0.1, Some(80.0));
        assert_eq!(cmd.rate, 1.0);
        assert!(
            (c.fill_kg() - 2.0 * 0.1).abs() < 1.0e-9,
            "fill integrates at exactly flow·dt when saturated"
        );
    }

    #[test]
    fn tank_capacity_is_bounded() {
        let mut c = BallastController::new(model(), 30.0, 2.0);
        for _ in 0..500 {
            c.update(0.0, 0.1, Some(100.0)); // impossible demand
        }
        assert_eq!(c.fill_kg(), 30.0, "tank cannot overfill");
        for _ in 0..500 {
            c.update(0.0, 0.1, Some(-100.0)); // impossible dump
        }
        assert_eq!(c.fill_kg(), 0.0, "tank cannot go negative");
    }

    #[test]
    fn payload_aboard_triggers_ballast_dump() {
        let mut c = BallastController::new(model(), 80.0, 2.0);
        run_to_trim(&mut c, 100.0, 1500);
        let before = c.fill_kg();
        // Collect a 3 kg sample: vehicle heavier → demand drops 3 kg → the
        // controller must dump ballast (negative rate).
        c.apply_payload_change(3.0);
        let cmd = c.update(100.0, 0.1, None);
        assert!(cmd.rate < 0.0, "must dump ballast after taking payload");
        // Demand shifted down by exactly the payload mass (within the trim
        // tolerance `run_to_trim` accepted).
        assert!(
            (cmd.target_kg - (before - 3.0)).abs() < 0.06,
            "demand must shift by the payload mass: {} vs {}",
            cmd.target_kg,
            before - 3.0
        );
    }

    #[test]
    fn payload_dropped_triggers_ballast_fill() {
        let mut c = BallastController::new(model(), 80.0, 2.0);
        run_to_trim(&mut c, 100.0, 1500);
        // Drop a 2 kg tool: vehicle lighter → pump water in.
        c.apply_payload_change(-2.0);
        let cmd = c.update(100.0, 0.1, None);
        assert!(cmd.rate > 0.0);
    }
}
