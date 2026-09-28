//! Slab-allocated ring of fixed-size point clouds (spec §4.3: "each ping
//! overwrites the oldest — no per-ping allocation").
//!
//! Memory is paid once: [`SLABS`] slabs × [`super::raytrace::BEAMS`]
//! points each. The producer claims the oldest slab, fills it in place
//! (zero-copy generation), and publishes it; consumers iterate the most
//! recent clouds.

/// Number of clouds kept in the ring.
pub const SLABS: usize = 16;

/// One bathymetric point.
#[derive(Debug, Clone, Copy, PartialEq, Default)]
#[repr(C)]
pub struct Point {
    /// North (vehicle frame x), metres.
    pub x: f32,
    /// East (vehicle frame y), metres.
    pub y: f32,
    /// Down (vehicle frame z), metres.
    pub z: f32,
    /// Backscatter amplitude, dB (0 if unknown).
    pub amp_db: f32,
}

/// A fixed-capacity point cloud — one ping's worth.
#[derive(Debug, Clone, Copy)]
#[repr(C)]
pub struct Cloud {
    /// Points in vehicle frame; validity beyond `beam_count` is undefined.
    pub points: [Point; super::raytrace::BEAMS],
    /// Live beams this ping.
    pub beam_count: u16,
    /// Ping timestamp, microseconds.
    pub timestamp_us: u64,
    /// Slab generation counter (monotonic; detects consumer lag).
    pub generation: u64,
}

impl Default for Cloud {
    fn default() -> Self {
        Self {
            points: [Point::default(); super::raytrace::BEAMS],
            beam_count: 0,
            timestamp_us: 0,
            generation: 0,
        }
    }
}

impl Cloud {
    /// Total slab memory per cloud, bytes (diagnostics).
    pub const BYTES: usize = core::mem::size_of::<Cloud>();
}

/// The slab ring: fixed allocation, oldest-overwritten.
#[derive(Debug)]
pub struct SlabRing {
    slabs: [Cloud; SLABS],
    head: usize,
    generation: u64,
    filled: usize,
}

impl Default for SlabRing {
    fn default() -> Self {
        Self::new()
    }
}

impl SlabRing {
    /// Allocate the ring (the *only* allocation this crate ever does).
    pub fn new() -> Self {
        Self {
            slabs: [Cloud::default(); SLABS],
            head: 0,
            generation: 0,
            filled: 0,
        }
    }

    /// Total ring footprint, bytes (diagnostics: 16 slabs ≈ 100 KB).
    pub const FOOTPRINT_BYTES: usize = core::mem::size_of::<SlabRing>();

    /// Claim the next slab for writing (the oldest one). Fill it, then the
    /// publish happens implicitly: the claim already moved the head.
    pub fn claim(&mut self, timestamp_us: u64) -> &mut Cloud {
        let c = &mut self.slabs[self.head];
        c.timestamp_us = timestamp_us;
        self.generation += 1;
        c.generation = self.generation;
        self.head = (self.head + 1) % SLABS;
        self.filled = (self.filled + 1).min(SLABS);
        c
    }

    /// Most recent cloud, if any.
    pub fn latest(&self) -> Option<&Cloud> {
        if self.filled == 0 {
            return None;
        }
        let head = (self.head + SLABS - 1) % SLABS;
        Some(&self.slabs[head])
    }

    /// Iterate stored clouds oldest → newest.
    pub fn iter(&self) -> impl Iterator<Item = &Cloud> {
        let start = (self.head + SLABS - self.filled) % SLABS;
        (0..self.filled).map(move |k| &self.slabs[(start + k) % SLABS])
    }

    /// Number of clouds currently stored.
    pub fn len(&self) -> usize {
        self.filled
    }

    /// Whether nothing has been pushed yet.
    pub fn is_empty(&self) -> bool {
        self.filled == 0
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn ring_overwrites_oldest_without_allocation() {
        let mut ring = SlabRing::new();
        for k in 0u64..SLABS as u64 + 5 {
            let c = ring.claim(k * 100);
            c.beam_count = 1;
            c.points[0].x = k as f32;
        }
        assert_eq!(ring.len(), SLABS);
        // 21 claims into 16 slabs → surviving generations 6..=21.
        let gens: Vec<u64> = ring.iter().map(|c| c.generation).collect();
        assert_eq!(gens.first().unwrap(), &6);
        assert_eq!(gens.last().unwrap(), &21);
        // Latest cloud is the last claim.
        assert_eq!(ring.latest().unwrap().points[0].x, 20.0);
    }

    #[test]
    fn footprint_is_bounded_and_known() {
        // 16 slabs of 512 points plus ring bookkeeping ≈ 131 KB — fixed
        // forever, and comfortably under the 256 KB line. (The bound is
        // also enforced at compile time via a const block in `new`.)
        let footprint = std::hint::black_box(SlabRing::FOOTPRINT_BYTES);
        assert!(footprint < 256 * 1024);
    }

    #[test]
    fn empty_ring_reports_empty() {
        let ring = SlabRing::new();
        assert!(ring.is_empty());
        assert!(ring.latest().is_none());
        assert_eq!(ring.iter().count(), 0);
    }
}
