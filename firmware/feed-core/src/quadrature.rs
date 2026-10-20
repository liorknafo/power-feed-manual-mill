//! Quadrature decoder for the speed dial (mechanical EC11-style encoder, 4 edges per detent).

/// Gray-code transition table: index = (prev << 2) | curr, value = -1/0/+1 (0 also for invalid jumps).
// clockwise = 11 -> 01 -> 00 -> 10 -> 11 (each step +1)
const TABLE: [i8; 16] = [0, -1, 1, 0, 1, 0, 0, -1, -1, 0, 0, 1, 0, 1, -1, 0];

#[derive(Default)]
pub struct Quadrature {
    prev: u8,
    sub: i8,
}

impl Quadrature {
    pub fn new(a: bool, b: bool) -> Self {
        Quadrature { prev: state(a, b), sub: 0 }
    }

    /// Feed the current A/B levels; returns whole detents moved (+1 clockwise, -1 counter-clockwise).
    pub fn update(&mut self, a: bool, b: bool) -> i32 {
        let curr = state(a, b);
        if curr == self.prev {
            return 0;
        }
        self.sub += TABLE[((self.prev << 2) | curr) as usize];
        self.prev = curr;
        // a detent rests at state 0b11 (both pulled up); count when we get back there
        if curr == 0b11 {
            let d = (self.sub + if self.sub >= 0 { 2 } else { -2 }) / 4;
            self.sub = 0;
            return d as i32;
        }
        0
    }
}

fn state(a: bool, b: bool) -> u8 {
    ((a as u8) << 1) | b as u8
}

/// Simple time-based acceleration: fast spins of the dial move in bigger steps.
pub struct DialAccel {
    last_ms: u32,
}

impl DialAccel {
    pub const fn new() -> Self {
        DialAccel { last_ms: 0 }
    }
    /// True if this detent came quickly after the previous one.
    pub fn is_fast(&mut self, now_ms: u32) -> bool {
        let fast = now_ms.wrapping_sub(self.last_ms) < 40;
        self.last_ms = now_ms;
        fast
    }
}

impl Default for DialAccel {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn run(q: &mut Quadrature, seq: &[(bool, bool)]) -> i32 {
        seq.iter().map(|&(a, b)| q.update(a, b)).sum()
    }

    // one clockwise detent: 11 -> 01 -> 00 -> 10 -> 11
    const CW: [(bool, bool); 4] = [(false, true), (false, false), (true, false), (true, true)];
    const CCW: [(bool, bool); 4] = [(true, false), (false, false), (false, true), (true, true)];

    #[test]
    fn counts_detents_both_ways() {
        let mut q = Quadrature::new(true, true);
        assert_eq!(run(&mut q, &CW), 1);
        assert_eq!(run(&mut q, &CW), 1);
        assert_eq!(run(&mut q, &CCW), -1);
    }

    #[test]
    fn contact_bounce_does_not_count() {
        let mut q = Quadrature::new(true, true);
        // wiggle 11 -> 01 -> 11 -> 01 -> 11 (never completes a detent)
        let seq = [(false, true), (true, true), (false, true), (true, true)];
        assert_eq!(run(&mut q, &seq), 0);
    }

    #[test]
    fn accel() {
        let mut a = DialAccel::new();
        assert!(!a.is_fast(1000));
        assert!(a.is_fast(1020));
        assert!(!a.is_fast(1200));
    }
}
