//! The random number generator of the `SET_SEED` / `PUSH_RAND` directives:
//! a port of C++ `std::mt19937` (the 32-bit Mersenne Twister, MT19937) and
//! of the `std::seed_seq` seeding that `PUSH_RAND` uses when no seed was
//! set, so a Rust sequencer produces the same numbers as the C++ one.
//!
//! Both algorithms are specified exactly by the C++ standard
//! (`[rand.eng.mers]`, `[rand.util.seedseq]`); the tests check them against
//! values printed by libstdc++.

const N: usize = 624;
const M: usize = 397;
const MATRIX_A: u32 = 0x9908_B0DF;
const UPPER_MASK: u32 = 0x8000_0000;
const LOWER_MASK: u32 = 0x7FFF_FFFF;

/// `std::mt19937`.
#[derive(Clone)]
pub struct Mt19937 {
    state: [u32; N],
    index: usize,
}

impl Mt19937 {
    /// `std::mt19937::default_seed`.
    pub const DEFAULT_SEED: u32 = 5489;

    /// A default-constructed engine (seeded with [`Self::DEFAULT_SEED`]).
    #[must_use]
    pub fn new() -> Self {
        let mut rng = Self {
            state: [0; N],
            index: N,
        };
        rng.seed(Self::DEFAULT_SEED);
        rng
    }

    /// `seed(result_type)`: the MT19937 linear initialization.
    pub fn seed(&mut self, value: u32) {
        self.state[0] = value;
        for i in 1..N {
            let prev = self.state[i - 1];
            self.state[i] = 1_812_433_253u32
                .wrapping_mul(prev ^ (prev >> 30))
                .wrapping_add(i as u32);
        }
        self.index = N;
    }

    /// `seed(std::seed_seq{values...})`: fill the state with
    /// `seed_seq::generate` and apply the all-zero fix-up of
    /// `[rand.eng.mers]`.
    pub fn seed_seq(&mut self, values: &[u32]) {
        seed_seq_generate(values, &mut self.state);
        // If the most significant w - r (= 1) bits of X(-n) and all other
        // words are zero, X(-n) becomes 2^(w-1).
        if self.state[0] & UPPER_MASK == 0 && self.state[1..].iter().all(|&x| x == 0) {
            self.state[0] = UPPER_MASK;
        }
        self.index = N;
    }

    /// `operator()`: the next 32-bit output.
    pub fn next_u32(&mut self) -> u32 {
        if self.index >= N {
            self.twist();
        }
        let mut y = self.state[self.index];
        self.index += 1;
        y ^= y >> 11;
        y ^= (y << 7) & 0x9D2C_5680;
        y ^= (y << 15) & 0xEFC6_0000;
        y ^= y >> 18;
        y
    }

    fn twist(&mut self) {
        for i in 0..N {
            let y = (self.state[i] & UPPER_MASK) | (self.state[(i + 1) % N] & LOWER_MASK);
            let mut next = self.state[(i + M) % N] ^ (y >> 1);
            if y & 1 != 0 {
                next ^= MATRIX_A;
            }
            self.state[i] = next;
        }
        self.index = 0;
    }
}

impl Default for Mt19937 {
    fn default() -> Self {
        Self::new()
    }
}

impl core::fmt::Debug for Mt19937 {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        f.debug_struct("Mt19937")
            .field("index", &self.index)
            .finish_non_exhaustive()
    }
}

/// `std::seed_seq(values).generate(out.begin(), out.end())`.
fn seed_seq_generate(values: &[u32], out: &mut [u32]) {
    let n = out.len();
    if n == 0 {
        return;
    }
    out.fill(0x8B8B_8B8B);
    let s = values.len();
    let t = if n >= 623 {
        11
    } else if n >= 68 {
        7
    } else if n >= 39 {
        5
    } else if n >= 7 {
        3
    } else {
        (n - 1) / 2
    };
    let p = (n - t) / 2;
    let q = p + t;
    let m = (s + 1).max(n);
    let tf = |x: u32| x ^ (x >> 27);
    for k in 0..m {
        let r1 =
            1_664_525u32.wrapping_mul(tf(out[k % n] ^ out[(k + p) % n] ^ out[(k + n - 1) % n]));
        let r2 = if k == 0 {
            r1.wrapping_add(s as u32)
        } else if k <= s {
            r1.wrapping_add((k % n) as u32).wrapping_add(values[k - 1])
        } else {
            r1.wrapping_add((k % n) as u32)
        };
        out[(k + p) % n] = out[(k + p) % n].wrapping_add(r1);
        out[(k + q) % n] = out[(k + q) % n].wrapping_add(r2);
        out[k % n] = r2;
    }
    for k in m..m + n {
        let r3 = 1_566_083_941u32.wrapping_mul(tf(out[k % n]
            .wrapping_add(out[(k + p) % n])
            .wrapping_add(out[(k + n - 1) % n])));
        let r4 = r3.wrapping_sub((k % n) as u32);
        out[(k + p) % n] ^= r3;
        out[(k + q) % n] ^= r4;
        out[k % n] = r4;
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    // Expected values printed by a libstdc++ program:
    //   std::mt19937 a; a();                 // default seed
    //   b.seed(123456789u); b();
    //   std::seed_seq s{2,7,123,456}; c.seed(s); c();

    #[test]
    fn default_seed_matches_libstdcxx() {
        let mut rng = Mt19937::new();
        assert_eq!(rng.next_u32(), 3_499_211_612);
        assert_eq!(rng.next_u32(), 581_869_302);
        assert_eq!(rng.next_u32(), 3_890_346_734);
    }

    #[test]
    fn ten_thousandth_output_is_the_standard_check_value() {
        // [rand.predef]: the 10000th consecutive invocation of a
        // default-constructed mt19937 produces 4123659995.
        let mut rng = Mt19937::new();
        for _ in 0..9999 {
            rng.next_u32();
        }
        assert_eq!(rng.next_u32(), 4_123_659_995);
    }

    #[test]
    fn integer_seed_matches_libstdcxx() {
        let mut rng = Mt19937::new();
        rng.seed(123_456_789);
        assert_eq!(rng.next_u32(), 2_288_500_408);
        assert_eq!(rng.next_u32(), 4_254_805_660);
        assert_eq!(rng.next_u32(), 2_294_099_250);
    }

    #[test]
    fn seed_seq_matches_libstdcxx() {
        let mut rng = Mt19937::new();
        rng.seed_seq(&[2, 7, 123, 456]);
        assert_eq!(rng.next_u32(), 469_824_542);
        assert_eq!(rng.next_u32(), 383_496_643);
        assert_eq!(rng.next_u32(), 3_718_868_230);

        rng.seed_seq(&[0, 0, 0, 0]);
        assert_eq!(rng.next_u32(), 927_617_601);
        assert_eq!(rng.next_u32(), 1_055_449_195);
        assert_eq!(rng.next_u32(), 5_268_542);

        rng.seed_seq(&[2, 0, 1, 0]);
        assert_eq!(rng.next_u32(), 1_460_940_338);
        assert_eq!(rng.next_u32(), 41_817_444);
        assert_eq!(rng.next_u32(), 949_229_711);
    }
}
