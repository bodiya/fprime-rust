//! # FPP-style code generation: `fpp_enum!`, `fpp_struct!`, `fpp_array!`
//!
//! Declarative (`macro_rules!`) replacements for the three user-defined data
//! types the C++ FPP autocoder emits as `Fw::Serializable` classes —
//! `<Name>EnumAc`, `<Name>SerializableAc`, `<Name>ArrayAc` (see
//! `docs/cpp-analysis/fpp-autocoder.md`, sections "Generated enum classes",
//! "Generated struct classes", "Generated array classes"). The macros
//! reproduce the generated API and, more importantly, the generated **wire
//! format**:
//!
//! | FPP type | wire format |
//! |----------|-------------|
//! | `enum E : R` | the representation value, `R`-width, big-endian; decode validates the exact declared values |
//! | `struct S {a: A, b: B}` | `A` then `B` in declaration order — no header, no count, no padding |
//! | `array X = [n] T` | `n` elements consecutively — no count prefix, no padding |
//!
//! Zero third-party dependencies and no proc macros: everything here is
//! `macro_rules!`, so the codegen layer costs the workspace nothing.
//!
//! ## Sizing
//!
//! [`FppSized`] carries the compile-time `SERIALIZED_SIZE` the C++ generated
//! classes expose as a static constant. It is the *maximum* on-wire size
//! (for variable-length members such as strings, `Serialize::serialized_size`
//! reports the actual size of a given value, exactly as C++ splits
//! `SERIALIZED_SIZE` from `getSerializedSize()`).
//!
//! ## What is deliberately NOT generated
//!
//! - `toString`/format strings (the FPP `format` qualifier): no consumer in
//!   the port yet; `Debug` covers diagnostics.
//! - String-typed members backed by `ExternalString` views: [`FwString`]
//!   is owned, so a string member is just a member.
//! - Getter/setter names are **not** synthesized from field names —
//!   `macro_rules!` cannot concatenate identifiers. Declare the accessor
//!   pair explicitly (`field: T { get_field, set_field }`) when the C++
//!   `get_*`/`set_*` API is wanted; the fields themselves are always `pub`.

use crate::serial::{Deserialize, Serialize};
use crate::string::FwString;
use crate::time::{Time, TimeInterval};

/// Compile-time on-wire size of an FPP type — the port of the
/// `SERIALIZED_SIZE` static constant on every autocoded `Fw::Serializable`.
///
/// For fixed-width types this equals `Serialize::serialized_size` for every
/// value; for variable-length ones (strings, and structs/arrays containing
/// them) it is the maximum, which is what the C++ constant is used for:
/// sizing message and packet buffers at compile time.
pub trait FppSized: Serialize + Deserialize {
    /// Maximum number of bytes `serialize_to` can write for this type.
    const SERIALIZED_SIZE: usize;
}

macro_rules! fpp_sized_prim {
    ($($ty:ty),+ $(,)?) => {
        $(
            impl FppSized for $ty {
                const SERIALIZED_SIZE: usize = size_of::<$ty>();
            }
        )+
    };
}

fpp_sized_prim!(u8, i8, u16, i16, u32, i32, u64, i64, f32, f64);

impl FppSized for bool {
    /// `0xFF`/`0x00`, one byte.
    const SERIALIZED_SIZE: usize = 1;
}

impl FppSized for Time {
    const SERIALIZED_SIZE: usize = Time::SERIALIZED_SIZE;
}

impl FppSized for TimeInterval {
    const SERIALIZED_SIZE: usize = TimeInterval::SERIALIZED_SIZE;
}

impl<const N: usize> FppSized for FwString<N> {
    /// `u16` length prefix + the full capacity (C++
    /// `STATIC_SERIALIZED_SIZE`).
    const SERIALIZED_SIZE: usize = FwString::<N>::SERIALIZED_SIZE;
}

// ---------------------------------------------------------------------------
// fpp_enum!
// ---------------------------------------------------------------------------

/// Declare an FPP enum type (`enum E : R { A = 0, ... } default A`).
///
/// Generates, mirroring `<Name>EnumAc`:
///
/// - the `#[repr(R)]` enum with the **exact** declared discriminants,
/// - `Default` from the `default` clause,
/// - `TryFrom<R>` (the raw value comes back in `Err`),
/// - `SERIALIZED_SIZE`, `VALUES`, `NUM_CONSTANTS`, `as_repr()`,
///   `is_valid()` (member) and `is_valid_repr(raw)` (static, the C++
///   `isValid` overload — non-contiguous value sets are checked exactly),
/// - [`Serialize`]/[`Deserialize`] at the representation width, with
///   **strict** decode: an undeclared value consumes its bytes and returns
///   `DeserFormatError`, leaving the target unmodified,
/// - [`FppSized`].
///
/// ```
/// use fprime_fw::{fpp_enum, Endianness, LinearBuffer, SerBuf, Serialize, SerializeStatus};
///
/// fpp_enum! {
///     /// Which way the widget points.
///     pub enum Direction : u16 {
///         /// Pointing up.
///         Up = 0x10,
///         /// Pointing down.
///         Down = 0x20,
///     }
///     default Up
/// }
///
/// assert_eq!(Direction::default(), Direction::Up);
/// assert_eq!(Direction::SERIALIZED_SIZE, 2);
/// assert!(Direction::is_valid_repr(0x20));
/// assert!(!Direction::is_valid_repr(0x11));
///
/// let mut buf = LinearBuffer::<8>::new();
/// assert_eq!(Direction::Down.serialize_to(&mut buf, Endianness::Big), SerializeStatus::Ok);
/// assert_eq!(buf.as_slice(), &[0x00, 0x20]);
/// ```
#[macro_export]
macro_rules! fpp_enum {
    (
        $(#[$meta:meta])*
        $vis:vis enum $name:ident : $repr:ty {
            $($(#[$vmeta:meta])* $var:ident = $val:literal),+ $(,)?
        }
        default $def:ident
    ) => {
        $(#[$meta])*
        #[repr($repr)]
        #[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
        $vis enum $name {
            $($(#[$vmeta])* $var = $val),+
        }

        impl $name {
            /// On-wire size: the width of the representation type
            /// (C++ `SERIALIZED_SIZE`).
            pub const SERIALIZED_SIZE: usize = size_of::<$repr>();

            /// Every declared constant, in declaration order.
            pub const VALUES: &'static [Self] = &[$(Self::$var),+];

            /// Number of declared constants (C++ `NUM_CONSTANTS`).
            pub const NUM_CONSTANTS: usize = Self::VALUES.len();

            /// The underlying representation value.
            #[must_use]
            pub const fn as_repr(self) -> $repr {
                self as $repr
            }

            /// C++ member `isValid()`. Always true in Rust — a value of this
            /// type cannot hold an undeclared discriminant — but kept so
            /// ported code reads 1:1 against the C++ it replaces.
            #[must_use]
            pub const fn is_valid(self) -> bool {
                true
            }

            /// C++ static `isValid(SerialType)`: is `raw` one of the exact
            /// declared values? Non-contiguous value sets are checked
            /// exactly (values *between* declared constants are invalid).
            #[must_use]
            // Contiguous value sets look like a range, but the FPP contract
            // is "exactly these declared constants" — keep the OR pattern so
            // adding a non-contiguous constant stays correct.
            #[allow(clippy::manual_range_patterns)]
            pub const fn is_valid_repr(raw: $repr) -> bool {
                matches!(raw, $($val)|+)
            }
        }

        impl Default for $name {
            fn default() -> Self {
                Self::$def
            }
        }

        impl TryFrom<$repr> for $name {
            type Error = $repr;
            /// Map a raw representation value to the enum; `Err` carries the
            /// unmatched raw value.
            fn try_from(v: $repr) -> Result<Self, $repr> {
                match v {
                    $($val => Ok(Self::$var),)+
                    other => Err(other),
                }
            }
        }

        impl $crate::serial::Serialize for $name {
            fn serialize_to(
                &self,
                buf: &mut dyn $crate::serial::SerBufAny,
                e: $crate::serial::Endianness,
            ) -> $crate::serial::SerializeStatus {
                $crate::serial::Serialize::serialize_to(&(*self as $repr), buf, e)
            }
            fn serialized_size(&self) -> usize {
                Self::SERIALIZED_SIZE
            }
        }

        impl $crate::serial::Deserialize for $name {
            fn deserialize_from(
                &mut self,
                buf: &mut dyn $crate::serial::SerBufAny,
                e: $crate::serial::Endianness,
            ) -> $crate::serial::SerializeStatus {
                let mut raw: $repr = <$repr as Default>::default();
                let status = $crate::serial::Deserialize::deserialize_from(&mut raw, buf, e);
                if status != $crate::serial::SerializeStatus::Ok {
                    return status;
                }
                // C++ parity: an undeclared value is DeserFormatError; the
                // raw bytes are consumed but the target is left unmodified.
                match Self::try_from(raw) {
                    Ok(v) => {
                        *self = v;
                        $crate::serial::SerializeStatus::Ok
                    }
                    Err(_) => $crate::serial::SerializeStatus::DeserFormatError,
                }
            }
        }

        impl $crate::fpp::FppSized for $name {
            const SERIALIZED_SIZE: usize = Self::SERIALIZED_SIZE;
        }
    };
}

// ---------------------------------------------------------------------------
// fpp_struct!
// ---------------------------------------------------------------------------

/// Declare an FPP struct type (`struct S { a: A, b: B } default { a = .. }`).
///
/// Generates, mirroring `<Name>SerializableAc`:
///
/// - the struct with all members `pub`, in declaration order,
/// - optional `get_*`/`set_*` accessor pairs per member (the C++ generated
///   API; the getter borrows, as C++ returns `const&` for class members),
/// - `new(..)` — the all-member constructor — and `set_all(..)` (C++ `set`),
/// - `Default` from the `default { .. }` clause; members not listed there
///   get their type's `Default`,
/// - `SERIALIZED_SIZE` (sum of the members' [`FppSized`] sizes) and
///   [`FppSized`],
/// - `Debug` + `PartialEq` (add `Clone`/`Copy`/`Eq`/... with a normal
///   `#[derive(..)]` above the declaration),
/// - [`Serialize`]/[`Deserialize`] in strict declaration order with no
///   header and no padding. Deserialization is **commit-on-success**: a
///   failure part-way leaves `self` untouched.
///
/// ```
/// use fprime_fw::{fpp_struct, Endianness, LinearBuffer, SerBuf, Serialize, SerializeStatus};
///
/// fpp_struct! {
///     /// A widget reading.
///     #[derive(Clone, Copy, Eq)]
///     pub struct Reading {
///         /// Sensor channel.
///         channel: u16 { get_channel, set_channel },
///         /// Raw counts.
///         counts: u32,
///         /// Whether the reading is trustworthy.
///         valid: bool,
///     }
///     default {
///         channel = 7,
///         valid = true,
///     }
/// }
///
/// assert_eq!(Reading::SERIALIZED_SIZE, 2 + 4 + 1);
/// let r = Reading::default();
/// assert_eq!(*r.get_channel(), 7);
/// assert!(r.valid);
///
/// let r = Reading::new(1, 0x0A0B0C0D, false);
/// let mut buf = LinearBuffer::<16>::new();
/// assert_eq!(r.serialize_to(&mut buf, Endianness::Big), SerializeStatus::Ok);
/// assert_eq!(buf.as_slice(), &[0x00, 0x01, 0x0A, 0x0B, 0x0C, 0x0D, 0x00]);
/// ```
#[macro_export]
macro_rules! fpp_struct {
    (
        $(#[$meta:meta])*
        $vis:vis struct $name:ident {
            $(
                $(#[$fmeta:meta])*
                $field:ident : $fty:ty $({ $getter:ident, $setter:ident })?
            ),* $(,)?
        }
        $(default { $($dfield:ident = $dval:expr),* $(,)? })?
    ) => {
        $(#[$meta])*
        #[derive(Debug, PartialEq)]
        $vis struct $name {
            $(
                $(#[$fmeta])*
                pub $field: $fty,
            )*
        }

        impl $name {
            /// Maximum on-wire size: the sum of the members' sizes in
            /// declaration order (C++ `SERIALIZED_SIZE`; no padding).
            pub const SERIALIZED_SIZE: usize =
                0 $(+ <$fty as $crate::fpp::FppSized>::SERIALIZED_SIZE)*;

            /// All-member constructor (the C++ generated full constructor).
            #[allow(clippy::too_many_arguments)]
            pub fn new($($field: $fty),*) -> Self {
                Self { $($field),* }
            }

            /// Assign every member at once (the C++ generated `set(..)`).
            #[allow(clippy::too_many_arguments)]
            pub fn set_all(&mut self, $($field: $fty),*) {
                $(self.$field = $field;)*
            }

            $($(
                #[doc = concat!("Get the `", stringify!($field), "` member (C++ `get_", stringify!($field), "`).")]
                #[must_use]
                pub fn $getter(&self) -> &$fty {
                    &self.$field
                }

                #[doc = concat!("Set the `", stringify!($field), "` member (C++ `set_", stringify!($field), "`).")]
                pub fn $setter(&mut self, value: $fty) {
                    self.$field = value;
                }
            )?)*
        }

        #[allow(clippy::derivable_impls)]
        impl Default for $name {
            /// Members listed in the `default { .. }` clause take those
            /// values; the rest take their type's `Default`.
            fn default() -> Self {
                #[allow(unused_mut)]
                let mut value = Self {
                    $($field: <$fty as Default>::default()),*
                };
                $($(value.$dfield = $dval;)*)?
                value
            }
        }

        impl $crate::serial::Serialize for $name {
            /// Members in declaration order; no header, count or padding.
            #[allow(unused_variables)]
            fn serialize_to(
                &self,
                buf: &mut dyn $crate::serial::SerBufAny,
                e: $crate::serial::Endianness,
            ) -> $crate::serial::SerializeStatus {
                $($crate::fw_try!(
                    $crate::serial::Serialize::serialize_to(&self.$field, buf, e)
                );)*
                $crate::serial::SerializeStatus::Ok
            }
            fn serialized_size(&self) -> usize {
                0 $(+ $crate::serial::Serialize::serialized_size(&self.$field))*
            }
        }

        impl $crate::serial::Deserialize for $name {
            /// Reads into a temporary and commits only on full success, so a
            /// partial/invalid message leaves `self` unmodified.
            #[allow(unused_variables)]
            fn deserialize_from(
                &mut self,
                buf: &mut dyn $crate::serial::SerBufAny,
                e: $crate::serial::Endianness,
            ) -> $crate::serial::SerializeStatus {
                #[allow(unused_mut)]
                let mut tmp = <Self as Default>::default();
                $($crate::fw_try!(
                    $crate::serial::Deserialize::deserialize_from(&mut tmp.$field, buf, e)
                );)*
                *self = tmp;
                $crate::serial::SerializeStatus::Ok
            }
        }

        impl $crate::fpp::FppSized for $name {
            const SERIALIZED_SIZE: usize = Self::SERIALIZED_SIZE;
        }
    };
}

// ---------------------------------------------------------------------------
// fpp_array!
// ---------------------------------------------------------------------------

/// Declare an FPP array type (`array A = [n] T default [..]`).
///
/// Generates, mirroring `<Name>ArrayAc`: a newtype over `[T; n]` with
/// `SIZE`, `SERIALIZED_SIZE`, [`FppSized`], `new`/`fill`/`From` conversions,
/// `elements()`/`elements_mut()`/`as_slice()`/`iter()`, `Index`/`IndexMut`,
/// `Debug` + `PartialEq`, `Default` (elementwise or from the `default`
/// clause), and elementwise [`Serialize`]/[`Deserialize`] — **no count
/// prefix, no padding**, exactly like the generated C++.
///
/// Three default forms:
///
/// ```text
/// (omitted)             every element = T::default()
/// default fill EXPR     every element = EXPR
/// default [E0, E1, ..]  element-by-element (must have exactly n entries)
/// ```
///
/// ```
/// use fprime_fw::{fpp_array, Endianness, LinearBuffer, SerBuf, Serialize, SerializeStatus};
///
/// fpp_array! {
///     /// Three cycle counts.
///     #[derive(Clone, Copy, Eq)]
///     pub array Counts = [u16; 3]
///     default fill 0xFFFF
/// }
///
/// assert_eq!(Counts::SIZE, 3);
/// assert_eq!(Counts::SERIALIZED_SIZE, 6);
/// assert_eq!(Counts::default()[1], 0xFFFF);
///
/// let c = Counts::new([1, 2, 3]);
/// let mut buf = LinearBuffer::<8>::new();
/// assert_eq!(c.serialize_to(&mut buf, Endianness::Big), SerializeStatus::Ok);
/// assert_eq!(buf.as_slice(), &[0, 1, 0, 2, 0, 3]);
/// ```
#[macro_export]
macro_rules! fpp_array {
    // default: elementwise T::default()
    (
        $(#[$meta:meta])*
        $vis:vis array $name:ident = [$elem:ty; $n:expr]
    ) => {
        $crate::fpp_array! {
            $(#[$meta])*
            $vis array $name = [$elem; $n]
            default fill <$elem as Default>::default()
        }
    };

    // default: fill every element with one expression
    (
        $(#[$meta:meta])*
        $vis:vis array $name:ident = [$elem:ty; $n:expr]
        default fill $fill:expr
    ) => {
        $crate::__fpp_array_impl! {
            $(#[$meta])*
            $vis array $name = [$elem; $n]
            default { ::std::array::from_fn(|_| $fill) }
        }
    };

    // default: an explicit element list
    (
        $(#[$meta:meta])*
        $vis:vis array $name:ident = [$elem:ty; $n:expr]
        default [$($dval:expr),* $(,)?]
    ) => {
        $crate::__fpp_array_impl! {
            $(#[$meta])*
            $vis array $name = [$elem; $n]
            default { [$($dval),*] }
        }
    };
}

/// Shared expansion for [`fpp_array!`]; not part of the public API.
#[doc(hidden)]
#[macro_export]
macro_rules! __fpp_array_impl {
    (
        $(#[$meta:meta])*
        $vis:vis array $name:ident = [$elem:ty; $n:expr]
        default { $default:expr }
    ) => {
        $(#[$meta])*
        #[derive(Debug, PartialEq)]
        $vis struct $name(
            /// The elements, in index order.
            pub [$elem; $n],
        );

        impl $name {
            /// Number of elements (C++ `SIZE`).
            pub const SIZE: usize = $n;

            /// Maximum on-wire size: `SIZE * element size`, no count prefix.
            pub const SERIALIZED_SIZE: usize =
                $n * <$elem as $crate::fpp::FppSized>::SERIALIZED_SIZE;

            /// Construct from a full element array.
            pub const fn new(elements: [$elem; $n]) -> Self {
                Self(elements)
            }

            /// Construct with every element set to `value` (the C++
            /// single-value fill constructor).
            pub fn fill(value: $elem) -> Self
            {
                Self(::std::array::from_fn(|_| value.clone()))
            }

            /// Borrow the backing element array.
            #[must_use]
            pub const fn elements(&self) -> &[$elem; $n] {
                &self.0
            }

            /// Mutably borrow the backing element array.
            pub const fn elements_mut(&mut self) -> &mut [$elem; $n] {
                &mut self.0
            }

            /// The elements as a slice.
            #[must_use]
            pub const fn as_slice(&self) -> &[$elem] {
                &self.0
            }

            /// Iterate over the elements in index order.
            pub fn iter(&self) -> ::std::slice::Iter<'_, $elem> {
                self.0.iter()
            }
        }

        impl Default for $name {
            fn default() -> Self {
                Self($default)
            }
        }

        impl ::std::ops::Index<usize> for $name {
            type Output = $elem;
            fn index(&self, index: usize) -> &$elem {
                &self.0[index]
            }
        }

        impl ::std::ops::IndexMut<usize> for $name {
            fn index_mut(&mut self, index: usize) -> &mut $elem {
                &mut self.0[index]
            }
        }

        impl From<[$elem; $n]> for $name {
            fn from(elements: [$elem; $n]) -> Self {
                Self(elements)
            }
        }

        impl From<$name> for [$elem; $n] {
            fn from(value: $name) -> Self {
                value.0
            }
        }

        impl<'a> IntoIterator for &'a $name {
            type Item = &'a $elem;
            type IntoIter = ::std::slice::Iter<'a, $elem>;
            fn into_iter(self) -> Self::IntoIter {
                self.0.iter()
            }
        }

        impl $crate::serial::Serialize for $name {
            /// Elements consecutively; no count prefix, no padding.
            fn serialize_to(
                &self,
                buf: &mut dyn $crate::serial::SerBufAny,
                e: $crate::serial::Endianness,
            ) -> $crate::serial::SerializeStatus {
                for element in &self.0 {
                    $crate::fw_try!(
                        $crate::serial::Serialize::serialize_to(element, buf, e)
                    );
                }
                $crate::serial::SerializeStatus::Ok
            }
            fn serialized_size(&self) -> usize {
                let mut total = 0;
                for element in &self.0 {
                    total += $crate::serial::Serialize::serialized_size(element);
                }
                total
            }
        }

        impl $crate::serial::Deserialize for $name {
            /// Reads into a temporary and commits only on full success.
            fn deserialize_from(
                &mut self,
                buf: &mut dyn $crate::serial::SerBufAny,
                e: $crate::serial::Endianness,
            ) -> $crate::serial::SerializeStatus {
                let mut tmp = <Self as Default>::default();
                for element in &mut tmp.0 {
                    $crate::fw_try!(
                        $crate::serial::Deserialize::deserialize_from(element, buf, e)
                    );
                }
                *self = tmp;
                $crate::serial::SerializeStatus::Ok
            }
        }

        impl $crate::fpp::FppSized for $name {
            const SERIALIZED_SIZE: usize = Self::SERIALIZED_SIZE;
        }
    };
}

#[cfg(test)]
mod tests {
    #![allow(dead_code)]

    use super::*;
    use crate::serial::{Endianness, LinearBuffer, SerBuf, SerializeStatus};
    use crate::string::CmdStringArg;

    // -- test types ---------------------------------------------------------

    fpp_enum! {
        /// Deliberately non-contiguous, u16-repr (C++ `IsValidTest`).
        pub enum Mode : u16 {
            /// Off.
            Off = 0,
            /// Standby.
            Standby = 5,
            /// Running.
            Running = 0x0100,
        }
        default Standby
    }

    fpp_array! {
        /// `array Triple = [3] I64` — the C++ `TestArray1` shape (24 bytes).
        #[derive(Clone, Copy, Eq)]
        pub array Triple = [i64; 3]
    }

    fpp_array! {
        /// Fill-clause defaults.
        #[derive(Clone, Copy, Eq)]
        pub array Filled = [u16; 4]
        default fill 0xBEEF
    }

    fpp_array! {
        /// Explicit element-list defaults.
        #[derive(Clone, Copy, Eq)]
        pub array Listed = [u8; 3]
        default [1, 2, 3]
    }

    fpp_struct! {
        /// A struct exercising every member kind: primitive, bool, enum,
        /// nested array, nested struct, string.
        #[derive(Clone)]
        pub struct Telemetry {
            /// Sequence counter.
            seq: u32 { get_seq, set_seq },
            /// Whether the reading is trustworthy.
            valid: bool { get_valid, set_valid },
            /// Operating mode (nested FPP enum).
            mode: Mode { get_mode, set_mode },
            /// Nested FPP array.
            samples: Triple,
            /// A fixed-capacity string member.
            label: CmdStringArg,
        }
        default {
            seq = 7,
            valid = true,
            mode = Mode::Running,
        }
    }

    fpp_struct! {
        /// The empty struct case (C++ `SERIALIZED_SIZE` 0).
        #[derive(Clone, Copy, Eq)]
        pub struct Empty {}
    }

    // -- FppSized -----------------------------------------------------------

    #[test]
    fn primitive_sizes_match_representation_widths() {
        assert_eq!(<u8 as FppSized>::SERIALIZED_SIZE, 1);
        assert_eq!(<i8 as FppSized>::SERIALIZED_SIZE, 1);
        assert_eq!(<u16 as FppSized>::SERIALIZED_SIZE, 2);
        assert_eq!(<i32 as FppSized>::SERIALIZED_SIZE, 4);
        assert_eq!(<f64 as FppSized>::SERIALIZED_SIZE, 8);
        assert_eq!(<bool as FppSized>::SERIALIZED_SIZE, 1);
        assert_eq!(<Time as FppSized>::SERIALIZED_SIZE, 11);
        assert_eq!(<TimeInterval as FppSized>::SERIALIZED_SIZE, 8);
        assert_eq!(<CmdStringArg as FppSized>::SERIALIZED_SIZE, 42);
    }

    // -- fpp_enum! ----------------------------------------------------------

    #[test]
    fn enum_is_valid_repr_rejects_values_between_declared_constants() {
        // C++ IsValidTest: values BETWEEN declared constants are invalid.
        assert!(Mode::is_valid_repr(0));
        assert!(Mode::is_valid_repr(5));
        assert!(Mode::is_valid_repr(0x0100));
        assert!(!Mode::is_valid_repr(1));
        assert!(!Mode::is_valid_repr(4));
        assert!(!Mode::is_valid_repr(0x00FF));
        assert!(!Mode::is_valid_repr(0x0101));
        // The member overload is trivially true — a Rust value cannot hold
        // an undeclared discriminant.
        assert!(Mode::Off.is_valid());
    }

    #[test]
    fn enum_metadata_matches_declaration() {
        assert_eq!(Mode::SERIALIZED_SIZE, 2);
        assert_eq!(<Mode as FppSized>::SERIALIZED_SIZE, 2);
        assert_eq!(Mode::NUM_CONSTANTS, 3);
        assert_eq!(Mode::VALUES, &[Mode::Off, Mode::Standby, Mode::Running]);
        assert_eq!(Mode::Running.as_repr(), 0x0100);
        assert_eq!(Mode::default(), Mode::Standby);
    }

    #[test]
    fn enum_serializes_at_repr_width_big_endian() {
        let mut buf = LinearBuffer::<8>::new();
        assert_eq!(
            Mode::Running.serialize_to(&mut buf, Endianness::Big),
            SerializeStatus::Ok
        );
        assert_eq!(buf.as_slice(), &[0x01, 0x00]);
        assert_eq!(Mode::Running.serialized_size(), 2);
    }

    #[test]
    fn enum_decode_rejects_undeclared_value_leaving_target_unmodified() {
        let mut buf = LinearBuffer::<8>::new();
        assert_eq!(buf.serialize_u16_be(4), SerializeStatus::Ok);
        let mut mode = Mode::Running;
        assert_eq!(
            mode.deserialize_from(&mut buf, Endianness::Big),
            SerializeStatus::DeserFormatError
        );
        assert_eq!(mode, Mode::Running);
    }

    // -- fpp_array! ---------------------------------------------------------

    #[test]
    fn array_size_constants_match_cpp_test_array() {
        // C++ TestArray1 = [3] I64 -> SERIALIZED_SIZE 24.
        assert_eq!(Triple::SIZE, 3);
        assert_eq!(Triple::SERIALIZED_SIZE, 24);
        assert_eq!(<Triple as FppSized>::SERIALIZED_SIZE, 24);
    }

    #[test]
    fn array_serializes_elements_with_no_count_prefix() {
        let a = Triple::new([1, -1, 0x0102_0304_0506_0708]);
        let mut buf = LinearBuffer::<64>::new();
        assert_eq!(
            a.serialize_to(&mut buf, Endianness::Big),
            SerializeStatus::Ok
        );

        // Byte-for-byte equality with hand-written elementwise serialization.
        let mut hand = LinearBuffer::<64>::new();
        assert_eq!(hand.serialize_i64_be(1), SerializeStatus::Ok);
        assert_eq!(hand.serialize_i64_be(-1), SerializeStatus::Ok);
        assert_eq!(
            hand.serialize_i64_be(0x0102_0304_0506_0708),
            SerializeStatus::Ok
        );
        assert_eq!(buf.as_slice(), hand.as_slice());
        assert_eq!(buf.as_slice().len(), 24);
        assert_eq!(a.serialized_size(), 24);
    }

    #[test]
    fn array_defaults_elementwise_fill_and_list() {
        assert_eq!(Triple::default(), Triple::new([0, 0, 0]));
        assert_eq!(Filled::default(), Filled::new([0xBEEF; 4]));
        assert_eq!(Listed::default(), Listed::new([1, 2, 3]));
        assert_eq!(Filled::fill(1), Filled::new([1, 1, 1, 1]));
    }

    #[test]
    fn array_index_iterate_and_convert() {
        let mut a = Triple::from([10, 20, 30]);
        assert_eq!(a[1], 20);
        a[1] = 21;
        assert_eq!(a.elements(), &[10, 21, 30]);
        a.elements_mut()[0] = 11;
        assert_eq!(a.as_slice(), &[11, 21, 30]);
        assert_eq!(a.iter().sum::<i64>(), 62);
        assert_eq!((&a).into_iter().count(), 3);
        let raw: [i64; 3] = a.into();
        assert_eq!(raw, [11, 21, 30]);
    }

    #[test]
    fn array_round_trips() {
        let a = Filled::new([1, 2, 3, 4]);
        let mut buf = LinearBuffer::<16>::new();
        assert_eq!(
            a.serialize_to(&mut buf, Endianness::Big),
            SerializeStatus::Ok
        );
        let mut out = Filled::default();
        assert_eq!(
            out.deserialize_from(&mut buf, Endianness::Big),
            SerializeStatus::Ok
        );
        assert_eq!(out, a);
    }

    #[test]
    fn array_short_buffer_leaves_target_unmodified() {
        let mut buf = LinearBuffer::<16>::new();
        assert_eq!(buf.serialize_u16_be(1), SerializeStatus::Ok);
        assert_eq!(buf.serialize_u16_be(2), SerializeStatus::Ok);
        let mut out = Filled::new([9, 9, 9, 9]);
        assert_eq!(
            out.deserialize_from(&mut buf, Endianness::Big),
            SerializeStatus::DeserBufferEmpty
        );
        assert_eq!(
            out,
            Filled::new([9, 9, 9, 9]),
            "commit only on full success"
        );
    }

    // -- fpp_struct! --------------------------------------------------------

    #[test]
    fn struct_serialized_size_is_the_sum_of_member_sizes() {
        // u32 + bool + Mode(u16) + Triple(24) + CmdStringArg(2 + 40)
        assert_eq!(Telemetry::SERIALIZED_SIZE, 4 + 1 + 2 + 24 + 42);
        assert_eq!(<Telemetry as FppSized>::SERIALIZED_SIZE, 73);
        assert_eq!(Empty::SERIALIZED_SIZE, 0);
    }

    #[test]
    fn struct_default_clause_wins_over_type_defaults() {
        let t = Telemetry::default();
        assert_eq!(t.seq, 7);
        assert!(t.valid);
        assert_eq!(t.mode, Mode::Running);
        // Members absent from the clause take their type default.
        assert_eq!(t.samples, Triple::default());
        assert_eq!(t.label.len(), 0);
    }

    #[test]
    fn struct_accessors_match_the_cpp_generated_api() {
        let mut t = Telemetry::default();
        assert_eq!(*t.get_seq(), 7);
        t.set_seq(9);
        assert_eq!(*t.get_seq(), 9);
        assert_eq!(*t.get_mode(), Mode::Running);
        t.set_mode(Mode::Off);
        assert_eq!(*t.get_mode(), Mode::Off);
        assert!(*t.get_valid());
        t.set_valid(false);
        assert!(!t.valid);

        let mut other = Telemetry::default();
        other.set_all(9, false, Mode::Off, t.samples, t.label.clone());
        assert_eq!(other, t);
    }

    #[test]
    fn struct_wire_format_equals_hand_written_member_order() {
        let t = Telemetry::new(0x0102_0304, true, Mode::Standby, Triple::new([1, 2, 3]), {
            let mut l = CmdStringArg::new();
            l.set("hi");
            l
        });
        let mut buf = LinearBuffer::<128>::new();
        assert_eq!(
            t.serialize_to(&mut buf, Endianness::Big),
            SerializeStatus::Ok
        );

        // Hand-written equivalent, members in declaration order, no padding.
        let mut hand = LinearBuffer::<128>::new();
        assert_eq!(hand.serialize_u32_be(0x0102_0304), SerializeStatus::Ok);
        assert_eq!(
            hand.serialize_bool(true, Endianness::Big),
            SerializeStatus::Ok
        );
        assert_eq!(hand.serialize_u16_be(5), SerializeStatus::Ok);
        for v in [1i64, 2, 3] {
            assert_eq!(hand.serialize_i64_be(v), SerializeStatus::Ok);
        }
        assert_eq!(hand.serialize_u16_be(2), SerializeStatus::Ok);
        assert_eq!(
            hand.serialize_bytes(
                b"hi",
                crate::serial::LengthMode::OmitLength,
                Endianness::Big
            ),
            SerializeStatus::Ok
        );
        assert_eq!(buf.as_slice(), hand.as_slice());

        // Variable-length member => serialized_size() is the ACTUAL size,
        // SERIALIZED_SIZE the maximum.
        assert_eq!(t.serialized_size(), 4 + 1 + 2 + 24 + 2 + 2);
        assert!(t.serialized_size() < Telemetry::SERIALIZED_SIZE);
        assert_eq!(buf.as_slice().len(), t.serialized_size());
    }

    #[test]
    fn struct_round_trips() {
        let t = Telemetry::new(42, false, Mode::Off, Triple::new([-1, 0, 1]), {
            let mut l = CmdStringArg::new();
            l.set("label");
            l
        });
        let mut buf = LinearBuffer::<128>::new();
        assert_eq!(
            t.serialize_to(&mut buf, Endianness::Big),
            SerializeStatus::Ok
        );
        let mut out = Telemetry::default();
        assert_eq!(
            out.deserialize_from(&mut buf, Endianness::Big),
            SerializeStatus::Ok
        );
        assert_eq!(out, t);
    }

    #[test]
    fn struct_invalid_nested_enum_leaves_target_unmodified() {
        let mut buf = LinearBuffer::<128>::new();
        assert_eq!(buf.serialize_u32_be(1), SerializeStatus::Ok);
        assert_eq!(
            buf.serialize_bool(true, Endianness::Big),
            SerializeStatus::Ok
        );
        assert_eq!(buf.serialize_u16_be(4), SerializeStatus::Ok); // undeclared Mode
        let mut out = Telemetry::default();
        assert_eq!(
            out.deserialize_from(&mut buf, Endianness::Big),
            SerializeStatus::DeserFormatError
        );
        assert_eq!(out, Telemetry::default(), "commit only on full success");
    }

    #[test]
    fn empty_struct_serializes_to_zero_bytes() {
        let e = Empty::default();
        let mut buf = LinearBuffer::<8>::new();
        assert_eq!(
            e.serialize_to(&mut buf, Endianness::Big),
            SerializeStatus::Ok
        );
        assert_eq!(buf.as_slice(), &[] as &[u8]);
        assert_eq!(e.serialized_size(), 0);
    }
}
