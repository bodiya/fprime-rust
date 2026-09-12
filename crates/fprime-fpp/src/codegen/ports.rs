//! Emission of port definitions as object-safe traits, in the shape the
//! workspace hand-writes in `fprime_comp::port`:
//!
//! ```text
//! pub trait <Name>Port: Send + Sync {
//!     fn invoke(&self, port_num: FwIndexType, <params>) [-> <ret>];
//! }
//! ```
//!
//! Parameter passing: FPP `ref` becomes `&mut T`; a value parameter is by
//! value for `Copy` types and `&T` otherwise; serializable buffers are
//! always `&mut T`; `Fw.Buffer` is always owned (it moves).

use super::{Generator, names};
use crate::analysis::SymId;
use crate::error::Result;

impl Generator<'_, '_> {
    /// `port P(params) -> T`
    pub(super) fn emit_port(&mut self, sym: SymId) -> Result<()> {
        let s = self.a.symbols.sym(sym);
        let docs = s.docs.clone();
        let name = format!("{}Port", s.name);
        let def = self.a.ports[&sym].clone();
        let mut params = Vec::new();
        for p in &def.params {
            params.push(format!(
                "{}: {}",
                names::snake_ident(&p.name),
                self.param_type(p)?
            ));
        }
        let ret = match &def.ret {
            Some(t) => format!(" -> {}", self.rust_type(t)?),
            None => String::new(),
        };
        self.line("");
        self.doc(&docs);
        self.line(&format!(
            "/// FPP port `{}` as an object-safe trait.",
            s.qualified_name()
        ));
        self.line(&format!("pub trait {name}: Send + Sync {{"));
        self.indent();
        for p in &def.params {
            if !p.docs.is_empty() {
                self.line(&format!("/// `{}`: {}", p.name, p.docs.join(" ")));
            }
        }
        if def.params.len() > 6 {
            self.line("#[allow(clippy::too_many_arguments)]");
        }
        let sig = if params.is_empty() {
            format!("fn invoke(&self, port_num: ::fprime_config::FwIndexType){ret};")
        } else {
            format!(
                "fn invoke(&self, port_num: ::fprime_config::FwIndexType, {}){ret};",
                params.join(", ")
            )
        };
        self.line(&sig);
        self.dedent();
        self.line("}");
        Ok(())
    }
}
