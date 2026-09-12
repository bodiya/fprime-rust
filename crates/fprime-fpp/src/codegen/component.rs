//! Emission of component bases (placeholder until the component back end
//! lands).

use super::Generator;
use crate::analysis::SymId;
use crate::error::Result;

impl Generator<'_, '_> {
    pub(super) fn emit_component(&mut self, sym: SymId) -> Result<()> {
        let name = self.a.symbols.sym(sym).name.clone();
        self.line(&format!("// component {name}: not generated yet"));
        Ok(())
    }
}
