//! Emission of topologies (placeholder until the topology back end lands).

use super::Generator;
use crate::analysis::SymId;
use crate::error::Result;

impl Generator<'_, '_> {
    pub(super) fn emit_topology(&mut self, sym: SymId) -> Result<()> {
        let name = self.a.symbols.sym(sym).name.clone();
        self.line(&format!("// topology {name}: not generated yet"));
        Ok(())
    }
}
