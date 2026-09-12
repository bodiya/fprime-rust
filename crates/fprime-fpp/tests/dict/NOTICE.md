The `.fpp` inputs and `.ref.json` expected outputs in this directory are
copied unchanged from the FPP compiler's `fpp-to-dict` test corpus
(https://github.com/nasa/fpp, `compiler/tools/fpp-to-dict/test/top`,
Apache-2.0) so that `tests/dictionary.rs` can check this crate's JSON
dictionary back end against the reference compiler's output without a
checkout of the Scala tree.
