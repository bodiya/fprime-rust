The `.fpp` files in this directory are copied from NASA's F Prime
(https://github.com/nasa/fprime, Apache-2.0) so that the demo model resolves
against the real framework definitions without a checkout of the C++ tree.

`../demo/SignalGen.fpp` and its `.fppi` includes are the upstream
`Ref/SignalGen` model, copied unchanged so the generator is exercised on a
real production component.

`AcConstants.fpp` (`default/config`), `ToCpp.fpp` (`Fpp/`, the
`Fpp.ToCpp.Phases` init specifiers name) and the `Svc.TlmPacketizer` model
(`TlmPacketizer.fpp`, `TlmPacketizerCfg.fpp`, `TlmPacketizerTypes.fpp`,
`TlmPacketizerPorts.fpp`) are copied unchanged as well; the `Ops` topology
instantiates the hand-written Rust `TlmPacketizer` through that model.
