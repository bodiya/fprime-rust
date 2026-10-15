@ The `Probe` machine of fprime-comp's tests/sm_traces.rs: an inner guarded
@ transition overriding an outer one, a transition to an ancestor, an
@ internal transition on a typed signal with an untyped guard, nested
@ initial transitions
state machine Probe {

  action a
  action b
  action exitS1
  action exitS2
  action enterS1
  action enterS2

  guard g

  signal s
  signal up
  signal v: U32

  initial enter S1

  state S1 {
    entry do { enterS1 }
    exit do { exitS1 }
    initial enter S2
    on s do { a } enter S3
    state S2 {
      entry do { enterS2 }
      exit do { exitS2 }
      on s if g do { b } enter S3
      on up enter S1
      on v if g do { a }
    }
  }

  state S3 {
    initial enter S4
    state S4 {
      initial enter S5
      state S5
    }
  }

}
