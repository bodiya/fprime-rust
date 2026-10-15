# Upstream declares this state machine inside `active component
# FpySequencer` (Svc/FpySequencer/FpySequencer.fpp); a module of the same
# name keeps the qualified names (Svc.FpySequencer.SequencerStateMachine)
# without the rest of the component.
module Svc {

  module FpySequencer {

    include "FpySequencerStateMachine.fppi"

  }

}
