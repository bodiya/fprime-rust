//! The interpreter: a store, its compiled code and its host modules.
//!
//! Port of the `spacewasm_c_api` entry points `Svc::WasmSequencer` calls
//! (`capi.rs`, `host.rs`, `stream.rs`), returning [`Error`] where the C API
//! returns `spacewasm_status_t`.

use alloc::sync::Arc;
use core::marker::PhantomData;
use core::ops::ControlFlow;

use spacewasm::{
    CodeBuilder, CompilerOptions, Engine, ExportDesc, HostFunction, HostFunctionBreak,
    HostFunctionResult, HostModule, HostName, HostValList, InnerVec, Interpreter,
    InterpreterResult, InterpreterRunner, Module, ModuleRef, RawValue, Rc, Ref, ValType, Value,
    Vec as SwVec, WasmMemoryAllocator, WasmRef, WasmStream,
};

use crate::error::Error;
use crate::guest::{GuestAllocator, GuestPool};
use crate::heap::{self, HeapGuard};

/// Maximum control-frame depth per function (the C API's default
/// `MAX_CONTROL_FRAMES`, which upstream's build keeps).
pub const MAX_CONTROL_FRAMES: usize = 64;
/// Maximum operand-stack depth per function during validation (the C API's
/// default `MAX_STACK_DEPTH`).
pub const MAX_STACK_DEPTH: usize = 256;

/// What a host function asks of the interpreter
/// (`spacewasm_hostcall_result_t`).
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum HostCallResult {
    /// Continue without a result (`SPACEWASM_CONTINUE_NONE`).
    ContinueNone,
    /// Continue with this result (`SPACEWASM_CONTINUE_SOME`).
    ContinueSome(Value),
    /// Trap the interpreter (`SPACEWASM_TRAP`).
    Trap,
    /// Pause the interpreter until [`WasmEngine::resume`] /
    /// [`WasmEngine::resume_value`] (`SPACEWASM_PAUSE`).
    Pause,
}

/// The host modules of a store being built (`spacewasm_host_t`):
/// allocated from the heap, so tied to the guard it was created under.
pub struct HostModules<'g> {
    modules: SwVec<HostModule>,
    slot: usize,
    _guard: PhantomData<&'g ()>,
}

impl<'g> HostModules<'g> {
    /// Room for `len` host modules (`spacewasm_host_new`).
    pub fn new(guard: &'g HeapGuard<'_>, len: usize) -> Result<Self, Error> {
        Ok(Self {
            modules: SwVec::new(len).map_err(Error::Alloc)?,
            slot: guard.slot(),
            _guard: PhantomData,
        })
    }

    /// Add a host module named `name` with room for `max_functions`
    /// functions and `max_globals` globals, returning its index
    /// (`spacewasm_add_host_module`).
    pub fn add_module(
        &mut self,
        name: &str,
        max_functions: usize,
        max_globals: usize,
    ) -> Result<u32, Error> {
        let functions = SwVec::new(max_functions).map_err(Error::Alloc)?;
        let globals = SwVec::new(max_globals).map_err(Error::Alloc)?;
        let name = HostName::try_from_str(name).map_err(|_| Error::NameTooLong)?;
        let module = HostModule {
            name,
            globals,
            functions,
            memory: SwVec::zero(),
            table: SwVec::zero(),
        };
        self.modules.try_push(module).map_err(|_| Error::Capacity)?;
        Ok((self.modules.len() - 1) as u32)
    }

    /// Add host function `name` to module `module_idx`, with parameter and
    /// result signatures in the `iIfd` alphabet, implemented by `f`
    /// (`spacewasm_add_host_function`).
    pub fn add_function<F>(
        &mut self,
        module_idx: u32,
        name: &str,
        params_sig: &str,
        returns_sig: &str,
        f: F,
    ) -> Result<(), Error>
    where
        F: Fn(&[Value]) -> HostCallResult + Send + 'static,
    {
        let name = HostName::try_from_str(name).map_err(|_| Error::NameTooLong)?;
        let params = HostValList::try_new(params_sig).map_err(Error::HostFunction)?;
        let returns = HostValList::try_new(returns_sig).map_err(Error::HostFunction)?;
        let host_fn = HostFunction::try_new(
            name,
            params,
            returns,
            move |_state: &mut Engine, args: &[Value]| -> HostFunctionResult {
                match f(args) {
                    HostCallResult::ContinueNone => ControlFlow::Continue(None),
                    HostCallResult::ContinueSome(v) => ControlFlow::Continue(Some(v)),
                    HostCallResult::Trap => ControlFlow::Break(HostFunctionBreak::Trap),
                    HostCallResult::Pause => ControlFlow::Break(HostFunctionBreak::Pause),
                }
            },
        )
        .map_err(Error::HostFunction)?;
        match self.modules.get_mut(module_idx as usize) {
            Some(m) => m.functions.try_push(host_fn).map_err(|_| Error::Capacity),
            None => Err(Error::NotFound),
        }
    }
}

/// The C API's `CEngine`: the engine and its compiled code.
struct Inner {
    engine: Engine,
    code_builder: CodeBuilder,
}

/// A store and its interpreter (the C API's `spacewasm_t` handle plus the
/// store's guest-memory allocator handle).
///
/// Everything it owns was allocated from one [`crate::Heap`]: the methods
/// that allocate take that heap's [`HeapGuard`], and dropping it makes the
/// heap current for the duration (waiting for the selection if another
/// heap holds it).
pub struct WasmEngine {
    inner: Option<spacewasm::Box<Inner>>,
    guest: Option<Rc<dyn WasmMemoryAllocator>>,
    slot: usize,
}

// SAFETY: spacewasm's `Engine` is `!Send` only because it is built on
// spacewasm's single-threaded `Rc` (and raw pointers into the heap and guest
// pools). Every `Rc` it holds points into the same object graph, which this
// value owns exclusively and moves as a whole: no method returns an `Rc`, a
// `&Engine` or a `&mut Engine` (host functions only see argument values), so
// no clone of an inner `Rc` can be left behind on another thread. The host
// functions it stores are required to be `Send`, the guest allocator is
// `Send + Sync`, and the pools outlive it.
#[allow(unsafe_code)]
unsafe impl Send for WasmEngine {}

impl core::fmt::Debug for WasmEngine {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        f.debug_struct("WasmEngine")
            .field("slot", &self.slot)
            .finish()
    }
}

/// A [`WasmStream`] over a pull callback that fills one chunk buffer, as
/// the C API's `CallbackStream` over upstream's `WasmFileReader` (whose
/// `LOAD_READ_CHUNK_SIZE` buffer lives on the caller's stack).
struct ChunkStream<'r, const CHUNK: usize> {
    buf: [u8; CHUNK],
    read: &'r mut dyn FnMut(&mut [u8]) -> Result<usize, ()>,
    errored: bool,
}

impl<const CHUNK: usize> WasmStream for ChunkStream<'_, CHUNK> {
    fn read(&mut self) -> Result<Option<InnerVec<u8>>, u8> {
        match (self.read)(&mut self.buf) {
            Err(()) => {
                self.errored = true;
                // Non-zero code signals a reader error to the decoder.
                Err(1)
            }
            Ok(0) => Ok(None),
            Ok(len) => {
                let len = len.min(CHUNK);
                // SAFETY: `buf` holds `len` initialized bytes and outlives
                // the chunk: spacewasm's reader returns a chunk before it
                // asks for the next one (the contract the C API's
                // `spacewasm_read_fn_t` documents), and the stream outlives
                // the load. `capacity` 0 marks the chunk as borrowed, so it
                // is never freed.
                #[allow(unsafe_code)]
                let chunk = unsafe { InnerVec::from_raw_parts(self.buf.as_mut_ptr(), 0, len) };
                Ok(Some(chunk))
            }
        }
    }

    fn return_(&mut self, _chunk: InnerVec<u8>) {
        // Borrowed chunk: nothing to free.
    }
}

impl WasmEngine {
    /// Build a store with the given host modules, a `stack_size`-word
    /// operand stack and room for `max_modules` guest modules, compiling
    /// with `options` (`spacewasm_new`), then its guest-memory allocator
    /// over `guest` (`spacewasm_allocator_new`), in upstream's order.
    pub fn new(
        guard: &HeapGuard<'_>,
        host: HostModules<'_>,
        stack_size: usize,
        max_modules: usize,
        options: CompilerOptions,
        guest: &Arc<GuestPool>,
    ) -> Result<WasmEngine, Error> {
        assert!(host.slot == guard.slot(), "host modules from another heap");
        if max_modules > 256 {
            return Err(Error::VecTooLong);
        }
        let engine = Engine::new(stack_size, max_modules, host.modules).map_err(Error::Memory)?;
        let code_builder = CodeBuilder::new(options).map_err(Error::Alloc)?;
        let inner = spacewasm::Box::new(Inner {
            engine,
            code_builder,
        })
        .map_err(Error::Alloc)?;
        let mut this = WasmEngine {
            inner: Some(inner),
            guest: None,
            slot: guard.slot(),
        };
        let allocator = Rc::new(GuestAllocator {
            pool: guest.clone(),
        })
        .map_err(Error::Alloc)?;
        this.guest = Some(allocator.into_wasm_memory_allocator());
        Ok(this)
    }

    fn inner(&self) -> &Inner {
        match &self.inner {
            Some(inner) => inner,
            None => panic!("engine already destroyed"),
        }
    }

    fn inner_mut(&mut self) -> &mut Inner {
        match &mut self.inner {
            Some(inner) => inner,
            None => panic!("engine already destroyed"),
        }
    }

    /// Destroy the store under its heap's guard (`spacewasm_destroy`, then
    /// `spacewasm_allocator_destroy`).
    pub fn destroy(mut self, guard: &HeapGuard<'_>) {
        assert!(
            self.slot == guard.slot(),
            "engine destroyed under another heap"
        );
        self.release();
    }

    fn release(&mut self) {
        drop(self.inner.take());
        drop(self.guest.take());
    }

    /// Stream a module named `name` into the store, pulling its bytes in
    /// `CHUNK`-byte pieces from `read` (which returns the number of bytes
    /// written, 0 at the end, or `Err` on an I/O error), and return its
    /// index (`spacewasm_load_module`). Does not run its start function.
    pub fn load_module<const CHUNK: usize>(
        &mut self,
        guard: &HeapGuard<'_>,
        name: &str,
        read: &mut dyn FnMut(&mut [u8]) -> Result<usize, ()>,
    ) -> Result<u32, Error> {
        assert!(
            self.slot == guard.slot(),
            "module loaded under another heap"
        );
        let alloc = match &self.guest {
            Some(guest) => guest.clone(),
            None => panic!("engine already destroyed"),
        };
        let inner = self.inner_mut();
        if !inner.engine.is_idle() {
            return Err(Error::WrongState);
        }
        let mut stream = ChunkStream::<CHUNK> {
            buf: [0; CHUNK],
            read,
            errored: false,
        };
        let module = match Module::new::<MAX_CONTROL_FRAMES, MAX_STACK_DEPTH>(
            name,
            &mut stream,
            &mut inner.engine.store,
            &mut inner.code_builder,
            alloc,
        ) {
            Ok(m) => m,
            // An I/O error from the reader wins over the parse failure it
            // causes.
            Err(_) if stream.errored => return Err(Error::ReaderError),
            Err(e) => return Err(Error::Parse(e)),
        };
        let module_ref = inner
            .engine
            .push_module(module)
            .map_err(|_| Error::Capacity)?;
        Ok(u32::from(module_ref.0))
    }

    /// The index of the first module named `name` (`spacewasm_find_module`).
    pub fn find_module(&self, name: &str) -> Result<u32, Error> {
        self.inner()
            .engine
            .store
            .modules()
            .iter()
            .position(|m| m.name == name)
            .map(|i| i as u32)
            .ok_or(Error::NotFound)
    }

    /// The function index of the function module `module_idx` exports as
    /// `name` (`spacewasm_find_export_func`).
    pub fn find_export_func(&self, module_idx: u32, name: &str) -> Result<u32, Error> {
        let module = self
            .inner()
            .engine
            .store
            .modules()
            .get(module_idx as usize)
            .ok_or(Error::NotFound)?;
        for e in &module.exports {
            if e.name == name {
                if let ExportDesc::Func(fi) = e.desc {
                    return match module.get_func_ref(fi) {
                        Some(Ref::Module(idx)) => Ok(u32::from(idx)),
                        _ => Err(Error::NotFound),
                    };
                }
            }
        }
        Err(Error::NotFound)
    }

    /// The (module index, function index) of module `module_idx`'s start
    /// function (`spacewasm_module_start`).
    pub fn module_start(&self, module_idx: u32) -> Result<(u32, u32), Error> {
        let engine = &self.inner().engine;
        if module_idx as usize >= engine.store.modules().len() {
            return Err(Error::NotFound);
        }
        match engine.module_start(ModuleRef(module_idx as u8)) {
            Some(start) => Ok((u32::from(start.module.0), u32::from(start.index))),
            None => Err(Error::NotFound),
        }
    }

    /// Check that function `func_idx` of module `module_idx` has the
    /// signature `params_sig -> returns_sig` (`spacewasm_check_func_signature`).
    pub fn check_func_signature(
        &self,
        module_idx: u32,
        func_idx: u32,
        params_sig: &str,
        returns_sig: &str,
    ) -> Result<(), Error> {
        let params = HostValList::try_new(params_sig).map_err(Error::HostFunction)?;
        let returns = HostValList::try_new(returns_sig).map_err(Error::HostFunction)?;
        let module = self
            .inner()
            .engine
            .store
            .modules()
            .get(module_idx as usize)
            .ok_or(Error::NotFound)?;
        let func = module
            .functions
            .get(func_idx as usize)
            .ok_or(Error::NotFound)?;
        let ty = module
            .types
            .get(func.ty.0 as usize)
            .ok_or(Error::NotFound)?;
        if params.len() != ty.params.len() || returns.len() != ty.returns.len() {
            return Err(Error::ParamLenMismatch);
        }
        if params.as_slice() != &ty.params[..] || returns.as_slice() != &ty.returns[..] {
            return Err(Error::ParamTypeMismatch);
        }
        Ok(())
    }

    /// The index (in the module's own globals) of the global module
    /// `module_idx` exports as `name` (`spacewasm_find_global`).
    pub fn find_global(&self, module_idx: u32, name: &str) -> Result<u32, Error> {
        let module = self
            .inner()
            .engine
            .store
            .modules()
            .get(module_idx as usize)
            .ok_or(Error::NotFound)?;
        for e in &module.exports {
            if e.name == name {
                if let ExportDesc::Global(gi) = e.desc {
                    return match module.get_global_ref(gi) {
                        Some(Ref::Module(idx)) => Ok(u32::from(idx)),
                        _ => Err(Error::NotFound),
                    };
                }
            }
        }
        Err(Error::NotFound)
    }

    /// Read global `global_idx` of module `module_idx`
    /// (`spacewasm_get_global`).
    pub fn get_global(&self, module_idx: u32, global_idx: u32) -> Result<Value, Error> {
        let module = self
            .inner()
            .engine
            .store
            .modules()
            .get(module_idx as usize)
            .ok_or(Error::NotFound)?;
        let global = module
            .globals
            .get(global_idx as usize)
            .ok_or(Error::NotFound)?;
        Ok(global.value())
    }

    /// Write `value` into global `global_idx` of module `module_idx`, which
    /// must be mutable and of the value's type (`spacewasm_set_global`).
    pub fn set_global(
        &mut self,
        module_idx: u32,
        global_idx: u32,
        value: Value,
    ) -> Result<(), Error> {
        let module = self
            .inner_mut()
            .engine
            .store
            .modules_mut()
            .get_mut(module_idx as usize)
            .ok_or(Error::NotFound)?;
        let global = module
            .globals
            .get_mut(global_idx as usize)
            .ok_or(Error::NotFound)?;
        let (ty, raw) = match value {
            Value::I32(i) => (ValType::I32, RawValue::from_i32(i)),
            Value::I64(i) => (ValType::I64, RawValue::from_i64(i)),
            Value::F32(f) => (ValType::F32, RawValue::from_f32(f)),
            Value::F64(f) => (ValType::F64, RawValue::from_f64(f)),
        };
        if ty != global.type_.ty {
            return Err(Error::GlobalTypeMismatch);
        }
        if !global.type_.mutable {
            return Err(Error::GlobalNotMutable);
        }
        global.value = raw;
        Ok(())
    }

    /// Set up a call of function `func_idx` of module `module_idx` with
    /// `params` (`spacewasm_invoke`); [`Self::run`] executes it.
    pub fn invoke(
        &mut self,
        module_idx: u32,
        func_idx: u32,
        params: &[Value],
    ) -> Result<(), Error> {
        if func_idx > u32::from(u16::MAX) {
            return Err(Error::BadArg);
        }
        let engine = &mut self.inner_mut().engine;
        if !engine.is_idle() {
            return Err(Error::WrongState);
        }
        let module = engine
            .store
            .modules()
            .get(module_idx as usize)
            .ok_or(Error::NotFound)?;
        if func_idx as usize >= module.functions.len() {
            return Err(Error::NotFound);
        }
        if params.len() > 64 {
            return Err(Error::Capacity);
        }
        let f_ref = WasmRef {
            module: ModuleRef(module_idx as u8),
            index: func_idx as u16,
        };
        engine.invoke(f_ref, params).map_err(Error::Invoke)
    }

    /// Run the pending call for up to `fuel` instructions
    /// (`spacewasm_run`). `None` when no call is pending (the C API reports
    /// that as a trap with `SPACEWASM_TRAP_NONE`).
    pub fn run(&mut self, fuel: usize) -> Option<InterpreterResult> {
        let inner = self.inner_mut();
        if inner.engine.is_idle() {
            return None;
        }
        Some(Interpreter.run(inner.code_builder.pages(), &mut inner.engine, fuel))
    }

    /// Resume after a host pause, with no result (`spacewasm_resume`).
    pub fn resume(&mut self) -> Result<(), Error> {
        self.inner_mut().engine.resume(None).map_err(Error::Resume)
    }

    /// Resume after a host pause with the host function's result
    /// (`spacewasm_resume_value`).
    pub fn resume_value(&mut self, value: Value) -> Result<(), Error> {
        self.inner_mut()
            .engine
            .resume(Some(value))
            .map_err(Error::Resume)
    }

    /// Abandon any call and return to idle (`spacewasm_reset`).
    pub fn reset(&mut self) {
        self.inner_mut().engine.reset();
    }

    /// The result of the last completed call as `expected`
    /// (`spacewasm_get_result`).
    pub fn get_result(&self, expected: ValType) -> Result<Value, Error> {
        self.inner()
            .engine
            .result
            .map(|raw| raw.to_value(expected))
            .ok_or(Error::NotFound)
    }

    /// Copy `dst.len()` bytes of the current module's linear memory at
    /// `addr` into `dst` (`spacewasm_mem_read`).
    pub fn mem_read(&self, addr: u32, dst: &mut [u8]) -> Result<(), Error> {
        let src = self
            .inner()
            .engine
            .memory
            .load(addr as usize, dst.len())
            .map_err(Error::Memory)?;
        dst.copy_from_slice(src);
        Ok(())
    }

    /// Write `src` into the current module's linear memory at `addr`
    /// (`spacewasm_mem_write`).
    pub fn mem_write(&mut self, addr: u32, src: &[u8]) -> Result<(), Error> {
        self.inner()
            .engine
            .memory
            .store(addr as usize, src)
            .map_err(Error::Memory)
    }
}

impl Drop for WasmEngine {
    fn drop(&mut self) {
        if self.inner.is_some() || self.guest.is_some() {
            let slot = self.slot;
            heap::with_current(slot, || self.release());
        }
    }
}
