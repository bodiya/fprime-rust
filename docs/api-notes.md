# Foundation crate API notes

Authoritative quick reference to the public APIs of the foundation crates,
written by their implementers. The source code is the final word; this exists
so component implementers do not have to re-derive signatures.

## fprime-config, fprime-fw

```text
CRATE fprime_config (all pub at root unless noted):
  type FwSizeType=u64; FwSignedSizeType=i64; FwIndexType=i16; FwAssertArgType=i32; FwIdType=u32; FwOpcodeType=FwChanIdType=FwEventIdType=FwPrmIdType=FwDpIdType=FwIdType; FwDpPriorityType=u32; FwEnumStoreType=i32; FwSizeStoreType=u16; FwBuffSizeType=FwSizeStoreType; FwPacketDescriptorType=u16; FwTimeBaseStoreType=u16; FwTimeContextStoreType=u8; FwTlmPacketizeIdType=u16; FwTraceIdType=u32; FwQueuePriorityType=u8; FwTaskPriorityType=u8; FwTaskIdType=i32
  const (usize unless noted): FW_COM_BUFFER_MAX_SIZE=512; FW_CMD_ARG_BUFFER_MAX_SIZE=FW_LOG_BUFFER_MAX_SIZE=FW_TLM_BUFFER_MAX_SIZE=FW_PARAM_BUFFER_MAX_SIZE=506; FW_CMD_STRING_MAX_SIZE=40; FW_LOG_STRING_MAX_SIZE=200; FW_TLM_STRING_MAX_SIZE=40; FW_PARAM_STRING_MAX_SIZE=40; FW_LOG_TEXT_BUFFER_SIZE=256; FW_FIXED_LENGTH_STRING_SIZE=256; FW_OBJ_NAME_BUFFER_SIZE=80; FW_QUEUE_NAME_BUFFER_SIZE=80; FW_TASK_NAME_BUFFER_SIZE=80; FW_SERIALIZE_TRUE_VALUE:u8=0xFF; FW_SERIALIZE_FALSE_VALUE:u8=0x00; FW_ASSERT_TEXT_SIZE=256; FILE_NAME_STRING_SIZE=240; FW_CONTEXT_DONT_CARE:u8=0xFF; CMD_DISPATCHER_COMMAND_PORTS=30; CMD_DISPATCHER_SEQUENCE_PORTS=5; RATE_GROUP_MEMBER_OUT_PORTS=10; RATE_GROUP_DRIVER_CYCLE_PORTS=3; HEALTH_PING_PORTS=25
  mod cmd_dispatcher { DISPATCH_TABLE_SIZE:usize=150; SEQUENCER_TABLE_SIZE:usize=25; INCLUDE_COMMAND_OPCODES_IN_EVENTS:bool=true }
  mod event_manager { FILTER_WARNING_HI_DEFAULT/WARNING_LO/COMMAND/ACTIVITY_HI/ACTIVITY_LO_DEFAULT:bool=true; FILTER_DIAGNOSTIC_DEFAULT:bool=false; ID_FILTER_SIZE:usize=25 }
  mod tlm_chan { NUM_TLM_HASH_SLOTS:usize=15; HASH_MOD_VALUE:u32=99; HASH_BUCKETS:usize=500; MAX_ENTRIES_PER_RUN:usize=500 }
  mod active_rate_group { OVERRUN_THROTTLE:u32=5 }
  mod buffer_manager { MAX_NUM_BINS:usize=10 }
  mod com_queue { COM_PORTS:usize=2; BUFFER_PORTS:usize=1 }

CRATE fprime_fw. Everything below is re-exported at crate root (fprime_fw::Time etc.); modules: serial, string, time, enums, com, buffer, packets, poly_type, assert, logger, fpp. Also `pub use fprime_config as config` (fw_assert! expansion needs this path; depend on fprime-fw and it resolves).

fpp (codegen layer; macros exported at the crate root — see "Codegen layer (FPP-style macros) - usage" below):
  macros fpp_enum! / fpp_struct! / fpp_array!; trait FppSized { const SERIALIZED_SIZE: usize } (impl'd for u8..i64, f32, f64, bool, Time, TimeInterval, FwString<N>, and every macro-generated type)

serial:
  #[must_use] #[repr(i32)] enum SerializeStatus { Ok=0, FormatError=1, NoRoomLeft=2, DeserBufferEmpty=3, DeserFormatError=4, DeserSizeMismatch=5, DeserTypeMismatch=6, DeserImmutable=7, DeserInvalidData=8, DiscardedExisting=9 }; fn is_ok(self)->bool
  enum Endianness { #[default] Big, Little }; enum LengthMode { #[default] IncludeLength, OmitLength }
  macro fw_try!(expr) — statement; early-returns non-Ok from a fn returning SerializeStatus
  trait SerBufAny { fn bytes(&self)->&[u8]; fn bytes_mut(&mut self)->&mut [u8]; fn capacity(&self)->usize; fn ser_loc(&self)->usize; fn set_ser_loc(&mut self,usize); fn deser_loc(&self)->usize; fn set_deser_loc(&mut self,usize); fn as_ser_buf_any(&mut self)->&mut dyn SerBufAny }
  trait SerBuf: SerBufAny (blanket impl for ALL T: SerBufAny + ?Sized, incl. dyn SerBufAny — just `use fprime_fw::SerBuf`):
    for each T in u8,i8,u16,i16,u32,i32,u64,i64,f32,f64,bool: fn serialize_T(&mut self, v:T, e:Endianness)->SerializeStatus; fn serialize_T_be(&mut self, v:T)->SerializeStatus; fn deserialize_T(&mut self, v:&mut T, e:Endianness)->SerializeStatus; fn deserialize_T_be(&mut self, v:&mut T)->SerializeStatus
    fn serialize_bytes(&mut self, data:&[u8], mode:LengthMode, e:Endianness)->SerializeStatus
    fn deserialize_bytes(&mut self, dest:&mut [u8], len:&mut usize, mode:LengthMode, e:Endianness)->SerializeStatus  // len in/out like C++ (in: max/exact, out: stored)
    fn serialize_buffer(&mut self, val:&dyn SerBufAny, e:Endianness)->SerializeStatus  // [u16 size][bytes], pre-checked
    fn deserialize_buffer(&mut self, val:&mut dyn SerBufAny, e:Endianness)->SerializeStatus
    fn serialize_size(&mut self, size:FwSizeType, e:Endianness)->SerializeStatus  // range-checks -> FormatError
    fn deserialize_size(&mut self, size:&mut FwSizeType, e:Endianness)->SerializeStatus
    fn serialize_skip(&mut self, n:usize)->SerializeStatus; fn deserialize_skip(&mut self, n:usize)->SerializeStatus
    fn move_ser_to_offset(&mut self, o:usize)->SerializeStatus; fn move_deser_to_offset(&mut self, o:usize)->SerializeStatus
    fn reset_ser(&mut self); fn reset_deser(&mut self); fn get_size(&self)->usize; fn deserialize_size_left(&self)->usize; fn serialize_size_left(&self)->usize
    fn set_buff(&mut self, src:&[u8])->SerializeStatus; fn set_buff_len(&mut self, len:usize)->SerializeStatus
    fn copy_raw(&mut self, dest:&mut dyn SerBufAny, size:usize)->SerializeStatus  // replaces dest
    fn copy_raw_offset(&mut self, dest:&mut dyn SerBufAny, size:usize)->SerializeStatus  // appends
    fn serialize(&mut self, val:&dyn Serialize, e:Endianness)->SerializeStatus; fn deserialize(&mut self, val:&mut dyn Deserialize, e:Endianness)->SerializeStatus
    fn as_slice(&self)->&[u8] /*[0..ser_loc]*/; fn remaining_slice(&self)->&[u8] /*[deser..ser]*/
  trait Serialize { fn serialize_to(&self, buf:&mut dyn SerBufAny, e:Endianness)->SerializeStatus; fn serialized_size(&self)->usize }  // impl'd for all primitives+bool and all types below
  trait Deserialize { fn deserialize_from(&mut self, buf:&mut dyn SerBufAny, e:Endianness)->SerializeStatus }
  struct LinearBuffer<const N:usize> (Clone, Default, PartialEq/Eq on content): const fn new(); assoc const SERIALIZED_SIZE=N+2; impls SerBufAny+Serialize+Deserialize
  struct ExtBuf<'a>: fn new(data:&'a mut [u8])->Self /*empty*/; fn with_len(data:&'a mut [u8], len:usize)->Self /*len valid, readable*/; impls SerBufAny
  type ComBuffer=LinearBuffer<512>; CmdArgBuffer/LogBuffer/TlmBuffer/ParamBuffer=LinearBuffer<506>

string (C++ parity: the Endianness parameters are IGNORED — the u16 length prefix is always big-endian, as ConstStringBase/StringBase never forward their mode; content is C-string based — set/append/deserialize commit only bytes before the first NUL, like the C++ NUL-scan length()):
  struct FwString<const N:usize> (Clone, Default, PartialEq/Eq, PartialEq<&str>, Display, fmt::Write, From<&str>, Serialize, Deserialize): const SERIALIZED_SIZE=N+2; const fn new(); fn set(&mut self,&str); set_bytes(&mut self,&[u8]); append(&mut self,&str); append_bytes; clear; len()->usize; is_empty; const fn max_length()->usize /*==N*/; as_bytes()->&[u8]; as_str()->Option<&str>; format(&mut self, fmt::Arguments); serialized_size()->usize; serialize_to_truncated(&self, buf:&mut dyn SerBufAny, max_len:usize, e:Endianness)->SerializeStatus; serialized_truncated_size(&self, max:usize)->usize
  type ObjectName=FwString<80>; CmdStringArg=FwString<40>; LogStringArg=FwString<200>; TextLogString=FwString<256>; TlmString=FwString<40>; ParamString=FwString<40>; FileNameString=FwString<240>; FwDefaultString=FwString<256>

time:
  #[repr(u16)] enum TimeBase { TbNone=0, TbProcTime=1, TbWorkstationTime=2, TbScTime=3, TbDontCare=0xFFFF } (Default=TbNone, TryFrom<u16>, Serialize/Deserialize u16-width validated)
  #[repr(i32)] enum TimeComparison { Lt=-1, Eq=0, Gt=1, Incomparable=2 }
  struct Time (Copy, Default=ZERO, PartialEq/PartialOrd via compare, Serialize, Deserialize): const SERIALIZED_SIZE=11; const ZERO:Time; fn new(TimeBase, u8, u32 sec, u32 usec)->Time /*asserts usec<1e6*/; from_seconds_useconds(u32,u32)->Time /*forces TbNone,ctx 0*/; zero(TimeBase)->Time; set(&mut self, u32, u32) /*preserves base+ctx*/; set_time_base(&mut self, TimeBase); set_time_context(&mut self, u8); get_seconds/get_useconds()->u32; get_time_base()->TimeBase; get_context()->u8; compare(&Time,&Time)->TimeComparison; add(&Time,&Time)->Time; sub(minuend:&Time, subtrahend:&Time)->Time; add_duration(&mut self, u32, u32); add_f64(&mut self, f64); from_f64(f64)->Time; to_f64(&self)->f64
  struct TimeInterval (Copy, Default, Eq, Ord, Serialize, Deserialize): const SERIALIZED_SIZE=8; fn new(u32 sec, u32 usec)->Self /*asserts usec<1e6*/; between(start:&Time, end:&Time)->Self; set(&mut self,u32,u32); get_seconds/get_useconds()->u32; compare(&Self,&Self)->TimeComparison; add(&Self,&Self)->Self; sub(&Self,&Self)->Self /*commutative absolute*/

enums (all #[repr(u8)], Copy, Eq, Hash, Default=listed first unless noted, TryFrom<u8>, Serialize/Deserialize at u8 width, invalid decode->DeserFormatError leaving target unmodified):
  Success{Failure=0,Success=1}; Enabled{Disabled=0,Enabled=1}; Wait{Wait=0,NoWait=1}; Completed{Completed=0,Canceled=1,Failed=2}; Health{Healthy=0,Sick=1,Failed=2}; CmdResponse{Ok=0,InvalidOpcode=1,ValidationError=2,FormatError=3,ExecutionError=4,Busy=5}; LogSeverity{Fatal=1,WarningHi=2,WarningLo=3,Command=4,ActivityHi=5,ActivityLo=6,Diagnostic=7}; ParamValid{Uninit=0,Valid=1,Invalid=2,Default=3} + fn is_ok(self)->bool; TlmValid{Valid=0,Invalid=1}; DeserialStatus{Ok=0,BufferEmpty=3,FormatError=4,SizeMismatch=5,TypeMismatch=6}

com:
  #[repr(u16)] enum ComPacketType { FwPacketCommand=0, FwPacketTelem=1, FwPacketLog=2, FwPacketFile=3, FwPacketPacketizedTlm=4, FwPacketDp=5, FwPacketIdle=6, FwPacketParam=7, FwPacketHand=0xFE, FwPacketUnknown=0xFF, SppIdlePacket=0x7FF, InvalidUninitialized=0x800 } (Default=InvalidUninitialized, TryFrom<u16>, Serialize/Deserialize u16-width); type Apid=ComPacketType
  #[repr(u8)] enum Pvn { SpacePacketProtocol=0, EncapsulationPacketProtocol=7, InvalidUninitialized=8 } (Default=InvalidUninitialized)
  const SA_INDEX_UNSET:u16=0xFFFF
  struct FrameContext { pub com_queue_index:FwIndexType, pub apid:Apid, pub has_sec_hdr:bool, pub sequence_flags:u8, pub sequence_count:u16, pub vc_id:u8, pub pvn:Pvn, pub send_now:bool, pub sa_index:u16 } (Copy, Eq, Default per ComCfg.fpp, Serialize/Deserialize, const SERIALIZED_SIZE=13, plus get_*/set_* accessor pairs and new()/set_all() — declared with fpp_struct!)

buffer:
  type BufferStorage = Box<[u8]>
  struct Buffer (Default=empty): const NO_CONTEXT:u32=0xFFFF_FFFF; fn empty()->Buffer; allocate(size:usize)->Buffer; from_storage(BufferStorage, context:u32)->Buffer; is_valid(&self)->bool; capacity/size/offset(&self)->usize; context(&self)->u32; set_context(&mut self,u32); advance(&mut self, amount:FwSignedSizeType) /*fw_asserts*/; set_size(&mut self, usize) /*fw_asserts*/; data(&self)->&[u8]; data_mut(&mut self)->&mut [u8]; get_serializer(&mut self)->ExtBuf<'_> /*empty, resetSer*/; get_deserializer(&mut self)->ExtBuf<'_> /*whole window readable*/; into_storage(self)->BufferStorage

packets (C++ parity: the leading u16 packet descriptor is always big-endian — ComPacket::serializeBase/deserializeBase never forward the endianness mode; only the fields after it honor it):
  struct CmdPacket (Default, Deserialize ONLY): fn new(); get_opcode()->FwOpcodeType; get_arg_buffer()->&CmdArgBuffer; get_arg_buffer_mut()->&mut CmdArgBuffer
  struct LogPacket (Default, Serialize+Deserialize): fn new(); set_id(FwEventIdType); set_time_tag(Time); set_log_buffer(&LogBuffer); get_id()->FwEventIdType; get_time_tag()->&Time; get_log_buffer()->&LogBuffer; get_log_buffer_mut()->&mut LogBuffer
  struct TlmPacket (Default; Serialize/Deserialize = the NON-GDS u64-numEntries format; accumulator methods are BE-only like the defaulted C++ calls): fn new(); reset_pkt_ser(&mut self)->SerializeStatus; reset_pkt_deser(&mut self)->SerializeStatus; get_num_entries(&self)->FwSizeType; get_buffer(&self)->&ComBuffer; get_buffer_mut(&mut self)->&mut ComBuffer; set_buffer(&mut self, ComBuffer); add_value(&mut self, id:FwChanIdType, time_tag:&Time, buffer:&TlmBuffer)->SerializeStatus; extract_value(&mut self, id:&mut FwChanIdType, time_tag:&mut Time, buffer:&mut TlmBuffer, buffer_size:usize)->SerializeStatus /*DeserBufferEmpty at end*/

poly_type:
  enum PolyType (Copy, PartialEq, Default=NoType, Serialize+Deserialize) { NoType, U8(u8), I8(i8), U16(u16), I16(i16), U32(u32), I32(i32), U64(u64), I64(i64), F32(f32), F64(f64), Bool(bool), Ptr(u64) }: const SERIALIZED_SIZE=12; fn wire_tag(&self)->FwEnumStoreType /*0..=12*/

assert:
  trait AssertHook: Send+Sync { fn report_assert(&self, file:&str, line:u32, args:&[FwAssertArgType]) /*default: format+print*/; fn print_assert(&self, msg:&str) /*default eprintln*/; fn do_assert(&self) /*default panic*/ }
  fn register_assert_hook(Box<dyn AssertHook>)->Option<Box<dyn AssertHook>>; fn deregister_assert_hook()->Option<Box<dyn AssertHook>>; fn format_assert_msg(file:&str, line:u32, args:&[FwAssertArgType])->String; fn assert_failure(file:&str, line:u32, args:&[FwAssertArgType]) /*NOT -> ! : returns if a hook's do_assert returns, C++ parity*/
  macro fw_assert!(cond); fw_assert!(cond, a1, ..., a6) — args are cast `as FwAssertArgType` in the expansion

logger:
  trait FwLogger: Send+Sync { fn write_message(&self, message:&str) }
  fn register_logger(&'static dyn FwLogger); fn deregister_logger(); fn log_message(&str) /*drops if unregistered, truncates to 256*/
  macro fw_log!(fmt, args...) — printf-style via format!
```

### Implementation notes / deviations

Deviations from ARCHITECTURE.md / C++ (all recorded in doc comments):
1. fw_try! expands to a statement (unit-valued match), not an expression returning Ok — avoids unused-must_use noise; semantics identical for the C++ status-ladder pattern.
2. SerBuf is object-safe and blanket-implemented over SerBufAny (impl<T: SerBufAny + ?Sized> SerBuf for T), so `&mut dyn SerBufAny` gets every codec method after `use fprime_fw::SerBuf`. SerBufAny carries one extra required method, as_ser_buf_any(), enabling generic serialize/deserialize from provided methods; concrete impls just return self.
3. PolyType::deserialize_from leaves self fully unmodified on an unknown tag or short value; C++ overwrites m_dataType with the unvalidated tag before rejecting — unrepresentable in a Rust tagged enum (documented on the impl). FrameContext deserialize likewise commits via a temporary.
4. assert_failure is not `-> !`: like C++, a registered hook whose do_assert returns lets execution continue past the failed fw_assert!. Default path (no hook) always panics with the formatted message. C++ fail-stop parity comes from the workspace release profile's `panic = "abort"` (a release-build fw_assert aborts the whole process like the C++ `assert(false)`); dev/test builds unwind so `#[should_panic]` tests work.
5. TimeComparison is a plain repr(i32) enum without Serialize (not a wire type in phase 1).
6. TlmPacket::add_value/extract_value/reset_pkt_* are big-endian only, matching the endianness-defaulted C++ calls; the trait Serialize/Deserialize on TlmPacket is the non-GDS [u64 numEntries][raw internal buffer] format with numEntries fixed at the u64 FwSizeType width (this is a real byte-format commitment noted in fw-services.md).
7. fprime-config extras beyond the task list: FwBuffSizeType/FwTraceIdType aliases, FW_CONTEXT_DONT_CARE, FW_ASSERT_TEXT_SIZE, FILE_NAME_STRING_SIZE — all needed by fprime-fw and verified against the C++ config; event_manager::ID_FILTER_SIZE is the C++ TELEM_ID_FILTER_SIZE per the task's naming.
8. C++ behaviors verified against source and captured in tests: serialize_bytes(IncludeLength) commits the u16 prefix before the body room check (vs serialize_buffer's pre-check); the prefix silently truncates to u16 while serialize_size range-checks; OmitLength deserialize has no empty-check (SizeMismatch on empty, Ok for len 0); serialize_skip does NOT reset the read cursor; deserialize_skip(0) on a consumed buffer is DeserBufferEmpty; bool decode does not advance on a bad byte.
9. Known gaps (intentional per ARCHITECTURE): no Fw::Buffer pointer-member serialization; no FilePacket family (not in phase 1); CmdPacket has no Serialize impl (C++ FW_ASSERT(false)); Time/FwString equality quirks documented (PartialEq ignores context for Time).
Test-suite note: assert-hook and default-panic tests share a lock and the hook keeps the default panicking do_assert, so concurrent should_panic tests in the same binary remain correct; verified stable across repeated runs.

## fprime-os

```text
CRATE fprime_os. Root re-exports: Queue, BlockingType, Task, OsMutex, ScopeLock, ConditionVariable, File, Directory, Console, ConsoleStream, CONSOLE, RawTime, IntervalTimer; fn init() (registers CONSOLE as the global fw logger). Status enums stay module-scoped (fprime_os::queue::Status etc.); all are #[must_use] #[repr(i32)] with exact C++ discriminants.

mod queue:
  enum Status { OpOk=0, AlreadyCreated=1, Empty=2, Uninitialized=3, SizeMismatch=4, SendError=5, ReceiveError=6, InvalidPriority=7, Full=8, NotSupported=9, AllocationFailed=10, UnknownError=11 }
  enum BlockingType { Blocking=0, NonBlocking=1 }
  type QueueString = FwString<80>
  struct Queue (Default; ALL methods &self — share as Arc<Queue>):
    fn new()->Queue
    fn create(&self, name:&str, depth:FwSizeType, message_size:FwSizeType)->Status  // fw_asserts depth>0,size>0; C++ id param dropped
    fn send(&self, buffer:&[u8], priority:FwQueuePriorityType, block_type:BlockingType)->Status
    fn receive(&self, destination:&mut [u8], block_type:BlockingType, actual_size:&mut FwSizeType, priority:&mut FwQueuePriorityType)->Status
    fn send_serial(&self, message:&dyn SerBufAny, priority:FwQueuePriorityType, block_type:BlockingType)->Status  // sends bytes[0..ser_loc]
    fn receive_serial(&self, destination:&mut dyn SerBufAny, block_type:BlockingType, priority:&mut FwQueuePriorityType)->Status  // resets dest, sets len
    fn get_messages_available(&self)->FwSizeType; fn get_message_high_water_mark(&self)->FwSizeType
    fn get_depth(&self)->FwSizeType; fn get_message_size(&self)->FwSizeType; fn get_name(&self)->QueueString
    fn get_num_queues()->FwSizeType  // static

mod task:
  enum Status { OpOk=0, InvalidHandle=1, InvalidParams=2, InvalidPriority=3, InvalidStack=4, UnknownError=5, InvalidAffinity=6, DelayError=7, JoinError=8, ErrorResources=9, ErrorPermission=10, NotSupported=11, InvalidState=12 }
  enum State { NotStarted=0, Starting=1, Running=2, SuspendedIntentionally=3, SuspendedUnintentionally=4, Exited=5, Unknown=6 }
  enum SuspensionType { Intentional=0, Unintentional=1 }
  const TASK_DEFAULT: FwSizeType = FwSizeType::MAX; TASK_PRIORITY_DEFAULT: FwTaskPriorityType = u8::MAX; TASK_IDENTIFIER_DEFAULT: FwTaskIdType = -1
  struct Arguments { pub name:String, pub routine:Box<dyn FnOnce()+Send+'static>, pub priority:FwTaskPriorityType, pub stack_size:FwSizeType, pub cpu_affinity:FwSizeType, pub identifier:FwTaskIdType }; fn new(name:&str, routine:Box<dyn FnOnce()+Send+'static>)->Arguments /*defaults*/
  struct Task (Default; &self methods, Arc-shareable):
    fn new(); fn start(&self, arguments:Arguments)->Status; fn join(&self)->Status
    fn get_state(&self)->State; fn get_name(&self)->String; fn get_priority(&self)->FwTaskPriorityType
    fn suspend(&self, SuspensionType) /*fw_assert(false)*/; fn resume(&self) /*fw_assert(false)*/; fn is_cooperative(&self)->bool /*false*/
    fn get_num_tasks()->FwSizeType; fn delay(interval:TimeInterval)->Status  // static

mod mutex:
  enum Status { OpOk=0, ErrorBusy=1, ErrorDeadlock=2, NotSupported=3, ErrorOther=4 }
  struct OsMutex (Default): const fn new(); fn lock(&self)->ScopeLock<'_>; fn try_lock(&self)->Result<ScopeLock<'_>, Status>
  struct ScopeLock<'a>  // RAII guard; unlocks on drop

mod condition:
  enum Status { OpOk=0, ErrorMutexNotHeld=1, ErrorDifferentMutex=2, ErrorNotImplemented=3, NotSupported=4, ErrorOther=5 }
  struct ConditionVariable (Default): const fn new(); fn pend<'a>(&self, lock:ScopeLock<'a>)->(Status, ScopeLock<'a>); fn wait<'a>(&self, lock:ScopeLock<'a>)->ScopeLock<'a> /*asserts OpOk*/; fn notify(&self); fn notify_all(&self)

mod file:
  const FW_FILE_CHUNK_SIZE: usize = 512; INITIAL_CRC: u32 = 0xFFFF_FFFF
  enum Mode (Default=OpenNoMode) { OpenNoMode=0, OpenRead=1, OpenCreate=2, OpenWrite=3, OpenSyncWrite=4, OpenAppend=5 }
  enum Status { OpOk=0, DoesntExist=1, NoSpace=2, NoPermission=3, BadSize=4, NotOpened=5, FileExists=6, NotSupported=7, InvalidMode=8, InvalidArgument=9, NoMoreResources=10, OtherError=11, OutsideSandbox=12 }
  enum OverwriteType { NoOverwrite=0, Overwrite=1 }; enum SeekType { Relative=0, Absolute=1 }; enum WaitType { NoWait=0, Wait=1 }
  struct File (Default; closes on drop):
    fn new(); fn open(&mut self, filepath:&str, requested_mode:Mode)->Status /*NoOverwrite*/; fn open_with_overwrite(&mut self, filepath:&str, requested_mode:Mode, overwrite:OverwriteType)->Status
    fn close(&mut self); fn is_open(&self)->bool; fn get_mode(&self)->Mode
    fn size(&mut self, size_result:&mut FwSizeType)->Status; fn position(&mut self, position_result:&mut FwSizeType)->Status
    fn seek(&mut self, offset:FwSignedSizeType, seek_type:SeekType)->Status; fn seek_absolute(&mut self, offset:FwSizeType)->Status; fn flush(&mut self)->Status
    fn read(&mut self, buffer:&mut [u8], size:&mut FwSizeType, wait:WaitType)->Status   // request = buffer.len(); *size = actual (out)
    fn write(&mut self, buffer:&[u8], size:&mut FwSizeType, wait:WaitType)->Status      // *size = written (out)
    fn readline(&mut self, buffer:&mut [u8], size:&mut FwSizeType, wait:WaitType)->Status  // *size includes '\n'
    fn incremental_crc(&mut self, size:&mut FwSizeType)->Status /*in: chunk<=512, out: read*/; fn finalize_crc(&mut self, crc:&mut u32)->Status /*returns UN-complemented register = !crc32*/; fn calculate_crc(&mut self, crc:&mut u32)->Status

mod filesystem (free functions):
  const FILE_SYSTEM_FILE_CHUNK_SIZE: usize = 512
  enum Status { OpOk=0, AlreadyExists=1, NoSpace=2, NoPermission=3, NotDir=4, IsDir=5, NotEmpty=6, InvalidPath=7, DoesntExist=8, FileLimit=9, Busy=10, NoMoreFiles=11, BufferTooSmall=12, ExdevError=13, OverflowError=14, NotSupported=15, OtherError=16 }
  enum PathType { File=0, Directory=1, Other=2, NotExist=3 }
  fn remove_directory(&str)->Status; remove_file(&str)->Status; rename(&str,&str)->Status; get_path_type(&str)->PathType; exists(&str)->bool
  fn create_directory(path:&str, error_if_already_exists:bool)->Status; touch(&str)->Status
  fn copy_file(source:&str, dest:&str)->Status; append_file(source:&str, dest:&str, create_missing_dest:bool)->Status; move_file(source:&str, dest:&str)->Status
  fn get_file_size(path:&str, size:&mut FwSizeType)->Status
  fn get_free_space(path:&str, total_bytes:&mut FwSizeType, free_bytes:&mut FwSizeType)->Status  // always NotSupported (see notes)
  fn get_working_directory(path:&mut FileNameString)->Status; change_working_directory(&str)->Status

mod directory:
  enum Status { OpOk=0, DoesntExist=1, NoPermission=2, NotOpened=3, NotDir=4, NoMoreFiles=5, FileLimit=6, BadDescriptor=7, AlreadyExists=8, NotSupported=9, OtherError=10 }
  enum OpenMode { Read=0, CreateIfMissing=1, CreateExclusive=2 }
  struct Directory (Default): fn new(); fn open(&mut self, path:&str, mode:OpenMode)->Status; fn is_open(&self)->bool; fn rewind(&mut self)->Status; fn read(&mut self, filename:&mut FileNameString)->Status; fn get_file_count(&mut self, file_count:&mut FwSizeType)->Status; fn read_directory(&mut self, filenames:&mut [FileNameString], filename_count:&mut FwSizeType)->Status; fn close(&mut self)

mod console:
  enum ConsoleStream (Default=Stdout) { Stdout, Stderr }
  struct Console (impl fprime_fw::FwLogger): const fn new(); fn set_output_stream(&self, ConsoleStream); fn write(&self, message:&str)
  static CONSOLE: Console; fn init()  // registers CONSOLE as global fw logger

mod rawtime:
  const FW_RAW_TIME_SERIALIZATION_MAX_SIZE: usize = 8
  enum Status { OpOk=0, OpOverflow=1, InvalidParams=2, NotSupported=3, OtherError=4 }
  struct RawTime (Copy, Default=zero, PartialEq /*same-microsecond*/, impl fprime_fw Serialize+Deserialize; wire = [u32 sec][u32 nsec], sec truncated from u64): const SERIALIZED_SIZE=8; fn new(); from_parts(seconds:u64, nanoseconds:u32)->RawTime; get_seconds(&self)->u64; get_nanoseconds(&self)->u32; now(&mut self)->Status; get_time_interval(&self, other:&RawTime, interval:&mut TimeInterval)->Status /*commutative absolute*/; get_diff_usec(&self, other:&RawTime, result:&mut u32)->Status /*saturates u32::MAX + OpOverflow*/

mod interval_timer:
  struct IntervalTimer (Copy, Default): fn new(); start(&mut self); stop(&mut self); get_diff_usec(&self)->u32 /*u32::MAX on overflow, status swallowed*/; get_time_interval(&self, interval:&mut TimeInterval)->rawtime::Status
```

### Implementation notes / deviations

Deviations from C++/ARCHITECTURE (each documented in code):
1) filesystem::get_free_space returns NotSupported: C++ uses statvfs, which has no zero-dependency std equivalent (std has no free-space API). Downstream users (e.g. a FileManager port) must tolerate NotSupported or this needs a cfg'd libc-free platform shim later.
2) OsMutex drops the C++ take()/release() raw pair in favor of the RAII ScopeLock (safe Rust cannot hold a std MutexGuard across unpaired calls); Status enum kept for parity/telemetry. Priority-inheritance/errorcheck pthread attributes are not reproduced by std (recursive lock deadlocks instead of asserting).
3) Task priority/affinity are recorded no-ops with a one-time fw_log notice (the documented C++ EPERM-degrade equivalent); stack_size IS honored via thread::Builder. Task identifier defaults to -1 (C++ TASK_DEFAULT cast to FwTaskIdType). Queue::create drops the C++ `id` parameter (only used by the optional QueueRegistry, not ported). Task/Queue registries not ported.
4) Queue::get_messages_available takes the lock (C++ reads unlocked, racy by design — locked read is semantically equivalent). Global queue/task counters are relaxed atomics instead of static-mutex-guarded counters.
5) File::size uses metadata() instead of the C++ seek-dance; seek_absolute uses a single u64 SeekFrom::Start instead of the 3-seek trick (API shape kept). File::preallocate not ported (not required by the task; no framework consumer in phase 1). ValidateFile/SandboxedFile/CountingSemaphore/Cpu/Memory not ported (not in scope per task).
6) errno mapping goes through io::ErrorKind, not raw errno: ELOOP (C++ -> DOESNT_EXIST) and ENAMETOOLONG (C++ -> INVALID_PATH) have no stable ErrorKind at the rust-version 1.85 floor and land in OtherError. ErrorKind::CrossesDevices (EXDEV, needed for the move_file fallback) and QuotaExceeded are believed stable as of 1.85; if the floor build ever fails there, those two arms are the place to look.
7) The CRC32 table is duplicated privately in file.rs (fprime-os may not depend on fprime-utils per the dependency DAG); a later refactor can unify with utils::hash.
8) RawTime stores u64 seconds internally (like timespec) and truncates to u32 only on serialization (exact C++ PosixRawTime behavior, including the truncate-then-borrow u32 arithmetic in get_time_interval).
9) SYNC_WRITE mode approximates O_SYNC by fsync-ing after every write in that mode (WAIT writes fsync in all modes, C++ parity).
10) Directory::open on an already-open Directory closes it first (C++ front class would leak the old stream; safe-Rust improvement, behavior otherwise identical).
Test count: 63 unit tests, all in-module; covers every gotcha in the os.md list that is reachable without OS fault injection (EINTR/ENOSPC paths exercised structurally via loop bounds, not fault-injected).

## fprime-utils

```text
CRATE fprime_utils (everything re-exported at crate root; modules: hash, crc_checker, types::{circular_buffer,queue}, rate_limiter, token_bucket). Uses fprime_fw::{SerializeStatus, Time}; statuses are the fprime_fw enum.

hash:
  const HASH_DIGEST_LENGTH: usize = 4; const HASH_EXTENSION_STRING: &str = ".CRC32"
  struct HashBuffer (Debug, Clone, Default, PartialEq/Eq on content; impls fprime_fw SerBufAny + Serialize + Deserialize — wire as value = [u16 size][digest bytes]; `use fprime_fw::SerBuf` for cursor methods):
    const SERIALIZED_SIZE: usize = 6; const fn new() -> Self; fn from_bytes(data: &[u8]) -> Self /*fw_asserts len<=4*/; fn as_big_endian_u32(&self) -> u32 /*folds raw 4-byte storage MSB-first*/
  struct Hash (Debug, Clone, Default):
    const fn new() -> Self /*register=0xFFFFFFFF*/; fn init(&mut self); fn update(&mut self, data: &[u8]) /*no final complement*/; fn finalize(&self) -> u32 /*!register = standard CRC-32; non-mutating*/; fn finalize_buffer(&self) -> HashBuffer /*standard value serialized BE*/; fn set_hash_value(&mut self, value: u32) /*stores !value*/; fn set_hash_value_buffer(&mut self, value: &mut HashBuffer) /*u32 BE at read cursor, advances it*/; fn hash(data: &[u8]) -> HashBuffer /*one-shot*/; fn hash_u32(data: &[u8]) -> u32 /*one-shot standard value*/

crc_checker:
  const CRC_FILE_READ_BLOCK: usize = 2048
  #[must_use] #[repr(i32)] enum CrcStat { PassedFileCrcCheck=0, PassedFileCrcWrite=1, FailedFileSize=2, FailedFileSizeCast=3, FailedFileOpen=4, FailedFileRead=5, FailedFileCrcOpen=6, FailedFileCrcRead=7, FailedFileCrcWrite=8, FailedFileCrcCheck=9 }
  fn create_checksum_file(fname: &str) -> CrcStat /*success = PassedFileCrcWrite; writes <fname>.CRC32 = 4 raw NATIVE-endian bytes of standard CRC-32*/
  fn read_crc32_from_file(fname: &str, checksum_from_file: &mut u32) -> CrcStat /*success = PassedFileCrcCheck; out untouched on failure*/
  fn verify_checksum(fname: &str, expected: &mut u32, actual: &mut u32) -> CrcStat /*expected=sidecar, actual=recomputed, written on both pass and FailedFileCrcCheck; untouched on earlier failures*/

types::circular_buffer (re-exported at types:: and crate root):
  struct CircularBuffer (Debug):
    fn new(size: usize) -> Self /*owns Box<[u8]> allocated here; fw_asserts size>0; setup-once by construction*/
    fn serialize(&mut self, buffer: &[u8]) -> SerializeStatus /*append; NoRoomLeft if > free, never overwrites*/
    fn peek_u8(&self, value: &mut u8, offset: usize) -> SerializeStatus /*DeserBufferEmpty if offset+1 > allocated*/
    fn peek_u32_be(&self, value: &mut u32, offset: usize) -> SerializeStatus /*4 bytes MSB-first*/
    fn peek_bytes(&self, dest: &mut [u8], offset: usize) -> SerializeStatus /*copies dest.len() bytes*/
    fn rotate(&mut self, amount: usize) -> SerializeStatus /*consume FRONT; DeserBufferEmpty if amount > allocated*/
    fn trim(&mut self, amount: usize) -> SerializeStatus /*drop BACK; same bound*/
    fn get_allocated_size(&self) -> usize; fn get_free_size(&self) -> usize; fn get_capacity(&self) -> usize /*== store size, full store usable*/; fn get_high_water_mark(&self) -> usize; fn clear_high_water_mark(&mut self)

types::queue (re-exported at types:: and crate root):
  #[repr(i32)] enum QueueMode { Fifo=0 (Default), Lifo=1 }; #[repr(i32)] enum QueueOverflowMode { DropNewest=0 (Default), DropOldest=1 } (both Debug, Clone, Copy, PartialEq, Eq)
  struct Queue (Debug):
    fn new(depth: usize, message_size: usize, mode: QueueMode, overflow_mode: QueueOverflowMode) -> Self /*owns ring of exactly depth*message_size bytes; fw_asserts both > 0*/
    fn enqueue(&mut self, message: &[u8]) -> SerializeStatus /*fw_asserts message.len()==message_size; Ok | NoRoomLeft (full+DropNewest) | DiscardedExisting (full+DropOldest: oldest rotated out, new stored)*/
    fn dequeue(&mut self, message: &mut [u8]) -> SerializeStatus /*fw_asserts message.len()>=message_size, writes first message_size bytes; FIFO=front, LIFO=back; empty -> DeserBufferEmpty*/
    fn pop_front(&mut self, message: &mut [u8]) -> SerializeStatus /*always front regardless of mode*/
    fn get_queue_size(&self) -> usize /*MESSAGE units*/; fn get_high_water_mark(&self) -> usize /*MESSAGE units*/; fn get_message_size(&self) -> usize; fn clear_high_water_mark(&mut self)

rate_limiter:
  struct RateLimiter (Debug, Clone; Default = cycles 0,0):
    fn new(counter_cycle: u32, time_cycle: u32) -> Self /*0 disables a dimension*/
    fn trigger(&mut self, time: Time) -> bool /*both cycles 0 -> always true; else OR of enabled criteria; updates enabled dimensions*/
    fn trigger_counter_only(&mut self) -> bool /*C++ arg-less trigger(); fw_asserts time_cycle==0*/
    fn set_counter_cycle(&mut self, u32); fn set_time_cycle(&mut self, u32); fn reset(&mut self); fn reset_counter(&mut self); fn reset_time(&mut self) /*back to negative infinity*/; fn set_counter(&mut self, u32); fn set_time(&mut self, Time) /*clears neg-infinity flag*/
    /*time criterion builds the cycle Time with TbNone; Time::add fw_asserts on base mismatch with the stored time (C++ parity)*/

token_bucket:
  const MAX_TOKEN_BUCKET_TOKENS: u32 = 1000
  struct TokenBucket (Debug, Clone):
    fn new(replenish_interval_us: u32, max_tokens: u32) -> Self /*rate=1, starts full, time=(0,0) TbNone; fw_asserts max<=1000*/
    fn with_state(replenish_interval_us: u32, max_tokens: u32, replenish_rate: u32, start_tokens: u32, start_time: Time) -> Self /*C++ 5-arg ctor; NO limit check*/
    fn trigger(&mut self, time: Time) -> bool /*replenish loop in whole intervals then consume one token*/
    fn replenish(&mut self) /*SETS tokens to max*/
    fn set_replenish_interval(&mut self, u32); fn set_max_tokens(&mut self, u32); fn set_replenish_rate(&mut self, u32); fn get_replenish_interval(&self) -> u32; fn get_max_tokens(&self) -> u32; fn get_replenish_rate(&self) -> u32; fn get_tokens(&self) -> u32
```

### Implementation notes / deviations

Deviations/gaps (all intentional): (1) The C++ Serializable/LinearBufferBase overloads of CircularBuffer::serialize/peek and Queue::enqueue/dequeue/popFront (staging-buffer object slots; default STAGING_BUFFER_SIZE=0 asserts on any wrapping slot) are not ported — the task scoped the byte-slice API, and phase-1 consumers (FrameAccumulator, ComQueue) use the byte-level paths; add later if a component needs them. (2) Types::SpscQueue not ported (no phase-1 consumer; listed as out of scope). (3) CONFIG_CRC_FILE_READ_BLOCK lives as crc_checker::CRC_FILE_READ_BLOCK because fprime-config (not writable by this agent) has no slot for it; move there if config-forking is wanted. (4) crc_checker uses std::fs directly per ARCHITECTURE.md (not fprime-os File); sidecar written with File::create (truncate) — equivalent to C++ OPEN_WRITE since the file is exactly 4 bytes at offset 0. (5) C++ overload pairs map to distinct names: RateLimiter::trigger() (arg-less) -> trigger_counter_only(); Hash::finalize(HashBuffer&) -> finalize_buffer(); TokenBucket 5-arg ctor -> with_state() (which, like C++, does NOT check MAX_TOKEN_BUCKET_TOKENS). (6) Queue::get_message_size() added (no C++ equivalent; trivial getter for downstream ComQueue). (7) Time-base semantics ported exactly: RateLimiter/TokenBucket build cycle/interval Times via Time::from_seconds_useconds (TbNone), so Time::add fw_asserts if a caller mixes bases and cross-base comparisons are false (Incomparable) — same crash/behavior as C++. Note for the orchestrator: mid-task `cargo build --workspace` failed transiently because a sibling agent was concurrently writing fprime-os; I waited until it stabilized and the final full-workspace build is green (fprime-utils itself never depended on it). Files: /home/user/fprime-rust/crates/fprime-utils/src/{lib.rs,hash.rs,crc_checker.rs,rate_limiter.rs,token_bucket.rs,types/mod.rs,types/circular_buffer.rs,types/queue.rs}.

## fprime-comp

```text
CRATE fprime_comp — everything re-exported at root (modules: obj, port, msg, queued, active, glue, escrow, macros).
Crate re-exports for macro expansions and downstream convenience: `fprime_comp::config` (= fprime_config), `fprime_comp::fw` (= fprime_fw), `fprime_comp::os` (= fprime_os).
Codegen macros (exported at the crate root, documented under "Codegen layer" below): component_msg_types!, input_port_adapter!, async_input_port_adapter!.

obj::PassiveBase: fn new(name:&str)->Self; Default ("NoName"); get_obj_name()->ObjectName; set_obj_name(&self,&str); set_id_base(&self,FwIdType); get_id_base()->FwIdType; set_instance(&self,FwEnumStoreType); get_instance()->FwEnumStoreType. All &self.

port::PortRef<P:?Sized> { pub target: Arc<P>, pub port_num: FwIndexType }: fn new(Arc<P>, FwIndexType)->Self; Clone, Debug.
port::OutputPort<P:?Sized> (Default, Debug): const fn new(); connect(&self, Arc<P>, FwIndexType) /*fw_asserts port_num>=0 and second connect*/; connect_to(&self, PortRef<P>); is_connected()->bool; get(&self)->&PortRef<P> /*fw_assert unconnected*/; try_get(&self)->Option<&PortRef<P>>.
Port traits (all `: Send + Sync`, object-safe; use as OutputPort<dyn XPort>/Arc<dyn XPort>):
  SchedPort::invoke(&self, port_num: FwIndexType, context: u32)
  CyclePort::invoke(&self, port_num, cycle_start: &fprime_os::RawTime)
  PingPort::invoke(&self, port_num, key: u32)
  WatchDogPort::invoke(&self, port_num, code: u32)
  CmdPort::invoke(&self, port_num, op_code: FwOpcodeType, cmd_seq: u32, args: &mut CmdArgBuffer)
  CmdRegPort::invoke(&self, port_num, op_code: FwOpcodeType)
  CmdResponsePort::invoke(&self, port_num, op_code, cmd_seq: u32, response: CmdResponse)
  LogPort::invoke(&self, port_num, id: FwEventIdType, time_tag: &mut Time, severity: LogSeverity, args: &mut LogBuffer)
  LogTextPort::invoke(&self, port_num, id, time_tag: &mut Time, severity, text: &mut TextLogString)
  TlmPort::invoke(&self, port_num, id: FwChanIdType, time_tag: &mut Time, val: &mut TlmBuffer)
  TimePort::invoke(&self, port_num, time: &mut Time)
  ComPort::invoke(&self, port_num, data: &mut ComBuffer, context: u32)
  BufferSendPort::invoke(&self, port_num, buffer: Buffer)              // by move
  BufferGetPort::invoke(&self, port_num, size: FwSizeType) -> Buffer   // sync only
  ComDataWithContextPort::invoke(&self, port_num, data: Buffer, context: &FrameContext)  // no return; return path = paired port
  SuccessConditionPort::invoke(&self, port_num, condition: &mut Success)
  PrmGetPort::invoke(&self, port_num, id: FwPrmIdType, val: &mut ParamBuffer) -> ParamValid
  PrmSetPort::invoke(&self, port_num, id: FwPrmIdType, val: &mut ParamBuffer)
  FatalEventPort::invoke(&self, port_num, id: FwEventIdType)

msg: const EXIT_MSG_TYPE: FwEnumStoreType = 0; EXIT_MSG_SIZE: usize = 4; EXIT_MSG_BYTES: [u8;4]; ENVELOPE_HEADER_SIZE: usize = 6.
  fn write_envelope_header(buf:&mut dyn SerBufAny, msg_type:FwEnumStoreType, port_num:FwIndexType)->SerializeStatus
  fn write_exit(buf:&mut dyn SerBufAny)->SerializeStatus
  fn read_msg_type(buf:&mut dyn SerBufAny, &mut FwEnumStoreType)->SerializeStatus
  fn read_port_num(buf:&mut dyn SerBufAny, &mut FwIndexType)->SerializeStatus
  enum QueueFullPolicy { #[default] Assert, Drop, Block, Hook } (Copy, Eq)

queued: #[must_use] #[repr(i32)] enum MsgDispatchStatus { Ok=0, Empty=1, Error=2, Exit=3 } (Copy, Eq).
  trait ComponentDispatch: Send+Sync { fn dispatch_message(&self, msg_type: FwEnumStoreType, msg: &mut dyn SerBufAny) -> MsgDispatchStatus; fn preamble(&self) {}; fn finalizer(&self) {} }
    // msg cursor is positioned AFTER msg_type; component must read port_num first via msg::read_port_num.
  struct QueuedBase (Default; all &self) { pub base: PassiveBase, .. }:
    fn new(name:&str)->Self
    fn create_queue(&self, depth: FwSizeType, msg_size: FwSizeType)   // queue name = object name; fw_asserts OK
    fn queue(&self)->&fprime_os::Queue
    fn get_num_msgs_dropped(&self)->FwSizeType; fn inc_num_msgs_dropped(&self)
    fn send_message(&self, message:&dyn SerBufAny, priority:FwQueuePriorityType, policy:QueueFullPolicy)->fprime_os::queue::Status  // #[must_use] result; sends bytes[0..ser_loc]
    fn do_dispatch(&self, component:&dyn ComponentDispatch, blocking: fprime_os::queue::BlockingType)->MsgDispatchStatus
    fn dispatch_available_messages(&self, component:&dyn ComponentDispatch)->MsgDispatchStatus  // snapshot-bounded

active: #[repr(i32)] enum Lifecycle { Created=0, Dispatching=1, Finalizing=2, Done=3 } (Copy, Eq).
  trait ActiveComponent: ComponentDispatch + 'static { fn active_base(&self)->&ActiveBase; }
  struct ActiveBase (Default) { pub queued: QueuedBase, .. }:
    fn new(name:&str)->Self
    fn start<C: ActiveComponent>(&self, component:&Arc<C>, priority:FwTaskPriorityType, stack_size:FwSizeType, cpu_affinity:FwSizeType)  // asserts component.active_base() is self; task named after object; fw_asserts task start
    fn exit(&self)      // 4-byte EXIT, priority 0, non-blocking, status ignored
    fn join(&self)->fprime_os::task::Status
    fn lifecycle(&self)->Lifecycle; fn task(&self)->&fprime_os::Task
  Component field access pattern: comp.active.queued.base (PassiveBase), comp.active.queued (QueuedBase).

glue: fn time_get(port:&OutputPort<dyn TimePort>)->Time  // Time::default() when unconnected
  struct CmdGlue (const new, Default) { pub cmd_reg_out: OutputPort<dyn CmdRegPort>, pub cmd_response_out: OutputPort<dyn CmdResponsePort> }:
    fn reg_commands(&self, id_base:FwIdType, opcodes:&[FwOpcodeType])       // invokes reg with id_base+opcode each; asserts unconnected
    fn cmd_response(&self, op_code:FwOpcodeType, cmd_seq:u32, response:CmdResponse)  // asserts unconnected
  struct EventThrottle: const fn new(limit:u32); fn ok_to_emit(&self)->bool /*counts; false once >= limit*/; fn clear(&self); fn get_count(&self)->u32
  struct EventGlue (const new, Default) { pub log_out: OutputPort<dyn LogPort>, pub text_log_out: OutputPort<dyn LogTextPort>, pub time_out: OutputPort<dyn TimePort> }:
    fn time_get(&self)->Time
    fn log_event<F: FnOnce(&mut LogBuffer)->SerializeStatus>(&self, id_base:FwIdType, local_id:FwEventIdType, severity:LogSeverity, text:&str, write_args:F)
      // stamps time once, invokes log_out (asserting write_args status) and text_log_out only when each is connected; throttling is caller's job (guard with EventThrottle::ok_to_emit)
  struct TlmGlue (const new, Default) { pub tlm_out: OutputPort<dyn TlmPort> }:
    fn tlm_write(&self, id_base:FwIdType, local_id:FwChanIdType, value:&dyn Serialize, time_tag:Time)  // no-op when unconnected
  struct PrmGlue (const new, Default) { pub prm_get_out: OutputPort<dyn PrmGetPort>, pub prm_set_out: OutputPort<dyn PrmSetPort> }:
    fn get_param(&self, id_base:FwIdType, local_id:FwPrmIdType, val:&mut ParamBuffer)->ParamValid  // asserts unconnected
    fn set_param(&self, id_base:FwIdType, local_id:FwPrmIdType, val:&mut ParamBuffer)

escrow::BufferEscrow (Default, Debug): fn new(); with_capacity(usize)->Self; deposit(&self, Buffer)->u64; claim(&self, token:u64)->Buffer /*fw_assert invalid/stale/double*/; len()->usize; is_empty()->bool. Token = (generation<<32)|slot; serialize as u64 (8 bytes, same slot C++ uses for the pointer).

COMPONENT PATTERN (normative; copy tests/example_component.rs — hand-written; tests/macro_component.rs is the same shapes built with the codegen macros): msg types start at 1; async input = adapter struct {comp: Arc<C>} impl Port trait — write_envelope_header + args + send_message(policy); sync/guarded input = impl trait on the component itself, handler locks the component's Mutex<State>; input factory: fn x_in(self:&Arc<Self>, port_num)->PortRef<dyn XPort>; async command envelope args = [opCode u32][cmdSeq u32][serialize_buffer(CmdArgBuffer)] and dispatch on op_code.wrapping_sub(id_base) with InvalidOpcode fallback, FormatError on deser failure or deserialize_size_left()!=0; topology order: construct -> set_id_base -> connect -> create_queue -> reg_commands -> start(&arc,..) -> exit -> join (EXIT is priority 0, so pending higher-priority traffic drains first — join is a deterministic sync point in tests).
```

### Implementation notes / deviations

Deviations/decisions (all C++-behavior-preserving):
1. doDispatch split: C++ generates one doDispatch per component; here QueuedBase::do_dispatch does receive + msg_type read + EXIT check (before port_num — per the documented gotcha), and the component's ComponentDispatch::dispatch_message does the switch starting with read_port_num. Wire format unchanged.
2. ActiveBase::start is generic (start<C: ActiveComponent>(&Arc<C>,..)) rather than taking Arc<dyn ActiveComponent>, because dyn-to-dyn trait upcasting needs Rust 1.86 and the workspace floor is 1.85. It fw_asserts that component.active_base() is the same base being started. The cooperative one-step-per-call s_taskStateMachine is not ported (os Task::is_cooperative() is always false); the Lifecycle stages are still tracked in an atomic for observability.
3. OutputPort second-connect fw_asserts (OnceLock), stricter than C++ silently overwriting a connection — catches wiring bugs only. Port tracing ("+ name for tracing" in ARCHITECTURE) not implemented: no tracing consumer exists yet; PortRef/OutputPort have Debug impls instead.
4. ObjRegistry/SimpleObjRegistry and serialize-ports (Input/OutputSerializePort) are not ported (raw-pointer registry is debug-only; GenericHub is a phase-1 non-goal). The envelope helpers keep the byte format hub-compatible for later.
5. receive-side non-Empty queue errors in do_dispatch are fw_assert (C++ generated code asserts them) rather than returning Error; dispatch_message deserialize failures return MsgDispatchStatus::Error (the softer of the C++ "ERROR/assert" options, letting active loops keep running — verified by a test).
6. EXIT is returned by do_dispatch for queued components too (dispatch_available_messages just breaks with it), matching C++.
7. EventGlue::log_event takes the pre-formatted text as &str; formatting cost is paid even when text logging is unconnected if the caller uses format! unconditionally — callers can guard with text_log_out.is_connected() if it matters. Event-string truncation to FW_LOG_STRING_MAX_SIZE is the arg-writer closure's job via FwString::serialize_to_truncated (documented on log_event).
8. BufferEscrow slots Vec can grow past with_capacity if more buffers are in flight than pre-sized (bounded in practice by queue depth); size with with_capacity for strict no-steady-state-alloc.
9. Queue-full policies: Drop asserts on non-OK statuses other than Full (e.g. SizeMismatch) — only Full is a countable drop; Hook returns Full so the ADAPTER invokes its overflow hook with the original args (the hook cannot live in the base, C++ parity).
Open questions: none blocking. git status shows uncommitted modifications to fprime-config/fw/os/utils lib.rs from the earlier implementer agents — untouched by me; I wrote only under crates/fprime-comp/.

# Codegen layer (FPP-style macros) — usage

Zero-dependency `macro_rules!` replacements for what the C++ FPP autocoder
generates. Data types live in `fprime_fw` (module `fpp`, macros exported at
the crate root); component boilerplate lives in `fprime_comp` (module
`macros`, macros exported at the crate root). No proc macros, so nothing is
added to the dependency graph.

```text
fprime_fw:   fpp_enum!  fpp_struct!  fpp_array!   trait FppSized { const SERIALIZED_SIZE: usize }
fprime_comp: component_msg_types!  input_port_adapter!  async_input_port_adapter!
```

`FppSized` is the compile-time (maximum) on-wire size — the C++ static
`SERIALIZED_SIZE`. It is implemented for `u8..i64`, `f32`, `f64`, `bool`,
`Time`, `TimeInterval`, `FwString<N>`, and for every type these macros
generate. `Serialize::serialized_size` remains the *actual* size of a value.

## `fpp_enum!` — FPP `enum E : R`

```rust
fpp_enum! {
    /// Event severity (`Fw::LogSeverity`).
    pub enum LogSeverity : u8 {
        /// A fatal non-recoverable event.
        Fatal = 1,
        /// A serious but recoverable event.
        WarningHi = 2,
    }
    default Fatal
}
```

Generates: `#[repr(u8)]` enum + `Debug, Clone, Copy, PartialEq, Eq, Hash`;
`Default` = the `default` constant; `TryFrom<u8>` (`Err` carries the raw
value); consts `SERIALIZED_SIZE`, `VALUES: &'static [Self]`, `NUM_CONSTANTS`;
`as_repr()`, `is_valid()` (member; always true in Rust), `is_valid_repr(raw)`
(static, exact declared values — values *between* declared constants are
invalid); `Serialize`/`Deserialize` at the representation width, big-endian,
with strict decode (undeclared value -> `DeserFormatError`, bytes consumed,
target unmodified); `FppSized`.

## `fpp_struct!` — FPP `struct S { .. } default { .. }`

```rust
fpp_struct! {
    /// Context passed between comms components (`ComCfg::FrameContext`).
    #[derive(Clone, Copy, Eq)]                 // Debug + PartialEq are automatic
    pub struct FrameContext {
        /// Queue index used by the ComQueue.
        com_queue_index: FwIndexType { get_com_queue_index, set_com_queue_index },
        /// 11-bit APID in CCSDS.
        apid: Apid { get_apid, set_apid },
        /// Secondary header flag.
        has_sec_hdr: bool,                     // accessor pair is optional
    }
    default {
        apid = Apid::FwPacketUnknown,          // members not listed use their type default
    }
}
```

Generates: the struct with all members `pub` in declaration order;
`SERIALIZED_SIZE` = sum of the members' `FppSized` sizes; `new(..)` (full
constructor) and `set_all(..)` (C++ `set`); the declared `get_*`/`set_*`
pairs (**getters borrow** — `*ctx.get_apid()` — because C++ returns `const&`
for class members; the `pub` field is there for everything else); `Default`
from the clause; `Serialize`/`Deserialize` strictly in member order with no
header/count/padding, deserialization committing only on full success.

`macro_rules!` cannot concatenate identifiers, so accessor names are spelled
out; omit the `{ .. }` clause on a member to get the field only. Add
`Clone`/`Copy`/`Eq`/`Hash` with a plain `#[derive(..)]` above the struct.

## `fpp_array!` — FPP `array A = [n] T`

```rust
fpp_array! {
    /// Three cycle counts.
    #[derive(Clone, Copy, Eq)]
    pub array Counts = [u16; 3]
    default fill 0xFFFF        // or: default [1, 2, 3]  — or omit for T::default()
}
```

Generates: `pub struct Counts(pub [u16; 3])`; `SIZE`, `SERIALIZED_SIZE`
(= `SIZE * element size`); `new([..])`, `fill(v)`, `From<[T; N]>` /
`From<Self> for [T; N]`; `elements()`, `elements_mut()`, `as_slice()`,
`iter()`, `IntoIterator for &Self`, `Index`/`IndexMut<usize>`; `Default`;
elementwise `Serialize`/`Deserialize` with **no count prefix and no
padding**, deserialization committing only on full success.

## `component_msg_types!`

```rust
component_msg_types! {
    /// Queue message types (0 is the EXIT sentinel).
    impl ActiveRateGroup {
        /// `CycleIn` async input port.
        MSG_TYPE_CYCLE_IN,     // = 1
        /// `PingIn` async input port.
        MSG_TYPE_PING_IN,      // = 2
    }
}
```

Emits `pub const <NAME>: FwEnumStoreType` on the component, numbered from 1
in declaration order. Refer to them as `Self::MSG_TYPE_CYCLE_IN`, including
in the `dispatch_message` match arms.

## Port-argument passing modes (both adapter macros)

| mode | trait parameter | queue write | queue read |
|------|-----------------|-------------|------------|
| `val x: T` | `x: T` | `Serialize` | `Deserialize` |
| `ref x: T` | `x: &T` | `Serialize` | `Deserialize` |
| `mut x: T` | `x: &mut T` | `Serialize` | `Deserialize` |
| `buf x: T` | `x: &mut T` | nested buffer (`u16` len + bytes) | nested buffer |

`mut` is the C++ non-buffer `ref` parameter (`Fw::Time&`, `Fw::Success&`).
Across an async queue the value is copy-in only — the caller's variable is
not written back, exactly as the generated C++ behaves.

## `input_port_adapter!` — SYNC / GUARDED input

```rust
input_port_adapter! {
    /// `dataIn` — GUARDED `Svc.ComDataWithContext` input: one frame.
    component: FprimeDeframer;
    adapter: DataInAdapter;          // adapter struct name (identifiers can't be synthesized)
    port: ComDataWithContextPort;
    input: pub data_in;              // factory name, with its visibility
    handler: data_in_handler;
    // returns: ParamValid;          // only for port traits whose invoke returns a value
    args { val data: Buffer, ref context: FrameContext }
}
```

Generates the adapter struct, `impl ComDataWithContextPort for DataInAdapter`
forwarding 1:1 to `self.comp.data_in_handler(port_num, data, context)` on the
caller's thread, and
`pub fn data_in(self: &Arc<Self>, port_num) -> PortRef<dyn ComDataWithContextPort>`.
Guarded semantics stay the handler's job (it locks the component's state
mutex) — in C++ only the generated lock differs, and here that lock is the
handler's `Mutex<State>`.

## `async_input_port_adapter!` — ASYNC input

```rust
async_input_port_adapter! {
    /// `CycleIn` — ASYNC `Svc.Cycle` input with the `drop` queue-full policy.
    component: ActiveRateGroup;
    adapter: CycleInAdapter;
    port: CyclePort;
    input: pub cycle_in;
    deserialize: cycle_in_deserialize;
    handler: cycle_in_handler;
    base: active.queued;                          // dotted path to the QueuedBase
    msg_type: ActiveRateGroup::MSG_TYPE_CYCLE_IN;
    msg_size: MSG_SIZE;                           // usize const; sizes LinearBuffer<{MSG_SIZE}>
    priority: CYCLE_IN_PRIORITY;
    queue_full: QueueFullPolicy::Drop;            // FPP assert / drop / block / hook
    args { ref cycle_start: RawTime }
    pre_msg_hook |comp, _port_num| {              // optional; SENDER's thread, before enqueue
        comp.cycle_started.store(true, Ordering::Relaxed);
    }
    // overflow_hook |comp, port_num| { .. }      // optional; runs when the send returned Full
}
```

Generates, on top of the sync macro's output, the byte-exact envelope
`[msg_type i32 BE][port_num i16 BE][args in declaration order]` (serialize
failures are `fw_assert!` — `msg_size` is the worst case, C++ parity), the
`send_message` call under the given policy, and

```rust
fn cycle_in_deserialize(msg: &mut dyn SerBufAny) -> Option<(RawTime,)>
```

which reads the arguments only (`msg_type`/`port_num` are already consumed by
the dispatch loop and by `dispatch_message`) and returns `None` on any decode
failure. Both hook closures bind `comp: &Component` and the port number under
names *you* choose (macro hygiene), and the port arguments are in scope too.

Use it from `dispatch_message`:

```rust
match msg_type {
    Self::MSG_TYPE_CYCLE_IN => match Self::cycle_in_deserialize(buf) {
        Some((cycle_start,)) => {
            self.cycle_in_handler(port_num, &cycle_start);
            MsgDispatchStatus::Ok
        }
        None => MsgDispatchStatus::Error,
    },
    _ => MsgDispatchStatus::Error,
}
```

Note the one-element tuple pattern `(cycle_start,)`; a `buf` argument comes
back owned, so bind it `Some((op_code, cmd_seq, mut args))` and pass
`&mut args`.

## When NOT to use a macro

Keep hand-writing (the macros deliberately do not cover these):

- **Async ports carrying an owned `Fw::Buffer`** — the `BufferEscrow`
  deposit/claim pairing is component state, not a per-port pattern.
- **The `dispatch_message` switch** — it is the component's own `doDispatch`.
- **Adapters that drop or reorder arguments** on the way to the handler
  (e.g. `FprimeFramer::data_return_in`, whose handler ignores `context`):
  forwarding is 1:1 or the macro does not apply.
- Command/event/telemetry emission — `CmdGlue` / `EventGlue` / `TlmGlue`
  already reduce that to ordinary calls.

Reference implementations: `crates/fprime-fw/src/com.rs` (`fpp_enum!` +
`fpp_struct!`), `crates/fprime-svc/src/active_rate_group.rs`
(`component_msg_types!` + `async_input_port_adapter!`),
`crates/fprime-svc/src/fprime_deframer.rs` (`input_port_adapter!`),
`crates/fprime-comp/tests/macro_component.rs` (all of them, with byte-level
assertions against hand-written serialization).

# Service crate API notes

## C&DH components (CmdDispatcher, EventManager, FatalHandler, PassiveTextLogger)

```text
All components live in crate fprime_svc (modules re-exported per lib.rs: fprime_svc::cmd_dispatcher, event_manager, fatal_handler, passive_text_logger).

CmdDispatcher (cmd_dispatcher.rs, ACTIVE):
  CmdDispatcher::new(name)->Arc<Self>; fields pub active: ActiveBase, cmd: CmdGlue, evt: EventGlue, tlm: TlmGlue, comp_cmd_send: [OutputPort<dyn CmdPort>; 30], seq_cmd_status: [OutputPort<dyn CmdResponsePort>; 5], ping_out: OutputPort<dyn PingPort>.
  init(&self, queue_depth: FwSizeType) creates the queue (msg size = cmd_dispatcher::QUEUE_MSG_SIZE = 524 = 6 envelope + 2+512 ComBuffer + 4 context; command envelope 522). reg_commands(&self) registers opcodes 0..=3.
  Input factories (all fn x(self:&Arc<Self>, port_num)->PortRef<...>): comp_cmd_reg_in (GUARDED dyn CmdRegPort, 0..30, asserts range), comp_cmd_stat_in (async dyn CmdResponsePort), seq_cmd_buff_in (async dyn ComPort, 0..5, HOOK overflow -> throttled(5) CommandDroppedQueueOverflow + dropped counter), ping_in (async dyn PingPort), run_in (async dyn SchedPort, emits on-change tlm), cmd_in (async dyn CmdPort for its own commands).
  Msg types 1..=5 (COMP_CMD_STAT, SEQ_CMD_BUFF, PING_IN, RUN, CMD); all queue priority 1.
  Assoc consts: OPCODE_CMD_NO_OP=0, OPCODE_CMD_NO_OP_STRING=1, OPCODE_CMD_TEST_CMD_1=2, OPCODE_CMD_CLEAR_TRACKING=3; EVENTID_* 0..=11 exactly per FPP (OP_CODE_REGISTERED..COMMAND_DROPPED_QUEUE_OVERFLOW, COMMAND_DROPPED_THROTTLE=5); CHANID_COMMANDS_DISPATCHED=0, CHANID_COMMAND_ERRORS=1, CHANID_COMMANDS_DROPPED=2 (U32, on change, run writes in C++ order 2,1,0).

EventManager (event_manager.rs, ACTIVE):
  EventManager::new(name)->Arc<Self> (severity defaults from config: DIAGNOSTIC filtered); fields pub active, cmd, evt, tlm, pkt_send: OutputPort<dyn ComPort>, fatal_announce: OutputPort<dyn FatalEventPort>, ping_out: OutputPort<dyn PingPort>.
  init(&self, queue_depth) (msg size event_manager::QUEUE_MSG_SIZE = 530 = 6 + 4 id + 11 Time + 1 severity + 2+506 LogBuffer). reg_commands registers opcodes 0,2,3.
  Input factories: log_recv_in (SYNC dyn LogPort — filters on caller thread, enqueues internal loqQueue msg with Drop policy, FatalAnnounce invoked on caller thread after enqueue), run_in (async dyn SchedPort, Drop policy), ping_in (async dyn PingPort), cmd_in (dyn CmdPort — SET_EVENT_FILTER opcode 0 executes SYNCHRONOUSLY on the caller thread; opcodes 2/3 enqueue; unknown -> InvalidOpcode immediately).
  Msg types 1..=4 (LOQ_QUEUE, RUN, PING_IN, CMD). PktSend emits byte-exact LogPacket [u16 2][id u32][Time 11B][raw args] with context 0; oversized packets dropped via fw_log.
  Assoc consts: OPCODE_SET_EVENT_FILTER=0, OPCODE_SET_ID_FILTER=2, OPCODE_DUMP_FILTER_STATE=3 (no opcode 1); EVENTID_SEVERITY_FILTER_STATE=0, ID_FILTER_ENABLED=1, ID_FILTER_LIST_FULL=2, ID_FILTER_REMOVED=3, ID_FILTER_NOT_FOUND=4; CHANID_EVENTS_DROPPED=0 (FwSizeType u64, on change, = queued.get_num_msgs_dropped()).
  Public types: enum FilterSeverity u8 {WarningHi=0..Diagnostic=5} (TryFrom<u8>, Serialize u8-width, name(), from_index(), to_log_severity(), NUM_CONSTANTS=6); enum EventManagerEnabled u8 {Enabled=0, Disabled=1} (REVERSED vs Fw Enabled, TryFrom<u8>); struct EventSeverityFilter (lock-free [AtomicBool;6]: new()/set_filter(LogSeverity,bool)/is_filtered/is_enabled; FATAL never filterable) — shared with PassiveTextLogger.

FatalHandler (fatal_handler.rs, PASSIVE):
  FatalHandler::new(name)->Arc<Self>; pub base: PassiveBase. fatal_receive_in(&arc, port_num)->PortRef<dyn FatalEventPort> (sync). Handler: fw_log "FATAL {} handled.\n", Task::delay(1s), then exit action. type FatalExitAction = Box<dyn Fn(FwEventIdType)+Send+Sync>; set_exit_action(&self, FatalExitAction) replaces the default (fw_log + std::process::exit(1)).

PassiveTextLogger (passive_text_logger.rs, PASSIVE):
  PassiveTextLogger::new(name)->Arc<Self> (severity defaults: DIAGNOSTIC filtered); pub base: PassiveBase. text_logger_in(&arc, port_num)->PortRef<dyn LogTextPort> (sync). Prints via global fw logger: "EVENT: (<id>) (<timeBase>:<sec>,<usec>) <SEVERITY>: <text>\n" (exact C++ format). configure(&self, &[FwEventIdType]) installs the <=25-entry ID filter (fw_assert over-size); set_severity_filter(&self, LogSeverity, bool). Module consts PASSIVE_TEXT_LOGGER_ID_FILTER_SIZE=25 + six FILTER_*_DEFAULT bools (from PassiveTextLoggerCfg.hpp; live here because fprime-config lacks a slot).

Topology order for all: construct -> set_id_base (via active.queued.base / base) -> connect ports -> init(depth) (active ones) -> reg_commands -> active.start(&arc, prio, stack, affinity) -> exit/join.
```

### Implementation notes / deviations

Deviations/decisions (all documented in code): (1) EventManager internal loqQueue envelope includes a port_num=0 field after msg_type — the fprime-comp dispatch contract reads port_num for every non-EXIT message; C++ internal-interface messages omit it. Internal-only, never crosses a hub. (2) EventManager LogRecv invalid-severity drop (C++ hub-bridge case) is unrepresentable — Rust LogSeverity cannot hold an invalid value; branch documented, not coded. (3) Invalid enum command args answer VALIDATION_ERROR per the workspace-wide CmdGlue discipline (C++ generated code would answer FORMAT_ERROR on enum deser failure; the C++ handler's own fromIndex check maps to VALIDATION_ERROR, which this matches in spirit). (4) FatalHandler default action logs + std::process::exit(1); raise(SIGABRT)/core dump is not reproducible in safe zero-dep Rust. The 1s pre-exit delay is fixed (C++ parity), so the fatal_handler test takes ~1s. (5) PassiveTextLoggerCfg constants (ID_FILTER_SIZE=25, severity defaults) are defined in passive_text_logger.rs because fprime-config has no passive_text_logger submodule and that crate is owned by another wave — flagged for migration. (6) EventSeverityFilter (Svc/Types shared type in C++) lives in event_manager.rs and is reused by passive_text_logger; implemented with relaxed AtomicBools to mirror the C++ lock-free (benignly racy) bool array. (7) CmdDispatcher dispatch table is BTreeMap capped at 150 by fw_assert (C++ RedBlackTreeMap insert-failure assert); sequence tracker is a fixed [Option<(u32,Entry)>;25] with ArrayMap overwrite-on-existing-key semantics. (8) TooManyCommands early return does NOT increment m_seq (matches C++ control flow; the analysis's summary sentence elides this but the C++ `return` precedes m_seq++). (9) On-change telemetry caches are per-component (Option<T> last-value), first write always emitted — matching autocoded tlmWrite behavior. (10) passive_text_logger tests capture the process-global fw logger with a static recorder serialized by a local mutex; if an unrelated sibling test registers its own global logger concurrently these tests could theoretically flake, mitigated by unique event-id markers. Sibling status: at final verification cargo build --workspace and cargo test -p fprime-svc were fully green (91 tests incl. sibling rate_group_driver); no broken sibling modules observed. Nothing committed per instructions.

## Rate/time components (RateGroupDriver, ActiveRateGroup, PassiveRateGroup, Health, PosixTime, LinuxTimer)

```text
All components: Arc-constructed, id_base via embedded base; ids/opcodes are FPP-relative associated consts. — RateGroupDriver: new(&str)->Arc; pub base: PassiveBase; pub cycle_out: [OutputPort<dyn CyclePort>; RATE_GROUP_DRIVER_CYCLE_PORTS=3]; struct Divider{divisor:FwSizeType, offset:FwSizeType} (Copy, Default=unused, const new(divisor,offset)); configure(&self, &[Divider;3]) once (second call fw_asserts); cycle_in(self:&Arc, port_num)->PortRef<dyn CyclePort> (sync). No events/tlm/cmds. — ActiveRateGroup: new(&str)->Arc; pub active: ActiveBase, pub evt: EventGlue (log/text/time), pub tlm: TlmGlue, pub rate_group_member_out: [OutputPort<dyn SchedPort>;10], pub ping_out: OutputPort<dyn PingPort>; configure(&self,[u32;10]); init(&self, queue_depth: FwSizeType) creates queue (msg size 32); inputs cycle_in(&Arc,n)->PortRef<dyn CyclePort> (async drop, msg type 1), ping_in(&Arc,n)->PortRef<dyn PingPort> (async assert, msg type 2); impl ComponentDispatch+ActiveComponent; lifecycle active.start(&arc,prio,stack,affinity)/exit/join; consts EVENTID_RATE_GROUP_STARTED=0, EVENTID_RATE_GROUP_CYCLE_SLIP=1, EVENTID_RATE_GROUP_TIME_GET_ERROR=2 (TIME_GET_ERROR_THROTTLE=5), CHANID_RG_MAX_TIME=0, CHANID_RG_CYCLE_SLIPS=1. — PassiveRateGroup: new(&str)->Arc; pub base: PassiveBase, pub cmd: CmdGlue, pub tlm: TlmGlue, pub time_out: OutputPort<dyn TimePort>, pub rate_group_member_out: [OutputPort<dyn SchedPort>;10]; configure(&self,[u32;10]); reg_commands(&self); inputs cycle_in (sync CyclePort), cmd_in (sync CmdPort — CLEAR_STATISTICS is a sync command); consts OPCODE_CLEAR_STATISTICS=0, CHANID_MAX_CYCLE_TIME=0, CHANID_CYCLE_TIME=1, CHANID_CYCLE_COUNT=2, CHANID_PORT_CYCLE_TIME=3, CHANID_PORT_CYCLE_TIME_HWM=4; pub struct CycleTimes(pub [u32;10]) impl Serialize (40 raw BE bytes); pub const PORT_CYCLE_TIME: bool = true. — Health: new(&str)->Arc; pub queued: QueuedBase, pub cmd: CmdGlue, pub evt: EventGlue, pub tlm: TlmGlue, pub ping_send: [OutputPort<dyn PingPort>;25], pub wdog_stroke: OutputPort<dyn WatchDogPort>; init(&self, queue_depth: FwSizeType) (queue msg size 128, stores drain bound); reg_commands(&self); set_ping_entries(&self, &[PingEntry], watch_dog_code: u32); pub struct PingEntry{warn_cycles:FwSizeType, fatal_cycles:FwSizeType, name:CmdStringArg} + PingEntry::new(warn,fatal,&str) (Clone, NOT Copy — FwString isn't Copy); inputs run_in (SYNC SchedPort), ping_return_in(&Arc, port_num)->PortRef<dyn PingPort> (async, msg type 1), cmd_in (async CmdPort, msg type 2); impl ComponentDispatch (no thread — drain happens inside Run); consts EVENTID_HLTH_PING_WARN=0, _PING_LATE=1, _PING_WRONG_KEY=2, _CHECK_ENABLE=3, _CHECK_PING=4, _CHECK_LOOKUP_ERROR=5, _PING_UPDATED=6, _PING_INVALID_VALUES=7; OPCODE_HLTH_ENABLE=0, _PING_ENABLE=1, _CHNG_PING=2; CHANID_PING_LATE_WARNINGS=0. Command wire args: Enabled as 1 raw u8; entry as u16-len+bytes string(40); warn/fatal u32 BE. — PosixTime: new(&str)->Arc; pub base: PassiveBase; set_time_context(&self, u8); time_get_port_in(&Arc, n)->PortRef<dyn TimePort> (sync; fills TbWorkstationTime/context/sec/usec). — LinuxTimer: new(&str)->Arc; pub base: PassiveBase; pub cycle_out: OutputPort<dyn CyclePort>; start_timer(&self, TimeInterval) blocks until quit; quit(&self); tick(&self) = one RawTime::now + CycleOut invocation (non-blocking helper for tests/manual drivers).
```

### Implementation notes / deviations

Deviations (all documented in code): (1) RateGroupDriver config is a OnceLock so the ISR-path handler is lock-free — a second configure() fw_asserts instead of replacing the table (C++ permits reconfigure; nothing in-tree reconfigures). (2) LinuxTimer loop follows the task spec (RawTime::now -> CycleOut -> sleep the REMAINING interval, drift-compensated) rather than the C++ TaskDelay ordering (delay first, fixed interval): the first tick fires immediately and quit is checked at loop top, so quit-during-sleep takes effect before the next tick; C++ latched raw-time-failure Fw::Logger message kept. (3) Enum command args follow the task's discipline: an invalid Enabled byte answers VALIDATION_ERROR (stock C++ generated code would fail at deserialize with FORMAT_ERROR); short/residual bytes are FORMAT_ERROR. (4) PassiveRateGroup's rawTimeSource configure parameter is not ported (the Rust OSAL has one RawTime source); PortCycleTime config constexpr is a module const (true). (5) 'update on change' telemetry is implemented per-channel in component state (first write always emits), since TlmGlue leaves on-change to the caller; HWM/max CAS loops use AtomicU32::fetch_max (identical semantics). (6) Health find_entry logs HLTH_CHECK_LOOKUP_ERROR for unknown names in BOTH HLTH_PING_ENABLE and HLTH_CHNG_PING (C++ parity: the helper logs). Health set_ping_entries takes the C++ watchDogCode parameter the task text omitted. (7) RateGroupTimeGetError (needs RawTime::now failure) and LinuxTimer's raw-time error log are unreachable without OS fault injection — implemented, untested at runtime. Untested-only path aside, every documented gotcha has a test (offset-0 first tick, rollover wrap, sender-thread slip flag with manual throttle up/down, fatal-before-warn equality, warn==fatal only FATAL, wrong-key-no-reset, bounded queue drain, clear-stats-keeps-cycles, timer quit). Verification: cargo build --workspace green; cargo test -p fprime-svc 91/91 (51 mine: 9 rate_group_driver + 9 active_rate_group + 11 passive_rate_group + 16 health + 3 posix_time + 3 linux_timer), full workspace test green; cargo clippy -p fprime-svc --all-targets -D warnings clean; cargo fmt -p fprime-svc --check clean (siblings converged mid-task; their transient cmd_dispatcher breakage resolved itself). Timing-based tests were stress-run 15x with no flakes. A scratch harness crate remains at the session scratchpad (svc-harness) — used only to test my modules in isolation while siblings were mid-write; not part of the repo.

## TlmChan + BufferManager

```text
TlmChan (crates/fprime-svc/src/tlm_chan.rs, module fprime_svc::tlm_chan):
- TlmChan::new(name: &str) -> Arc<TlmChan>. Pub fields: active: ActiveBase; evt: EventGlue (eventOut/eventOutText/timeCaller); pkt_send: OutputPort<dyn ComPort> (PktSend, context 0); ping_out: OutputPort<dyn PingPort>.
- Input factories (all fn(self: &Arc<Self>, port_num: FwIndexType)): tlm_recv_in -> PortRef<dyn TlmPort> (guarded sync); tlm_get_in -> PortRef<dyn TlmGetPort> (guarded sync); run_in -> PortRef<dyn SchedPort> (async, Assert policy — no drop); ping_in -> PortRef<dyn PingPort> (async, Assert policy).
- pub trait TlmGetPort: Send + Sync { fn invoke(&self, port_num: FwIndexType, id: FwChanIdType, time_tag: &mut Time, val: &mut TlmBuffer) -> TlmValid } — defined and exported HERE (does not exist in fprime-comp).
- Consts: MSG_TYPE_RUN=1, MSG_TYPE_PING_IN=2, QUEUE_MESSAGE_SIZE: FwSizeType = 16, RUN_PRIORITY=PING_IN_PRIORITY=1; assoc consts TlmChan::EVENTID_TLM_CHAN_EPOCH_PROCESSING_CAP_REACHED=0, TlmChan::EVENTID_TLM_CHAN_BUCKET_POOL_EXHAUSTED=1, TlmChan::BUCKET_POOL_EXHAUSTED_THROTTLE=10.
- No commands, no telemetry channels, no parameters. Topology: new -> active.queued.base.set_id_base -> connect pkt_send/ping_out/evt.{log_out,text_log_out,time_out} -> active.queued.create_queue(depth, QUEUE_MESSAGE_SIZE) -> active.start(&arc, prio, stack, affinity) -> ... -> active.exit()/join(). Implements ComponentDispatch + ActiveComponent; also impls TlmPort and TlmGetPort directly (guarded ports).

BufferManager (crates/fprime-svc/src/buffer_manager.rs, module fprime_svc::buffer_manager):
- BufferManager::new(name: &str) -> Arc<BufferManager>. Pub fields: base: PassiveBase; evt: EventGlue (eventOut/textEventOut/timeCaller); tlm: TlmGlue (tlmOut).
- pub struct BufferBin { pub buffer_size: FwSizeType, pub num_buffers: u16 }.
- Config: setup(&self, mgr_id: u16, bins: &[BufferBin]) — call after wiring, before any port traffic (handlers fw_assert setup); asserts >MAX_NUM_BINS(10) bins, total slots > u16::MAX, double setup. cleanup(&self) drops the pool (idempotent; re-setup allowed after).
- Input factories (all guarded sync, fn(self: &Arc<Self>, port_num)): buffer_get_callee_in -> PortRef<dyn BufferGetPort>; buffer_send_in -> PortRef<dyn BufferSendPort>; sched_in -> PortRef<dyn SchedPort>. Component itself impls BufferGetPort/BufferSendPort/SchedPort.
- Handed-out Buffer: context=(mgr_id<<16)|slot_index, size=request, capacity=slot size; failure returns Buffer::empty() (is_valid()==false). Returned buffers must come back via bufferSendIn with the original context and storage (bad returns fw_assert).
- Assoc consts: EVENTID_NO_BUFFS_AVAILABLE=0x00 (WARNING_HI, throttle 10, arg = FwSizeType u64 BE), EVENTID_NULL_EMPTY_BUFFER=0x01 (WARNING_HI, throttle 10, no args), EVENT_THROTTLE=10; CHANID_TOTAL_BUFFS=0, CHANID_CURR_BUFFS=1, CHANID_HI_BUFFS=2, CHANID_NO_BUFFS=3, CHANID_EMPTY_BUFFS=4 (all U32, on-change suppression built into the schedIn handler). No commands, no queue.
```

### Implementation notes / deviations

Deviations (documented in code): (1) TlmChan per-run cap: the private run_with_cap(cap) takes the cap as a parameter so tests can exercise deferral (with the default config MAX_ENTRIES_PER_RUN == HASH_BUCKETS deferral is unreachable); the flight path run_handler always passes the config const. (2) Events are emitted after releasing the guarded/state mutex (C++ guarded handlers hold theirs across log_* calls) — behaviorally equivalent, deadlock-safe. (3) TlmChan Run drains the inactive store with per-bucket locking (C++ reads it lock-free and locks only to clear the updated flag); the inactive store has no other writer so contention is unchanged. (4) Seed's C++ stack-address fold is reproduced with a safe &local-to-usize pointer cast; only the u32-FwChanIdType Murmur3 hash path is compiled (Wang16/u8 paths apply to narrower config types). (5) BufferManager owned-storage adaptation: memId/MemAllocator params dropped (slots own Box<[u8]> that moves out on get and back on return); the C++ data-pointer range asserts map to a storage-capacity == slot-size assert; double setup without cleanup fw_asserts (C++ would leak). Clippy caveat: my two modules produce zero clippy/rustfmt diagnostics and cargo test -p fprime-svc was fully green (173 tests, 36 mine); however at final check `cargo clippy -p fprime-svc --all-targets -- -D warnings` still fails on a SIBLING agent's in-progress file (fprime_deframer.rs lines 340/406/442, clippy::field_reassign_with_default) — polled for ~10 minutes without convergence; per instructions I did not touch their file. clippy_clean=true refers to my owned modules.

## Comms stack (Framer, Deframer, FrameAccumulator, Router, ComQueue, ComStub)

```text
Shared protocol constants live in fprime_svc::fprime_framer: START_WORD: u32 = 0xdeadbeef, HEADER_SIZE=8, TRAILER_SIZE=4, MIN_FRAME_SIZE=12.

FprimeFramer: FprimeFramer::new(name)->Arc<Self>; pub base: PassiveBase, evt: EventGlue; OutputPort fields buffer_allocate: OutputPort<dyn BufferGetPort>, buffer_deallocate: OutputPort<dyn BufferSendPort>, data_out/data_return_out: OutputPort<dyn ComDataWithContextPort>, com_status_out: OutputPort<dyn SuccessConditionPort>. Inputs: data_in(&Arc,port)->PortRef<dyn ComDataWithContextPort>, data_return_in(..)->PortRef<dyn ComDataWithContextPort>, com_status_in(..)->PortRef<dyn SuccessConditionPort>. clear_no_buffer_available_throttle(). Consts EVENTID_NO_BUFFER_AVAILABLE=0 (throttle 5).

FprimeDeframer: new(name)->Arc; base, evt; data_out, data_return_out: OutputPort<dyn ComDataWithContextPort>. Inputs data_in (guarded), data_return_in -> PortRef<dyn ComDataWithContextPort>. EVENTID_INVALID_BUFFER_RECEIVED=0, INVALID_START_WORD=1, INVALID_LENGTH_RECEIVED=2, INVALID_CHECKSUM=3, PAYLOAD_TOO_SHORT=4.

FrameAccumulator: new(name)->Arc; configure(&self, detector: Box<dyn FrameDetector>, store_size: usize) (allocator params dropped; CircularBuffer owns storage); base, evt; buffer_allocate: OutputPort<dyn BufferGetPort>, buffer_deallocate: OutputPort<dyn BufferSendPort>, data_out, data_return_out: OutputPort<dyn ComDataWithContextPort>. Inputs data_in (guarded), data_return_in. pub trait FrameDetector { fn detect(&self, ring:&CircularBuffer)->DetectorStatus } with pub enum DetectorStatus{FrameDetected(usize),NoFrameDetected,MoreDataNeeded(usize)}; pub struct FprimeFrameDetector (const new()). EVENTID_NO_BUFFER_AVAILABLE=0, FRAME_DETECTION_SIZE_ERROR=1 (arg FwSizeType u64 BE), FRAME_DETECTION_VALID_FRAME_DROPPED=2.

FprimeRouter: new(name)->Arc; base, evt; command_out: OutputPort<dyn ComPort> (must be connected), file_out: OutputPort<dyn BufferSendPort>, unknown_data_out: OutputPort<dyn ComDataWithContextPort>, data_return_out: OutputPort<dyn ComDataWithContextPort>. Inputs data_in (guarded ComDataWithContextPort), file_buffer_return_in (guarded BufferSendPort), cmd_response_in (sync CmdResponsePort, no-op). BUFFER_CONTEXT_TABLE_SIZE=50; EVENTID_SERIALIZATION_ERROR=0, DESERIALIZATION_ERROR=1 (declared, never emitted — C++ parity), FILE_OUT_CONTEXT_TABLE_FULL=2, UNKNOWN_DATA_OUT_CONTEXT_TABLE_FULL=3, BUFFER_CONTEXT_NOT_FOUND=4.

ComQueue (ACTIVE): new(name)->Arc; pub active: ActiveBase, cmd: CmdGlue, evt: EventGlue, tlm: TlmGlue; data_out: OutputPort<dyn ComDataWithContextPort>, buffer_return_out: [OutputPort<dyn BufferSendPort>; BUFFER_PORT_COUNT]. configure(&self, &QueueConfigurationTable) — QueueConfigurationTable{entries:[QueueConfigurationEntry; TOTAL_PORT_COUNT]} with QueueConfigurationEntry{depth:FwSizeType, priority:FwIndexType (0..TOTAL, lower first), mode:QueueMode, overflow_mode:QueueOverflowMode} (Default depth 0 — must be set >0). reg_commands(). Inputs: com_status_in->PortRef<dyn SuccessConditionPort> (async assert, msg 1), com_packet_queue_in(port 0..2)->PortRef<dyn ComPort> (async drop, msg 2, envelope [i32][i16][u16-len ComBuffer][u32 ctx]), buffer_queue_in(port 0..1)->PortRef<dyn BufferSendPort> (async hook via escrow token u64, msg 3), run_in->PortRef<dyn SchedPort> (async drop, msg 4), data_return_in->PortRef<dyn ComDataWithContextPort> (SYNC), cmd_in->PortRef<dyn CmdPort> (async, msg 5). Topology: configure -> connect -> active.queued.create_queue(depth, com_queue::MSG_SIZE /*=524*/) -> reg_commands -> active.start(&arc,..). Consts: COM_PORT_COUNT=2, BUFFER_PORT_COUNT=1, TOTAL_PORT_COUNT=3, OPCODE_FLUSH_QUEUE=0/FLUSH_ALL_QUEUES=1/SET_QUEUE_PRIORITY=2, EVENTID_QUEUE_OVERFLOW=0 (args [QueueType u8][index i16])/QUEUE_PRIORITY_CHANGED=1 ([u8][i16][i16]), CHANID_COM_QUEUE_DEPTH=0/BUFF_QUEUE_DEPTH=1; pub enum QueueType{ComQueue=0,BufferQueue=1} (TryFrom<u8>); pub ComQueueDepth([u32;2])/BuffQueueDepth([u32;1]) impl Serialize (raw u32 BE elements). Command args on the wire: FLUSH_QUEUE=[u8 queueType][i16 index]; SET_QUEUE_PRIORITY=[u8][i16][i16].

ComStub: new(name)->Arc; base; data_out, data_return_out: OutputPort<dyn ComDataWithContextPort>, com_status_out: OutputPort<dyn SuccessConditionPort>, drv_send_out: OutputPort<dyn ByteStreamSendPort>, drv_receive_return_out: OutputPort<dyn BufferSendPort>. Inputs: data_in, data_return_in -> PortRef<dyn ComDataWithContextPort>; drv_connected -> PortRef<dyn ByteStreamReadyPort>; drv_receive_in -> PortRef<dyn ByteStreamDataPort>. RETRY_LIMIT: FwIndexType = 10. Defines pub enum ByteStreamStatus{OpOk=0,SendRetry=1,RecvNoData=2,OtherError=3} (repr u8) and pub traits ByteStreamSendPort{invoke(&self,port,&mut Buffer)->ByteStreamStatus}, ByteStreamReadyPort{invoke(&self,port)}, ByteStreamDataPort{invoke(&self,port,Buffer,ByteStreamStatus)} — the ref topology bridges these to fprime-drv's identical traits with one-line adapter shims.
```

### Implementation notes / deviations

Deviations/decisions (all documented in module headers): (1) FprimeRouter context table keys on the buffer's data address (`data().as_ptr()`) exactly like the C++ `getData()` key — the Rust `Buffer` owns its `Box<[u8]>` storage, so the heap address is stable across port moves (and the async escrow) and unique among outstanding allocations; the buffer's context word is never touched, an un-tracked (table-full) buffer can never match another entry on return, and observable behavior (table-full events, restoration, BufferContextNotFound+default on miss) is identical. (2) ComQueue storage: com queues persist fixed 514-byte [u16 len][bytes][pad] records and buffer queues persist 8-byte BufferEscrow tokens (C++ serializes ComBuffer/Fw::Buffer objects; the token occupies the same 8-byte slot as the C++ pointer); DROP_OLDEST pre-emptive pop still returns ownership before overwrite. (3) ComQueue send path builds an owned Buffer from recycled BufferStorage instead of aliasing m_dequeued_com_buffer; com-buffer returns on dataReturnIn recycle the storage (C++ drops them) so steady state is allocation-free after the first send. (4) C++ casts the leading packet descriptor to Apid unchecked (ComQueue) — Rust maps unknown descriptors to Apid::InvalidUninitialized; the deframer's mapping matches C++ exactly. (5) ComStub implements only the synchronous driver path; the async ports (drvAsyncSendOut/drvAsyncSendReturnIn) are not ported in phase 1 (no async byte-stream driver exists) — noted in com_stub.rs. (6) ByteStreamStatus + ByteStream port traits are defined publicly in com_stub.rs because fprime-svc may not depend on fprime-drv; fprime-drv defines its own identical traits and fprime-ref glues them with tiny shims. (7) Deframer keeps the exact C++ handler order: APID extraction (and a possible PayloadTooShort WARNING_LO) happens BEFORE the CRC check, so a short-payload bad-CRC frame emits both events; drops always use the ORIGINAL context. (8) FrameAccumulator::configure drops the MemAllocator params (CircularBuffer owns its ring); C++'s unchecked peek into a smaller-than-requested allocation (UB) is a clean fw_assert here. (9) All three ComQueue commands (FLUSH_QUEUE, FLUSH_ALL_QUEUES, SET_QUEUE_PRIORITY) are implemented — nothing deferred; invalid QueueType enum args answer ValidationError, deser failures/residual bytes answer FormatError per the task's discipline (C++ generated code folds enum validity into FormatError). (10) Event ids follow FPP declaration order (0,1,2,...) per the .fpp files; ComQueue telemetry ids 0/1 are explicit in the fppi. No sibling-module breakage observed: full workspace build and test are green (504 passed / 0 failed), fprime-svc full-crate green seen repeatedly.

## lib.rs, byte_stream.rs, socket_helper.rs, tcp_client.rs, tcp_server.rs, tcp_loopback.rs

```text
CRATE fprime_drv (all re-exported at root; modules: byte_stream, socket_helper, tcp_client, tcp_server).

byte_stream:
  #[repr(u8)] enum ByteStreamStatus { OpOk=0 (Default), SendRetry=1, RecvNoData=2, OtherError=3 } (Copy, Eq, Hash, TryFrom<u8>, fprime_fw Serialize/Deserialize at u8 width, strict decode)
  trait ByteStreamSendPort: Send+Sync { fn invoke(&self, port_num: FwIndexType, buffer: &mut Buffer) -> ByteStreamStatus }
  trait ByteStreamDataPort: Send+Sync { fn invoke(&self, port_num: FwIndexType, buffer: Buffer, status: ByteStreamStatus) }  // buffer by move
  trait ByteStreamReadyPort: Send+Sync { fn invoke(&self, port_num: FwIndexType) }
  pub use fprime_comp::BufferSendPort  // recvReturn direction
  CANONICAL traits: fprime-svc's com_stub duplicates them locally (crates may not depend on each other); ref deployment writes 2-line shims bridging svc traits -> these.

socket_helper:
  #[must_use] #[repr(i32)] enum SocketIpStatus { Success=0, FailedToGetSocket=-1, FailedToGetHostIp=-2, InvalidIpAddress=-3, FailedToConnect=-4, FailedToSetSocketOptions=-5, InterruptedTryAgain=-6, ReadError=-7, Disconnected=-8, FailedToBind=-9, FailedToListen=-10, FailedToAccept=-11, SendError=-13, NotStarted=-14, FailedToReadBackPort=-15, NoDataAvailable=-16, AnotherThreadOpening=-17, AutoConnectDisabled=-18, InvalidCall=-19 }
  const SOCKET_MAX_ITERATIONS: usize = 0xFFFF
  struct Timing (Copy, Default=C++ values) { pub reconnect_check_interval: Duration /*50ms*/, reconnect_wait_interval /*10ms*/, reconnect_wait_timeout /*1s*/, retry_interval /*1s*/, read_timeout: Option<Duration> /*Some(500ms); None=pure C++ blocking*/, write_timeout: Option<Duration> /*Some(1s)*/, connect_timeout: Duration /*1s*/ }
  trait SocketWorker: Send+Sync+'static { fn helper(&self)->&SocketHelper; fn open_protocol(&self)->Result<TcpStream,SocketIpStatus>; fn get_buffer(&self)->Buffer; fn send_buffer(&self, Buffer, SocketIpStatus); fn connected(&self); fn read_loop(&self) /*default = helper().read_loop_body(self); TcpServer overrides*/ }
  struct SocketHelper (Default): fn new(); set_timing(&self, Timing) /*before start*/; timing()->Timing; start<C:SocketWorker>(&self, &Arc<C>, name:&str) /*spawns reconnect then read thread; fw_asserts double-start*/; open<C>(&self,&C)->SocketIpStatus; is_opened()->bool; set_automatic_open(&self,bool); get_automatic_open()->bool; send(&self,&[u8])->SocketIpStatus; recv(&self,&mut [u8],&mut usize)->SocketIpStatus; shutdown(); close(); stop(); running()->bool; running_reconnect()->bool; stop_reconnect(); join()->fprime_os::task::Status; request_reconnect(); wait_for_reconnect()->SocketIpStatus; read_loop_body<C>(&self,&C); reconnect_loop_body<C>(&self,&C)
  fn byte_stream_recv_status(SocketIpStatus)->ByteStreamStatus  // Success->OpOk, NoDataAvailable->RecvNoData, else OtherError
  fn byte_stream_send_status(SocketIpStatus)->ByteStreamStatus  // InterruptedTryAgain->SendRetry, Success->OpOk, else OtherError

tcp_client::TcpClient / tcp_server::TcpServer (identical surface unless noted):
  fields: pub base: PassiveBase; pub allocate_out: OutputPort<dyn BufferGetPort>; pub deallocate_out: OutputPort<dyn BufferSendPort>; pub recv_out: OutputPort<dyn ByteStreamDataPort>; pub ready_out: OutputPort<dyn ByteStreamReadyPort>
  fn new(name:&str)->Arc<Self>
  fn configure(&self, hostname:&str, port:u16, buffer_size:FwSizeType, reconnect:bool)->SocketIpStatus
    // client: fw_asserts port!=0; returns Success (address validated at open, inet_pton parity: dotted-quad only, no DNS)
    // server: port 0 = ephemeral; binds+listens NOW and returns startup status
  fn start(self:&Arc<Self>) /*spawns both threads; fw_asserts configured*/; fn stop(&self); fn join(&self)->fprime_os::task::Status; fn socket_helper(&self)->&SocketHelper /*set_timing etc.*/
  input factories: fn send_in(self:&Arc<Self>, port_num)->PortRef<dyn ByteStreamSendPort> (guarded); fn recv_return_in(self:&Arc<Self>, port_num)->PortRef<dyn BufferSendPort> (guarded, forwards to deallocate_out)
  TcpServer extra: fn startup(&self)->SocketIpStatus /*idempotent while listening*/; fn get_listen_port(&self)->u16; fn is_started(&self)->bool; fn terminate(&self)
  Both impl SocketWorker (open_protocol = connect / accept), ByteStreamSendPort, BufferSendPort.
  recv_out delivers with port_num from the wired connection; buffers are Buffer::allocate'd via allocate_out with the configured buffer_size; recv delivery statuses include periodic RecvNoData when Timing::read_timeout is set (benign, ComStub returns them immediately).
  Topology wiring order: new -> set_id_base -> connect allocate/deallocate/recv/ready outs -> configure -> start; teardown: stop -> join (server terminate() also closes the listener).
```

### Implementation notes / deviations

Full workspace build green; cargo test -p fprime-svc also fully green (208 tests) — sibling modules converged, no blockers. Documented behavior-preserving divergences (all in doc comments): (1) TcpStream handles are duplicated per operation via try_clone (the C++ copy-fd-out-of-lock idiom); close()/stop() use TcpStream::shutdown(Both) to break blocking recvs, plus an optional per-recv read timeout (Timing::read_timeout, default 500 ms, None = exact C++ blocking behavior) as belt-and-braces — a timeout maps to SOCK_NO_DATA_AVAILABLE/RecvNoData exactly like C++ EAGAIN, so upstream sees periodic benign RecvNoData deliveries. (2) TcpServer: std cannot set listen backlog (C++ uses 1) nor SO_REUSEADDR, and cannot shutdown a TcpListener, so the listener is non-blocking and open_protocol polls accept at reconnect_wait_interval checking stop flags (equivalent responsiveness); std bind() conflates bind/listen failures into FailedToBind. (3) Client connect uses connect_timeout (Timing::connect_timeout, 1 s) to keep the reconnect loop responsive; C++ blocks in connect(2). (4) EBADF is unrepresentable through std io::ErrorKind; ConnectionReset/ConnectionAborted cover the C++ ECONNRESET/EBADF -> DISCONNECTED branch. (5) configure() drops the C++ send-timeout parameters per the task signature; timeouts live in SocketHelper::set_timing (defaults = IpCfg.hpp values). (6) The full C++ send-when-closed path is ported (requestReconnect + bounded waitForReconnect, then send on success), not just the OtherError shortcut — covered by the send_before_connect integration test. Loopback tests run 6x consecutively green (~60 ms/suite); all waits are bounded deadline polls. Did not commit or push, wrote only under crates/fprime-drv/.

# Remaining-subsystem crate API notes

## PrmDb

```text
CRATE fprime_svc, module `prm_db` (fprime_svc::prm_db::*).

consts: NUM_DB_ENTRIES: usize = 25; ENTRY_DELIMITER: u8 = 0xA5; MIN_RECORD_SIZE: u32 = 4; MAX_RECORD_SIZE: u32 = 510; QUEUE_MSG_SIZE: usize = 522.

fpp_enum types (u8 repr, Serialize/Deserialize/TryFrom/as_repr/VALUES):
  PrmDbType { DbActive=0, DbStaging=1 } (default DbActive)
  PrmDbFileLoadState { Idle=0, LoadingFileUpdates=1, FileUpdatesStaged=2 } (default Idle)
  PrmReadError { Open=0, Delimiter=1, DelimiterSize=2, DelimiterValue=3, RecordSize=4, RecordSizeSize=5, RecordSizeValue=6, ParameterId=7, ParameterIdSize=8, ParameterValue=9, ParameterValueSize=10, Crc=11, CrcSize=12, CrcBuffer=13, SeekZero=14 }
  PrmWriteError { Open=0, Delimiter=1, DelimiterSize=2, RecordSize=3, RecordSizeSize=4, ParameterId=5, ParameterIdSize=6, ParameterValue=7, ParameterValueSize=8, CrcPlace=9, CrcReal=10, CurrPosition=11, SeekZero=12, SeekPosition=13 }
  Merge { Merge=0, Reset=1 } (default Merge)
  PrmLoadAction { SetParameter=0, SaveFileCommand=1, LoadFileCommand=2, CommitStagedCommand=3 }
plain enums: PrmUpdateType { NoSlots, ParamAdded, ParamUpdated }; PrmLoadStatus { Success, Error }.
pub struct PrmDbStore (Debug): pub fn len()->usize; is_empty()->bool; iter()->impl Iterator<Item=(FwPrmIdType, &ParamBuffer)> (insertion order). Construction/mutation are internal to the component.

pub struct PrmDb (use as Arc<PrmDb>):
  pub fields: active: ActiveBase, cmd: CmdGlue, evt: EventGlue, ping_out: OutputPort<dyn PingPort>
  fn new(name:&str)->Arc<Self>
  fn init(&self, queue_depth: FwSizeType)                 // creates the queue, msg size QUEUE_MSG_SIZE
  fn configure(&self, file:&str)                          // parameter file name (C++ configure)
  fn configure_load_sandbox(&self, directory:&str)        // optional; restricts commanded PRM_LOAD_FILE paths
  fn read_param_file(&self)                               // boot-time load into ACTIVE; run BEFORE active.start
  fn reg_commands(&self)                                  // registers opcodes 0,1,2
  input factories (all fn x(self:&Arc<Self>, port_num: FwIndexType) -> PortRef<dyn ...>):
    get_prm  -> dyn PrmGetPort   (GUARDED, returns ParamValid)
    set_prm  -> dyn PrmSetPort   (ASYNC)
    ping_in  -> dyn PingPort     (ASYNC)
    cmd_in   -> dyn CmdPort      (ASYNC, own commands)
  msg types: PrmDb::MSG_TYPE_SET_PRM=1, MSG_TYPE_PING_IN=2, MSG_TYPE_CMD=3
  opcodes: OPCODE_PRM_SAVE_FILE=0x00, OPCODE_PRM_LOAD_FILE=0x01, OPCODE_PRM_COMMIT_STAGED=0x02
  event ids: EVENTID_PRM_ID_NOT_FOUND=0 (WARNING_LO, PRM_ID_NOT_FOUND_THROTTLE=5), EVENTID_PRM_ID_UPDATED=1 (ACTIVITY_HI), EVENTID_PRM_DB_FULL=2 (WARNING_HI), EVENTID_PRM_ID_ADDED=3 (ACTIVITY_HI), EVENTID_PRM_FILE_WRITE_ERROR=4 (WARNING_HI), EVENTID_PRM_FILE_SAVE_COMPLETE=5 (ACTIVITY_HI), EVENTID_PRM_FILE_READ_ERROR=6 (WARNING_HI), EVENTID_PRM_FILE_LOAD_COMPLETE=7 (ACTIVITY_HI), EVENTID_PRM_DB_COMMIT_COMPLETE=8 (ACTIVITY_HI), EVENTID_PRM_DB_COPY_ALL_COMPLETE=9 (ACTIVITY_HI), EVENTID_PRM_DB_FILE_LOAD_FAILED=10 (WARNING_HI), EVENTID_PRM_DB_FILE_LOAD_INVALID_ACTION=11 (WARNING_LO), EVENTID_PRM_FILE_BAD_CRC=12 (WARNING_HI)
  no telemetry channels.
  impls ComponentDispatch + ActiveComponent.

Topology order: PrmDb::new -> set_id_base -> connect (cmd.cmd_reg_out, cmd.cmd_response_out, evt.log_out/text_log_out/time_out, ping_out; wire get_prm/set_prm/ping_in/cmd_in into the users, dispatcher and health) -> init(depth) -> configure(path) [-> configure_load_sandbox(dir)] -> read_param_file() -> reg_commands() -> active.start(&arc, prio, stack, affinity) -> ... -> active.exit()/join(). Components' load_parameters() must run after read_param_file().
```

### Implementation notes / deviations

Deviations / decisions (each documented in the source):
1. Two owned stores + `std::mem::swap` under the component lock replace the C++ `PrmDbStore*` pair; the swap is inside the same lock the guarded getPrm takes, so getPrm never observes a half-swapped pair. Backing Vecs are `with_capacity(25)` at construction and never grow.
2. `getPrm` drops the state lock before emitting PrmIdNotFound (C++ holds the guarded mutex across the log call). CONVENTIONS.md requires output-port invocations outside the state lock; behavior is otherwise identical. The save loop DOES hold the lock across the whole file write, matching the C++ lock()/unLock() bracket, and emits nothing while holding it.
3. **[Superseded in phase 2 — see "Phase 2" below; the helpers now live in `fprime-os`.]** `Os::SandboxedFile` / `Os::FilePathUtils` were not ported in fprime-os, so the sandbox was implemented inline in this module: `resolve_path`/`resolve_from_cwd` (lexical `.`/`..`/`//` resolution against the cwd, MAX_PATH_LENGTH = FILE_NAME_STRING_SIZE = 240) plus `checkContainment` semantics, mapping a violation onto `file::Status::OutsideSandbox` reported as PrmFileReadError(Open, 0, 12). Unit-tested against the C++ algorithm's cases. If fprime-os later gains SandboxedFile, these two helpers should be deleted in favor of it.
4. C++ parity quirks kept deliberately (each has a test): (a) `OPEN_WRITE` does not truncate, so a shorter image over a longer file leaves residue and the next load fails the CRC (the CRC covers everything to EOF); (b) `PRM_LOAD_FILE`'s file name is a `Fw::CmdStringArg` (40 bytes) even though the FPP model declares `string size FileNameStringSize` (240) — a longer name is a FormatError, exactly as the C++ generated handler behaves (verified against PrmDbImpl.hpp and CmdSequencer's identical shape); (c) records past the 25th are silently ignored; (d) a dropped record fails the load only after the remaining records are processed.
5. U32 fields are read/written with `to_be_bytes`/`from_be_bytes` rather than through a `WorkingBuffer` — identical encoding to the Fw serialize layer, locked down by the literal-byte file test.
6. Events with `string` args serialize via `LogStringArg::serialize_to_truncated(.., 80, ..)` (FPP default string size).
7. Test scratch directories are deliberately short (`/tmp/fpdb<pid>_<n>`) because PRM_LOAD_FILE path arguments are capped at 40 bytes; a very long `TMPDIR` would break those tests.
8. Nothing was deliberately left unported. Sibling report: `cargo clippy -p fprime-svc --all-targets -- -D warnings` failed for a while on `crates/fprime-svc/src/file_uplink.rs:448` (clippy::int_plus_one, another agent's file — I did not touch it); it was fixed by that agent and the final whole-crate clippy, build (`cargo build --workspace`), `cargo test -p fprime-svc` (244 tests) and `cargo doc` runs are all green, with `rustfmt --edition 2024 --check` clean on prm_db.rs. I ran rustfmt on my file only rather than `cargo fmt -p fprime-svc`, to avoid rewriting siblings' in-progress files.

## File services (FilePacket, CFDP checksum, FileUplink/Downlink/Manager)

```text
```text
fprime_fw::file_packet (module fprime_fw::file_packet; NOT re-exported at the crate root — use the full path)
  const HEADER_SIZE: usize = 5; DATA_PACKET_HEADER_SIZE: usize = 11; PATH_NAME_MAX_LENGTH: usize = 255
  fpp_enum FilePacketType : u8 { Start=0, Data=1, End=2, Cancel=3, None=255 } (Default=None, TryFrom<u8>, Serialize/Deserialize u8, as_repr(), SERIALIZED_SIZE)
  struct Header (Copy, Eq, Default) { pub packet_type: FilePacketType, pub sequence_index: u32 }: const fn new(FilePacketType, u32); const fn buffer_size()->usize
  struct StartPacket<'a> (Copy, Eq) { pub header: Header, pub file_size: u32, pub source_path: &'a [u8], pub destination_path: &'a [u8] }
    fn initialize(file_size: u32, source_path: &'a [u8], destination_path: &'a [u8]) -> Self   // forces sequence_index = 0, truncates paths to 255
    fn buffer_size(&self) -> usize
  struct DataPacket<'a> (Copy, Eq) { pub header, pub byte_offset: u32, pub data_size: u16, pub data: &'a [u8] }
    fn initialize(sequence_index: u32, byte_offset: u32, data_size: u16, data: &'a [u8]) -> Self  // trims data to data_size
    const fn fixed_length_size(&self)->usize; fn buffer_size(&self)->usize
  struct EndPacket (Copy, Eq) { pub header, pub checksum_value: u32 }: const fn initialize(seq: u32, checksum_value: u32); const fn buffer_size()->usize
  struct CancelPacket (Copy, Eq) { pub header }: const fn initialize(seq: u32); const fn buffer_size()->usize
  enum FilePacket<'a> (Copy, Eq) { Start(StartPacket<'a>), Data(DataPacket<'a>), End(EndPacket), Cancel(CancelPacket) }
    fn header(&self)->Header; fn packet_type(&self)->FilePacketType; fn buffer_size(&self)->usize
    fn serialize_to(&self, buf: &mut dyn SerBufAny) -> SerializeStatus          // always big-endian
    fn from_buffer(data: &'a [u8]) -> Result<FilePacket<'a>, SerializeStatus>   // Err carries the C++ status (DecodeError arg)

fprime_utils::cfdp (module fprime_utils::cfdp; NOT re-exported at the crate root)
  struct Checksum (Debug, Clone, Copy, Default, PartialEq, Eq, Hash):
    const fn new(); const fn from_value(u32); const fn get_value(&self)->u32
    fn update(&mut self, data: &[u8], file_offset: u32)     // any alignment/order; equals the whole-file value
    fn add_byte_at_offset(&mut self, byte: u8, offset: u8)  // offset is a WORD offset 0..=3

fprime_svc::file_uplink (ACTIVE)
  pub trait FileAnnouncePort: Send + Sync { fn invoke(&self, port_num: FwIndexType, file_name: &mut FileNameString) }
  pub const QUEUE_MSG_SIZE: FwSizeType = 14; MAX_PATH_LENGTH: usize = 240
  pub enum PathStatus { Valid=0, OutsideSandbox=1, InvalidPath=2, TooLong=3 }
  pub fn resolve_path(path: &str, base_dir: &str, resolved: &mut FileNameString) -> PathStatus
  pub fn resolve_from_cwd(path: &str, resolved: &mut FileNameString) -> PathStatus
  pub fn check_containment(resolved_path: &str, allowed_directory: &str) -> PathStatus
  pub struct SandboxedFile (Default = fail-open "/"): new(); configure(&mut self, dir: &str); sandbox_directory()->&FileNameString; is_configured()->bool;
      open(&mut self, path:&str, mode: os::file::Mode)->os::file::Status; close(); is_open(); size(&mut FwSizeType); seek(i64, SeekType); read(&mut [u8], &mut FwSizeType, WaitType); write(&[u8], &mut FwSizeType, WaitType)
  pub struct FileUplink { pub active: ActiveBase, pub evt: EventGlue, pub tlm: TlmGlue,
      pub buffer_send_out: OutputPort<dyn BufferSendPort>, pub ping_out: OutputPort<dyn PingPort>,
      pub file_announce: OutputPort<dyn FileAnnouncePort> }
    fn new(name:&str)->Arc<Self>; fn init(&self, queue_depth: FwSizeType); fn configure(&self, directory: &str)
    fn buffer_send_in(self:&Arc<Self>, port_num)->PortRef<dyn BufferSendPort>   // async
    fn ping_in(self:&Arc<Self>, port_num)->PortRef<dyn PingPort>                // async
    impl ComponentDispatch + ActiveComponent. No commands.
    consts EVENTID_BAD_CHECKSUM=0, FILE_OPEN_ERROR=1, FILE_RECEIVED=2, FILE_WRITE_ERROR=3, INVALID_RECEIVE_MODE=4,
      PACKET_OUT_OF_BOUNDS=5, PACKET_OUT_OF_ORDER=6, PACKET_DUPLICATE=7, UPLINK_CANCELED=8, DECODE_ERROR=9,
      INVALID_PACKET_RECEIVED=10; THROTTLE_5=5, THROTTLE_20=20;
      CHANID_FILES_RECEIVED=0, PACKETS_RECEIVED=1, WARNINGS=2, FILES_RECEIVED_FAILED=3
  Topology: new -> active.queued.base.set_id_base -> connect(evt.log_out/evt.text_log_out/evt.time_out, tlm.tlm_out,
      buffer_send_out, ping_out, [file_announce]) -> init(depth) -> active.start(&arc,prio,stack,affinity) -> exit()/join()

fprime_svc::file_downlink (ACTIVE)
  pub const INTERNAL_BUFFER_SIZE=512; MAX_DATA_SIZE=499; FILE_DOWN_COMPLETE_PORTS=1;
        COMMAND_FAILURES_DISABLED: bool = true; QUEUE_MSG_SIZE: FwSizeType = 522
  fpp_enum SendFileStatus : u8 { StatusOk=0, StatusError=1, StatusInvalid=2, StatusBusy=3 }
  fpp_struct SendFileResponse { status: SendFileStatus {get_status,set_status}, context: u32 {get_context,set_context} }
        (Copy, Eq, SERIALIZED_SIZE=5, new(status, context), wire = [u8][u32 BE])
  pub type SendFileNameArg = FwString<100>
  pub trait SendFileRequestPort { fn invoke(&self, port_num, source: &SendFileNameArg, dest: &SendFileNameArg, offset: u32, length: u32) -> SendFileResponse }
  pub trait SendFileCompletePort { fn invoke(&self, port_num, resp: SendFileResponse) }
  pub enum Mode { Idle=0, Downlink=1, Cancel=2, Wait=3, Cooldown=4 }; pub enum CallerSource { Command=0, Port=1 }
  pub struct FileEntry { src_filename/dest_filename: FileNameString, offset, length: u32, source: CallerSource, op_code: FwOpcodeType, cmd_seq, context: u32 } (Default)
  pub struct FileDownlink { pub active: ActiveBase, pub cmd: CmdGlue, pub evt: EventGlue, pub tlm: TlmGlue,
      pub buffer_send_out: OutputPort<dyn BufferSendPort>,
      pub file_complete: [OutputPort<dyn SendFileCompletePort>; 1],
      pub ping_out: OutputPort<dyn PingPort> }
    fn new(name:&str)->Arc<Self>; fn configure(&self, cooldown: u32, cycle_time: u32, file_queue_depth: usize);
    fn configure_sandbox(&self, directory: &str); fn init(&self, queue_depth: FwSizeType); fn reg_commands(&self); fn mode(&self)->Mode
    fn run_in(&Arc,port)->PortRef<dyn SchedPort> (async, msg 1); fn buffer_return_in(..)->PortRef<dyn BufferSendPort> (async, msg 2, escrow);
    fn ping_in(..)->PortRef<dyn PingPort> (async, msg 3); fn cmd_in(..)->PortRef<dyn CmdPort> (async, msg 4);
    fn send_file_in(..)->PortRef<dyn SendFileRequestPort> (GUARDED, component impls the trait)
    consts OPCODE_SEND_FILE=0x00, OPCODE_CANCEL=0x01, OPCODE_SEND_PARTIAL=0x02;
      EVENTID_FILE_OPEN_ERROR=0x00, FILE_READ_ERROR=0x01, FILE_SENT=0x02, DOWNLINK_CANCELED=0x03,
      DOWNLINK_PARTIAL_WARNING=0x05, DOWNLINK_PARTIAL_FAIL=0x06, SEND_DATA_FAIL=0x07, SEND_STARTED=0x08,
      DOWNLINK_ZERO_SIZE_FILE=0x09, FILENAME_SOURCE_OVERFLOW=0x10, FILENAME_DESTINATION_OVERFLOW=0x11,
      SOURCE_OUT_OF_SANDBOX=0x12; CHANID_FILES_SENT=0x00, PACKETS_SENT=0x01, WARNINGS=0x02
    Command wire args: SendFile = [CmdStringArg src][CmdStringArg dst]; Cancel = none;
      SendPartial = [CmdStringArg src][CmdStringArg dst][u32 startOffset][u32 length]
  Topology: new -> set_id_base -> connect(cmd.cmd_reg_out/cmd.cmd_response_out, evt.*, tlm.tlm_out, buffer_send_out,
      file_complete[0], ping_out) -> configure(cooldown,cycle,depth) [-> configure_sandbox] -> init(depth)
      -> reg_commands() -> active.start(&arc,..) -> exit()/join()

fprime_svc::file_manager (ACTIVE)
  pub const FILES_PER_RATE_TICK=1, GENERATE_DP_MAX_CHUNK_SIZE=1024, CHUNKS_PER_RATE_TICK=1, DEFAULT_DP_PRIORITY=10;
        QUEUE_MSG_SIZE: FwSizeType = 522
  fpp_enum GenerateDpStage : i32 { Open=0, Size=1, Seek=2, Read=3, Serialize=4, Busy=5 }
  fpp_enum GenerateDpMode : i32 { Paced=0, Immediate=1 }
  fpp_enum StringFormatStatus : u8 { Success=0, Overflowed=1, InvalidFormatString=2, SizeOverflow=3, OtherError=4 }
  pub struct FileManager { pub active: ActiveBase, pub cmd: CmdGlue, pub evt: EventGlue, pub tlm: TlmGlue,
      pub ping_out: OutputPort<dyn PingPort> }
    fn new(name:&str)->Arc<Self>; fn init(&self, queue_depth: FwSizeType); fn reg_commands(&self)
    fn ping_in(&Arc,port)->PortRef<dyn PingPort> (async, msg 1); fn cmd_in(..)->PortRef<dyn CmdPort> (async, msg 2);
    fn sched_in(..)->PortRef<dyn SchedPort> (SYNC — component impls SchedPort; internal run message is msg 3, DROP policy)
    consts OPCODE_CREATE_DIRECTORY=0x00, MOVE_FILE=0x01, REMOVE_DIRECTORY=0x02, REMOVE_FILE=0x03, APPEND_FILE=0x05,
      FILE_SIZE=0x06, LIST_DIRECTORY=0x07, CALCULATE_CRC=0x08, GENERATE_DP=0x09   (0x04 is a gap)
      EVENTID_* 0x00..0x22 exactly per Events.fppi (gaps 0x04, 0x07, 0x0D);
      CHANID_COMMANDS_EXECUTED=0x00, CHANID_ERRORS=0x01
    Command wire args (all strings are CmdStringArg = [u16 len][bytes], max 40): CreateDirectory/RemoveDirectory/
      FileSize/ListDirectory/CalculateCrc = [str]; MoveFile/AppendFile = [str][str];
      RemoveFile = [str][bool 0xFF/0x00]; GenerateDp = [str][u32 chunkSize][u64 begin][u64 end][u32 priority][i32 mode]
  Topology: new -> set_id_base -> connect(cmd.*, evt.*, tlm.tlm_out, ping_out) -> init(depth) -> reg_commands()
      -> active.start(&arc,..); wire sched_in into a rate group -> exit()/join()
```
```

### Implementation notes / deviations

DEVIATIONS (each documented in the code)

1. `FilePacket::from_buffer` returns `Result<FilePacket<'_>, SerializeStatus>` instead of the C++ in-place `SerializeStatus fromBuffer(&Buffer)`. The task's enum has no `T_NONE` variant, so there is nothing to fill in place; the `Err` carries the exact C++ status that `FileUplink` logs as `DecodeError`. An unknown/`T_NONE` type byte is `DeserFormatError` (the C++ `FW_ASSERT(false)` in the caller's dispatch switch would crash on ground-supplied data).

2. **[Superseded in phase 2 — see "Phase 2" below.]** `Os::SandboxedFile` and `Os::FilePathUtils` were NOT in fprime-os (that crate's notes list SandboxedFile as "not ported" and it is owned by an earlier wave, so I may not edit it). I ported them into `file_uplink.rs` as public items (`SandboxedFile`, `PathStatus`, `resolve_path`, `resolve_from_cwd`, `check_containment`, `MAX_PATH_LENGTH`) and `file_downlink.rs` imports `SandboxedFile` from there. They should move to `fprime-os` when that crate is next touched. Semantics are exact: fail-open default (`/`, configured), purely textual resolution (no `canonicalize`, no symlink following), and the resolved path is what gets opened.

3. `Fw.StringFormatStatus` is declared in `file_manager.rs` for the same reason (fprime-fw's `enums` module is not mine to edit). Move it to `fprime_fw::enums` later.

4. FileDownlink buffer arenas: C++ keeps two static `[u8;512]` arrays and wraps `Fw::Buffer` views around them. Rust `Buffer`s own their storage and move through ports, so I keep a two-slot `BufferStorage` pool that recycles whatever returns on `bufferReturn` — INCLUDING buffers whose return is otherwise ignored (a stale return would otherwise leak the storage). The load-bearing part, the context bookkeeping (`context + 1 == last_buffer_id`, one id per `get_buffer`, a second one for a CANCEL packet), is reproduced exactly and tested.

5. FileDownlink's internal `Os::Queue` of memcpy'd `FileEntry` PODs is a bounded `VecDeque<FileEntry>` (capacity = `file_queue_depth`, never grown, FIFO, non-blocking, full => the C++ send-failure path). The analysis explicitly sanctions a typed queue here; nothing on the wire depends on it.

6. Non-UTF-8 paths: Rust file APIs take `&str`. FileUplink's `File::open` returns `BAD_SIZE` (surfacing as `FileOpenError`, the same event the C++ produces for its own `BAD_SIZE`) and FileDownlink's returns `OTHER_ERROR` (surfacing as `FileOpenError`) for a path that is not valid UTF-8. C++ passes raw bytes to `open(2)`.

7. FileManager's internal `run` message carries a `port_num = 0` field after the msg type; C++ internal-interface messages omit it. This matches the existing fprime-comp dispatch contract and the EventManager precedent — internal only, never crosses a hub.

8. FileUplink/FileManager/FileDownlink hold their `Mutex<State>` across event/telemetry emission where the C++ component has no mutex at all (all the mutating paths are async, i.e. component-thread only). That preserves the C++ event/telemetry interleaving exactly and cannot deadlock, since no output port can re-enter the component synchronously. FileDownlink's `Mode` has its own mutex (`mode()`/`set_mode()` lock briefly), mirroring the C++ `m_mode`; state and mode are never held simultaneously in a way that can invert.

NOT PORTED (deliberate)

- **[Superseded in phase 2 — see "Phase 2" below; the chunking loop is ported.]** FileManager `GenerateDp` (0x09) chunking: `Fw/Dp`, `DpContainer` and the `Fw.DataProductSync` ports do not exist yet (`fprime-fw/src/dp.rs`, `dp_manager.rs`, `dp_writer.rs` are still one-line placeholders being written by another wave). The handler validates its arguments (`FormatError` / `ValidationError` on the `GenerateDpMode` enum) and then takes the exact path the C++ takes when `productGetOut`/`productSendOut` are unconnected: `GenerateDpBufferFailed` + `cmdResponse OK`. The `GenerateDpStage`/`GenerateDpMode` enums and the event ids are in place; the paced chunk loop, `FileChunkHeader` record and the DP-pacing half of `run_internalInterfaceHandler` are the only work left, and `run_internal_handler` has the hook comment marking where it goes.
- No shell/exec command: opcode 0x04 is a deliberate upstream gap (the historic `ShellCommand` was removed). The task text said "ShellCommand if present"; it is not present in `Svc/FileManager/Commands.fppi`, and the analysis says explicitly not to add one. A test asserts 0x04 answers `InvalidOpcode`.
- FileDownlink `FilenameSourceOverflow`/`FilenameDestinationOverflow` are implemented but unreachable in the stock configuration (both C++ and Rust): command strings are `Fw::CmdStringArg` (40 bytes) and port strings are 100 bytes, both below the 240-byte `FileNameString` capacity they are compared against. Kept for control-flow parity.
- `FileUplink::File::open`'s `BAD_SIZE`-on-long-path branch is likewise unreachable (a `PathName` length is a `U8` <= 255 and the C++ buffer is 256), kept for parity.

CONFIG CONSTANTS: the analysis's list of new `fprime-config` entries (`FW_FILE_BUFFER_MAX_SIZE`, `FileDownCompletePorts`, `FILEDOWNLINK_*`, `FileManagerConfig::*`) live as module consts in the owning component modules, because `fprime-config` belongs to an earlier wave. They are flagged in the doc comments for migration.

TESTS: 116 new tests. Literal-byte tests cover every FilePacket variant (START/DATA/END/CANCEL), both parse and serialize directions, every error status, path truncation and zero-copy aliasing; the CFDP checksum is checked against hand-computed vectors, a byte-by-byte reference, unaligned starts 0..8, and chunked/out-of-order equivalence. Every documented gotcha in `file-services.md` has a named test. The end-to-end test downlinks a 2345-byte temp file (START + 5 DATA + END, exact chunk sizes and sequence indices), feeds the produced buffers straight into FileUplink, and asserts the reconstructed file is byte-identical with `FileReceived` as the only event.

TEST-PATH CAVEAT: command string arguments are `Fw::CmdStringArg` (40 bytes) in both C++ and this port, so the FileDownlink/FileManager tests deliberately use short temp directory names (`/tmp/fpr<pid>_<n>`); longer paths get silently truncated by the command path, which is C++ behavior, not a port bug.

SIBLINGS: no sibling module was edited. At final verification `cargo build --workspace`, `cargo test --workspace` (all crates green, fprime-svc 330 tests incl. my 86), `cargo clippy -p fprime-fw -p fprime-utils -p fprime-svc --all-targets -- -D warnings` and `cargo fmt --check` were all clean; fprime-svc was re-run three times with no flakes. Nothing was committed or pushed.

## CmdSequencer

```text
CRATE fprime_svc, module `cmd_sequencer` (already declared in lib.rs; use fully-qualified paths, e.g. `fprime_svc::cmd_sequencer::CmdSequencer`). Nothing was added to lib.rs.

```text
MODULE CONSTS
  SEQUENCE_ARGUMENTS_MAX_SIZE: usize = 255
  SEQUENCE_HEADER_SIZE: usize = 11; SEQUENCE_CRC_SIZE: usize = 4
  QUEUE_MSG_SIZE: usize = 522        // pass to create_queue via CmdSequencer::init
  NO_SEQ: &str = "<no seq>"

SVC/SEQ TYPES (all Serialize/Deserialize, Default)
  pub struct SeqArgsBuffer(pub [u8; 255])  // fpp_array!, Clone+Copy+Eq; SIZE, SERIALIZED_SIZE=255
  pub struct SeqArgs { pub size: FwSizeType, pub buffer: SeqArgsBuffer }  // fpp_struct!, Clone+Copy+Eq
      SeqArgs::SERIALIZED_SIZE = 263; new(size, buffer); set_all(..); get_size/set_size; get_buffer/set_buffer
      wire = [size u64 BE][255 raw bytes]
  pub enum BlockState : u8 { Block = 0, NoBlock = 1 }   default Block; TryFrom<u8>, as_repr(), VALUES
  pub enum SeqMode : u8 { Step = 0, Auto = 1 }          default Step   (the REPORTED mode; inverted vs StepMode)
  pub enum FileReadStage : u8 { ReadHeader=0, ReadHeaderSize=1, DeserSize=2, DeserNumRecords=3,
                                DeserTimeBase=4, DeserTimeContext=5, ReadSeqCrc=6, ReadSeqData=7,
                                ReadSeqDataSize=8 }     default ReadHeader

PORT TRAITS (defined here; `Send + Sync`, object-safe — use as OutputPort<dyn X> / PortRef<dyn X>)
  pub trait CmdSeqInPort     { fn invoke(&self, port_num: FwIndexType, filename: &FileNameString, args: &SeqArgs) }
  pub trait CmdSeqCancelPort { fn invoke(&self, port_num: FwIndexType) }
  pub trait FileDispatchPort { fn invoke(&self, port_num: FwIndexType, file_name: &mut FileNameString) }

INTERNAL STATE ENUMS (public for introspection/tests)
  pub enum RunMode  { Stopped = 0, Running = 1 }   (repr i32, Default = Stopped)
  pub enum StepMode { Auto = 0, Manual = 1 }       (repr i32, Default = Auto)
  pub enum RecordDescriptor { Absolute = 0, Relative = 1, EndOfSequence = 2 }  (repr u8, Default = EndOfSequence)

SEQUENCE FORMAT
  pub struct SequenceHeader { pub file_size: u32, pub num_records: u32, pub time_base: TimeBase, pub time_context: u8 }
      Default = {0, 0, TbDontCare, FW_CONTEXT_DONT_CARE}
  pub struct SequenceRecord { pub descriptor: RecordDescriptor, pub time_tag: Time, pub command: ComBuffer }
  pub enum SequenceLoadEvent { FileNotFound, FileReadError, FileInvalid{stage,error}, FileSizeError{size},
      FileCrcFailure{stored,computed}, RecordInvalid{record_number,error}, RecordMismatch{header_records,extra_bytes},
      TimeBaseMismatch{current,seq}, TimeContextMismatch{current,seq}, NoRecords }
  pub trait Sequence: Send {
      fn allocate_buffer(&mut self, identifier: FwEnumStoreType, bytes: usize)   // fw_assert bytes >= 11
      fn deallocate_buffer(&mut self); fn capacity(&self) -> usize
      fn set_file_name(&mut self, &CmdStringArg)
      fn file_name(&self) -> &CmdStringArg; fn log_file_name(&self) -> &LogStringArg
      fn string_file_name(&self) -> &FwDefaultString; fn header(&self) -> &SequenceHeader
      fn load_file(&mut self, &CmdStringArg, current_time: &Time, event: &mut Option<SequenceLoadEvent>) -> bool
      fn has_more_records(&self) -> bool; fn next_record(&mut self, &mut SequenceRecord)  // fw_asserts on bad deser
      fn reset(&mut self)   // rewind read cursor
      fn clear(&mut self)   // drop data (both cursors)
  }
  pub struct FPrimeSequence : Sequence  (Default) — fn new(); allocator_id() -> FwEnumStoreType; stored_crc() -> u32

  pub struct Timer (Copy, Default) — const new(); set(Time); clear(); is_expired_at(&Time) -> bool
      (`compare(exp, now) != Gt`, so INCOMPARABLE counts as EXPIRED); expiration_time() -> Time; is_armed() -> bool

COMPONENT
  pub struct CmdSequencer {
      pub active: ActiveBase,
      pub cmd: CmdGlue,                                   // cmdRegOut / cmdResponseOut
      pub evt: EventGlue,                                 // logOut / LogText / timeCaller
      pub tlm: TlmGlue,                                   // tlmOut
      pub com_cmd_out:   OutputPort<dyn ComPort>,         // Fw.Com — the sequenced command packets
      pub seq_done:      OutputPort<dyn CmdResponsePort>, // Fw.CmdResponse
      pub seq_start_out: OutputPort<dyn CmdSeqInPort>,    // Svc.CmdSeqIn
      pub ping_out:      OutputPort<dyn PingPort>,
  }
  fn new(name: &str) -> Arc<CmdSequencer>
  fn init(&self, queue_depth: FwSizeType)                          // create_queue(depth, QUEUE_MSG_SIZE)
  fn reg_commands(&self)                                           // registers opcodes 0..=7
  fn set_sequence_format(&self, Box<dyn Sequence>)                 // C++ setSequenceFormat
  fn allocate_buffer(&self, identifier: FwEnumStoreType, bytes: usize)  // Ref: (0, 5*1024)
  fn deallocate_buffer(&self)
  fn set_timeout(&self, timeout_seconds: u32)                      // 0 = disabled (default)
  fn load_sequence(&self, &CmdStringArg)                           // fw_asserts run_mode == Stopped
  fn run_mode(&self) -> RunMode; fn step_mode(&self) -> StepMode   // introspection
  impl ComponentDispatch + ActiveComponent

  Input-port factories (all `fn x(self: &Arc<Self>, port_num: FwIndexType) -> PortRef<dyn ...>`, all ASYNC,
  `assert` queue-full policy, queue priority 1):
      seq_cancel_in   -> PortRef<dyn CmdSeqCancelPort>
      cmd_response_in -> PortRef<dyn CmdResponsePort>
      ping_in         -> PortRef<dyn PingPort>
      seq_run_in      -> PortRef<dyn CmdSeqInPort>
      seq_dispatch_in -> PortRef<dyn FileDispatchPort>
      sched_in        -> PortRef<dyn SchedPort>
      cmd_in          -> PortRef<dyn CmdPort>
  Msg types: CmdSequencer::MSG_TYPE_SEQ_CANCEL_IN=1, MSG_TYPE_CMD_RESPONSE_IN=2, MSG_TYPE_PING_IN=3,
             MSG_TYPE_SEQ_RUN_IN=4, MSG_TYPE_SEQ_DISPATCH_IN=5, MSG_TYPE_SCHED_IN=6, MSG_TYPE_CMD_IN=7

  Associated consts (FPP-relative):
    OPCODE_CS_RUN=0, OPCODE_CS_VALIDATE=1, OPCODE_CS_CANCEL=2, OPCODE_CS_START=3,
    OPCODE_CS_STEP=4, OPCODE_CS_AUTO=5, OPCODE_CS_MANUAL=6, OPCODE_CS_JOIN_WAIT=7
    EVENTID_CS_SEQUENCE_LOADED=0, _SEQUENCE_CANCELED=1, _FILE_READ_ERROR=2, _FILE_INVALID=3,
    _RECORD_INVALID=4, _FILE_SIZE_ERROR=5, _FILE_NOT_FOUND=6, _FILE_CRC_FAILURE=7,
    _COMMAND_COMPLETE=8, _SEQUENCE_COMPLETE=9, _COMMAND_ERROR=10, _INVALID_MODE=11,
    _RECORD_MISMATCH=12, _TIME_BASE_MISMATCH=13, _TIME_CONTEXT_MISMATCH=14,
    _PORT_SEQUENCE_STARTED=15, _UNEXPECTED_COMPLETION=16, _MODE_SWITCHED=17,
    _NO_SEQUENCE_ACTIVE=18, _SEQUENCE_VALID=19, _SEQUENCE_TIMEOUT=20, _CMD_STEPPED=21,
    _CMD_STARTED=22, _JOIN_WAITING=23, _JOIN_WAITING_NOT_COMPLETE=24, _NO_RECORDS=25
    CHANID_CS_LOAD_COMMANDS=0, _CANCEL_COMMANDS=1, _ERRORS=2, _COMMANDS_EXECUTED=3,
    _SEQUENCES_COMPLETED=4, _CURRENT_SEQUENCE=5 (string 240, update on change)

  Command wire args: CS_RUN = [u16 len][name bytes][u8 BlockState]; CS_VALIDATE = [u16 len][name bytes];
  the other six take no arguments. File names deserialize into `Fw::CmdStringArg` (40) exactly as the C++
  generated handler does, so a longer name answers FormatError.

  Topology order: CmdSequencer::new -> active.queued.base.set_id_base -> connect
  (cmd.cmd_reg_out, cmd.cmd_response_out, evt.log_out, evt.text_log_out, evt.time_out, tlm.tlm_out,
   com_cmd_out -> CmdDispatcher::seq_cmd_buff_in, seq_done, seq_start_out, ping_out) ->
  init(depth) -> allocate_buffer(0, 5*1024) -> set_timeout(..) -> reg_commands() ->
  [load_sequence(..)] -> active.start(&arc, prio 20, stack, affinity) -> ... -> active.exit() / active.join()
  -> deallocate_buffer().
  Ref wiring: rateGroup2Comp.RateGroupMemberOut[0] -> cmdSeq.sched_in;
  cmdSeq.com_cmd_out -> CmdDispatcher.seq_cmd_buff_in; CmdDispatcher.seq_cmd_status -> cmdSeq.cmd_response_in.
```
```

### Implementation notes / deviations

DEVIATIONS (each documented in code)

1. Exactly-once CS_RUN response. C++ ends `CS_RUN_cmdHandler` with `if (NO_BLOCK == m_blockState) cmdResponse(OK)` reading the *member*. When a sequence completes synchronously inside `performCmd_Step` (a file whose FIRST record is END_OF_SEQUENCE) `sequenceComplete` has already answered the BLOCK caller and reset `m_blockState`, so stock C++ answers a second time. Because the task mandates exactly-once responses, the port tests the block state the command ARRIVED with. Every other path is byte-identical; covered by `block_mode_answers_exactly_once_for_an_immediately_complete_sequence`.

2. `Header::validateTime` calls `component.getTime()` in C++; here the component samples `timeCaller` once in `load_file` and passes the `Time` into `Sequence::load_file`. Equivalent (synchronous port, single thread), and it keeps the `Sequence` trait free of a back-pointer to the component.

3. `Sequence::Events` (which calls back into the component) became a returned `Option<SequenceLoadEvent>`: every failing load path emits at most one event and returns immediately, so this is faithful. The component maps it to the event id and calls `error()` for all variants except `RecordMismatch` (C++ TODO).

4. `deserialize_header` commits header fields only on full success; C++ writes them as it goes. Unobservable — after a failed load the component clears the sequence and nothing reads the header.

5. `MemAllocator`/`ExternalSerializeBuffer` -> owned `Box<[u8]>` plus explicit `ser_loc`/`deser_loc`. `allocate_buffer` keeps the `identifier` argument as an inert field for API parity (`FPrimeSequence::allocator_id()`); `recoverable` is dropped. Reading the records *over* the header is not needed: the CRC takes the 11 header bytes as read, then the record slice — bit-identical.

6. A file name that is not valid UTF-8 cannot reach `fprime_os::File::open`, so it reports `CS_FileReadError` (C++ would hand the bytes to open(2) and fail there).

7. `Sequence::capacity()` is an addition (no C++ equivalent) used by the buffer-size check and tests.

8. State is held in a `Mutex<CmdSequencerState>` per workspace convention and the lock is held across output-port invocations. C++ has no mutex at all here; every input port is ASYNC so there is no re-entrancy and no guarded port, which makes this safe and semantically identical.

NOT PORTED (per the analysis)
- `formats/AMPCSSequence` (alternate format with a `.CRC32` sidecar) — the `Sequence` trait keeps it addable.
- `Os::ValidateFile` — a dead include in `CmdSequencerImpl.hpp`.

C++ PARITY QUIRKS DELIBERATELY KEPT (all tested)
- `fileSize` includes the trailing CRC; CRC covers header(11)+records(fileSize-4) only.
- `recordSize + sizeof(FwPacketDescriptorType) > 512` is over-strict by 2 (max usable 510).
- `Timer::is_expired_at` treats INCOMPARABLE as EXPIRED, while `performCmd_Step_ABSOLUTE`'s `>=` does not.
- `perform_cmd_cancel` uses `reset()` (rerunnable), complete/validate/load-failure use `clear()`.
- `CS_RecordMismatch` does not bump `CS_Errors`.
- `CS_JOIN_WAIT` logs the PREVIOUS cmdSeq/opCode.
- `do_sequence_run` invokes `seqDone` unguarded on its error paths (fw_asserts when unconnected) while cancel/complete guard — covered by a `#[should_panic]` test.
- `schedIn`'s `else if` (a due dispatch suppresses the timeout check) and its second `getTime()`.
- `set_cmd_timeout` only when `timeout > 0` AND step mode AUTO.
- A malformed record time tag with `useconds >= 1e6` fail-stops through `fw_assert!`, exactly like the C++ `Fw::Time::set` FW_ASSERT.

VERIFICATION
`cargo build --workspace` green; `cargo test -p fprime-svc` fully green (428 tests, 84 of them mine); `cargo clippy -p fprime-svc --all-targets -- -D warnings` clean; `rustfmt --edition 2024 --check` clean on my file.

SIBLING NOTE (not a defect in my work): `cargo fmt -p fprime-svc -- --check` currently reports formatting drift in `crates/fprime-svc/src/dp_writer.rs` (a sibling agent's in-progress module) — I did not touch it. Caution for the orchestrator: earlier in this session I ran `cargo fmt -p fprime-svc`, which formats the whole crate and may have applied whitespace-only rustfmt changes to sibling files present at that moment; the current drift shows they have since been rewritten. Nothing was committed or pushed, and no other file was edited by hand.

## Data products (DpContainer, DpManager, DpWriter, DpCatalog)

```text
## fprime_fw::dp (NOT re-exported at the crate root — use `fprime_fw::dp::*`)

```text
const CONTAINER_USER_DATA_SIZE: usize = 32; HASH_DIGEST_LENGTH: usize = 4
fn crc32(data: &[u8]) -> u32   // standard complemented CRC-32 (= Utils::Hash value)

fpp_enum DpState : u8 { Untransmitted=0, Partial=1, Transmitted=2 } default Untransmitted
fpp_enum ProcType : u8 { None=0x00, ZlibDeflate=0x01, One=0x02, Two=0x04 } default None
  // bit MASK: DpWriter fans out on bit index n -> proc_buffer_send_out[n]

struct Header (offsets, all `usize` assoc consts): PACKET_DESCRIPTOR_OFFSET=0, ID_OFFSET=2,
  PRIORITY_OFFSET=6, TIME_TAG_OFFSET=10, PROC_TYPES_OFFSET=21, USER_DATA_OFFSET=22,
  DP_STATE_OFFSET=54, DATA_SIZE_OFFSET=55, SIZE=57

struct DpContainer (Debug, Default):
  consts HEADER_HASH_OFFSET=57, DATA_OFFSET=61, MIN_PACKET_SIZE=65
  const fn packet_size_for_data_size(FwSizeType) -> FwSizeType
  pub user_data: [u8; 32]
  fn new() -> Self; with_buffer(id: FwDpIdType, buffer: Buffer) -> Self /*asserts size>=65*/
  id/set_id, priority/set_priority, time_tag/set_time_tag (Time), proc_types/set_proc_types (u8),
    state()->DpState / set_dp_state, data_size/set_data_size (FwSizeType)
  fn packet_size(&self)->FwSizeType; data_hash_offset(&self)->FwSizeType
  fn buffer(&self)->&Buffer; buffer_mut(&mut self)->&mut Buffer
  fn set_buffer(&mut self, Buffer) /*asserts >=65, resets data_size*/
  fn take_buffer(&mut self)->Buffer  // Rust form of invalidateBuffer
  fn shrink_buffer_size(&mut self)   // asserts shrink-only
  fn data_capacity(&self)->FwSizeType; data(&self)->&[u8]; data_region_mut(&mut self)->&mut [u8]
  fn data_serializer(&mut self)->ExtBuf<'_>   // the C++ m_dataBuffer, cursor at 0
  fn serialize_header(&mut self) /*asserts; also updates the header hash*/
  fn deserialize_header(&mut self)->SerializeStatus /*FormatError on bad descriptor*/
  fn header_hash/compute_header_hash(&self)->u32; set_header_hash(u32); update_header_hash()
  fn check_header_hash(&self)->(Success, stored: u32, computed: u32)
  fn data_hash/compute_data_hash(&self)->u32; set_data_hash(u32); update_data_hash()
  fn check_data_hash(&self)->(Success, stored: u32, computed: u32)

Port traits (all `: Send + Sync`, object-safe, use as OutputPort<dyn XPort>):
  DpGetPort::invoke(&self, port_num, id: FwDpIdType, data_size: FwSizeType, buffer: &mut Buffer) -> Success
  DpRequestPort::invoke(&self, port_num, id: FwDpIdType, data_size: FwSizeType)
  DpResponsePort::invoke(&self, port_num, id: FwDpIdType, buffer: Buffer /*by move*/, status: Success)
  DpSendPort::invoke(&self, port_num, id: FwDpIdType, buffer: Buffer /*by move*/)
```

## fprime_svc::dp_manager

```text
const DP_MANAGER_NUM_PORTS: usize = 5; QUEUE_MSG_SIZE: FwSizeType = 522
DpManager::new(name)->Arc<Self>; init(&self, queue_depth); reg_commands(&self)
fields: pub active: ActiveBase, cmd: CmdGlue, evt: EventGlue, tlm: TlmGlue,
  product_response_out: [OutputPort<dyn DpResponsePort>; 5],
  buffer_get_out: [OutputPort<dyn BufferGetPort>; 5],
  product_send_out: [OutputPort<dyn BufferSendPort>; 5]
input factories fn x(self:&Arc<Self>, port_num)->PortRef<..>:
  product_get_in (SYNC dyn DpGetPort, 0..5, asserts range), product_request_in (async dyn DpRequestPort),
  product_send_in (async dyn DpSendPort, escrowed buffer), sched_in (async dyn SchedPort),
  cmd_in (async dyn CmdPort)
msg types 1..=4 (SCHED_IN, PRODUCT_REQUEST_IN, PRODUCT_SEND_IN, CMD_IN), priority 1
consts: OPCODE_CLEAR_EVENT_THROTTLE=0; EVENTID_BUFFER_ALLOCATION_FAILED=0;
  CHANID_NUM_SUCCESSFUL_ALLOCATIONS=0, NUM_FAILED_ALLOCATIONS=1, NUM_DATA_PRODUCTS=2, NUM_BYTES=3
```

## fprime_svc::dp_writer

```text
const DP_WRITER_NUM_PROC_PORTS: usize = 5; DP_EXT = ".fdp"; QUEUE_MSG_SIZE = 522
fn format_dp_file_name(base_dir:&str, id:FwDpIdType, seconds:u32, useconds:u32)
   -> (FileNameString, StringFormatStatus)      // "<dir>/Dp_%08_%08_%08.fdp"; >240 -> Overflowed
trait DpWrittenPort: Send+Sync { fn invoke(&self, port_num, file_name:&FileNameString,
   priority: FwDpPriorityType, size: FwSizeType) }        // Svc.DpWritten lives here
trait DpProcPort:   Send+Sync { fn invoke(&self, port_num, buffer:&mut Buffer) }  // procBufferSendOut
DpWriter::new(name)->Arc<Self>; init(&self, queue_depth); configure(&self, dp_file_name_prefix:&str);
  reg_commands(&self)
fields: pub active, cmd, evt, tlm, proc_buffer_send_out: [OutputPort<dyn DpProcPort>; 5],
  dp_written_out: OutputPort<dyn DpWrittenPort> (optional), dealloc_buffer_send_out: OutputPort<dyn BufferSendPort>
input factories: buffer_send_in (async dyn BufferSendPort, escrowed), sched_in (async dyn SchedPort),
  cmd_in (async dyn CmdPort)
msg types 1..=3 (SCHED_IN, BUFFER_SEND_IN, CMD_IN), priority 1
consts: OPCODE_CLEAR_EVENT_THROTTLE=0; EVENTID_ INVALID_BUFFER=0, BUFFER_TOO_SMALL_FOR_PACKET=1,
  INVALID_HEADER_HASH=2, INVALID_HEADER=3, BUFFER_TOO_SMALL_FOR_DATA=4, FILE_NAME_FORMAT_ERROR=5,
  FILE_OPEN_ERROR=6, FILE_WRITE_ERROR=7, FILE_WRITTEN=8;
  CHANID_ NUM_BUFFERS_RECEIVED=0, NUM_BYTES_WRITTEN=1, NUM_SUCCESSFUL_WRITES=2, NUM_FAILED_WRITES=3, NUM_ERRORS=4
```

## fprime_svc::dp_catalog

```text
const DP_MAX_DIRECTORIES=2; DP_MAX_FILES=127; STATE_FILE_RECORD_SIZE=31; QUEUE_MSG_SIZE=522
fpp_enum DpHdrField : u8 { Descriptor=0, Id=1, Priority=2, Crc=3 }
fpp_struct DpRecord (Clone, Copy; SERIALIZED_SIZE=29) { id: FwDpIdType, t_sec: u32, t_sub: u32,
  priority: u32, size: u64, blocks: u32, state: DpState } + get_*/set_* pairs
struct DpStateEntry { pub dir: FwIndexType, pub record: DpRecord }  // Ord == compare_entries
fn compare_entries(&DpStateEntry, &DpStateEntry) -> std::cmp::Ordering  // priority, tSec, tSub, id; dir EXCLUDED
DpCatalog::new(name)->Arc<Self>; init(&self, queue_depth); reg_commands(&self);
  configure(&self, directories: &[FileNameString], state_file: &FileNameString)
  configure_with_slots(&self, dirs, state_file, requested_slots: FwSizeType)  // 0 = the no-memory path
  shutdown(&self); catalog_size(&self)->usize
fields: pub active, cmd, evt, tlm, ping_out: OutputPort<dyn PingPort>,
  file_out: OutputPort<dyn SendFileRequestPort>
input factories: ping_in (async dyn PingPort), file_done (async dyn SendFileCompletePort),
  add_to_cat (async dyn DpWrittenPort), cmd_in (async dyn CmdPort)
msg types 1..=4 (PING_IN, FILE_DONE, ADD_TO_CAT, CMD_IN), priority 1
consts: OPCODE_ BUILD_CATALOG=0, START_XMIT_CATALOG=1 (wait: Fw.Wait, remainActive: bool),
  STOP_XMIT_CATALOG=2, CLEAR_CATALOG=3
  EVENTID_* with the FPP explicit ids: DIRECTORY_OPEN_ERROR=0 .. DIRECTORY_NOT_MANAGED=5,
  CATALOG_XMIT_STARTED=10 (never emitted), CATALOG_XMIT_STOPPED=11, CATALOG_XMIT_COMPLETED=12,
  SENDING_PRODUCT=13, PRODUCT_COMPLETE=14, COMPONENT_NOT_INITIALIZED=20 .. FILE_SIZE_ERROR=31,
  NO_DP_MEMORY=32, XMIT_NOT_ACTIVE=34, STATE_FILE_*=35..40 (DP_DUPLICATE=28 never emitted),
  DP_FILE_XMIT_ERROR=41, DP_FILE_SEND_ERROR=42, DP_FILE_ADDED=43, NOT_LOADED=44, DP_FILE_SKIPPED=45,
  XMIT_UNBUILT_CATALOG=46, INVALID_FILE_NAME=47, FILE_CORRUPTED_DATA_ERROR=48, FILE_NAME_FORMAT_ERROR=49
  CHANID_CATALOG_DPS=0, CHANID_DPS_SENT=1  (declared, never written — C++ parity)
```

Topology wiring: DpManager.product_send_out[i] -> DpWriter.buffer_send_in; DpWriter.dp_written_out ->
DpCatalog.add_to_cat; DpCatalog.file_out -> FileDownlink.send_file_in; FileDownlink's completion port ->
DpCatalog.file_done. Order per component: new -> set_id_base -> connect -> init(queue_depth) ->
configure (DpWriter/DpCatalog) -> reg_commands -> active.start.
```

### Implementation notes / deviations

DELIBERATE DEVIATIONS (each documented in the module headers)

1. zlib is OUT OF SCOPE, as instructed: `Svc::DpZLibCompressor` and `Svc::DpCompressProc` are not ported (they call libz; the workspace has zero third-party deps). `ProcType::ZlibDeflate` (0x01) is kept for wire parity and DpWriter's 5-port `procTypes & (1<<portNum)` fan-out is ported verbatim, so any processing component can be wired later. With nothing connected procTypes is 0x00 and `perform_processing` is a no-op; the rest of the chain is fully functional.

2. Home of the port traits: DpGet/DpRequest/DpResponse/DpSend are defined in `fprime_fw::dp` (they are declared in `Fw/Dp/Dp.fpp` next to `DpState` and are meaningless without `DpContainer`; `fprime-comp` is not writable by this task). `Svc.DpWritten` is defined in `fprime_svc::dp_writer`, next to its only emitter (Svc/DpPorts holds nothing else). Both are `: Send + Sync` object-safe traits and work with `fprime_comp::OutputPort<dyn ...>` unchanged.

3. `procBufferSendOut` is FPP `Fw.BufferSend`, but C++ keeps using the buffer after the synchronous call (`ref Fw::Buffer`). The framework's `BufferSendPort` moves the buffer, so the fan-out uses a new `DpProcPort` taking `&mut Buffer` — same call graph, same in-place mutation, no ownership loss. `bufferSendIn`/`productSendIn`/`deallocBufferSendOut` stay real `BufferSendPort`s.

4. `DpContainer` owns its `Buffer` (the Rust `Fw::Buffer` owns storage). `invalidateBuffer` becomes `take_buffer() -> Buffer` so ownership is handed back rather than dropped, and the C++ `m_dataBuffer` alias into the packet is replaced by `data_region_mut()` / `data_serializer()` plus explicit `set_data_size`. The C++ double `setBuffer` in DpWriter's `deserializePacketHeader` is preserved literally (take + set) because the data-size reset it performs is load-bearing.

5. Hashes are `u32` instead of `Utils::HashBuffer` (4-byte digest stored big-endian, so this is exactly `HashBuffer::asBigEndianU32()`, which is what every event argument uses). The CRC-32 table/algorithm is duplicated in `fprime-fw/src/dp.rs` because `fprime-fw` cannot depend on `fprime-utils` (the DAG runs utils -> fw); this mirrors the duplication `fprime-os` already carries. A test pins it against the standard vectors (`"123456789"` -> 0xCBF43926) and against the literal 57-byte header CRC.

6. `Fw.StringFormatStatus` is reused from `crate::file_manager` (an earlier wave declared it there with a note that it belongs in `fprime-fw::enums`); I did not duplicate it. `Svc.SendFileStatus`/`SendFileResponse`/`SendFileRequestPort`/`SendFileCompletePort` come from `crate::file_downlink`, and `DpCatalog` truncates its 240-byte file names into the port's `string size 100` exactly as C++ does.

7. `FwSizeType` event arguments (DpWriter's `bufferSize`) are serialized as 8 raw big-endian bytes, matching the repo precedent set by EventManager's `EventsDropped` FwSizeType channel. If the FPP autocoder is later found to use `serializeSize` (U16) for size-typed event args, DpWriter's three affected events are the only place to change.

8. DpCatalog's `MemAllocator` is replaced by `configure_with_slots(.., requested_slots)`, which sizes a `Box<[DpStateFileEntry]>` and derives `num_dp_slots = min(requested, DP_MAX_FILES)`; `requested_slots == 0` reproduces the C++ short/failed-allocation path (`ComponentNoMemory` / `NoDpMemory`). The catalog itself keeps its fixed 127-entry capacity independent of the slot count, as in C++.

9. `Fw::RedBlackTreeSet` is a sorted fixed-capacity `Vec` (capacity allocated once at construction) with binary-search find/insert/remove; the order is `compare_entries` verbatim and `first()` is `begin()`.

C++ QUIRKS REPRODUCED ON PURPOSE (all covered by tests)
- DpManager: `NumBytes` adds the whole `Fw::Buffer` size, not the packet size; `productRequestIn` always answers, with an invalid buffer + FAILURE on allocation failure; responses/sends mirror the request port index.
- DpWriter: `CLEAR_EVENT_THROTTLE` clears 7 of the 8 throttled events (FileNameFormatError, id 5, is throttled but never cleared); the double `setBuffer`; `updateHeaderHash()` followed by `serializeHeader()` (which re-hashes) after processing; the file is never explicitly closed (Rust `File` closes on drop); `OPEN_CREATE` defaults to NO_OVERWRITE so rewriting an existing product is a FileOpenError.
- DpCatalog: `FileHdrError` passes (exp = computed, act = stored) — the reverse of the argument names; `dir` is excluded from ordering/equality so the same product in two managed directories is a duplicate, while `getFileState` DOES require a `dir` match; `CLEAR_CATALOG` neither checks nor clears the xmit flags (a later `fileDone` unwinds with EXECUTION_ERROR); `STOP_XMIT_CATALOG` does not cancel the in-flight transfer; `START_XMIT_CATALOG` arms the waited response before starting; `addToCat` ignores the port's priority/size arguments; only insert failure and slot overflow return QUIT while every other file error continues the scan; a directory open/read/count error aborts the entire build; the state file is unframed, appended without dedup, and a short trailing record is a benign truncation; `CatalogXmitStarted` (10), `DpDuplicate` (28) and both telemetry channels are declared but never emitted/written.
- A rebuild does NOT skip a product that was transmitted in a previous session: C++ never rewrites the file's own header, so only a header whose `DpState` is TRANSMITTED is skipped; the state file only merges `state`/`blocks` into the catalog entry. A test pins this (it is easy to mis-implement as a skip).

OPEN QUESTIONS / NOTES FOR THE ORCHESTRATOR
- `docs/api-notes.md` was not edited (docs/ is off-limits for this task); the API block above is ready to paste under a "Data products" heading.
- `fprime-fw/src/lib.rs` was not touched, so `dp` is reachable as `fprime_fw::dp::...` only. Add `pub use dp::{DpContainer, DpState, ProcType};` there if a crate-root re-export is wanted.
- Mid-task, `cargo build -p fprime-svc` once failed with an unclosed delimiter in a sibling's `cmd_sequencer.rs`, and two `ccsds::space_packet_framer` should_panic tests failed transiently; both cleared on their own. Final state: `cargo build --workspace`, `cargo test --workspace` (all crates green, fprime-svc 519 tests), `cargo clippy -p fprime-fw -p fprime-svc --all-targets -D warnings` and `cargo fmt --check` are all clean. `cargo fmt` touched only my own files (verified by mtime).

## CCSDS stack (CRC-16, types, ApidManager, SpacePacket/TM/TC)

```text
All items live under `fprime_svc::ccsds::{crc16, types, apid_manager, space_packet_framer, space_packet_deframer, tm_framer, tc_deframer}` (nothing re-exported at the crate root; fully-qualified paths work).

crc16:
  const POLYNOMIAL:u16=0x1021; INIT:u16=0xFFFF; XOR_OUT:u16=0x0000
  struct Crc16 (Debug,Clone,Copy,Eq,Default): const fn new(); fn update(&mut self,u8); fn update_all(&mut self,&[u8]); const fn finalize(&self)->u16; const fn register(&self)->u16; fn compute(&[u8])->u16

types:
  const SPACECRAFT_ID:u16=0x0044; TM_FRAME_FIXED_SIZE:usize=1024; AOS_MAX_FRAME_FIXED_SIZE:usize=1536; AGGREGATION_SIZE:usize=1009
  fpp_enum FrameError:u8 {SpInvalidPacket=0,SpInvalidLength=1,TcInvalidScid=2,TcInvalidLength=3,TcInvalidVcid=4,TcInvalidCrc=5,AosInvalidScid=6,AosInvalidLength=7,AosInvalidVcid=8,AosInvalidCrc=9,AosInvalidVersion=10,AosInvalidEpp=11,AosVcFrameCountGap=12,SdlsDecryptionFailure=13} default SpInvalidPacket
  fpp_enum SdlsStatus:u8 {Success=0,UnknownSa=1,UnknownPort=2,EncryptionFailure=3,DecryptionFailure=4,KeyError=5}; fpp_enum Tfvn:u8 {TmTc=0,Aos=1,ProxOne=2,Uslp=3,InvalidUninitialized=4} default InvalidUninitialized
  mod space_packet_subfields {PVN_MASK 0xE000, PKT_TYPE_MASK 0x1000, SEC_HDR_MASK 0x0800, APID_MASK 0x07FF, PVN_OFFSET 13, PKT_TYPE_OFFSET 12, SEC_HDR_OFFSET 11, SEQ_FLAGS_MASK 0xC000, SEQ_COUNT_MASK 0x3FFF, SEQ_FLAGS_OFFSET 14, APID_WIDTH 11, SEQ_COUNT_WIDTH 14}
  mod tm_subfields {FRAME_VERSION_OFFSET 14, SPACECRAFT_ID_OFFSET 4, VIRTUAL_CHANNEL_ID_OFFSET 1, SEG_LENGTH_OFFSET 11, FRAME_VERSION_MASK 0xC000, SPACECRAFT_ID_MASK 0x3FF0, VIRTUAL_CHANNEL_ID_MASK 0x000E, OCF_FLAG_MASK 0x0001, SEC_HDR_FLAG_MASK 0x8000, SYNC_FLAG_MASK 0x4000, PACKET_ORDER_FLAG_MASK 0x2000, SEG_LENGTH_MASK 0x1800, FIRST_HEADER_POINTER_MASK 0x07FF}
  mod tc_subfields {FRAME_VERSION_MASK 0xC000, BYPASS_FLAG_MASK 0x2000, CONTROL_FLAG_MASK 0x1000, RESERVED_MASK 0x0C00, SPACECRAFT_ID_MASK 0x03FF, BYPASS_FLAG_OFFSET 13, VC_ID_MASK 0xFC00, FRAME_LENGTH_MASK 0x03FF, VC_ID_OFFSET 10}
  mod m_pdu_subfields {FHP_NO_PACKET_START 0xFFFF, FHP_IDLE_DATA_ONLY 0xFFFE}
  struct SpacePacketHeader (Clone,Copy,Eq,Debug,PartialEq,Default,Serialize/Deserialize,SERIALIZED_SIZE=6) {pub packet_identification:u16, packet_sequence_control:u16, packet_data_length:u16} + get_*/set_* pairs + new(a,b,c)/set_all
    const fn build_packet_identification(pvn:u8, packet_type:u8, has_sec_hdr:bool, apid:u16)->u16 (all fields masked)
    const fn build_packet_sequence_control(sequence_flags:u8, sequence_count:u16)->u16
    const fn pvn()->u8; packet_type()->u8; has_sec_hdr()->bool; apid_value()->u16; sequence_flags()->u8; sequence_count()->u16; data_field_length()->u32 (token+1, widened); length_token(data_field_length:u16)->u16
  struct TMHeader (SERIALIZED_SIZE=6) {global_vc_id:u16, master_frame_count:u8, virtual_frame_count:u8, data_field_status:u16}
    const fn build_global_vc_id(spacecraft_id:u16, vc_id:u8, ocf_flag:bool)->u16  // UNMASKED, reproduces the C++ overflow gotcha
    const fn build_data_field_status(sec_hdr:bool, sync:bool, packet_order:bool, segment_length_id:u8, first_header_pointer:u16)->u16
    const fn frame_version()->u8; spacecraft_id()->u16; vc_id()->u8; ocf_flag()->bool; segment_length_id()->u8; first_header_pointer()->u16
  struct TMTrailer{fecf:u16} (2); struct TCTrailer{fecf:u16} (2); struct AOSTrailer{fecf:u16} (2)
  struct TCHeader (SERIALIZED_SIZE=5) {flags_and_sc_id:u16, vc_id_and_length:u16, frame_sequence_num:u8}
    const fn build_flags_and_sc_id(bypass:bool, control_command:bool, spacecraft_id:u16)->u16  // (true,false,0x0044)==0x2044, the frame-detector token
    const fn build_vc_id_and_length(vc_id:u8, total_frame_length:u16)->u16  // stores length-1
    const fn frame_version()->u8; bypass_flag()->bool; control_command_flag()->bool; spacecraft_id()->u16; vc_id()->u8; total_frame_length()->u16 (token+1)
  struct AOSHeader{global_vc_id:u16, frame_count_and_signaling:u32} (6); struct MPduHeader{first_header_pointer:u16} (2, default 0xFFFF); struct SaMapEntry{security_association_index:u16, port_index:FwIndexType}
  trait ApidSequenceCountPort: Send+Sync { fn invoke(&self, port_num:FwIndexType, apid:Apid, sequence_count:u16)->u16 }
  trait ErrorNotifyPort: Send+Sync { fn invoke(&self, port_num:FwIndexType, error_code:FrameError) }

apid_manager::ApidManager (PASSIVE, both inputs guarded):
  fn new(name:&str)->Arc<Self>; pub base: PassiveBase; pub evt: EventGlue
  input factories: get_apid_seq_count_in(self:&Arc<Self>, port_num)->PortRef<dyn ApidSequenceCountPort>; validate_apid_seq_count_in(..)->PortRef<dyn ApidSequenceCountPort>
  no output ports besides evt; const MAX_TRACKED_APIDS:usize=12 (=Apid::NUM_CONSTANTS); EVENTID_UNEXPECTED_SEQUENCE_COUNT=0 (WARNING_LO, args [u16 transmitted][u16 expected])
  fn owns nothing else; const fn calculate_next_seq_count(u16)->u16; module const SEQ_COUNT_MODULUS:u32=1<<14
  Topology: new -> set_id_base -> connect evt.log_out/text_log_out/time_out -> hand its input factories to the framer/deframer output ports.

space_packet_framer::SpacePacketFramer (PASSIVE, all inputs sync):
  fn new(name:&str)->Arc<Self>; pub base: PassiveBase; pub evt: EventGlue
  output ports: buffer_allocate: OutputPort<dyn BufferGetPort>, buffer_deallocate: OutputPort<dyn BufferSendPort>, get_apid_seq_count: OutputPort<dyn ApidSequenceCountPort>, data_out / data_return_out: OutputPort<dyn ComDataWithContextPort>, com_status_out: OutputPort<dyn SuccessConditionPort>
  input factories: data_in(..)->PortRef<dyn ComDataWithContextPort>; data_return_in(..)->PortRef<dyn ComDataWithContextPort>; com_status_in(..)->PortRef<dyn SuccessConditionPort>
  fn clear_no_buffer_available_throttle(&self)
  module consts: EVENTID_NO_BUFFER_AVAILABLE:FwEventIdType=0 (WARNING_HI, no args), EVENTID_NO_BUFFER_AVAILABLE_THROTTLE:u32=5
  Requires buffer_allocate, buffer_deallocate, get_apid_seq_count, data_out, data_return_out connected (unconnected .get() asserts); com_status_out is optional.

space_packet_deframer::SpacePacketDeframer (PASSIVE, dataIn guarded):
  fn new(name:&str)->Arc<Self>; pub base, evt
  output ports: data_out, data_return_out: OutputPort<dyn ComDataWithContextPort>; validate_apid_seq_count: OutputPort<dyn ApidSequenceCountPort> (REQUIRED, invoked unconditionally); error_notify: OutputPort<dyn ErrorNotifyPort> (OPTIONAL, connection-checked)
  input factories: data_in(..), data_return_in(..) -> PortRef<dyn ComDataWithContextPort>
  module consts: EVENTID_INVALID_PACKET=0 (WARNING_HI, no args), EVENTID_INVALID_LENGTH=1 (WARNING_HI, args [u64 transmitted][u64 actual])

tm_framer::TmFramer (PASSIVE, all inputs sync):
  fn new(name:&str)->Arc<Self>; pub base: PassiveBase (NO evt/tlm/cmd — the component has none)
  output ports: data_out, data_return_out: OutputPort<dyn ComDataWithContextPort>, com_status_out: OutputPort<dyn SuccessConditionPort>
  input factories: data_in(..), data_return_in(..) -> PortRef<dyn ComDataWithContextPort>; com_status_in(..)->PortRef<dyn SuccessConditionPort>
  fn owns_frame_buffer(&self)->bool (observability for the OWNED/NOT_OWNED handshake)
  module consts: IDLE_DATA_PATTERN:u8=0x44; TRAILER_OFFSET:usize=1022; TM_PAYLOAD_CAPACITY:usize=1016; MIN_IDLE_PACKET_SIZE:usize=7; MAX_PAYLOAD_SIZE:usize=1009; SEGMENT_LENGTH_ID:u8=3; IDLE_SEQUENCE_FLAGS:u8=3
  Ownership contract: exactly one frame in flight; dataIn asserts the buffer is owned, dataReturnIn asserts it is not owned and that capacity==1024. Downstream MUST copy the frame before returning it.

tc_deframer::TcDeframer (PASSIVE, dataIn guarded):
  fn new(name:&str)->Arc<Self>; pub base, evt
  fn configure(&self, vc_id:u16, spacecraft_id:u16, accept_all_vcid:bool)  // defaults: vc_id 0, SPACECRAFT_ID, accept_all=true
  output ports: data_out, data_return_out: OutputPort<dyn ComDataWithContextPort>; error_notify: OutputPort<dyn ErrorNotifyPort> (OPTIONAL)
  input factories: data_in(..), data_return_in(..) -> PortRef<dyn ComDataWithContextPort>
  module consts: EVENTID_INVALID_PACKET=0 (WARNING_LO, no args), EVENTID_INVALID_SPACECRAFT_ID=1 (WARNING_LO, [u16][u16]), EVENTID_INVALID_FRAME_LENGTH=2 (WARNING_HI, [u16 transmitted][u64 actual]), EVENTID_INVALID_VC_ID=3 (ACTIVITY_LO, [u16][u16]), EVENTID_INVALID_CRC=4 (WARNING_HI, [u16 computed][u16 transmitted] — inverted vs the FPP declaration, C++ parity); MIN_TC_FRAME_SIZE:usize=7

No component here has async ports, a queue, commands, telemetry or parameters, so there are no msg types and no queue-message sizing.
Typical ComCcsds wiring: comQueue.dataOut -> spacePacketFramer.data_in; spacePacketFramer.data_out -> tmFramer.data_in; tmFramer.data_out -> comStub; returns mirror backwards through data_return_in/data_return_out; spacePacketFramer.get_apid_seq_count -> apidManager.get_apid_seq_count_in; uplink: frameAccumulator -> tcDeframer.data_in -> spacePacketDeframer.data_in -> fprimeRouter.data_in, with spacePacketDeframer.validate_apid_seq_count -> apidManager.validate_apid_seq_count_in.
```

### Implementation notes / deviations

Deviations (all documented in the module doc comments):
1. `getApidSeqCount` / `validateApidSeqCount` are ordinary `OutputPort`s whose `.get()` fw_asserts when unconnected, matching the C++ generated return-value output ports (which FW_ASSERT connectivity). ccsds.md porting note 7 suggests making them optional; I did not, because making them optional would silently frame packets with sequence count 0 instead of failing like C++. `errorNotify` IS connection-checked (`try_get`), as in C++.
2. TmFramer's static `U8 m_frameBuffer[1024]` + `BufferOwnershipState` become `Option<BufferStorage>` (Some == OWNED). The C++ "returned pointer lies within m_frameBuffer" assert becomes: not currently owned AND capacity == TM_FRAME_FIXED_SIZE. Same crash on the same misuse.
3. Events are emitted after the state mutex is released (ApidManager, TcDeframer) — the house pattern already used by BufferManager; C++ emits under the guarded mutex. Behaviorally equivalent, deadlock-safe. The map read/insert pair in ApidManager::validate stays inside one critical section, so the resync remains atomic.
4. TcDeframer's InvalidCrc argument inversion (declared `(transmitted, computed)`, emitted `(computed, transmitted)`) is reproduced verbatim for byte-compatibility with C++ flight software, with a loud comment and a test that pins it. Flagging it here as the analysis asks.
5. `M_PDUHeader` is spelled `MPduHeader` (Rust identifier conventions); every other C++ type name is preserved (TMHeader, TCHeader, AOSHeader...).
6. ComCfg CCSDS constants (SPACECRAFT_ID, TM_FRAME_FIXED_SIZE, AOS_MAX_FRAME_FIXED_SIZE, AGGREGATION_SIZE) live in `ccsds::types` because I may not edit fprime-config; AGGREGATION_SIZE is derived from the others, not hardcoded. If config-forking is wanted later they should move to `fprime-config` (a mechanical move; nothing outside ccsds/ references them today).
7. The CCSDS port traits (ApidSequenceCountPort, ErrorNotifyPort) are declared in `ccsds::types` following the com_stub precedent for crate-local port traits.
8. `TM_FRAME_FIXED_SIZE` is a plain const, not a const generic — the C++ is equally static (`ComCfg::TmFrameFixedSize`); const-parameterizing TmFramer was judged unnecessary complexity for phase 1 (noted as ccsds.md porting note 9's "ideally").
9. Deliberately NOT ported (outside my file list / out of scope per the analysis): CcsdsTcFrameDetector **[ported in phase 2 — see "Phase 2" below]** (belongs in `frame_accumulator.rs`, which I do not own — nothing in the workspace implements it yet, so the CCSDS uplink has no frame synchronizer; it would be ~40 lines over `Crc16` + `TCHeader::build_flags_and_sc_id(true,false,SPACECRAFT_ID)==0x2044` and the existing `FrameDetector` trait), ComAggregator, and the AOS/SDLS/CFDP siblings. AOSHeader/AOSTrailer/MPduHeader/SaMapEntry/SdlsStatus/Tfvn are declared for Types.fpp parity but nothing consumes them yet.
10. No async ports exist anywhere in this subsystem (all five components are passive with sync/guarded inputs), so the queue-message sizing rule does not apply here.
No sibling breakage observed: `cargo build --workspace`, `cargo clippy -p fprime-svc --all-targets -- -D warnings`, `cargo fmt -p fprime-svc` clean; `cargo test -p fprime-svc` green (558 passed, 85 of them in ccsds) and a full `cargo test --workspace` run was green (22 test binaries, 0 failures). `cargo doc -p fprime-svc --no-deps` is warning-free for these files. Nothing committed or pushed; only files under crates/fprime-svc/src/ccsds/ were modified (mod.rs untouched).

## Svc utilities (TlmPacketizer, ComLogger, SystemResources)

```text
## TlmPacketizer (fprime_svc::tlm_packetizer, ACTIVE)

```text
TlmPacketizer::new(name: &str) -> Arc<Self>
pub fields: active: ActiveBase; cmd: CmdGlue; evt: EventGlue; tlm: TlmGlue; prm: PrmGlue;
            pkt_send: [OutputPort<dyn ComPort>; TELEMETRY_SEND_PORTS=2]; ping_out: OutputPort<dyn PingPort>
init(&self, queue_depth: FwSizeType)                       // queue msg size = QUEUE_MSG_SIZE (usize 522)
reg_commands(&self)                                        // opcodes 0..=5
set_packet_list(&self, packets: &[TlmPacketizerPacket<'_>], ignore_list: &[TlmPacketizerChannelEntry], start_level: FwChanIdType)
load_parameters(&self)                                     // no-op when prm_get_out unconnected
serialize_param(&self, base_id, local_id: FwPrmIdType, buf: &mut dyn SerBufAny) -> SerializeStatus
deserialize_param(&self, base_id, local_id, prm_stat: ParamValid, buf: &mut dyn SerBufAny) -> SerializeStatus

Input factories (fn(self: &Arc<Self>, port_num: FwIndexType) -> PortRef<...>):
  tlm_recv_in  -> PortRef<dyn TlmPort>                       (SYNC)
  tlm_get_in   -> PortRef<dyn crate::tlm_chan::TlmGetPort>   (SYNC, returns TlmValid)
  control_in   -> PortRef<dyn EnableSectionPort>             (async, Assert, msg 1)
  ping_in      -> PortRef<dyn PingPort>                      (async, Assert, msg 2)
  run_in       -> PortRef<dyn SchedPort>                     (async, Assert, msg 3)
  configure_section_group_rate_in -> PortRef<dyn ConfigureGroupRatePort> (async, Assert, msg 4)
  cmd_in       -> PortRef<dyn CmdPort>                       (async, Assert, msg 5)
impls ComponentDispatch + ActiveComponent.

New public port traits (defined here, absent from fprime-comp):
  trait EnableSectionPort { fn invoke(&self, port_num, section: TelemetrySection, enabled: Enabled) }
  trait ConfigureGroupRatePort { fn invoke(&self, port_num, section: TelemetrySection, tlm_group: FwChanIdType,
                                           rate_logic: RateLogic, min_delta: u32, max_delta: u32) }

Public types:
  fpp_enum TelemetrySection : i32 { Realtime=0, Recorded=1, NumSections=2 }   (NumSections IS a valid wire value)
  fpp_enum RateLogic : i32 { Silenced=0, EveryMax=1, OnChangeMin=2, OnChangeMinOrEveryMax=3 }
  struct GroupConfig { enabled: Enabled, force_enabled: Enabled, rate_logic: RateLogic, min: u32, max: u32 }
         (Copy, Default = DEFAULT_GROUP_CONFIG, SERIALIZED_SIZE = 14, Serialize/Deserialize/FppSized, const new(..))
  fpp_array GroupConfigs = [GroupConfig; 4] (56 B); fpp_array SectionConfigs = [GroupConfigs; 2] (112 B)
  struct SectionEnabled(pub [Enabled; 2])  (Default = all ENABLED, SERIALIZED_SIZE = 2, Index/IndexMut)
  struct TlmPacketizerChannelEntry { id: FwChanIdType, size: FwSizeType }  + const new(id, size)
  struct TlmPacketizerPacket<'a> { channels: &'a [TlmPacketizerChannelEntry], id: FwTlmPacketizeIdType, level: FwChanIdType } + const new(..)
  const IGNORE_OMIT_LIST: &[TlmPacketizerChannelEntry] = &[]

Module consts: MAX_PACKETIZER_PACKETS=50, MAX_PACKETIZER_CHANNELS=200, TLMPACKETIZER_MAX_MISSING_TLM_CHECK=25,
  MAX_CONFIGURABLE_TLMPACKETIZER_GROUP=3, NUM_CONFIGURABLE_TLMPACKETIZER_GROUPS=4, NUM_SECTIONS=2,
  TELEMETRY_SEND_PORTS=2, TELEMETRY_SEND_PORT_MAPPING=[[0,0,0,0],[1,1,1,1]], QUEUE_MSG_SIZE=522, PACKET_HEADER_SIZE=15
Assoc consts: OPCODE_SET_LEVEL=0, OPCODE_SEND_PKT=1, OPCODE_ENABLE_SECTION=2, OPCODE_ENABLE_GROUP=3,
  OPCODE_FORCE_GROUP=4, OPCODE_CONFIGURE_GROUP_RATES=5; EVENTID_NO_CHAN=0, LEVEL_SET=1, MAX_LEVEL_EXCEED=2,
  PACKET_SENT=3, PACKET_NOT_FOUND=4, SECTION_UNCONFIGURABLE=5, OVERSIZED_CHANNEL=6 (OVERSIZED_CHANNEL_THROTTLE=10);
  CHANID_GROUP_CONFIGS=0, CHANID_SECTION_ENABLED=1; PARAMID_SECTION_ENABLED=0, PARAMID_SECTION_CONFIGS=1;
  MSG_TYPE_CONTROL_IN=1, MSG_TYPE_PING_IN=2, MSG_TYPE_RUN=3, MSG_TYPE_CONFIGURE_SECTION_GROUP_RATE=4, MSG_TYPE_CMD_IN=5

Topology: new -> active.queued.base.set_id_base -> connect(pkt_send[0..2], ping_out, cmd/evt/tlm/prm ports)
  -> init(depth) -> set_packet_list(...) -> reg_commands -> load_parameters -> active.start(&arc, prio, stack, aff)
  -> ... -> active.exit()/join().  Packet wire layout: [u16 0x0004][u16 pktId][Time 11B][raw channel values].
```

## ComLogger (fprime_svc::com_logger, ACTIVE)

```text
ComLogger::new(name) -> Arc<Self>                                   // UNINITIALIZED (drops buffers)
ComLogger::with_log_file(name, file_prefix: &str, max_file_size: u32, store_buffer_length: bool) -> Arc<Self>
init_log_file(&self, file_prefix, max_file_size, store_buffer_length)
init(&self, queue_depth: FwSizeType)                                // queue msg size = QUEUE_MSG_SIZE (usize 524)
reg_commands(&self)                                                 // opcode 0x00
file_name(&self) -> FileNameString;  is_file_open(&self) -> bool
pub fields: active: ActiveBase; cmd: CmdGlue; evt: EventGlue; ping_out: OutputPort<dyn PingPort>
Input factories: com_in -> PortRef<dyn ComPort> (async, msg 1); ping_in -> PortRef<dyn PingPort> (async, msg 2);
                 cmd_in -> PortRef<dyn CmdPort> (async, msg 3).  impls ComponentDispatch + ActiveComponent + Drop.
Assoc consts: OPCODE_CLOSE_FILE=0x00; EVENTID_FILE_OPEN_ERROR=0x00, FILE_WRITE_ERROR=0x01,
  FILE_VALIDATION_ERROR=0x02, FILE_CLOSED=0x03, FILE_NOT_INITIALIZED=0x04 (FILE_NOT_INITIALIZED_THROTTLE=5);
  MSG_TYPE_COM_IN=1, MSG_TYPE_PING_IN=2, MSG_TYPE_CMD_IN=3
Module items: pub const QUEUE_MSG_SIZE: usize = 524; pub const VFILE_HASH_CHUNK_SIZE: usize = 256;
  pub enum ValidateStatus (repr u32, Os::ValidateFile ordinals 0..=9);
  pub fn create_validation(file_name: &str, hash_file_name: &str) -> ValidateStatus  // BIG-endian .CRC32 sidecar
Topology: new/with_log_file -> set_id_base -> connect -> init(depth) -> reg_commands -> active.start -> exit/join.
```

## SystemResources (fprime_svc::system_resources, PASSIVE)

```text
SystemResources::new(name) -> Arc<Self>                             // default ProcSampler
SystemResources::with_sampler(name, Box<dyn ResourceSampler>) -> Arc<Self>
reg_commands(&self); cpu_count(&self) -> usize; is_enabled(&self) -> bool
pub fields: base: PassiveBase; cmd: CmdGlue; evt: EventGlue (log/text/time); tlm: TlmGlue
Input factories: run_in -> PortRef<dyn SchedPort> (GUARDED); cmd_in -> PortRef<dyn CmdPort> (GUARDED command)
Public types: fpp_enum SystemResourceEnabled : u8 { Disabled=0, Enabled=1 };
  enum GenericStatus { OpOk=0, Error=1 }; struct UsedTotal { used: FwSizeType, total: FwSizeType };
  trait ResourceSampler { cpu_count(&self,&mut FwSizeType)->GenericStatus;
      cpu_ticks(&self,&mut UsedTotal, cpu_index: FwSizeType)->GenericStatus;
      memory_usage(&self,&mut UsedTotal)->GenericStatus;
      free_space(&self, path:&str, total:&mut FwSizeType, free:&mut FwSizeType)->fprime_os::filesystem::Status }
  struct ProcSampler (default impl)
Module consts: CPU_COUNT=16; NON_VOLATILE_PATH="/"
Assoc consts: OPCODE_ENABLE=0; CHANID_MEMORY_TOTAL=0, MEMORY_USED=1, NON_VOLATILE_TOTAL=2, NON_VOLATILE_FREE=3,
  CPU=4, CPU_00=5 (CPU_nn = CPU_00 + nn, up to 20)
Topology: new -> base.set_id_base -> connect cmd/evt/tlm/time ports -> reg_commands -> wire run_in to a rate group.
```
```

### Implementation notes / deviations

**Verification.** `cargo build --workspace` green; `cargo test -p fprime-svc` 616/616 green (58 of them mine: 33 tlm_packetizer, 14 com_logger, 11 system_resources); `cargo test --workspace` fully green (no sibling breakage observed at any point); `cargo clippy -p fprime-svc --all-targets -- -D warnings` clean; `cargo fmt -p fprime-svc --check` clean (verified 3 consecutive full-crate runs, no flakes). Nothing committed; only my three files were written (verified by mtime that `cargo fmt -p fprime-svc` did not touch sibling modules).

**Deviations, each documented in code:**
1. `Os::Cpu`/`Os::Memory` are not ported in fprime-os, so — as the task allows — the sampling lives INSIDE `system_resources.rs` behind the `ResourceSampler` trait: `ProcSampler` reads `/proc/stat` (a direct port of `getCpuData`, incl. the "cpu" prefix check, the CPU-index field check and USER+NICE+SYSTEM/+IDLE arithmetic) and `/proc/meminfo` (`MemTotal`/`MemFree` ×1024 — deliberately NOT `MemAvailable`, matching `sysinfo`'s `freeram`), with `std::thread::available_parallelism()` as the non-Linux core-count fallback.
2. Free space goes through `fprime_os::filesystem::get_free_space`, which is `NotSupported` in this workspace (no zero-dependency statvfs), so `NON_VOLATILE_TOTAL/FREE` are skipped by default — behaviorally identical to the C++ VxWorks path. A project supplies a real one through the OSAL seam or its own `ResourceSampler`.
3. `Os::ValidateFile` is not ported either, so `com_logger::create_validation` + `ValidateStatus` + the status-translation table are implemented in the module (256-byte chunked hashing, BIG-endian `HashBuffer` sidecar — explicitly NOT the native-endian `Utils::CRCChecker` encoding).
4. `GroupConfig` and `SectionEnabled` are hand-written instead of `fpp_struct!`/`fpp_array!`-generated because their members are `Fw::Enabled`, which has no `FppSized` impl in fprime-fw and cannot get one here (orphan rule). `GroupConfigs`/`SectionConfigs` DO use `fpp_array!` (I added `impl FppSized for GroupConfig`, a local type). Byte formats verified by test: 14/56/112 and 2 bytes.
5. Lock scope: the C++ `m_lock` is reproduced exactly (per-packet in `TlmRecv`/`TlmGet`, twice per packet in `Run`, `has_value` written inside it). The channel table sits behind an `RwLock` (C++ reads it lock-free after init) and the `[section][group]` config/flags behind a second mutex; lock order is always table → packets → ctrl, so no inversion. In `Run` the flag/counter updates happen just before the port invocations rather than just after, so the ports are invoked outside the ctrl lock — nothing reads those fields in between. Same pattern in SystemResources (`run` samples under the mutex, writes telemetry after release, order preserved) and in the tlm_packetizer command handlers.
6. Enum command args answer `ValidationError` (workspace discipline; stock C++ generated code would answer `FORMAT_ERROR` at deserialize) — including the SEND_PKT/ENABLE_* `TelemetrySection` and `RateLogic` args. Short/residual args are `FormatError`. `TelemetrySection::NumSections` is a declared FPP constant, so it deserializes fine and is rejected by the range check with `ValidationError`, exactly as in C++.
7. `ComLogger`'s C++ `FW_ASSERT`ed `format()` statuses become explicit length `fw_assert!`s (Rust `FwString::set` truncates silently, C++ asserts).
8. Not ported deliberately: the C++ `Fw::ParamExternalDelegate` registration machinery (fprime-comp has no external-parameter framework) — the delegate methods are public on the component and `load_parameters()` drives them through `PrmGlue`; and the `configureSectionGroupRate` port's C++ `FW_ASSERT`-on-bad-args behavior is kept (bad port args crash, unlike the command path).

**Test coverage highlights:** literal-byte packetized packets (descriptor 0x0004, packet id, the 11 time bytes rewritten in place, per-channel offsets, zeros for unwritten channels), the 112-byte `SectionConfigs` default, both async envelopes, packet-table lookup/missing-channel/ignore/oversize paths, `TlmGet` padding, `set_packet_list` re-entrancy and all four configuration asserts (`#[should_panic]`), every rate-logic branch (ON_CHANGE_MIN, EVERY_MAX, SILENCED counter freeze, section disable + force override, unconnected ports, the retained-REQUESTED-flag gotcha), all command status branches; ComLogger file naming, record bytes with and without length prefixes, rotation at the strict `>` boundary, the `.CRC32` sidecar (canonical `123456789` vector), destructor close-without-event, uninitialized throttle, one-shot open-error latch; SystemResources full channel order with an injected sampler, backwards-counter skip, zero-delta 100%, hard-coded "/" path, per-sampler error isolation, ENABLE gating and status branches.

**Open questions / for the doc wave:** the config constants for all three components (`MAX_PACKETIZER_*`, `TELEMETRY_SEND_PORT_MAPPING`, `CPU_COUNT`, `VFILE_HASH_CHUNK_SIZE`, …) live as module consts because `fprime-config` has no slots for them and is owned by another wave — flagged for migration, same as `PassiveTextLoggerCfg`. `crate::tlm_chan::TlmGetPort` is reused for the `TlmGet` port rather than redeclaring it.

## Linux hardware drivers (GPIO, UART, I2C, SPI)

```text
All items are reached by fully-qualified path (lib.rs was not edited): `fprime_drv::gpio::*`, `::uart::*`, `::i2c::*`, `::spi::*`.

GPIO (gpio.rs)
  enum Logic : u8 {Low=0, High=1} (fpp_enum: Default=Low, TryFrom<u8>, Serialize/Deserialize u8, VALUES/NUM_CONSTANTS/SERIALIZED_SIZE)
  enum GpioStatus : u8 {OpOk=0, NotOpened=1, InvalidMode=2, UnknownError=3} (same fpp_enum surface)
  enum GpioConfiguration {GpioOutput, GpioInput, GpioInterruptRisingEdge, GpioInterruptFallingEdge, GpioInterruptBothRisingAndFallingEdges}: const fn is_interrupt()->bool; edge_fires(from:Logic,to:Logic)->bool; sysfs_edge()->&'static str
  trait GpioWritePort: Send+Sync { fn invoke(&self, port_num: FwIndexType, state: Logic) -> GpioStatus }
  trait GpioReadPort: Send+Sync  { fn invoke(&self, port_num: FwIndexType, state: &mut Logic) -> GpioStatus }
  fn errno_to_file_status(&io::Error)->fprime_os::file::Status; fn errno_to_gpio_status(&io::Error)->GpioStatus
  struct GpioChipInfo { pub name: String, pub label: String, pub pin_message: String }: fn new(name,label)->Self (pin_message="Unknown")
  enum GpioOpenError { Chip(FileStatus), Pin{pin_message:String, status:FileStatus} }: const fn status(&self)->FileStatus
  enum PollOutcome { Interrupt, NoInterrupt, ReadError{expected:u32, got:u32}, PollError(i32) }
  trait GpioBackend: Send+Sync { fn open(&self, device:&str, gpio:u32, configuration:GpioConfiguration, default_state:Logic, consumer:&str) -> Result<GpioChipInfo, GpioOpenError>; fn read(&self)->Result<Logic,GpioStatus>; fn write(&self, state:Logic)->GpioStatus; fn poll(&self, timeout:Duration)->PollOutcome; fn close(&self) {} }
  struct StubGpioBackend: const fn new()
  struct SysfsGpioBackend: fn new(); with_root<P:AsRef<Path>>(P); with_sample_interval(Duration)->Self (builder); sample_interval()->Duration; line()->Option<u32>
  consts: DEFAULT_SAMPLE_INTERVAL: Duration = 10ms; SYSFS_GPIO_ROOT = "/sys/class/gpio"; fn default_backend()->Box<dyn GpioBackend>
  struct LinuxGpioDriver { pub base: PassiveBase, pub evt: EventGlue, pub gpio_interrupt_out: OutputPort<dyn CyclePort> }
    fn new(name)->Arc<Self>; fn with_backend(name, Box<dyn GpioBackend>)->Arc<Self>
    fn open(&self, device:&str, gpio:u32, configuration:GpioConfiguration, default_state:Logic) -> fprime_os::file::Status
    fn close(&self); fn configuration(&self)->Option<GpioConfiguration>
    fn start(self:&Arc<Self>, priority:FwTaskPriorityType, stack_size:FwSizeType, cpu_affinity:FwSizeType)->GpioStatus  // InvalidMode unless an interrupt mode
    fn stop(&self); fn join(&self)->fprime_os::task::Status   // stop() MUST precede join()
    input factories: fn gpio_read_in(self:&Arc<Self>, port_num)->PortRef<dyn GpioReadPort>; fn gpio_write_in(..)->PortRef<dyn GpioWritePort>
    assoc consts: EVENTID_OPEN_CHIP=0, EVENTID_OPEN_CHIP_ERROR=1, EVENTID_OPEN_PIN_ERROR=2, EVENTID_INTERRUPT_READ_ERROR=3, EVENTID_POLLING_ERROR=4, EVENTID_INTERRUPT_TIME_ERROR=5; GPIO_POLL_TIMEOUT: u64 = 500 (ms); CONSUMER_LABEL_SIZE=32
    Wiring: connect evt.log_out/text_log_out/time_out; connect gpio_interrupt_out to an ASYNC input (it fires on the poll thread). No Tlm, no Cmd ports (FPP parity).

UART (uart.rs) — reuses crate::byte_stream::{ByteStreamStatus, ByteStreamSendPort, ByteStreamDataPort, ByteStreamReadyPort}
  enum UartBaudRate (repr u32, discriminants ARE the baud numbers): Baud9600..Baud4000K; const fn as_u32()->u32
  enum UartFlowControl {NoFlow=0, HwFlow=1} (Default NoFlow); enum UartParity {ParityNone=0, ParityOdd=1, ParityEven=2} (Default ParityNone)
  enum UartConfigPolicy {Reject (Default), Stty, TrustExternal}
  fn stty_arguments(device:&str, baud, flow_control, parity) -> Vec<String>
  trait SerialBackend: Send+Sync { fn open(&self, device:&str)->io::Result<()>; fn read(&self, dest:&mut [u8])->io::Result<usize>; fn write(&self, data:&[u8])->io::Result<usize>; fn is_open(&self)->bool; fn close(&self) {} }
  struct FileSerialBackend: fn new()
  struct LinuxUartDriver { pub base: PassiveBase, pub evt: EventGlue, pub tlm: TlmGlue, pub allocate_out: OutputPort<dyn BufferGetPort>, pub deallocate_out: OutputPort<dyn BufferSendPort>, pub recv_out: OutputPort<dyn ByteStreamDataPort>, pub ready_out: OutputPort<dyn ByteStreamReadyPort> }
    fn new(name)->Arc<Self>; fn with_backend(name, Box<dyn SerialBackend>)->Arc<Self>
    fn set_config_policy(&self, UartConfigPolicy); fn config_policy(&self)->UartConfigPolicy
    fn open(&self, device:&str, baud:UartBaudRate, fc:UartFlowControl, parity:UartParity, allocation_size:FwSizeType)->bool
    fn open_preconfigured(&self, device:&str, allocation_size:FwSizeType)->bool
    fn start(self:&Arc<Self>, priority, stack_size, cpu_affinity); fn quit_read_thread(&self); fn join(&self)->task::Status; fn close(&self)
    fn bytes_sent(&self)->FwSizeType; fn bytes_received(&self)->FwSizeType
    input factories: send_in -> PortRef<dyn ByteStreamSendPort>; recv_return_in -> PortRef<dyn BufferSendPort>; run_in -> PortRef<dyn SchedPort>
    assoc consts: EVENTID_OPEN_ERROR=0, EVENTID_CONFIG_ERROR=1, EVENTID_WRITE_ERROR=2, EVENTID_READ_ERROR=3, EVENTID_PORT_OPENED=4, EVENTID_NO_BUFFERS=5, EVENTID_BUFFER_TOO_SMALL=6 (reserved); ERROR_EVENT_THROTTLE=5, NO_BUFFERS_THROTTLE=20; CHANID_BYTES_SENT=0, CHANID_BYTES_RECV=1; NO_BUFFER_RETRY_US=50_000, IDLE_READ_RETRY_US=50_000
    Topology order: new -> set_id_base -> connect (allocate/deallocate/recv/ready/evt/tlm) -> set_config_policy -> open*/open_preconfigured -> start -> ... -> quit_read_thread -> join -> close

I2C (i2c.rs)
  const I2C_DRIVER_PORTS: usize = 10
  enum I2cStatus : u8 {I2cOk=0, I2cAddressErr=1, I2cWriteErr=2, I2cReadErr=3, I2cOpenErr=4, I2cOtherErr=5} (fpp_enum)
  trait I2cPort { fn invoke(&self, port_num, addr:u32, ser_buffer:&mut Buffer)->I2cStatus }
  trait I2cWriteReadPort { fn invoke(&self, port_num, addr:u32, write_buffer:&mut Buffer, read_buffer:&mut Buffer)->I2cStatus }
  port types only (no implementation in tree): I2cRequestPort, I2cWriteReadRequestPort, I2cCallbackPort, I2cWriteReadCallbackPort
  trait I2cBackend: Send+Sync { fn open(&self, device:&str)->bool; fn write(&self, addr:u32, data:&[u8])->I2cStatus; fn read(&self, addr:u32, dest:&mut [u8])->I2cStatus; fn write_read(&self, addr:u32, write:&[u8], read:&mut [u8])->I2cStatus /* must report failures as I2cOtherErr */; fn close(&self) {} }
  struct StubI2cBackend: const fn new()  (open->true, all transfers->I2cOk: upstream stub parity)
  struct LinuxI2cDriver { pub base: PassiveBase }
    fn new(name)->Arc<Self>; fn with_backend(name, Box<dyn I2cBackend>)->Arc<Self>; fn open(&self, device:&str)->bool; fn is_open(&self)->bool; fn device(&self)->String; fn close(&self)
    input factories (all GUARDED): write_in -> PortRef<dyn I2cPort>; read_in -> PortRef<dyn I2cPort>; write_read_in -> PortRef<dyn I2cWriteReadPort>
    No events/telemetry/commands and no special ports (FPP parity).

SPI (spi.rs)
  enum SpiStatus : u8 {SpiOk=0, SpiOpenErr=1, SpiConfigErr=2, SpiMismatchErr=3, SpiWriteErr=4, SpiOtherErr=5} (fpp_enum)
  enum SpiFrequency (repr u32): SpiFrequency1Mhz..SpiFrequency20Mhz; const fn as_hz()->u32
  enum SpiMode (repr u8, Default SpiModeCpolLowCphaLow): 0..3; const fn as_kernel_mode()->u8
  trait SpiWriteReadPort { fn invoke(&self, port_num, write_buffer:&mut Buffer, read_buffer:&mut Buffer)->SpiStatus }
  trait SpiReadWritePort  { fn invoke(&self, port_num, write_buffer:&mut Buffer, read_buffer:&mut Buffer) }  // DEPRECATED upstream
  struct SpiConfigMismatch { pub parameter: String, pub write_value: u32, pub read_value: u32 }
  enum SpiOpenError { Open(i32), Config(i32) }
  trait SpiBackend: Send+Sync { fn open(&self, device:FwIndexType, select:FwIndexType, clock:SpiFrequency, mode:SpiMode)->Result<Vec<SpiConfigMismatch>, SpiOpenError>; fn write_read(&self, write:&[u8], read:&mut [u8])->Result<(), i32>; fn close(&self) {} }
  struct StubSpiBackend: const fn new()  (DEFAULT: open fails with Open(-1), transfer Ok)
  struct HalfDuplexSpidevBackend: fn new_write_then_read_not_full_duplex()->Self; with_device_root<P:Into<PathBuf>>(P)->Self; device_path(device, select)->PathBuf; const SPIDEV_ROOT = "/dev"
  struct LinuxSpiDriver { pub base: PassiveBase, pub evt: EventGlue, pub tlm: TlmGlue }
    fn new(name)->Arc<Self>; fn with_backend(name, Box<dyn SpiBackend>)->Arc<Self>
    fn open(&self, device:FwIndexType, select:FwIndexType, clock:SpiFrequency, mode:SpiMode)->bool  // fw_asserts device>=0, select>=0
    fn is_open(&self)->bool; fn bytes(&self)->FwSizeType; fn close(&self)
    input factories: spi_write_read_in -> PortRef<dyn SpiWriteReadPort> (GUARDED); spi_read_write_in -> PortRef<dyn SpiReadWritePort> (deprecated, routed through the same mutex)
    assoc consts: EVENTID_SPI_OPEN_ERROR=0, EVENTID_SPI_CONFIG_ERROR=1, EVENTID_SPI_WRITE_ERROR=2, EVENTID_SPI_CONFIG_MISMATCH=3, EVENTID_SPI_PORT_OPENED=4 (reserved, never emitted); WRITE_ERROR_THROTTLE=5; CHANID_SPI_BYTES=0
```

### Implementation notes / deviations

DEVIATIONS (all documented in rustdoc on the public API, not just here):
1. No ioctl anywhere, so: GPIO character-device access, UART termios, all I2C transactions and SPI full duplex/config are unreachable. Each driver is the full component surface over a backend trait (GpioBackend / SerialBackend / I2cBackend / SpiBackend) so a downstream crate that permits libc/unsafe can drop in hardware access without forking.
2. GPIO interrupts via SysfsGpioBackend are LEVEL SAMPLED (poll(2) unavailable): pulses shorter than the sample interval are missed, latency is bounded by the interval (default 10 ms, configurable), two edges in one interval collapse, and the timestamp is the sampler's RawTime::now(). Stated loudly on the type and in the module header. The `edge` attribute is still written best-effort. /dev/gpiochipN -> sysfs global line is resolved by indexing gpiochip* dirs sorted by `base` (the only mapping plain file reads allow) — documented as a heuristic; missing root/chip -> FileStatus::NotSupported.
3. GPIO ApiVersion (v1/v2 uAPI selection) not ported — meaningless without ioctl. The C++ stub's silent open() becomes an OpenChipError event here because the event path lives in the component; the returned status is identical.
4. UART: configuration is opt-in. Default UartConfigPolicy::Reject emits ConfigError (FPP id 1, never emitted upstream) and does NOT open, rather than silently running at the wrong baud; Stty applies settings via std::process::Command (external `stty` dependency, argument vector exposed and unit-tested); TrustExternal / open_preconfigured skip configuration. O_NOCTTY cannot be requested (the process may acquire the tty as controlling terminal) — documented. OpenError's `error` field is always -1 here (C++ passes the fd, which is -1 on open failure but a valid fd on later termios failures — there is no termios stage here). The read retry sleeps IDLE_READ_RETRY_US=50 ms because VMIN/VTIME cannot be applied; the C++ loop relies on VTIME. quit_read_thread is only re-checked between reads (same caveat as C++). Device string is OWNED (C++ keeps the caller's const char*).
5. I2C: stub backend only. StubI2cBackend reproduces LinuxI2cDriverStub.cpp exactly (open->true, all handlers I2cOk). The trait documents that write_read must report failures only as I2cOtherErr.
6. SPI: StubSpiBackend is the default; because the component keeps the REAL driver's open-gate, a stub-backed driver answers SpiOpenErr from transfers where upstream's stub returned SPI_OK (documented divergence; the stub backend's own transfer still returns Ok, matching upstream at the seam). HalfDuplexSpidevBackend exists only behind new_write_then_read_not_full_duplex() and is documented as two separate bus transactions, NOT SpiWriteRead's full-duplex exchange, with mode/speed/bits coming from the device tree (so SPI_ConfigError/SPI_ConfigMismatch are unreachable with it). Both SPI ports are routed through the same mutex-locked handler — an improvement over C++'s unguarded deprecated port, documented.
7. Fw.Logic is declared in gpio.rs (fpp_enum!, byte-identical to Fw/Types/Types.fpp) because fprime-fw was outside my file set; moving it later is a source-only change.
8. Os.FileStatus / Os.RawTimeStatus event arguments are serialized as U8 (their FPP width) by casting the repr(i32) fprime-os enums — noted at each call site.
9. No lib.rs edits: the four modules were already declared; everything is reached by fully-qualified path (fprime_drv::gpio::LinuxGpioDriver, etc.). If the orchestrator wants root re-exports, add them to lib.rs centrally.

VERIFICATION: 83 new unit tests (gpio 30, uart 20, i2c 13, spi 20); fprime-drv totals 107 lib + 6 integration tests, all passing. `cargo build --workspace` green; full `cargo test --workspace` observed fully green (1122 tests, including siblings' fprime-svc 616). `cargo clippy -p fprime-drv --all-targets -- -D warnings` clean; `cargo fmt -p fprime-drv -- --check` clean; `cargo doc -p fprime-drv --no-deps` emits no warnings. Literal-byte tests cover every event/telemetry payload emitted (GPIO OpenChip/OpenChipError/OpenPinError/InterruptReadError/PollingError, UART ConfigError/PortOpened/OpenError/WriteError/ReadError/NoBuffers + both u64 channels, SPI OpenError/ConfigError/ConfigMismatch/WriteError + SPI_Bytes) plus the exact sysfs file contents (export/unexport/direction "low"/"high"/"in"/edge/value "0"/"1") and the exact stty argument vector. No sibling-file problems encountered.

OPEN QUESTION (non-blocking): the sysfs chip-index-by-base mapping cannot be verified without real hardware; a deployment that knows its global line numbers may prefer a backend that takes them directly.


# Phase 2 API notes

Ordered backlog: `docs/ROADMAP.md`. Each item below records what changed and
the deviations, in the same spirit as the phase-1 notes above.

## fprime-os: `file_path_utils` + `sandboxed_file`

`Os::FilePathUtils` (`MAX_PATH_LENGTH`, `PathStatus { Valid=0,
OutsideSandbox=1, InvalidPath=2, TooLong=3 }`, `resolve_path`,
`resolve_from_cwd`, `check_containment`) and `Os::SandboxedFile`
(re-exported as `fprime_os::SandboxedFile`) moved verbatim out of
`fprime-svc::file_uplink`. `FileUplink`, `FileDownlink` and `PrmDb` now share
the one implementation; `PrmDb`'s private `Option<String>`-returning copy is
deleted and its two call sites (`configure_load_sandbox`,
`read_param_file_impl`) use the `PathStatus`/`FileNameString` API. Semantics
are unchanged: purely lexical resolution, fail-open default sandbox (`/`),
every resolution or containment failure reported as
`file::Status::OutsideSandbox`. The `file_uplink` re-exports were NOT kept —
the crate is pre-1.0 and the only in-tree consumer (`file_downlink`) was
repointed.

## fprime-svc: `CcsdsTcFrameDetector`

`Svc::FrameDetectors::CcsdsTcFrameDetector` lives next to
`FprimeFrameDetector` in `frame_accumulator.rs` (the C++ directory layout),
using `ccsds::types::{TCHeader, TCTrailer, tc_subfields}` and
`ccsds::crc16::Crc16`. `new()` matches `(1 << BypassFlagOffset) |
ComCfg::SpacecraftId` = `0x2044`; `for_spacecraft(id)` is an addition for
deployments with a different ID (C++ hard-codes the config constant).
Deviations: none in behavior. Unlike the Rust `FprimeFrameDetector` there is
deliberately no ring-capacity check (C++ parity) — an oversized announcement
is `MoreDataNeeded`, and `FrameAccumulator` answers with
`FrameDetectionSizeError` and a one-byte slide (tested). A lookalike token
in garbage stalls the accumulator until the announced length has arrived
and only then fails the CRC and resyncs (tested; this is the C++ behavior
too, since a TC frame has no start word).

## fprime-svc: `FileManager::GenerateDp`

The command is now fully ported from `FileManager.cpp` (`GenerateDp_cmdHandler`,
`processDpChunks`, `finishDpGeneration`, the `run_internalInterfaceHandler`
DP-pacing half). New public surface: `product_get_out: OutputPort<dyn
DpGetPort>`, `product_send_out: OutputPort<dyn DpSendPort>`,
`CONTAINER_ID_FILE_DP = 0`, `RECORD_ID_FILE_CHUNK_HEADER = 0`,
`RECORD_ID_FILE_CHUNK_DATA = 1`, `SIZE_OF_FILE_CHUNK_HEADER_RECORD` (= 4 +
254), `size_of_file_chunk_data_record(n)` (= 6 + n), and the `fpp_struct!`
`FileChunkHeader { file_name: FileNameString, offset: u64, data_size: u32 }`.

Wire format per container (matching the autocoded
`serializeRecord_FileChunkHeaderRecord` / `..DataRecord`): the data region is
`[base+0 u32][u16 len][name][u64 offset][u32 dataSize][base+1 u32][u16 n][n
bytes]`; record ids are absolute (base id + record id), the container id is
`base + 0`, the priority is the command's or `DEFAULT_DP_PRIORITY` (10) for
zero, the time tag comes from the time port, and the data hash is left to
`DpWriter` (C++ parity). The buffer requested from `productGetOut` is
`DpContainer::packet_size_for_data_size(SIZE_OF_FILE_CHUNK_HEADER_RECORD +
size_of_file_chunk_data_record(readSize))` — i.e. sized for the string at
full capacity, as the autocoder's `SIZE_OF_..._RECORD` constant is — while
`dataSize` in the header is the bytes actually written.

Ported quirks (all tested): every failure path (BUSY, unconnected ports,
open/size/seek/read/serialize failures, buffer failure, invalid range)
emits its WARNING_HI event and still answers `OK`; `PACED` mode defers the
response to `finishDpGeneration` (one chunk per `schedIn` tick via the
`run` internal port, before the listing tick); `chunkSize` 0 or above
`GENERATE_DP_MAX_CHUNK_SIZE` clamps to the maximum; `endOffset` 0 or past
the end means end-of-file; an empty file (or empty range) reports
`Started(0)` + `Complete(0)` with no containers; `CommandsExecuted`/`Errors`
are untouched. Deviation: the newer upstream `resolveInSandbox` step is not
ported (this `FileManager` has no sandbox — ROADMAP item 2), so the command
opens the path as given.

`Ref` topology: `fileManager.productGetOut/productSendOut -> dpMgr[1]`, and
`dpMgr.bufferGetOut[1]`/`productSendOut[1]` are wired to the same
`dpBufferManager`/`dpWriter` inputs as index 0 (upstream FPP auto-numbers
the second producer the same way). `tests/subsystems_test.rs` drives a
framed `GenerateDp` of a 100-byte file at chunk 40 and checks three `.fdp`
files with byte-exact records.

## fprime-fpp: `fpp-to-rust`

`crates/fprime-fpp/README.md` is the reference. Notes that matter for the
rest of the workspace:

- The front end is a production-for-production port of the reference
  compiler. Deviations are only in error wording. One extra: `--check`
  reports success counts.
- The back end emits the *existing* macro layer (`fpp_enum!`,
  `fpp_struct!`, `fpp_array!`) for data types, so generated and
  hand-written types are the same kinds of Rust items; a struct member
  `x: [n] T` becomes a helper `fpp_array!` type `S_x_Array` because the
  macro layer needs `FppSized` on every member type.
- Names: FPP modules, types, constants and enum constants are kept
  verbatim (with lint allows on the generated modules); functions, fields
  and ports are `snake_case`. Rust keywords are `r#`-escaped. FPP
  parameters that collide with generated signature names get an `_arg`
  suffix; generator locals are `fpp_`-prefixed.
- Generated component bases embed `fprime_comp` glue and cores exactly as
  the hand-written components do, so a generated component and a
  hand-written one can be wired together by hand; the generated topology
  only wires generated components (it needs the generated port names).
- Bound framework ports may carry per-parameter passing modes because the
  hand-written traits are not uniform about `ref Fw.Buffer`
  (`BufferSend` moves it, `DpGet` fills it); see
  `Bindings::framework()`.
- `MSG_SIZE` is a `const` expression over `FppSized::SERIALIZED_SIZE` of
  the argument types (buffers use `2 + capacity`; escrowed buffers 8),
  evaluated by rustc, so a bound type without `FppSized` fails to compile
  rather than silently mis-sizing the queue.
- Not generated: state machine instances (error), serial ports (error),
  telemetry packet sets (ignored), the dictionary.

### JSON dictionary (`codegen::dictionary`)

`generate_dictionaries(&Analysis, &DictOptions) -> Result<Vec<DictFile>>`;
`DictOptions { project_version, framework_version, library_versions, targets }`
(defaults `"[no value specified]"`, like the reference); `DictFile { name, json: Json }`
with `name` = `<Topology>TopologyDictionary.json` / `<Qualified_System>SystemDictionary.json`;
`DICTIONARY_SPEC_VERSION = "1.0.0"`. `json::Json { Null, Bool, Int(i128), Float, Str, Arr, Obj(Vec<(String, Json)>) }`
with `obj/push/push_opt/get/as_str/as_arr`, `to_pretty()` and `Json::parse(&str)`.
Supporting analysis additions: `analysis::uses::Uses { found: BTreeSet<SymId> }` with
`expr/type_name/params/def/component_specifiers/resolve_deep` (enum constants count
as uses of their enum); `TopologyModel.packet_sets: Vec<TlmPacketSetModel { name, packets:
Vec<TlmPacketModel { name, id, group, members: Vec<TlmChannelRef>, loc }>, omitted, loc }>`;
`EventDef.every: Option<(u64, u32)>` (throttle interval, `useconds <= 999999`, count > 0
enforced); `Format: Display` (source form, braces re-escaped); enum constants are now
resolved separately from the enum's default (`ensure_enum_constants`) so a constant
defined as `E.A` may be `E`'s default. Reference quirks kept: parameter commands are
named `<PARAM>_PRM_SET`/`_PRM_SAVE` (upper case) and annotated with the parameter's
annotation; `Integer`-typed constants are typed `U64` (`I64` if negative); anonymous
array/struct constants are omitted; `bool` is reported as 8 bits; unsized strings use
`FW_FIXED_LENGTH_STRING_SIZE`.

CLI: `fpp-to-rust --dict DIR [-p VER] [-f VER] [-l LIB,...] -i ... FILE...`. The demo's
`build.rs` also writes `$OUT_DIR/DemoTopologyDictionary.json` (`tests/dictionary.rs`
checks it against the generated constants).

`crates/fprime-ref/fpp/{Ref,SignalGen}.fpp` model the Rust Ref's instances (C++ base
ids) and the Rust SignalGen (which is not the upstream component: commands 0..3 +
`Amplitude` param set/save 4/5, events 0..5, channels `SignalValue`/`SignalType`,
`SignalType: U8 { Sine, Triangle }`); `generate-dictionary.sh <fprime checkout>`
regenerates `crates/fprime-ref/dictionary/RefTopologyDictionary.json` from the upstream
model (every other Rust component's opcodes/ids match upstream; `systemResources`
declares fewer CPU channels and `fileManager` lacks `PathOutsideSandbox`, harmless
supersets). `tools/gds-crosscheck.py` is the live check (fprime-gds 4.3.1: `-n -g none
--framing-selection fprime`, GDS as TCP server on 50000, Ref with `-a 127.0.0.1 -p 50000`;
`fprime-cli events`/`channels` in text mode because `-j` crashes on enum arguments in
4.3.1; command string arguments are capped at `FW_CMD_STRING_MAX_SIZE` = 40 on both sides).
