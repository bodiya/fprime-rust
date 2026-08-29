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

CRATE fprime_fw. Everything below is re-exported at crate root (fprime_fw::Time etc.); modules: serial, string, time, enums, com, buffer, packets, poly_type, assert, logger. Also `pub use fprime_config as config` (fw_assert! expansion needs this path; depend on fprime-fw and it resolves).

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

string:
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
  struct FrameContext { pub com_queue_index:FwIndexType, pub apid:Apid, pub has_sec_hdr:bool, pub sequence_flags:u8, pub sequence_count:u16, pub vc_id:u8, pub pvn:Pvn, pub send_now:bool, pub sa_index:u16 } (Copy, Eq, Default per ComCfg.fpp, Serialize/Deserialize, const SERIALIZED_SIZE=13)

buffer:
  type BufferStorage = Box<[u8]>
  struct Buffer (Default=empty): const NO_CONTEXT:u32=0xFFFF_FFFF; fn empty()->Buffer; allocate(size:usize)->Buffer; from_storage(BufferStorage, context:u32)->Buffer; is_valid(&self)->bool; capacity/size/offset(&self)->usize; context(&self)->u32; set_context(&mut self,u32); advance(&mut self, amount:FwSignedSizeType) /*fw_asserts*/; set_size(&mut self, usize) /*fw_asserts*/; data(&self)->&[u8]; data_mut(&mut self)->&mut [u8]; get_serializer(&mut self)->ExtBuf<'_> /*empty, resetSer*/; get_deserializer(&mut self)->ExtBuf<'_> /*whole window readable*/; into_storage(self)->BufferStorage

packets:
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
4. assert_failure is not `-> !`: like C++, a registered hook whose do_assert returns lets execution continue past the failed fw_assert!. Default path (no hook) always panics with the formatted message.
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
CRATE fprime_comp — everything re-exported at root (modules: obj, port, msg, queued, active, glue, escrow).

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

COMPONENT PATTERN (normative; copy tests/example_component.rs): msg types start at 1; async input = adapter struct {comp: Arc<C>} impl Port trait — write_envelope_header + args + send_message(policy); sync/guarded input = impl trait on the component itself, handler locks the component's Mutex<State>; input factory: fn x_in(self:&Arc<Self>, port_num)->PortRef<dyn XPort>; async command envelope args = [opCode u32][cmdSeq u32][serialize_buffer(CmdArgBuffer)] and dispatch on op_code.wrapping_sub(id_base) with InvalidOpcode fallback, FormatError on deser failure or deserialize_size_left()!=0; topology order: construct -> set_id_base -> connect -> create_queue -> reg_commands -> start(&arc,..) -> exit -> join (EXIT is priority 0, so pending higher-priority traffic drains first — join is a deterministic sync point in tests).
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

Deviations/decisions (all documented in module headers): (1) FprimeRouter context table keys on a router-generated token stamped into the forwarded buffer's context word (original word restored on return) instead of the C++ data-pointer key — Rust owned Buffers have no observable pointer identity; observable behavior (table-full events, restoration, BufferContextNotFound+default on miss) is identical. (2) ComQueue storage: com queues persist fixed 514-byte [u16 len][bytes][pad] records and buffer queues persist 8-byte BufferEscrow tokens (C++ serializes ComBuffer/Fw::Buffer objects; the token occupies the same 8-byte slot as the C++ pointer); DROP_OLDEST pre-emptive pop still returns ownership before overwrite. (3) ComQueue send path builds an owned Buffer from recycled BufferStorage instead of aliasing m_dequeued_com_buffer; com-buffer returns on dataReturnIn recycle the storage (C++ drops them) so steady state is allocation-free after the first send. (4) C++ casts the leading packet descriptor to Apid unchecked (ComQueue) — Rust maps unknown descriptors to Apid::InvalidUninitialized; the deframer's mapping matches C++ exactly. (5) ComStub implements only the synchronous driver path; the async ports (drvAsyncSendOut/drvAsyncSendReturnIn) are not ported in phase 1 (no async byte-stream driver exists) — noted in com_stub.rs. (6) ByteStreamStatus + ByteStream port traits are defined publicly in com_stub.rs because fprime-svc may not depend on fprime-drv; fprime-drv defines its own identical traits and fprime-ref glues them with tiny shims. (7) Deframer keeps the exact C++ handler order: APID extraction (and a possible PayloadTooShort WARNING_LO) happens BEFORE the CRC check, so a short-payload bad-CRC frame emits both events; drops always use the ORIGINAL context. (8) FrameAccumulator::configure drops the MemAllocator params (CircularBuffer owns its ring); C++'s unchecked peek into a smaller-than-requested allocation (UB) is a clean fw_assert here. (9) All three ComQueue commands (FLUSH_QUEUE, FLUSH_ALL_QUEUES, SET_QUEUE_PRIORITY) are implemented — nothing deferred; invalid QueueType enum args answer ValidationError, deser failures/residual bytes answer FormatError per the task's discipline (C++ generated code folds enum validity into FormatError). (10) Event ids follow FPP declaration order (0,1,2,...) per the .fpp files; ComQueue telemetry ids 0/1 are explicit in the fppi. No sibling-module breakage observed: full workspace build and test are green (504 passed / 0 failed), fprime-svc full-crate green seen repeatedly.

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

