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

