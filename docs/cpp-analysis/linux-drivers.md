# Drv Linux hardware drivers: LinuxGpioDriver, LinuxUartDriver, LinuxI2cDriver, LinuxSpiDriver + Drv/Interfaces (Gpio, I2c, AsyncI2c, AsyncGuardedI2c, Spi, ByteStreamDriver) + Drv/Ports (GpioDriverPorts, I2cDriverPorts, SpiDriverPorts)

> Analysis of the C++ F Prime implementation (github.com/nasa/fprime) produced to guide this Rust port.
> File paths refer to the C++ tree.

## Overview

Four passive components wrapping Linux kernel userspace device APIs. GPIO uses the /dev/gpiochip* character-device uAPI (v2 preferred, v1 fallback) with a dedicated poll(2) interrupt thread; UART uses termios on /dev/tty* with a dedicated blocking-read thread and the ByteStreamDriver interface; I2C uses /dev/i2c-N (ioctl I2C_SLAVE + read/write, or ioctl I2C_RDWR for combined transfers); SPI uses /dev/spidevD.S (ioctl SPI_IOC_* config + SPI_IOC_MESSAGE(1) full-duplex transfer). All four ship a stub .cpp selected by CMake (FPRIME_USE_STUBBED_DRIVERS / non-Linux platform); GPIO is Linux-only, UART is restrict_platforms(Linux Darwin). Every device-touching operation except UART read/write and (half-duplex) spidev read/write requires ioctl(2), which safe zero-dependency std Rust cannot issue.

## Key items

### Drv.GpioStatus (enum, U8)

Files: `Drv/Ports/GpioDriverPorts.fpp`

OP_OK=0, NOT_OPENED=1, INVALID_MODE=2, UNKNOWN_ERROR=3. Implicit sequential values (no explicit assignments).

### Drv.GpioWrite / Drv.GpioRead ports

Files: `Drv/Ports/GpioDriverPorts.fpp`, `Drv/Interfaces/Gpio.fpp`

GpioWrite(state: Fw.Logic) -> GpioStatus. GpioRead(ref state: Fw.Logic) -> GpioStatus (out-param). Fw.Logic is U8 enum LOW=0, HIGH=1 (Fw/Types/Types.fpp:47). Interface Drv.Gpio = sync input gpioWrite, sync input gpioRead, output gpioInterrupt: Svc.Cycle. Svc.Cycle port = Cycle(ref cycleStart: Os.RawTime).

### Drv::LinuxGpioDriver component

Files: `Drv/LinuxGpioDriver/LinuxGpioDriver.fpp`, `Drv/LinuxGpioDriver/LinuxGpioDriver.hpp`, `Drv/LinuxGpioDriver/LinuxGpioDriver.cpp`, `Drv/LinuxGpioDriver/LinuxGpioDriverCommon.cpp`, `Drv/LinuxGpioDriver/LinuxGpioDriverStub.cpp`

passive, imports Gpio; special ports: Log, LogText, Time (NO Tlm, NO Cmd). One line per instance. Members: m_poller(Os::Task), m_lock(Os::Mutex), m_configuration(GpioConfiguration, init MAX_GPIO_CONFIGURATION=5), m_apiVersion(ApiVersion, init API_VERSION_UNSET=2), m_fd(int, init -1), m_running(bool, init false). const GPIO_POLL_TIMEOUT=500 (ms).
GpioConfiguration: GPIO_OUTPUT=0, GPIO_INPUT=1, GPIO_INTERRUPT_RISING_EDGE=2, GPIO_INTERRUPT_FALLING_EDGE=3, GPIO_INTERRUPT_BOTH_RISING_AND_FALLING_EDGES=4, MAX_GPIO_CONFIGURATION=5. ApiVersion: API_V2=0, API_V1=1, API_VERSION_UNSET=2.
open(device: const char*, gpio: U32, configuration, default_state=Fw::Logic::LOW) -> Os::File::Status. Steps: (1) FW_ASSERT(device!=nullptr); FW_ASSERT(0<=configuration<MAX). (2) Os::File::open(device, OPEN_WRITE); on failure log_WARNING_HI_OpenChipError(device, status) and return. (3) ioctl(GPIO_GET_CHIPINFO_IOCTL) -> gpiochip_info; failure -> errno_to_file_status, OpenChipError, return. (4) if gpio >= chip_info.lines: status=DOESNT_EXIST, log_WARNING_HI_OpenPinError(device, gpio, "Does Not Exist", status), return (SDD claims it returns OP_OK here — SDD is WRONG, code returns DOESNT_EXIST). (5) probe v2: ioctl(GPIO_V2_GET_LINEINFO_IOCTL, gpio_v2_line_info{offset=gpio}); rc==0 => supports_v2=true and pin_message = format("%s%s%s", info.name, has_consumer?" with current consumer ":"", consumer). Else ioctl(GPIO_GET_LINEINFO_IOCTL, gpioline_info{line_offset=gpio}) for the same message; pin_message default "Unknown". (6) if supports_v2 -> setupLineRequestV2 for ALL five configurations; else v1: GPIO_OUTPUT/GPIO_INPUT -> setupLineHandle (GPIO_GET_LINEHANDLE_IOCTL), the three interrupt modes -> setupLineEvent (GPIO_GET_LINEEVENT_IOCTL). (7) on error log_WARNING_HI_OpenPinError(device, gpio, pin_message, status); on success log_DIAGNOSTIC_OpenChip(chip_info.name, chip_info.label, gpio, pin_message) and commit m_fd/m_configuration/m_apiVersion. Consumer label = component object name (FW_OPTIONAL_NAME(getObjName())), truncated to 32 bytes.
gpioRead_handler: returns INVALID_MODE unless m_configuration==GPIO_INPUT (interrupt modes deliberately cannot be read). V2: ioctl(GPIO_V2_LINE_GET_VALUES_IOCTL, gpio_v2_line_values{mask=1}); state = (bits & 1) ? HIGH : LOW. V1: ioctl(GPIOHANDLE_GET_LINE_VALUES_IOCTL, gpiohandle_data); state = values[0] ? HIGH : LOW. Error -> errno_to_gpio_status(errno).
gpioWrite_handler: returns INVALID_MODE unless m_configuration==GPIO_OUTPUT. V2: gpio_v2_line_values{mask=1, bits=(state==HIGH)?1:0} + GPIO_V2_LINE_SET_VALUES_IOCTL. V1: gpiohandle_data.values[0]=(state==HIGH)?1:0 + GPIOHANDLE_SET_LINE_VALUES_IOCTL.
start(priority=TASK_PRIORITY_DEFAULT, stackSize=TASK_DEFAULT, cpuAffinity=TASK_DEFAULT, identifier=TASK_DEFAULT) -> GpioStatus: returns INVALID_MODE unless GPIO_INTERRUPT_RISING_EDGE <= m_configuration < MAX (i.e. 2,3,4). Sets m_running=true under m_lock, task name = format("%s.interrupt", objName) (may truncate), starts m_poller; task start failure -> UNKNOWN_ERROR. stop(): m_running=false under lock. join(): m_poller.join(). getRunning(): reads m_running under lock.
pollLoop(): while getRunning(): pollfd{fd=m_fd, events=POLLIN}; poll(fds,1,500). rc>0: expected_bytes = sizeof(gpioevent_data)=16 (v1) or sizeof(gpio_v2_line_event)=48 (v2); read(m_fd,&u,expected_bytes). If read_bytes==expected_bytes: Os::RawTime ts; ts.now(); on non-OP_OK log_WARNING_HI_InterruptTimeError(status) BUT STILL invoke gpioInterrupt_out(0, ts). Else log_WARNING_HI_InterruptReadError(expected_bytes, read_bytes) (read_bytes cast from ssize_t, so -1 becomes 0xFFFFFFFF). rc<0: log_WARNING_HI_PollingError(errno). Note the event payload (edge id, seqno, timestamp) is DISCARDED; only the local RawTime is forwarded. Interrupt is delivered on the poller thread — consumers should use an async CycleIn.
errno_to_file_status: 0->OP_OK, EBADF->NOT_OPENED, EINVAL->INVALID_ARGUMENT, ENODEV->DOESNT_EXIST, ENOMEM->NO_SPACE, EPERM->NO_PERMISSION, ENXIO->INVALID_MODE, everything else (EFAULT/EWOULDBLOCK/EBUSY/EIO/default)->OTHER_ERROR.
errno_to_gpio_status: EBADF->NOT_OPENED, ENXIO->INVALID_MODE, everything else->UNKNOWN_ERROR.
Destructor: close(m_fd) if >=0. Stub: open/setup* return Os::File::Status::NOT_SUPPORTED, both handlers return GpioStatus::UNKNOWN_ERROR, pollLoop delays 500ms per iteration.

### LinuxGpioDriver events (auto-assigned ids, declaration order)

Files: `Drv/LinuxGpioDriver/LinuxGpioDriver.fpp`

id 0 OpenChip(chip: string[80], chipLabel: string[80], pin: U32, pinMessage: string[80]) severity DIAGNOSTIC, fmt "Opened GPIO chip {}[{}] pin {}[{}]".
id 1 OpenChipError(chip: string[80], status: Os.FileStatus) severity WARNING_HI, fmt "Failed to open GPIO chip {}: {}".
id 2 OpenPinError(chip: string[80], pin: U32, pinMessage: string[80], status: Os.FileStatus) severity WARNING_HI, fmt "Failed to open GPIO chip {} pin {} [{}]: {}".
id 3 InterruptReadError(expected: U32, got: U32) severity WARNING_HI, fmt "Interrupt data read expected {} byes and got {}" (typo 'byes' is in upstream — preserve).
id 4 PollingError(error_number: I32) severity WARNING_HI, fmt "Interrupt polling returned errno: {}".
id 5 InterruptTimeError(status: Os.RawTimeStatus) severity WARNING_HI, fmt "Failed to read interrupt timestamp: {}".
No explicit `id` clauses, no throttles. Bare `string` in FPP defaults to size 80. Os.FileStatus (U8): OP_OK=0, DOESNT_EXIST=1, NO_SPACE=2, NO_PERMISSION=3, BAD_SIZE=4, NOT_OPENED=5, FILE_EXISTS=6, NOT_SUPPORTED=7, INVALID_MODE=8, INVALID_ARGUMENT=9, NO_MORE_RESOURCES=10, OTHER_ERROR=11, OUTSIDE_SANDBOX=12. Os.RawTimeStatus (U8): OP_OK=0, OP_OVERFLOW=1, INVALID_PARAMS=2, NOT_SUPPORTED=3, OTHER_ERROR=4.

### Drv::LinuxUartDriver component

Files: `Drv/LinuxUartDriver/LinuxUartDriver.fpp`, `Drv/LinuxUartDriver/LinuxUartDriver.hpp`, `Drv/LinuxUartDriver/LinuxUartDriver.cpp`

passive, imports ByteStreamDriver. Ports: output ready: Drv.ByteStreamReady; output recv: Drv.ByteStreamData; guarded input send: Drv.ByteStreamSend -> ByteStreamStatus; guarded input recvReturnIn: Fw.BufferSend; output allocate: Fw.BufferGet; output deallocate: Fw.BufferSend; sync input run: Svc.Sched; special ports Log, Tlm, LogText, Time.
Members: m_fd(int, -1), m_allocationSize(FwSizeType, 0), m_device(const char*, "NOT_EXIST" — stores the CALLER's pointer, not a copy), m_readTask, atomic<FwSizeType> m_bytesSent/m_bytesReceived, atomic<bool> m_quitReadThread.
UartBaudRate enum values are the literal baud numbers: 9600,19200,38400,57600,115200(BAUD_115K),230400(BAUD_230K), then conditionally 460800,921600,1000000,1152000,1500000,2000000,2500000,3000000,3500000,4000000. UartFlowControl: NO_FLOW=0, HW_FLOW=1. UartParity: PARITY_NONE=0, PARITY_ODD=1, PARITY_EVEN=2.
open(device, baud, fc, parity, allocationSize) -> bool. (1) ::open(device, O_RDWR|O_NOCTTY); fd==-1 -> log_WARNING_HI_OpenError(device, fd, strerror(errno)) and false. (2) tcgetattr; set c_cc[VMIN]=0, c_cc[VTIME]=10 (1 s no-data timeout); tcsetattr(TCSANOW). (3) if HW_FLOW: fresh tcgetattr, t.c_cflag |= CRTSCTS, tcsetattr(TCSANOW). (4) map baud enum -> Bxxxxx speed_t. (5) fresh tcgetattr into newtio; newtio.c_cflag |= CS8|CLOCAL|CREAD; PARITY_ODD -> |= PARENB|PARODD; PARITY_EVEN -> |= PARENB; PARITY_NONE -> &= ~PARENB. (6) cfsetispeed + cfsetospeed. (7) newtio.c_oflag=0; newtio.c_lflag=0; newtio.c_iflag=INPCK. (8) tcflush(fd, TCIFLUSH). (9) tcsetattr(TCSANOW). Every failure closes fd and emits OpenError. On success: m_fd=fd, log_ACTIVITY_HI_PortOpened(device), and if isConnected_ready_OutputPort(0) then ready_out(0).
run_handler(portNum, context): tlmWrite_BytesSent(m_bytesSent); tlmWrite_BytesRecv(m_bytesReceived). Unconditional (not on-change).
send_handler(portNum, serBuffer) -> ByteStreamStatus: if m_fd==-1 || data==nullptr || size==0 -> OTHER_ERROR (no event). Else ::write(m_fd, data, size); if rc==-1 or rc!=size -> log_WARNING_HI_WriteError(m_device, (I32)rc) and OTHER_ERROR; else m_bytesSent += rc and OP_OK. Buffer ownership stays with caller.
recvReturnIn_handler: deallocate_out(0, fwBuffer) unconditionally.
serialReadTaskEntry: while(!m_quitReadThread): buff = allocate_out(0, m_allocationSize). If buff.getData()==nullptr -> log_WARNING_HI_NoBuffers(m_device), recv_out(0, buff, OTHER_ERROR), Os::Task::delay(0 s / 50000 us), continue. Else inner loop `while (stat==0 && !m_quitReadThread) stat = read(m_fd, buff.getData(), buff.getSize())` (spins across VTIME timeouts). Then buff.setSize(0). stat==-1 -> log_WARNING_HI_ReadError(m_device, stat) + OTHER_ERROR; stat>0 -> buff.setSize(stat), OP_OK, m_bytesReceived += stat; stat==0 (quit requested) -> OTHER_ERROR just to return the buffer. Always recv_out(0, buff, status).
start(priority, stackSize, cpuAffinity): task name literal "SerReader"; FW_ASSERT(taskStatus==OP_OK). quitReadThread(): m_quitReadThread=true. join(): m_readTask.join(). Destructor closes m_fd if != -1.

### LinuxUartDriver events & telemetry (explicit ids)

Files: `Drv/LinuxUartDriver/Events.fppi`, `Drv/LinuxUartDriver/Telemetry.fppi`

EVENTS: id 0 OpenError(device: string size 40, error: I32, name: string size 40) WARNING_HI "Error opening UART device {}: {} {}". id 1 ConfigError(device: string size 40, error: I32) WARNING_HI "Error configuring UART device {}: {}" — DECLARED BUT NEVER EMITTED by the .cpp. id 2 WriteError(device: string size 40, error: I32) WARNING_HI throttle 5 "Error writing UART device {}: {}". id 3 ReadError(device: string size 40, error: I32) WARNING_HI throttle 5 "Error reading UART device {}: {}". id 4 PortOpened(device: string size 40) ACTIVITY_HI "UART Device {} configured". id 5 NoBuffers(device: string size 40) WARNING_HI throttle 20 "UART Device {} ran out of buffers". id 6 BufferTooSmall(device: string size 40, size: U32, needed: U32) WARNING_HI "UART Device {} target buffer too small. Size: {} Needs: {}" — DECLARED BUT NEVER EMITTED.
TELEMETRY: id 0 BytesSent: FwSizeType (u64). id 1 BytesRecv: FwSizeType (u64).

### Drv.ByteStreamStatus + ByteStream ports

Files: `Drv/ByteStreamDriverModel/ByteStreamDriverModel.fpp`, `Drv/Interfaces/ByteStreamDriver.fpp`

enum ByteStreamStatus : U8 { OP_OK=0, SEND_RETRY=1, RECV_NO_DATA=2, OTHER_ERROR=3 }. port ByteStreamData(ref buffer: Fw.Buffer, status: ByteStreamStatus) — no return. port ByteStreamSend(ref sendBuffer: Fw.Buffer) -> ByteStreamStatus. port ByteStreamReady() — no args, no return. Already ported in fprime-rust as fprime_drv::byte_stream (same discriminants) — reuse those traits verbatim for the UART port.

### Drv::LinuxI2cDriver component

Files: `Drv/LinuxI2cDriver/LinuxI2cDriver.fpp`, `Drv/LinuxI2cDriver/LinuxI2cDriver.hpp`, `Drv/LinuxI2cDriver/LinuxI2cDriver.cpp`, `Drv/LinuxI2cDriver/LinuxI2cDriverStub.cpp`

passive, imports Drv.I2c ONLY. No Log/Tlm/Time ports at all -> NO events, NO telemetry, NO commands. Single member: int m_fd = -1 (compiled out under STUBBED_LINUX_I2C_DRIVER).
Ports (all guarded input): write: Drv.I2c(addr: U32, ref serBuffer: Fw.Buffer) -> I2cStatus; read: same signature; writeRead: Drv.I2cWriteRead(addr: U32, ref writeBuffer: Fw.Buffer, ref readBuffer: Fw.Buffer) -> I2cStatus (readBuffer size must be preset by caller).
open(device: const char*) -> bool: FW_ASSERT(device!=nullptr); m_fd = ::open(device, O_RDWR); return m_fd != -1. Typical device "/dev/i2c-1".
write_handler: m_fd==-1 -> I2C_OPEN_ERR. ioctl(m_fd, I2C_SLAVE=0x0703, addr) == -1 -> I2C_ADDRESS_ERR. FW_ASSERT(data!=nullptr). write(m_fd, data, size); rc==-1 or rc!=size -> I2C_WRITE_ERR. Else I2C_OK.
read_handler: identical, but read(); short/failed read -> I2C_READ_ERR.
writeRead_handler: m_fd==-1 -> I2C_OPEN_ERR. FW_ASSERTs both data pointers non-null; FW_ASSERT_NO_OVERFLOW(addr,U16), (writeBuffer.size,U16), (readBuffer.size,U16). Builds struct i2c_msg rdwr_msgs[2]: [0] = {addr=(U16)addr, flags=0 (write), len=writeBuffer.size, buf=writeBuffer.data}; [1] = {addr=(U16)addr, flags=I2C_M_RD=0x0001, len=readBuffer.size, buf=readBuffer.data}. i2c_rdwr_ioctl_data{msgs=rdwr_msgs, nmsgs=2}; ioctl(m_fd, I2C_RDWR=0x0707, &rdwr_data). rc==-1 -> I2C_OTHER_ERR (no finer classification possible), else I2C_OK. Combined transfer = repeated START, no STOP between write and read.
Addresses are 7-bit (no I2C_TENBIT handling). Buffers are caller-owned; driver never deallocates.
Stub: open() returns TRUE and all three handlers return I2C_OK (the SDD claims the stub 'reports open failures' — SDD is WRONG).

### Drv.I2cStatus and async I2C ports (not implemented by LinuxI2cDriver)

Files: `Drv/Ports/I2cDriverPorts.fpp`, `Drv/Interfaces/AsyncI2c.fpp`, `Drv/Interfaces/AsyncGuardedI2c.fpp`, `default/config/AsyncI2cCfg.fpp`

enum I2cStatus : U8 { I2C_OK=0, I2C_ADDRESS_ERR=1, I2C_WRITE_ERR=2, I2C_READ_ERR=3, I2C_OPEN_ERR=4, I2C_OTHER_ERR=5 } (explicit).
Async port types (no driver in-tree implements them; port defs only): I2cRequest(addr: U32, ref buffer: Fw.Buffer) no return; I2cWriteReadRequest(addr: U32, ref writeBuffer, ref readBuffer) no return; I2cCallback(ref buffer: Fw.Buffer, status: Drv.I2cStatus); I2cWriteReadCallback(ref writeBuffer, ref readBuffer, status: Drv.I2cStatus). Interfaces AsyncI2c (async input write/read/writeRead) and AsyncGuardedI2c (guarded input, same names), both arrays of Drv.AsyncI2cCfg.I2cDriverPorts = 10, plus output writeComplete/readComplete (I2cCallback) and writeReadComplete (I2cWriteReadCallback), also size 10.

### Drv::LinuxSpiDriverComponentImpl

Files: `Drv/LinuxSpiDriver/LinuxSpiDriver.fpp`, `Drv/LinuxSpiDriver/LinuxSpiDriverComponentImpl.hpp`, `Drv/LinuxSpiDriver/LinuxSpiDriverComponentImpl.cpp`, `Drv/LinuxSpiDriver/LinuxSpiDriverComponentImplCommon.cpp`, `Drv/LinuxSpiDriver/LinuxSpiDriverComponentImplStub.cpp`

passive, imports Drv.Spi; special ports Log, Tlm, LogText, Time. Members: m_fd(-1), m_device(FwIndexType, -1), m_select(FwIndexType, -1), m_bytes(FwSizeType, 0).
Ports: guarded input SpiWriteRead: Drv.SpiWriteRead(ref writeBuffer: Fw.Buffer, ref readBuffer: Fw.Buffer) -> SpiStatus; sync input SpiReadWrite: Drv.SpiReadWrite(same args, NO return) — DEPRECATED, forwards to SpiWriteRead_handler and discards the status.
enum SpiStatus : U8 { SPI_OK=0, SPI_OPEN_ERR=1, SPI_CONFIG_ERR=2, SPI_MISMATCH_ERR=3, SPI_WRITE_ERR=4, SPI_OTHER_ERR=5 } (explicit). NOTE: only SPI_OPEN_ERR, SPI_OTHER_ERR and SPI_OK are ever returned; SPI_CONFIG_ERR/SPI_MISMATCH_ERR/SPI_WRITE_ERR are dead values.
enum SpiFrequency (C++ only, not FPP): SPI_FREQUENCY_1MHZ=1000000, 5MHZ=5000000, 10MHZ=10000000, 15MHZ=15000000, 20MHZ=20000000. enum SpiMode: SPI_MODE_CPOL_LOW_CPHA_LOW=0, SPI_MODE_CPOL_LOW_CPHA_HIGH=1, SPI_MODE_CPOL_HIGH_CPHA_LOW=2, SPI_MODE_CPOL_HIGH_CPHA_HIGH=3 -> kernel SPI_MODE_0..3 (0, SPI_CPHA=0x1, SPI_CPOL=0x2, 0x3).
open(device: FwIndexType, select: FwIndexType, clock: SpiFrequency, spiMode = SPI_MODE_CPOL_LOW_CPHA_LOW) -> bool: FW_ASSERT(device>=0), FW_ASSERT(select>=0). Path formatted as "/dev/spidev%d.%d" into Fw::FileNameString; FW_ASSERT(format success). ::open(path, O_RDWR); fd==-1 -> log_WARNING_HI_SPI_OpenError(device, select, fd /* passes fd, i.e. -1, NOT errno */) and false. Then, each failing ioctl -> log_WARNING_HI_SPI_ConfigError(device, select, ret) + close(fd) + return false: ioctl(SPI_IOC_WR_MODE, &U8 mode); ioctl(SPI_IOC_RD_MODE, &U8 read_mode) then if mismatch log_WARNING_LO_SPI_ConfigMismatch(device, select, "MODE", mode, read_mode); ioctl(SPI_IOC_WR_BITS_PER_WORD, &U8 bits=8); ioctl(SPI_IOC_RD_BITS_PER_WORD, &U8 read_bits) + mismatch event with "BITS_PER_WORD"; ioctl(SPI_IOC_WR_MAX_SPEED_HZ, &clock); ioctl(SPI_IOC_RD_MAX_SPEED_HZ, &read_clock) + mismatch event with "MAX_SPEED_HZ". m_fd = fd only after full success. NOTE: SPI_PortOpened (event id 4) is NEVER emitted.
SpiWriteRead_handler: FW_ASSERT(portNum>=0), FW_ASSERT(writeBuffer.isValid()), FW_ASSERT(readBuffer.isValid()), FW_ASSERT(writeBuffer.getSize()==readBuffer.getSize()). m_fd==-1 -> SPI_OPEN_ERR (no event). Builds zeroed spi_ioc_transfer{tx_buf=(u64)writeBuffer.data, rx_buf=(u64)readBuffer.data, len=(u32)writeBuffer.getSize(), everything else 0 -> device defaults for speed/bits/delay}. ioctl(m_fd, SPI_IOC_MESSAGE(1), &tr). rc<1 -> log_WARNING_HI_SPI_WriteError(m_device, m_select, rc) and SPI_OTHER_ERR. Else m_bytes += readBuffer.getSize(); tlmWrite_SPI_Bytes(m_bytes); return SPI_OK.
SpiReadWrite_handler: same asserts (portNum>=0, both buffers valid), then (void)SpiWriteRead_handler(...).
Destructor: close(m_fd) unconditionally (closes -1 when never opened — harmless but note). Stub: open() returns false, SpiWriteRead_handler returns SPI_OK, SpiReadWrite_handler is empty, destructor empty.

### LinuxSpiDriver events & telemetry (explicit ids)

Files: `Drv/LinuxSpiDriver/Events.fppi`, `Drv/LinuxSpiDriver/Telemetry.fppi`

EVENTS: id 0 SPI_OpenError(device: I32, select: I32, error: I32) WARNING_HI "Error opening SPI device {}.{}: {}". id 1 SPI_ConfigError(device: I32, select: I32, error: I32) WARNING_HI "Error configuring SPI device {}.{}: {}". id 2 SPI_WriteError(device: I32, select: I32, error: I32) WARNING_HI throttle 5 "Error writing/reading SPI device {}.{}: {}". id 3 SPI_ConfigMismatch(device: I32, select: I32, parameter: string[80], write_value: U32, read_value: U32) WARNING_LO "SPI device {}.{} configuration mismatch for {}: wrote {}, read {}". id 4 SPI_PortOpened(device: I32, select: I32) ACTIVITY_HI "SPI Device {}.{} configured" — declared, NEVER emitted.
TELEMETRY: id 0 SPI_Bytes: FwSizeType (u64), cumulative bytes, written on every successful transfer.

## Wire formats

### struct gpiochip_info (GPIO_GET_CHIPINFO_IOCTL, _IOR(0xB4,0x01))

char name[32]; char label[32]; __u32 lines; — 68 bytes (padded to 72). name/label are NUL-terminated C strings fed to the OpenChip event.

### struct gpiohandle_request (GPIO_GET_LINEHANDLE_IOCTL, _IOWR(0xB4,0x03))

__u32 lineoffsets[64]; __u32 flags; __u8 default_values[64]; char consumer_label[32]; __u32 lines; int fd; — driver sets lineoffsets[0]=gpio, lines=1, default_values[0]=(HIGH?1:0), consumer_label=objName, flags=GPIOHANDLE_REQUEST_OUTPUT(1<<1=0x02) for GPIO_OUTPUT else GPIOHANDLE_REQUEST_INPUT(1<<0=0x01); fd is returned by the kernel.

### struct gpiohandle_data (GPIOHANDLE_GET/SET_LINE_VALUES_IOCTL, _IOWR(0xB4,0x08/0x09))

__u8 values[64] — only values[0] is used (1 = HIGH).

### struct gpioevent_request (GPIO_GET_LINEEVENT_IOCTL, _IOWR(0xB4,0x04))

__u32 lineoffset; __u32 handleflags; __u32 eventflags; char consumer_label[32]; int fd; — handleflags = GPIOHANDLE_REQUEST_INPUT(0x01); eventflags = GPIOEVENT_REQUEST_RISING_EDGE(1<<0=0x01) / FALLING(1<<1=0x02) / both(0x03).

### struct gpioevent_data (v1 read payload)

__u64 timestamp; __u32 id; — sizeof == 16 (u64 alignment padding). id: GPIOEVENT_EVENT_RISING_EDGE=0x01, GPIOEVENT_EVENT_FALLING_EDGE=0x02. F Prime reads exactly 16 bytes and discards both fields.

### struct gpio_v2_line_event (v2 read payload)

__aligned_u64 timestamp_ns; __u32 id; __u32 offset; __u32 seqno; __u32 line_seqno; __u32 padding[6]; — sizeof == 48. id: GPIO_V2_LINE_EVENT_RISING_EDGE=1, FALLING_EDGE=2. Also discarded.

### struct gpio_v2_line_values (GPIO_V2_LINE_GET/SET_VALUES_IOCTL, _IOWR(0xB4,0x0E/0x0F))

__aligned_u64 bits; __aligned_u64 mask; — 16 bytes. Driver always uses mask=1 and bit 0.

### struct gpio_v2_line_request (GPIO_V2_GET_LINE_IOCTL, _IOWR(0xB4,0x07))

__u32 offsets[64]; char consumer[32]; struct gpio_v2_line_config config; __u32 num_lines; __u32 event_buffer_size; __u32 padding[5]; __s32 fd. config = { __u64 flags; __u32 num_attrs; __u32 padding[5]; struct gpio_v2_line_config_attribute attrs[10] } where each attr = { struct gpio_v2_line_attribute attr {__u32 id; __u32 padding; union{__u64 flags; __u64 values; __u32 debounce_period_us}}; __u64 mask }. Driver sets offsets[0]=gpio, num_lines=1, fd=-1, consumer=objName, config.flags from GPIO_V2_LINE_FLAG_INPUT=1<<2(0x04) / OUTPUT=1<<3(0x08) / EDGE_RISING=1<<4(0x10) / EDGE_FALLING=1<<5(0x20); for GPIO_OUTPUT also num_attrs=1, attrs[0].attr.id=GPIO_V2_LINE_ATTR_ID_OUTPUT_VALUES=2, attrs[0].attr.values=(HIGH?1:0), attrs[0].mask=1.

### struct i2c_msg / i2c_rdwr_ioctl_data (I2C_RDWR = 0x0707)

i2c_msg { __u16 addr; __u16 flags; __u16 len; __u8 *buf; }. flags: 0 = write, I2C_M_RD = 0x0001 = read. i2c_rdwr_ioctl_data { struct i2c_msg *msgs; __u32 nmsgs; } with nmsgs=2. I2C_SLAVE = 0x0703 (address passed as the ioctl arg value, not a pointer).

### struct spi_ioc_transfer (SPI_IOC_MESSAGE(1) = _IOW('k', 0, char[32]))

__u64 tx_buf; __u64 rx_buf; __u32 len; __u32 speed_hz; __u16 delay_usecs; __u8 bits_per_word; __u8 cs_change; __u8 tx_nbits; __u8 rx_nbits; __u8 word_delay_usecs; __u8 pad; — 32 bytes, identical in 32- and 64-bit userspace. F Prime zeroes everything except tx_buf/rx_buf/len. Config ioctls: SPI_IOC_WR_MODE=_IOW('k',1,__u8), SPI_IOC_RD_MODE=_IOR('k',1,__u8), SPI_IOC_WR_BITS_PER_WORD=_IOW('k',3,__u8), SPI_IOC_RD_BITS_PER_WORD=_IOR('k',3,__u8), SPI_IOC_WR_MAX_SPEED_HZ=_IOW('k',4,__u32), SPI_IOC_RD_MAX_SPEED_HZ=_IOR('k',4,__u32).

### termios settings applied by LinuxUartDriver

c_cc[VMIN]=0, c_cc[VTIME]=10 (1.0 s inter-read timeout, so read() returns 0 on idle); optional c_cflag |= CRTSCTS for HW_FLOW; c_cflag |= CS8|CLOCAL|CREAD; parity via PARENB/PARODD; cfsetispeed/cfsetospeed to the Bxxxxx constant; c_oflag=0; c_lflag=0 (raw, non-canonical, no echo); c_iflag=INPCK only; tcflush(TCIFLUSH) before the final tcsetattr(TCSANOW). Note c_cflag/c_iflag are OR-ed onto whatever the port already had except where explicitly cleared, so the driver relies on the kernel's default line settings for stop bits (1) and flow control off.

## Threading / concurrency

GPIO: one dedicated Os::Task per interrupt-configured instance, named \"<objName>.interrupt\", body = pollLoop(). m_running is a plain bool guarded by Os::Mutex m_lock (start sets true, stop sets false, poll loop reads via getRunning()). poll() timeout 500 ms bounds shutdown latency, so stop() then join() terminates within one timeout. gpioInterrupt_out is invoked ON the poller thread — the receiving component must be async (e.g. Svc::ActiveRateGroup::CycleIn) or thread-safe. gpioRead/gpioWrite are sync input ports executing on the caller's thread with NO locking against the poll thread (safe in practice only because read/write and interrupt modes are mutually exclusive).\nUART: one dedicated Os::Task named \"SerReader\" running serialReadTaskEntry. m_quitReadThread, m_bytesSent, m_bytesReceived are std::atomic (bytes counters are read unsynchronized from the rate-group thread in run_handler). send is a guarded input port (component mutex) and recvReturnIn is guarded; recv_out fires from the read thread. Inner read loop spins on stat==0 (VTIME timeouts) and re-checks the quit flag each iteration. Buffer allocation happens on the read thread via allocate_out.\nI2C: no threads. All three ports are guarded -> serialized by the component mutex, executing on the caller's thread. The I2C_SLAVE address selection and the following read/write must be atomic, which is exactly why the ports are guarded.\nSPI: no threads. SpiWriteRead is guarded (mutex-serialized); SpiReadWrite is sync (NOT guarded) but simply calls the guarded handler's implementation function directly, so a caller invoking SpiReadWrite bypasses the mutex — a real race if both ports are used concurrently.

## Porting notes

Crate placement: extend fprime-drv (it already holds byte_stream.rs with the exact ByteStreamStatus discriminants and the TcpClient/TcpServer PassiveBase + OutputPort wiring pattern). New modules: gpio.rs (GpioStatus, GpioWritePort/GpioReadPort traits), i2c.rs (I2cStatus, I2cPort/I2cWriteReadPort traits + async request/callback traits), spi.rs (SpiStatus, SpiWriteReadPort/SpiReadWritePort traits), linux_gpio_driver.rs, linux_uart_driver.rs, linux_i2c_driver.rs, linux_spi_driver.rs.\nPort traits (object-safe, follow the fprime_drv::byte_stream shape):\n  trait GpioWritePort: Send+Sync { fn invoke(&self, port_num: FwIndexType, state: Logic) -> GpioStatus }\n  trait GpioReadPort: Send+Sync { fn invoke(&self, port_num: FwIndexType, state: &mut Logic) -> GpioStatus }\n  trait I2cPort: Send+Sync { fn invoke(&self, port_num, addr: u32, ser_buffer: &mut Buffer) -> I2cStatus }\n  trait I2cWriteReadPort: Send+Sync { fn invoke(&self, port_num, addr: u32, write_buffer: &mut Buffer, read_buffer: &mut Buffer) -> I2cStatus }\n  trait SpiWriteReadPort: Send+Sync { fn invoke(&self, port_num, write_buffer: &mut Buffer, read_buffer: &mut Buffer) -> SpiStatus }\n  trait SpiReadWritePort: Send+Sync { fn invoke(&self, port_num, write_buffer: &mut Buffer, read_buffer: &mut Buffer) }  // deprecated\nAdd Fw::Logic as an fpp_enum! (u8, Low=0, High=1) in fprime-fw types if not already present; reuse fprime_os::RawTime for gpioInterrupt (fprime-comp already defines CyclePort::invoke(&self, port_num, cycle_start: &RawTime)).\nUART is the only one that should be functionally complete: build it exactly like TcpClient — pub base: PassiveBase, pub evt: EventGlue, pub tlm: TlmGlue, pub allocate_out: OutputPort<dyn BufferGetPort>, pub deallocate_out: OutputPort<dyn BufferSendPort>, pub recv_out: OutputPort<dyn ByteStreamDataPort>, pub ready_out: OutputPort<dyn ByteStreamReadyPort>; input factories send_in (guarded, returns ByteStreamStatus), recv_return_in (guarded, forwards to deallocate_out), run_in (sync SchedPort). Store the device path as an owned FileNameString/String (C++ stores a borrowed const char* — fix that, it is a latent dangling pointer). Use std::fs::OpenOptions::new().read(true).write(true).open(path) for /dev/tty*, spawn the read thread with fprime_os::Task named \"SerReader\", keep m_bytes_sent/m_bytes_received as AtomicU64 and m_quit_read_thread as AtomicBool, and reproduce the exact read-loop shape (allocate -> spin while rc==0 && !quit -> set_size(0) -> classify -> recv_out with the possibly-zero-sized buffer). Emit the exact event ids/severities/throttles from Events.fppi (throttle 5 on WriteError/ReadError, 20 on NoBuffers) and telemetry ids 0/1 as FwSizeType. Keep ConfigError(id 1) and BufferTooSmall(id 6) defined-but-unemitted for id-space parity.\nGPIO: implement the component surface (ports, GpioStatus, all six events with the exact ids/severities, the mode gating, start/stop/join with a poll thread and a Mutex-guarded running flag, GPIO_POLL_TIMEOUT=500 ms) against a swappable backend trait, then supply a sysfs backend (see rust_feasibility) and a Stub backend mirroring LinuxGpioDriverStub (open -> FileStatus::NotSupported, handlers -> GpioStatus::UnknownError, poll loop just sleeps 500 ms). Preserve errno_to_file_status / errno_to_gpio_status as functions mapping std::io::ErrorKind + raw_os_error() (io::Error::raw_os_error() gives the real errno in std, so the tables port exactly: 9->NotOpened, 22->InvalidArgument, 19->DoesntExist, 12->NoSpace, 1->NoPermission, 6->InvalidMode).\nI2C and SPI: port the component shells, status enums, all events/telemetry ids, the mode/validity assertions (FW_ASSERT equivalents via fw_assert!), and ship the stub behavior as the only backend — matching the upstream stub .cpp exactly (I2C stub: open()->true, all handlers -> I2cStatus::I2cOk; SPI stub: open()->false, SpiWriteRead->SpiStatus::SpiOk, SpiReadWrite->no-op). That keeps a topology compilable and testable without pretending to drive hardware.\nEverywhere: preserve the exact status discriminants (GpioStatus 0..3 implicit, I2cStatus 0..5 explicit, SpiStatus 0..5 explicit, ByteStreamStatus 0..3 implicit) at u8 width with strict decode, matching the existing fprime-rust enum convention.

## Rust feasibility (safe, zero-dependency std)

IMPOSSIBLE in safe, zero-dependency std Rust (all require ioctl(2), which std does not expose and which cannot be issued without libc/nix or `unsafe extern`):\n- Every GPIO character-device operation: GPIO_GET_CHIPINFO_IOCTL, GPIO_GET_LINEINFO_IOCTL, GPIO_V2_GET_LINEINFO_IOCTL, GPIO_GET_LINEHANDLE_IOCTL, GPIO_GET_LINEEVENT_IOCTL, GPIOHANDLE_GET/SET_LINE_VALUES_IOCTL, GPIO_V2_GET_LINE_IOCTL, GPIO_V2_LINE_GET/SET_VALUES_IOCTL. Also poll(2) itself is unavailable in std (no poll/select/epoll wrapper), so the edge-triggered blocking wait cannot be reproduced even if the fd existed.\n- All UART configuration: tcgetattr/tcsetattr are TCGETS/TCSETS ioctls; cfsetispeed/cfsetospeed mutate a struct destined for that ioctl; tcflush is TCFLSH. So baud rate, parity, CS8/CLOCAL/CREAD, CRTSCTS, VMIN/VTIME and the input flush are ALL unreachable. O_NOCTTY cannot be requested either (std's OpenOptions has no such flag without OpenOptionsExt/libc).\n- All I2C transactions: I2C_SLAVE (0x0703) is mandatory before any read()/write() on /dev/i2c-N, and I2C_RDWR (0x0707) is the only way to do the repeated-start combined transfer. There is no file-only substitute — plain read/write on a freshly opened /dev/i2c-N targets no slave and fails with EINVAL. LinuxI2cDriver is therefore 100% unportable to safe zero-dep Rust.\n- SPI full duplex (SPI_IOC_MESSAGE(1)) and all SPI configuration (SPI_IOC_WR/RD_MODE, WR/RD_BITS_PER_WORD, WR/RD_MAX_SPEED_HZ).\nACHIEVABLE with plain std file I/O:\n- Legacy sysfs GPIO (/sys/class/gpio, deprecated but present when CONFIG_GPIO_SYSFS=y): write the line number to `export`/`unexport`; write \"in\"|\"out\"|\"low\"|\"high\" to `gpio<N>/direction`; read/write \"0\"/\"1\" to `gpio<N>/value` (reopen or seek(0) before each read — the value file does not auto-rewind); write \"none\"|\"rising\"|\"falling\"|\"both\" to `gpio<N>/edge`. This covers GPIO_INPUT and GPIO_OUTPUT (with default_state via direction \"low\"/\"high\") faithfully.\n- UART data transfer on an ALREADY-CONFIGURED /dev/tty*: File::write (send_handler) and File::read (read thread) work exactly as the C++ ::write/::read do, including the blocking semantics inherited from whatever VMIN/VTIME the port already carries.\n- SPI HALF-duplex only: spidev's file_operations implement plain write() (TX-only, RX discarded) and read() (RX-only, TX zeros). So a write-then-read sequence is possible with std::fs; true simultaneous full-duplex (which SpiWriteRead promises, and which many devices require) is not.\nRECOMMENDED SAFE ZERO-DEP PORT:\n1. LinuxUartDriver — implement fully, minus configuration. `open()` opens the device read/write and returns true/false; add a documented precondition that the port must already be configured externally (`stty -F /dev/ttyUSB0 115200 raw -echo -crtscts min 0 time 10`). Keep the UartBaudRate/UartFlowControl/UartParity enums in the API for parity, but have `open()` emit ConfigError (id 1 — finally putting the declared-unused event to work) or return false when a non-default configuration is requested and cannot be applied; alternatively offer an opt-in `configure_via_stty()` helper that shells out with std::process::Command (still safe and zero-dep) and document it as a divergence. Document that O_NOCTTY cannot be set, so the process may acquire the tty as its controlling terminal if it has none.\n2. LinuxGpioDriver — implement the full component surface with a `GpioBackend` trait and a **sysfs backend** supporting GPIO_OUTPUT and GPIO_INPUT exactly. For the three interrupt modes, sysfs edge detection needs poll(2)/POLLPRI, which is unavailable: implement a *level-sampling* poller instead — the thread re-reads `gpio<N>/value` every GPIO_POLL_TIMEOUT/N ms, and emits gpioInterrupt_out(0, RawTime::now()) on each configured transition — and document loudly that this is edge *sampling*, not kernel edge interrupts (missed fast pulses, latency bounded by the sample period, no hardware timestamp). Keep the chip-device API (`open(\"/dev/gpiochip0\", line, ...)`) in the signature and translate `/dev/gpiochipN` + line -> a sysfs global number only when the base is discoverable via `/sys/class/gpio/gpiochipM/base`+`ngpio` (plain file reads); otherwise return FileStatus::NotSupported. Ship the Stub backend as the default on non-Linux.\n3. LinuxI2cDriver — port the component shell, ports, and I2cStatus, backed only by the upstream stub semantics. Document as UNSUPPORTED: \"requires ioctl(I2C_SLAVE)/ioctl(I2C_RDWR); a real backend needs an unsafe FFI shim or a libc/nix dependency, both excluded by the zero-dep + forbid(unsafe_code) rules.\"\n4. LinuxSpiDriver — port the component shell, ports, SpiStatus, all five events and SPI_Bytes telemetry. Either (a) stub-only, or (b) offer an explicitly-named `half_duplex` backend using File::write + File::read on /dev/spidevD.S, returning SpiStatus::SpiOtherErr and documenting that mode/speed/bits-per-word come from the device tree and cannot be set or read back (so SPI_ConfigError/SPI_ConfigMismatch are unreachable, and SPI_OK from a half-duplex sequence is NOT equivalent to a full-duplex transfer). Prefer (a) as the default and gate (b) behind an opt-in constructor so no one silently gets half-duplex where full-duplex is required.

## Gotchas

- LinuxSpiDriver NEVER emits SPI_PortOpened (event id 4) even though it is declared and documented in the SDD — reserve id 4 but do not emit it.
- LinuxUartDriver NEVER emits ConfigError (id 1) or BufferTooSmall (id 6); all open/configure failures use OpenError (id 0). Preserve the id space.
- LinuxUartDriver's OpenError third argument is populated with `fd`, not errno: on the very first failure fd is -1, but on every subsequent tcgetattr/tcsetattr/cfset* failure it passes the VALID file descriptor as the 'error code'. The strerror(errno) string in the third arg is the only real diagnostic. Reproduce the field meanings exactly (or document the divergence if you choose to pass the real errno).
- LinuxSpiDriver's SPI_OpenError also passes `fd` (always -1) as the error code, not errno.
- LinuxUartDriver stores `m_device` as the caller's `const char*` with no copy — the topology must keep the string literal alive. The Rust port should own the string.
- LinuxUartDriver's read loop spins `while (stat==0 && !quit)` — with VTIME=10 that is a 1 Hz busy retry, not a tight spin, but if VMIN/VTIME are not applied (as in a safe-Rust port on a raw-configured tty) it can become a hot spin. Bound it with a small sleep in the port and document it.
- On the UART read-error path the buffer is delivered via recv_out with size set to 0 and status OTHER_ERROR — the receiver must return it. On buffer-allocation failure an INVALID (null) buffer is delivered with OTHER_ERROR and the thread sleeps 50 ms.
- LinuxGpioDriver discards the kernel event payload entirely (edge id, seqno, hardware timestamp_ns) and timestamps the interrupt with a fresh Os::RawTime::now() taken after the read. Both edges of a BOTH_EDGES configuration are therefore indistinguishable downstream.
- On an Os::RawTime::now() failure the GPIO poller logs InterruptTimeError and STILL invokes gpioInterrupt_out with the (possibly zero/invalid) timestamp. Do not skip the invocation.
- GPIO InterruptReadError casts a ssize_t read() result of -1 through FwSizeType, so a hard read error reports got=0xFFFFFFFF (after the U32 cast) rather than -1.
- GPIO interrupt modes deliberately reject gpioRead — a line that must be both polled and interrupt-driven requires two component instances. gpioWrite is rejected in every mode but GPIO_OUTPUT.
- GPIO start() only accepts configurations 2,3,4 (the interrupt modes); calling it in INPUT/OUTPUT mode returns INVALID_MODE and starts nothing. stop() must precede join() or join() never returns.
- The GPIO SDD (docs/sdd.md §5.3) claims a line number beyond the chip's line count logs OpenPinError but returns OP_OK. The code actually sets and returns Os::File::Status::DOESNT_EXIST. Follow the code.
- The I2C SDD (§3.3) claims the stub 'reports open failures for all transactions'. LinuxI2cDriverStub.cpp actually returns open()==true and I2cStatus::I2C_OK from all three handlers. Follow the code if you want upstream parity.
- LinuxSpiDriver's deprecated sync SpiReadWrite port is NOT guarded, but it calls straight into the guarded handler's body — concurrent use of both ports races on m_fd/m_bytes. In Rust, route both through the same mutex-locked handler (a behavior improvement worth documenting).
- SpiWriteRead FW_ASSERTs writeBuffer.getSize() == readBuffer.getSize(); mismatched sizes are a programmer error, not a status return. Same for isValid() on both buffers and portNum >= 0.
- LinuxSpiDriver::open() writes m_fd only after every ioctl succeeds, so a partially-configured device leaves the driver closed — but a SPI_ConfigMismatch (WARNING_LO) does NOT abort the open; the device is used with whatever the kernel actually accepted.
- LinuxSpiDriver's destructor calls close(m_fd) unconditionally, including close(-1) when never opened.
- SpiStatus::SPI_CONFIG_ERR(2), SPI_MISMATCH_ERR(3) and SPI_WRITE_ERR(4) are never returned by any code path; only SPI_OK(0), SPI_OPEN_ERR(1) and SPI_OTHER_ERR(5) appear. Keep the discriminants anyway.
- GpioStatus::NOT_OPENED(1) is only ever produced by errno_to_gpio_status on EBADF — an unopened pin actually returns INVALID_MODE(2) because m_configuration defaults to MAX_GPIO_CONFIGURATION and the mode gate fires first.
- The I2C writeRead combined transfer collapses every failure mode into I2C_OTHER_ERR(5) because ioctl(I2C_RDWR) reports a single result — never map it to ADDRESS/WRITE/READ errors.
- I2C addresses are 7-bit and downcast with FW_ASSERT_NO_OVERFLOW(addr, U16) in writeRead only; write/read pass the raw U32 straight to ioctl(I2C_SLAVE) with no range check.
- Buffers passed to I2C and SPI ports are caller-owned; the drivers never allocate or deallocate. Only LinuxUartDriver participates in the allocate/deallocate ownership protocol (allocate on the read thread, deallocate on recvReturnIn).
- The GPIO consumer label sent to the kernel is the component object name truncated to 32 bytes, and is empty when FW_OBJECT_NAMES is off (FW_OPTIONAL_NAME).
- Bare `string` in these FPP files (GPIO events, SPI_ConfigMismatch's `parameter`) defaults to size 80; UART events explicitly use `string size 40`.
- LinuxUartDriver only invokes ready_out(0) if isConnected_ready_OutputPort(0) — an unconnected ready port must not assert.

