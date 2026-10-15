//! Loading and validating a sequence — port of
//! `FpySequencerValidationState.cpp`.
//!
//! The file format (all big-endian, the FPP serialization of the `Fpy`
//! types):
//!
//! ```text
//! Header   majorVersion U8, minorVersion U8, patchVersion U8,
//!          schemaVersion U8 (= SCHEMA_VERSION), argumentCount U8,
//!          statementCount U16, bodySize U32                      11 bytes
//! Body     argumentCount × ArgSpec  { argName: [u16 len][bytes],
//!                                     typeName: [u16 len][bytes],
//!                                     argSize: U32 }
//!          statementCount × Statement { opCode: U8 (DirectiveId),
//!                                       argBuf: [u16 len][bytes] }
//!                                                       bodySize bytes
//! Footer   crc U32: CRC-32 (`Utils::Hash`) of header and body     4 bytes
//! ```
//!
//! Each stage is read into the start of the sequence buffer and
//! deserialized from there; the body may hold bytes after the last
//! statement (they are covered by the CRC and otherwise ignored), but the
//! file must end right after the footer.

use super::{FpySequencer, FpyState, types::*};
use crate::cmd_sequencer::SeqArgsBuffer;
use fprime_config::FwSizeType;
use fprime_config::fpy_sequencer::{
    MAX_SEQUENCE_ARG_COUNT, MAX_SEQUENCE_STATEMENT_COUNT, MAX_STACK_SIZE,
};
use fprime_fw::{Endianness, SerBuf, SerBufAny, SerializeStatus, Success, fw_assert};
use fprime_os::file::{File, Mode as FileMode, Status as FileStatus, WaitType};

impl FpySequencer {
    /// C++ `validate`: load the sequence file and check its header, body,
    /// CRC and length, and the sequence arguments against it.
    pub(crate) fn validate(&self, st: &mut FpyState) -> Success {
        if st.sequence_file_path.is_empty() {
            self.log_warning_hi_file_open_error(
                st.sequence_file_path.as_bytes(),
                FileStatus::InvalidArgument as i32,
            );
            return Success::Failure;
        }
        // the CRC must start from its initial value
        st.computed_crc.init();

        // a stack copy of the path: no allocation, and `st` stays free
        let path_copy = st.sequence_file_path.clone();
        let path = String::from_utf8_lossy(path_copy.as_bytes());
        let mut file = File::new();
        let open_status = file.open(&path, FileMode::OpenRead);
        if open_status != FileStatus::OpOk {
            self.log_warning_hi_file_open_error(
                st.sequence_file_path.as_bytes(),
                open_status as i32,
            );
            return Success::Failure;
        }

        if self.read_bytes(
            st,
            &mut file,
            Header::SERIALIZED_SIZE as FwSizeType,
            FileReadStage::Header,
            true,
        ) != Success::Success
        {
            return Success::Failure;
        }
        if self.read_header(st) != Success::Success {
            return Success::Failure;
        }
        let body_size = FwSizeType::from(st.sequence_obj.header.body_size);
        if self.read_bytes(st, &mut file, body_size, FileReadStage::Body, true) != Success::Success
        {
            return Success::Failure;
        }
        if self.read_body(st) != Success::Success {
            return Success::Failure;
        }
        // the footer is not part of the CRC
        if self.read_bytes(
            st,
            &mut file,
            Footer::SERIALIZED_SIZE as FwSizeType,
            FileReadStage::Footer,
            false,
        ) != Success::Success
        {
            return Success::Failure;
        }
        if self.read_footer(st) != Success::Success {
            return Success::Failure;
        }

        // the file must end here. size() and position() can fail on a file
        // system error or a concurrent change: a validation failure, not
        // an assertion.
        let mut file_size: FwSizeType = 0;
        let size_status = file.size(&mut file_size);
        if size_status != FileStatus::OpOk {
            self.log_warning_hi_file_api_error(
                st.sequence_file_path.as_bytes(),
                size_status as i32,
            );
            return Success::Failure;
        }
        let mut position: FwSizeType = 0;
        let position_status = file.position(&mut position);
        if position_status != FileStatus::OpOk {
            self.log_warning_hi_file_api_error(
                st.sequence_file_path.as_bytes(),
                position_status as i32,
            );
            return Success::Failure;
        }
        if file_size != position {
            self.log_warning_hi_extra_bytes_in_sequence(file_size.wrapping_sub(position));
            return Success::Failure;
        }

        let args_size = st.sequence_args.size;
        if args_size > MAX_STACK_SIZE as FwSizeType {
            self.log_warning_hi_arg_total_size_exceeds_stack_limit(args_size as StackSizeType);
            return Success::Failure;
        }
        // the size travels separately from the buffer, so it can describe
        // more bytes than the buffer holds: reject before anything reads
        // the buffer by it
        let capacity = SeqArgsBuffer::SIZE as FwSizeType;
        if args_size > capacity {
            self.log_warning_hi_arg_size_exceeds_capacity(args_size, capacity);
            return Success::Failure;
        }
        Success::Success
    }

    /// C++ `readHeader`: deserialize the header from the sequence buffer
    /// and check the schema version and the argument and statement counts.
    pub(crate) fn read_header(&self, st: &mut FpyState) -> Success {
        let status = st
            .sequence_buffer
            .deserialize(&mut st.sequence_obj.header, Endianness::Big);
        if status != SerializeStatus::Ok {
            self.deserialize_error(st, FileReadStage::Header, status);
            return Success::Failure;
        }
        let header = st.sequence_obj.header;
        if header.schema_version != SCHEMA_VERSION {
            self.log_warning_hi_wrong_schema_version(SCHEMA_VERSION, header.schema_version);
            return Success::Failure;
        }
        if usize::from(header.argument_count) > MAX_SEQUENCE_ARG_COUNT {
            self.log_warning_hi_too_many_sequence_args(
                header.argument_count,
                MAX_SEQUENCE_ARG_COUNT as u8,
            );
            return Success::Failure;
        }
        if usize::from(header.statement_count) > MAX_SEQUENCE_STATEMENT_COUNT {
            self.log_warning_hi_too_many_sequence_directives(
                header.statement_count,
                MAX_SEQUENCE_STATEMENT_COUNT as u16,
            );
            return Success::Failure;
        }
        Success::Success
    }

    /// C++ `readBody`: deserialize the argument specs (their sizes must add
    /// up to the sequence arguments' size) and the statements.
    pub(crate) fn read_body(&self, st: &mut FpyState) -> Success {
        let argument_count = usize::from(st.sequence_obj.header.argument_count);
        st.total_expected_arg_size = 0;
        for i in 0..argument_count {
            let status = st
                .sequence_buffer
                .deserialize(&mut st.sequence_obj.args[i], Endianness::Big);
            if status != SerializeStatus::Ok {
                self.deserialize_error(st, FileReadStage::Body, status);
                return Success::Failure;
            }
            // C++ parity: a U32 sum, wrapping
            st.total_expected_arg_size = st
                .total_expected_arg_size
                .wrapping_add(st.sequence_obj.args[i].arg_size);
        }
        if st.total_expected_arg_size as usize > MAX_STACK_SIZE {
            self.log_warning_hi_arg_total_size_exceeds_stack_limit(st.total_expected_arg_size);
            return Success::Failure;
        }
        if FwSizeType::from(st.total_expected_arg_size) != st.sequence_args.size {
            self.log_warning_hi_arg_size_mismatch(
                st.total_expected_arg_size,
                st.sequence_args.size,
                st.sequence_file_path.as_bytes(),
            );
            return Success::Failure;
        }

        let statement_count = usize::from(st.sequence_obj.header.statement_count);
        for i in 0..statement_count {
            let status = st
                .sequence_buffer
                .deserialize(&mut st.sequence_obj.statements[i], Endianness::Big);
            if status != SerializeStatus::Ok {
                self.deserialize_error(st, FileReadStage::Body, status);
                return Success::Failure;
            }
        }
        Success::Success
    }

    /// C++ `readFooter`: deserialize the footer and compare its CRC with
    /// the one computed while reading.
    pub(crate) fn read_footer(&self, st: &mut FpyState) -> Success {
        let status = st
            .sequence_buffer
            .deserialize(&mut st.sequence_obj.footer, Endianness::Big);
        if status != SerializeStatus::Ok {
            self.deserialize_error(st, FileReadStage::Footer, status);
            return Success::Failure;
        }
        let computed = st.computed_crc.finalize();
        if computed != st.sequence_obj.footer.crc {
            self.log_warning_hi_wrong_crc(st.sequence_obj.footer.crc, computed);
            return Success::Failure;
        }
        Success::Success
    }

    /// C++ `readBytes`: read `expected_len` bytes into the start of the
    /// sequence buffer, folding them into the CRC unless told otherwise.
    /// Asserts that the file is open.
    pub(crate) fn read_bytes(
        &self,
        st: &mut FpyState,
        file: &mut File,
        expected_len: FwSizeType,
        stage: FileReadStage,
        update_crc: bool,
    ) -> Success {
        fw_assert!(file.is_open());
        let capacity = SerBufAny::capacity(&st.sequence_buffer) as FwSizeType;
        // the buffer passed to allocate_buffer is too small for this file
        if expected_len > capacity {
            self.log_warning_hi_insufficient_buffer_space(
                capacity,
                st.sequence_file_path.as_bytes(),
            );
            return Success::Failure;
        }
        let len = expected_len as usize;
        let mut actual_len = expected_len;
        let status = file.read(
            &mut st.sequence_buffer.data[..len],
            &mut actual_len,
            WaitType::Wait,
        );
        if status != FileStatus::OpOk {
            self.log_warning_hi_file_read_error(
                stage,
                st.sequence_file_path.as_bytes(),
                status as i32,
            );
            return Success::Failure;
        }
        if actual_len < expected_len {
            self.log_warning_hi_end_of_file_error(stage, st.sequence_file_path.as_bytes());
            return Success::Failure;
        }
        fw_assert!(
            expected_len == actual_len,
            expected_len as i32,
            actual_len as i32
        );
        let status = st.sequence_buffer.set_buff_len(len);
        fw_assert!(status.is_ok(), status as i32);
        if update_crc {
            st.computed_crc.update(&st.sequence_buffer.data[..len]);
        }
        Success::Success
    }

    /// `FileReadDeserializeError` with the buffer's remaining and total
    /// sizes.
    fn deserialize_error(&self, st: &FpyState, stage: FileReadStage, status: SerializeStatus) {
        self.log_warning_hi_file_read_deserialize_error(
            stage,
            st.sequence_file_path.as_bytes(),
            status as i32,
            st.sequence_buffer.deserialize_size_left() as u64,
            st.sequence_buffer.get_size() as u64,
        );
    }
}
