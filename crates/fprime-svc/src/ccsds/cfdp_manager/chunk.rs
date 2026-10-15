//! Chunk lists (file-segment gap tracking) — port of
//! `Svc/Ccsds/CfdpManager/Chunk.{hpp,cpp}`, derived from NASA CF 3.0.0
//! `cf_chunks.{h,c}`.
//!
//! Copyright (c) 2019 United States Government as represented by the
//! Administrator of the National Aeronautics and Space Administration.
//! All Rights Reserved. NASA Docket No. GSC-18,447-1. Licensed under the
//! Apache License, Version 2.0 (see `ATTRIBUTION` in the module header of
//! `cfdp_manager`).
//!
//! A list of `(offset, size)` chunks kept sorted by offset, merging
//! contiguous or overlapping chunks. Receive transactions record the file
//! data received (gaps become NAK segment requests); send transactions
//! record the NAKed ranges to retransmit. When the list is full, a new
//! chunk evicts the smallest one if it is larger.
//!
//! Upstream the list points into a per-channel chunk memory block; here each
//! list owns its preallocated `Box<[Chunk]>`, created once at configure
//! time and never resized.

use super::types::FileSize;
use fprime_fw::fw_assert;

/// C++ `ChunkIdx`.
pub type ChunkIdx = u16;

/// C++ `Chunk`: a piece of a file.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct Chunk {
    /// Start offset within the file.
    pub offset: FileSize,
    /// Size in bytes.
    pub size: FileSize,
}

/// C++ `CfdpChunkList`.
#[derive(Debug)]
pub struct ChunkList {
    count: ChunkIdx,
    max_chunks: ChunkIdx,
    chunks: Box<[Chunk]>,
}

impl ChunkList {
    /// A list holding up to `max_chunks` chunks (asserts `max_chunks > 0`).
    #[must_use]
    pub fn new(max_chunks: ChunkIdx) -> Self {
        fw_assert!(max_chunks > 0);
        Self {
            count: 0,
            max_chunks,
            chunks: vec![Chunk::default(); usize::from(max_chunks)].into_boxed_slice(),
        }
    }

    /// Number of chunks.
    #[must_use]
    pub fn get_count(&self) -> ChunkIdx {
        self.count
    }

    /// Capacity.
    #[must_use]
    pub fn get_max_chunks(&self) -> ChunkIdx {
        self.max_chunks
    }

    /// The chunks in offset order.
    #[must_use]
    pub fn chunks(&self) -> &[Chunk] {
        &self.chunks[..usize::from(self.count)]
    }

    /// Empty the list (and zero the storage, C++ parity).
    pub fn reset(&mut self) {
        self.count = 0;
        self.chunks.fill(Chunk::default());
    }

    /// Add `(offset, size)`, merging with neighbors. A zero-size chunk is
    /// ignored (it covers nothing and may come from a received PDU).
    pub fn add(&mut self, offset: FileSize, size: FileSize) {
        if size == 0 {
            return;
        }
        let chunk = Chunk { offset, size };
        let i = self.find_insert_position(&chunk);
        // C++ parity: offset + size must not wrap.
        fw_assert!(
            offset.checked_add(size).is_some(),
            offset as i32,
            size as i32
        );
        self.insert(i, &chunk);
    }

    /// The first (lowest-offset) chunk, if any.
    #[must_use]
    pub fn get_first_chunk(&self) -> Option<Chunk> {
        if self.count > 0 {
            Some(self.chunks[0])
        } else {
            None
        }
    }

    /// Consume `size` bytes from the front of the first chunk, erasing it
    /// when emptied.
    pub fn remove_from_first(&mut self, size: FileSize) {
        let chunk = &mut self.chunks[0];
        let size = size.min(chunk.size);
        chunk.size -= size;
        if chunk.size == 0 {
            self.erase_chunk(0);
        } else {
            chunk.offset += size;
        }
    }

    /// C++ `computeGaps`: walk the gaps up to `total` starting at `start`,
    /// calling `callback` for each (at most `max_gaps`, beyond an initial
    /// gap before the first chunk), and return the number found. A total
    /// of 0 has no gaps; an empty list is one gap `[0, total)` whatever
    /// `start` is (C++ parity).
    pub fn compute_gaps(
        &self,
        max_gaps: ChunkIdx,
        total: FileSize,
        start: FileSize,
        mut callback: Option<&mut dyn FnMut(&Chunk)>,
    ) -> u32 {
        let mut ret: u32 = 0;
        if total == 0 {
            return 0;
        }
        fw_assert!(start < total, start as i32, total as i32);
        if self.count == 0 {
            let chunk = Chunk {
                offset: 0,
                size: total,
            };
            if let Some(cb) = callback.as_mut() {
                cb(&chunk);
            }
            return 1;
        }
        if start < self.chunks[0].offset {
            let chunk = Chunk {
                offset: start,
                size: self.chunks[0].offset - start,
            };
            if let Some(cb) = callback.as_mut() {
                cb(&chunk);
            }
            ret = 1;
        }
        let mut i: ChunkIdx = 0;
        while ret < u32::from(max_gaps) && i < self.count {
            let next_off = if i == self.count - 1 {
                total
            } else {
                self.chunks[usize::from(i) + 1].offset
            };
            let current = self.chunks[usize::from(i)];
            let gap_start = current.offset.wrapping_add(current.size);
            let offset = if gap_start > start { gap_start } else { start };
            let chunk = Chunk {
                offset,
                size: next_off.wrapping_sub(offset),
            };
            if gap_start >= total {
                break;
            } else if start < next_off {
                if let Some(cb) = callback.as_mut() {
                    cb(&chunk);
                }
                ret += 1;
            }
            i += 1;
        }
        ret
    }

    fn insert_chunk(&mut self, index: ChunkIdx, chunk: &Chunk) {
        fw_assert!(
            self.count < self.max_chunks,
            i32::from(self.count),
            i32::from(self.max_chunks)
        );
        fw_assert!(index <= self.count, i32::from(index), i32::from(self.count));
        let index = usize::from(index);
        let count = usize::from(self.count);
        if count > 0 && index != count {
            self.chunks.copy_within(index..count, index + 1);
        }
        self.chunks[index] = *chunk;
        self.count += 1;
    }

    fn erase_chunk(&mut self, index: ChunkIdx) {
        fw_assert!(self.count > 0);
        fw_assert!(index < self.count, i32::from(index), i32::from(self.count));
        let index = usize::from(index);
        let count = usize::from(self.count);
        self.chunks.copy_within(index + 1..count, index);
        self.count -= 1;
    }

    fn erase_range(&mut self, start: ChunkIdx, end: ChunkIdx) {
        fw_assert!(end <= self.count, i32::from(end), i32::from(self.count));
        if start < end {
            let count = usize::from(self.count);
            self.chunks
                .copy_within(usize::from(end)..count, usize::from(start));
            self.count -= end - start;
        }
    }

    /// Binary search for the first chunk whose offset is not below the new
    /// chunk's.
    fn find_insert_position(&self, chunk: &Chunk) -> ChunkIdx {
        let mut first: ChunkIdx = 0;
        let mut count = self.count;
        while count > 0 {
            let step = count / 2;
            let i = first + step;
            if self.chunks[usize::from(i)].offset < chunk.offset {
                first = i + 1;
                count -= step + 1;
            } else {
                count = step;
            }
        }
        first
    }

    fn combine_next(&mut self, i: ChunkIdx, chunk: &Chunk) -> bool {
        let mut combined_i = i;
        let mut chunk_end = chunk.offset.wrapping_add(chunk.size);
        // C++ parity: a rollover is only possible as a bug.
        fw_assert!(
            chunk_end > chunk.offset,
            chunk_end as i32,
            chunk.offset as i32
        );
        while combined_i < self.count {
            if chunk_end < self.chunks[usize::from(combined_i)].offset {
                break;
            }
            combined_i += 1;
        }
        if i != combined_i {
            let last = self.chunks[usize::from(combined_i) - 1];
            chunk_end = chunk_end.max(last.offset + last.size);
            self.chunks[usize::from(i)] = Chunk {
                offset: chunk.offset,
                size: chunk_end - chunk.offset,
            };
            self.erase_range(i + 1, combined_i);
            return true;
        }
        false
    }

    fn combine_previous(&mut self, i: ChunkIdx, chunk: &Chunk) -> bool {
        fw_assert!(
            i <= self.max_chunks,
            i32::from(i),
            i32::from(self.max_chunks)
        );
        if i > 0 {
            let chunk_end = chunk.offset + chunk.size;
            let prev = &mut self.chunks[usize::from(i) - 1];
            let prev_end = prev.offset + prev.size;
            if chunk.offset <= prev_end {
                if prev_end < chunk_end {
                    prev.size = chunk_end - prev.offset;
                }
                return true;
            }
        }
        false
    }

    fn insert(&mut self, i: ChunkIdx, chunk: &Chunk) {
        if self.combine_next(i, chunk) {
            let merged = self.chunks[usize::from(i)];
            if self.combine_previous(i, &merged) {
                self.erase_chunk(i);
            }
        } else if !self.combine_previous(i, chunk) {
            if self.count < self.max_chunks {
                self.insert_chunk(i, chunk);
            } else {
                let smallest_i = self.find_smallest_size();
                if self.chunks[usize::from(smallest_i)].size < chunk.size {
                    self.erase_chunk(smallest_i);
                    let at = self.find_insert_position(chunk);
                    self.insert_chunk(at, chunk);
                }
            }
        }
    }

    fn find_smallest_size(&self) -> ChunkIdx {
        let mut smallest: ChunkIdx = 0;
        for i in 1..self.count {
            if self.chunks[usize::from(i)].size < self.chunks[usize::from(smallest)].size {
                smallest = i;
            }
        }
        smallest
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::vec::Vec;

    fn gaps(list: &ChunkList, max: ChunkIdx, total: FileSize, start: FileSize) -> Vec<Chunk> {
        let mut out = Vec::new();
        let mut cb = |c: &Chunk| out.push(*c);
        list.compute_gaps(max, total, start, Some(&mut cb));
        out
    }

    #[test]
    fn adds_merge_contiguous_and_overlapping_chunks() {
        let mut list = ChunkList::new(8);
        list.add(0, 10);
        list.add(10, 10);
        assert_eq!(
            list.chunks(),
            &[Chunk {
                offset: 0,
                size: 20
            }]
        );
        list.add(30, 5);
        list.add(25, 10);
        assert_eq!(
            list.chunks(),
            &[
                Chunk {
                    offset: 0,
                    size: 20
                },
                Chunk {
                    offset: 25,
                    size: 10
                }
            ]
        );
        // Bridging chunk merges both neighbours.
        list.add(15, 12);
        assert_eq!(
            list.chunks(),
            &[Chunk {
                offset: 0,
                size: 35
            }]
        );
    }

    #[test]
    fn full_list_evicts_the_smallest_chunk_for_a_larger_one() {
        let mut list = ChunkList::new(2);
        list.add(0, 1);
        list.add(10, 5);
        list.add(20, 3);
        assert_eq!(
            list.chunks(),
            &[
                Chunk {
                    offset: 10,
                    size: 5
                },
                Chunk {
                    offset: 20,
                    size: 3
                }
            ]
        );
        // Smaller than the smallest: dropped.
        list.add(40, 2);
        assert_eq!(list.get_count(), 2);
        assert_eq!(
            list.chunks()[1],
            Chunk {
                offset: 20,
                size: 3
            }
        );
    }

    #[test]
    fn compute_gaps_reports_the_holes() {
        let mut list = ChunkList::new(8);
        list.add(10, 10);
        list.add(30, 10);
        assert_eq!(
            gaps(&list, 8, 50, 0),
            [
                Chunk {
                    offset: 0,
                    size: 10
                },
                Chunk {
                    offset: 20,
                    size: 10
                },
                Chunk {
                    offset: 40,
                    size: 10
                }
            ]
        );
        // The initial gap counts against max_gaps.
        assert_eq!(gaps(&list, 1, 50, 0).len(), 1);
        assert_eq!(gaps(&list, 2, 50, 0).len(), 2);
        // A start inside the file skips the gaps that end before it.
        assert_eq!(
            gaps(&list, 8, 50, 25),
            [
                Chunk {
                    offset: 25,
                    size: 5
                },
                Chunk {
                    offset: 40,
                    size: 10
                }
            ]
        );
        // A file exactly covered has no gap.
        let mut full = ChunkList::new(2);
        full.add(0, 50);
        assert_eq!(full.compute_gaps(1, 50, 0, None), 0);
    }

    #[test]
    fn compute_gaps_of_empty_list_is_the_whole_file_and_zero_total_has_none() {
        let list = ChunkList::new(4);
        assert_eq!(
            gaps(&list, 4, 100, 30),
            [Chunk {
                offset: 0,
                size: 100
            }]
        );
        assert_eq!(list.compute_gaps(4, 0, 0, None), 0);
    }

    #[test]
    fn zero_size_chunks_are_ignored() {
        let mut list = ChunkList::new(4);
        list.add(5, 0);
        assert_eq!(list.get_count(), 0);
    }

    #[test]
    fn remove_from_first_consumes_then_erases() {
        let mut list = ChunkList::new(4);
        list.add(0, 10);
        list.add(20, 10);
        list.remove_from_first(4);
        assert_eq!(list.get_first_chunk(), Some(Chunk { offset: 4, size: 6 }));
        list.remove_from_first(100);
        assert_eq!(
            list.get_first_chunk(),
            Some(Chunk {
                offset: 20,
                size: 10
            })
        );
        list.reset();
        assert_eq!(list.get_first_chunk(), None);
    }
}
