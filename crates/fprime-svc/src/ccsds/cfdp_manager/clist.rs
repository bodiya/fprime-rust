//! Circular doubly linked lists — port of `Svc/Ccsds/CfdpManager/Clist.{hpp,cpp}`,
//! derived from NASA CF 3.0.0 `cf_clist.{h,c}`.
//!
//! Copyright (c) 2019 United States Government as represented by the
//! Administrator of the National Aeronautics and Space Administration.
//! All Rights Reserved. NASA Docket No. GSC-18,447-1. Licensed under the
//! Apache License, Version 2.0.
//!
//! Upstream the lists are intrusive: a `CListNode` embedded in each
//! transaction, history entry and chunk wrapper, recovered with
//! `container_of`. Here the nodes of one pool are indices into that pool and
//! the links live in a [`CListLinks`] table beside it (one `next`/`prev` per
//! pool element); a list head is an `Option<NodeIdx>`. The algorithms — and
//! so the list orders the engine depends on — are the C ones, step for step,
//! including the traversal that tolerates the visited node removing itself.

use fprime_fw::fw_assert;

/// Index of a node in its pool.
pub type NodeIdx = u16;

/// A list head (C++ `CListNode*`; `None` is the empty list).
pub type ListHead = Option<NodeIdx>;

/// C++ `CListTraverseStatus`.
#[repr(u8)]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum CListTraverseStatus {
    /// Keep traversing.
    Continue = 0,
    /// Stop traversing.
    Exit = 1,
}

/// The `next`/`prev` links of one node pool.
#[derive(Debug, Clone)]
pub struct CListLinks {
    next: Box<[NodeIdx]>,
    prev: Box<[NodeIdx]>,
}

impl CListLinks {
    /// Links for `len` nodes, each initialized to point at itself.
    #[must_use]
    pub fn new(len: usize) -> Self {
        fw_assert!(len <= usize::from(NodeIdx::MAX), len as i32);
        let ids: Box<[NodeIdx]> = (0..len).map(|i| i as NodeIdx).collect();
        Self {
            next: ids.clone(),
            prev: ids,
        }
    }

    /// The node after `node`.
    #[must_use]
    pub fn next(&self, node: NodeIdx) -> NodeIdx {
        self.next[usize::from(node)]
    }

    /// The node before `node`.
    #[must_use]
    pub fn prev(&self, node: NodeIdx) -> NodeIdx {
        self.prev[usize::from(node)]
    }

    /// C++ `CfdpCListInitNode`: a node linked to itself.
    pub fn init_node(&mut self, node: NodeIdx) {
        self.next[usize::from(node)] = node;
        self.prev[usize::from(node)] = node;
    }

    fn assert_detached(&self, node: NodeIdx) {
        fw_assert!(self.next(node) == node, i32::from(node));
        fw_assert!(self.prev(node) == node, i32::from(node));
    }

    /// C++ `CfdpCListInsertFront`.
    pub fn insert_front(&mut self, head: &mut ListHead, node: NodeIdx) {
        self.assert_detached(node);
        if let Some(h) = *head {
            let last = self.prev(h);
            self.next[usize::from(node)] = h;
            self.prev[usize::from(node)] = last;
            self.next[usize::from(last)] = node;
            self.prev[usize::from(h)] = node;
        }
        *head = Some(node);
    }

    /// C++ `CfdpCListInsertBack`.
    pub fn insert_back(&mut self, head: &mut ListHead, node: NodeIdx) {
        self.assert_detached(node);
        match *head {
            None => *head = Some(node),
            Some(h) => {
                let last = self.prev(h);
                self.next[usize::from(node)] = h;
                self.prev[usize::from(h)] = node;
                self.prev[usize::from(node)] = last;
                self.next[usize::from(last)] = node;
            }
        }
    }

    /// C++ `CfdpCListPop`: remove and return the head.
    pub fn pop(&mut self, head: &mut ListHead) -> Option<NodeIdx> {
        let ret = *head;
        if let Some(node) = ret {
            self.remove(head, node);
        }
        ret
    }

    /// C++ `CfdpCListRemove`: unlink `node` (asserts a non-empty list) and
    /// re-initialize it.
    pub fn remove(&mut self, head: &mut ListHead, node: NodeIdx) {
        let h = match *head {
            Some(h) => h,
            None => {
                fw_assert!(false, i32::from(node));
                return;
            }
        };
        if self.next(node) == node {
            // Only node in the list.
            fw_assert!(node == h, i32::from(node), i32::from(h));
            *head = None;
        } else if h == node {
            // Removing the first node: the second becomes the head.
            let last = self.prev(h);
            let second = self.next(node);
            self.next[usize::from(last)] = second;
            *head = Some(second);
            self.prev[usize::from(second)] = self.prev(node);
        } else {
            let next = self.next(node);
            let prev = self.prev(node);
            self.prev[usize::from(next)] = prev;
            self.next[usize::from(prev)] = next;
        }
        self.init_node(node);
    }

    /// C++ `CfdpCListInsertAfter`: insert `after` following `start` in a
    /// non-empty list.
    pub fn insert_after(&mut self, head: &ListHead, start: NodeIdx, after: NodeIdx) {
        fw_assert!(head.is_some());
        fw_assert!(start != after, i32::from(start));
        let start_next = self.next(start);
        self.next[usize::from(after)] = start_next;
        self.next[usize::from(start)] = after;
        self.prev[usize::from(after)] = start;
        self.prev[usize::from(start_next)] = after;
    }
}

/// C++ `CfdpCListTraverse`: visit the list starting at `start` in order,
/// stopping early when `f` returns `Exit`. `f` may remove (or move) the
/// node it is visiting. `links` selects the link table of the traversed
/// pool inside `ctx`, which `f` mutates. Asserts if the list is longer
/// than `max_iterations` (upstream's guard against a corrupted list).
pub fn traverse<T: ?Sized>(
    ctx: &mut T,
    links: fn(&T) -> &CListLinks,
    start: ListHead,
    max_iterations: u32,
    mut f: impl FnMut(&mut T, NodeIdx) -> CListTraverseStatus,
) {
    let Some(mut start) = start else {
        return;
    };
    let mut node = start;
    let mut last = false;
    let mut i: u32 = 0;
    while i < max_iterations && !last {
        // Read next first: the callback may remove this node.
        let node_next = links(ctx).next(node);
        if node_next == start {
            last = true;
        }
        if f(ctx, node) != CListTraverseStatus::Continue {
            break;
        }
        // If the start node removed itself, the next node is the new start.
        if start == node && links(ctx).next(node) != node_next {
            start = node_next;
        }
        node = node_next;
        i += 1;
    }
    fw_assert!(last || i < max_iterations, i as i32);
}

/// C++ `CfdpCListTraverseR`: visit the list ending at `end` backwards,
/// from `end`'s predecessor (the tail when `end` is the head).
pub fn traverse_r<T: ?Sized>(
    ctx: &mut T,
    links: fn(&T) -> &CListLinks,
    end: ListHead,
    max_iterations: u32,
    mut f: impl FnMut(&mut T, NodeIdx) -> CListTraverseStatus,
) {
    let Some(end) = end else {
        return;
    };
    let mut node = links(ctx).prev(end);
    let mut end = node;
    let mut last = false;
    let mut i: u32 = 0;
    while i < max_iterations && !last {
        let node_next = links(ctx).prev(node);
        if node_next == end {
            last = true;
        }
        if f(ctx, node) != CListTraverseStatus::Continue {
            break;
        }
        if end == node && links(ctx).prev(node) != node_next {
            end = node_next;
        }
        node = node_next;
        i += 1;
    }
    fw_assert!(last || i < max_iterations, i as i32);
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::vec::Vec;

    struct Pool {
        links: CListLinks,
        head: ListHead,
    }

    fn pool(n: usize) -> Pool {
        Pool {
            links: CListLinks::new(n),
            head: None,
        }
    }

    fn order(p: &mut Pool) -> Vec<NodeIdx> {
        let mut out = Vec::new();
        let head = p.head;
        traverse(
            p,
            |p| &p.links,
            head,
            100,
            |_, n| {
                out.push(n);
                CListTraverseStatus::Continue
            },
        );
        out
    }

    #[test]
    fn insert_front_and_back_order_nodes() {
        let mut p = pool(4);
        p.links.insert_back(&mut p.head, 0);
        p.links.insert_back(&mut p.head, 1);
        p.links.insert_front(&mut p.head, 2);
        assert_eq!(order(&mut p), [2, 0, 1]);
        assert_eq!(p.head, Some(2));
    }

    #[test]
    fn pop_and_remove_unlink_and_reinit() {
        let mut p = pool(4);
        for n in 0..4 {
            p.links.insert_back(&mut p.head, n);
        }
        assert_eq!(p.links.pop(&mut p.head), Some(0));
        assert_eq!(p.links.next(0), 0);
        p.links.remove(&mut p.head, 2);
        assert_eq!(order(&mut p), [1, 3]);
        p.links.remove(&mut p.head, 1);
        p.links.remove(&mut p.head, 3);
        assert_eq!(p.head, None);
        assert_eq!(p.links.pop(&mut p.head), None);
    }

    #[test]
    fn insert_after_links_into_the_middle() {
        let mut p = pool(4);
        p.links.insert_back(&mut p.head, 0);
        p.links.insert_back(&mut p.head, 1);
        p.links.insert_after(&p.head, 0, 3);
        assert_eq!(order(&mut p), [0, 3, 1]);
    }

    #[test]
    fn traverse_survives_the_visited_node_removing_itself() {
        let mut p = pool(4);
        for n in 0..4 {
            p.links.insert_back(&mut p.head, n);
        }
        let mut seen = Vec::new();
        let head = p.head;
        traverse(
            &mut p,
            |p| &p.links,
            head,
            100,
            |p, n| {
                seen.push(n);
                if n % 2 == 0 {
                    p.links.remove(&mut p.head, n);
                }
                CListTraverseStatus::Continue
            },
        );
        assert_eq!(seen, [0, 1, 2, 3]);
        assert_eq!(order(&mut p), [1, 3]);
    }

    #[test]
    fn traverse_stops_on_exit_and_reverse_walks_from_the_tail() {
        let mut p = pool(4);
        for n in 0..4 {
            p.links.insert_back(&mut p.head, n);
        }
        let mut seen = Vec::new();
        let head = p.head;
        traverse(
            &mut p,
            |p| &p.links,
            head,
            100,
            |_, n| {
                seen.push(n);
                if n == 1 {
                    CListTraverseStatus::Exit
                } else {
                    CListTraverseStatus::Continue
                }
            },
        );
        assert_eq!(seen, [0, 1]);
        let mut rev = Vec::new();
        traverse_r(
            &mut p,
            |p| &p.links,
            head,
            100,
            |_, n| {
                rev.push(n);
                CListTraverseStatus::Continue
            },
        );
        assert_eq!(rev, [3, 2, 1, 0]);
        // Empty lists are a no-op.
        traverse(&mut p, |p| &p.links, None, 100, |_, _| unreachable!());
        traverse_r(&mut p, |p| &p.links, None, 100, |_, _| unreachable!());
    }
}
