/* Copyright (c) 2020, 2023, Oracle and/or its affiliates.

   This program is free software; you can redistribute it and/or modify
   it under the terms of the GNU General Public License, version 2.0,
   as published by the Free Software Foundation.

   This program is also distributed with certain software (including
   but not limited to OpenSSL) that is licensed under separate terms,
   as designated in a particular file or component or in included license
   documentation.  The authors of MySQL hereby grant you an additional
   permission to link the program and your derivative works with the
   separately licensed software that they have included with MySQL.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License, version 2.0, for more details.

   You should have received a copy of the GNU General Public License
   along with this program; if not, write to the Free Software
   Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA 02110-1301  USA */

#ifndef SUBGRAPH_ENUMERATION_H
#define SUBGRAPH_ENUMERATION_H 1

/**
  @file

  This file implements the DPhyp algorithm for enumerating connected
  subgraphs of hypergraphs (see hypergraph.h for a hypergraph definition).

  The core idea of the algorithm is that if the join structure of a
  query is expressed as a hypergraph, where the relations are nodes
  and the join predicates are hyperedges, one can efficiently find
  all legal join orders without Cartesian products by finding all
  possible subpartitions of the hypergraph. (Simple inner joins will
  have regular edges, but outer joins, antijoins etc., can be encoded
  as hyperedges to constrain the allowed join orderings, so that we
  do not join e.g. an inner and outer table together before said inner
  table has been joined to the entire set. Also, hyper-predicates such
  as t1.a + t2.b = t3.c will naturally give rise to hyperedges.)

  The algorithm is described in the paper “Dynamic Programming Strikes
  Back” by Neumann and Moerkotte. There is a somewhat extended version
  of the paper (that also contains a few corrections) in Moerkotte's
  treatise “Building Query Compilers”. Some critical details are still
  missing, which we've had to fill in ourselves. We don't currently
  implement the extension to generalized hypergraphs, but it should be
  fairly straightforward to do later. The algorithm is simple in concept
  but hard to grasp; we will only give a very rough outline here:

    1. Pick a seed node of the graph.
    2. Grow that seed along hyperedges, taking care never to make an
       unconnected graph or seeing the same subgraph twice.
    3. For each connected subgraph (csg): Repeat steps 1–2 independently
       to create a separate connected subgraph (the so-called complement,
       cmp), and try to connect the subgraph and its complement to create
       a larger graph (a so-called csg-cmp-pair).
    4. When such a csg-cmp-pair is found, call the receiver back with the
       csg and cmp. This is a valid subjoin that can be costed.

  The entry point for doing this is EnumerateAllConnectedPartitions().

  For complex joins, we may have to run DPhyp multiple times in a mode
  where we just count the number of partitions over various constrained
  graphs, and this will be a critical part of query planning time.
  Thus, it is coded as a template over a receiver type that gets callbacks
  for each partition. If the receiver just interested in counting,
  this saves a significant amount of call overhead. The templatization
  also allows the microbenchmarks to more accurately measure changes in
  the algorithm itself without having to benchmark the receiver.
 */

#include <assert.h>
#include <string>
#include "sql/join_optimizer/bit_utils.h"
#include "sql/join_optimizer/hypergraph.h"

// You can change 0 to 1 here to get debug traces of the algorithm
// as it is working.
#define DEBUGGING_DPHYP 1

#if DEBUGGING_DPHYP
#include <stdio.h>
#define HYPERGRAPH_PRINTF printf
#else
#define HYPERGRAPH_PRINTF(...)
#endif

namespace hypergraph {

template <class Receiver>
bool EnumerateAllConnectedPartitions(Receiver *receiver);

inline std::string PrintSet(NodeMap x) {
  std::string ret = "{";
  bool first = true;
  for (size_t node_idx : BitsSetIn(x)) {
    if (!first) {
      ret += ",";
    }
    first = false;

    char buf[256];
    snprintf(buf, sizeof(buf), "R%zu", node_idx + 1);
    ret += buf;
  }
  return ret + "}";
}

inline void PrintDynamicTable(size_t lowest_node_idx, NodeMap subgraph, NodeMap forbidden, NodeMap Neighborhood, NodeMap Complement, const char* FunctionName) {
  HYPERGRAPH_PRINTF("DynamicTable: |%25s|%20ld|%20s|%20s|%20s|%20s|\n",
                    FunctionName, lowest_node_idx, PrintSet(subgraph).c_str(), PrintSet(forbidden).c_str(), PrintSet(Neighborhood).c_str(), PrintSet(Complement).c_str());
}
inline void PrintDynamicTableHeader() {
  HYPERGRAPH_PRINTF("DynamicTable: |%-25s|%-20s|%-20s|%-20s|%-20s|%-20s|\n",
                    "FunctionName", "lowest_node_idx", "subgraph", "forbidden", "Neighborhood", "Complement");
}
inline void PrintDynamicTableLine() {
  HYPERGRAPH_PRINTF("DynamicTable: ------------------------------------------------------------------------------------------------------------------------------------\n");
}

/*
  FindNeighborhood() (see below) is crucial for speed. We can speed it up
  somewhat by observing that it is often being called many times with the
  same forbidden set and subgraphs that keep increasing; e.g., when we
  have the neighborhood {R1,R2}, we need to calculate the neighborhood of
  {R1}, {R2} and {R1,R2} -- the latter will start with calculating the
  neighborhood of {R1} and then add {R2} from there. We cannot just union
  the two neighborhoods due to hyperedges, but we can reuse the start.

  To this end, NeighborhoodCache implements a simple one-element cache.
  If we start a neighborhood computation that is a superset of the element
  in the cache, we can just continue with the neighborhood it calculated
  and add the missing elements. The overhead of managing the cache is a
  ~15-20% loss for simple graphs with low degrees (e.g. chains), but a
  huge speedup (60% or more) for graphs with high degrees, such as stars.
  Given that the simple graphs are already so fast that their time is
  hardly noticeable, this seems like a good overall tradeoff.

  The default enumeration of power sets given by NonzeroSubsetsOf
  (e.g. 000, 001, 010, 011, 100, etc. for three nodes in the neighborhood)
  is not optimal for caching. E.g., for four bits, we can brute-force the
  optimal order to be

    0001 *0010 0011 *0100 0101 *0110 0111 1110 *1000 1010 1100 *1001
    1011 *1110 1111

  where we overwrite the element in the cache every time we process a subset
  marked by *. This yields an optimal 17 loop iterations saved, leaving only 15.
  However, it is not clear how to efficiently enumerate these orders and choice
  of elements to cache realtime without huge precalculated tables (e.g., what is
  the optimal order for 14 potential neighbors?), so instead, we keep the normal
  order and add a simple heuristic: Keep every other item. The lowest bit will
  change between 0 and 1 every iteration, so one that ends in 1 cannot possibly
  be a subset of the the next enumerated subset. This yields:

    0001 *0010 0011 *0100 0101 *0110 0111 *1000 1001 *1010 1011 *1100
    1101 *1110 1111

  which saves 16 loop iterations, nearly as good. (This pattern does not seem
  to change markedly for larger subsets; the best pattern for five bits is
  1 *2 3 6 *10 11 14 *4 *5 7 21 *13 15 *8 9 12 *24 25 26 *28 29 30 *16 17 20
  *18 22 *19 23 *27 31, saving 49 bits where the heuristic saves 44. Optimal
  patterns for more than five bits are not known.)

  The only thing that really matters is keeping track of what the lowest bit
  is; we call it the “taboo bit”, as when we process such a subset, it
  signals that the result shouldn't replace whatever is in the cache.

  Note that you cannot reuse the cache across calls with different forbidden
  subsets; that would yield wrong results.
 */
class NeighborhoodCache {
 public:
  explicit NeighborhoodCache(NodeMap neighborhood)
      : m_taboo_bit(IsolateLowestBit(neighborhood)) {}

  // Tell the cache we intend to start a neighborhood search.
  // If the cache can reduce our workload, it will update the
  // two neighborhoods. Returns the actual set of bits we need
  // to compute the neighborhood for (whether it could save
  // anything or not).
  inline NodeMap InitSearch(NodeMap just_grown_by, NodeMap *neighborhood,
                            NodeMap *full_neighborhood) {
    if (IsSubset(m_last_just_grown_by, just_grown_by)) {
      // We can use our cache from the last node and continue the search from
      // there.
      *full_neighborhood |= m_last_full_neighborhood;
      *neighborhood = m_last_neighborhood;
      return just_grown_by & ~m_last_just_grown_by;
    }

    // We need to do the entire search as usual.
    return just_grown_by;
  }

  // Tell the cache we just computed a neighborhood. It can choose to
  // store it to accelerate future InitSearch() calls.
  inline void Store(int depth, NodeMap just_grown_by, NodeMap neighborhood,
                    NodeMap full_neighborhood) {
    HYPERGRAPH_PRINTF(
      "%sNeighborhoodCache->Store(just_grown_by:%s, neighborhood:%s, full_neighborhood:%s)     &cache:%p, cache.m_taboo_bit:%s\n",
      std::string(depth * 4, ' ').c_str(), PrintSet(just_grown_by).c_str(), PrintSet(neighborhood).c_str(), PrintSet(full_neighborhood).c_str(), 
      (void*)this, PrintSet(m_taboo_bit).c_str());

    assert(IsSubset(neighborhood, full_neighborhood));
    if (Overlaps(just_grown_by, m_taboo_bit)) return;

    m_last_just_grown_by = just_grown_by;
    m_last_full_neighborhood = full_neighborhood;
    m_last_neighborhood = neighborhood;
  }

 public:
  const NodeMap m_taboo_bit;
  NodeMap m_last_just_grown_by =
      ~0;  // Don't try to use the cache the first iteration.
  NodeMap m_last_full_neighborhood = 0;
  NodeMap m_last_neighborhood = 0;
};

/**
  Find the neighborhood of the given subgraph (S); informally, the set of nodes
  immediately reachable from that subgraph. There's an additional constraint
  in that the edges used to do so must not touch the forbidden set of nodes (X).
  The DPhyp paper calls this function N(S, X) (with a calligraphic N).

  How to calculate the neighborhood efficiently is one of the least explicitly
  described parts of the paper. The definition goes about as follows:

    1. Find E↓'(S,X), the set of “interesting hypernodes” (outgoing edge
       destinations from S). These are the (endpoints of) edges that have one
       side entirely within S, that have the other side entirely _outside_ S,
       and none of the sides touch the forbidden set X.
    2. Minimize E↓'(S,X) by removing all “subsumed hypernodes”, giving E↓(S,X).
       u subsumes v if it is a proper subset; if so, we can never go to where v
       points to before we've been at u, so it's pointless to keep v.
    3. For each hypernode in E↓(S,X), pick node with lowest index as a
       representative, because our subset enumeration algorithms cannot
       enumerate subsets of hypernodes, only subsets of normal nodes.
       (Actually, any node that's part of the hypernode would do; it does not
       even need to be consistent.) These nodes together constitute the
       neighborhood.

  There are a couple of points to note here:

  First, adding more nodes than needed to the neighborhood does not affect
  correctness of the algorithm, only speed. We try all combinations of
  included/excluded for the neighborhood (2^N in the number of nodes),
  so this covers all potential subgraphs; in theory, we could even just choose
  all non-forbidden nodes and reduce to the algorithm known as DPhyp, it just
  wouldn't be very efficient.

  Second, step 3 means that we may very well end up with a non-connected
  subgraph. This is harmless; we may eventually grow it to a connected one or
  we may not, we just won't start looking for any complements until we have a
  connected one (and we know whether it's connected or not based on whether we
  saw it as a csg-cmp pair in the algorithm earlier).

  Third, due to the way we grow our subgraph, only the nodes that we have just
  grown by can contribute to the E↓'(S,X). The reason is simple; every node
  from the previous neighborhood will have been added either to S or to X,
  and both exclude them from the new neighborhood. (Step 2 doesn't affect this,
  as any hypernode that was subsumed would also have to touch S or X.
  But there's an exception in that occasionally, we can remove nodes from X;
  see ExpandSubgraph().)

  Fourth, perfect minimization seems to be impossible to actually implement
  efficiently. This is known as the minimum set problem, and the best known
  algorithms to do this are in O(n² / log n) time (see e.g. Pritchard: ”An Old
  Sub-Quadratic Algorithm for Finding Extremal Sets”), which can be quite a
  lot when there are lots of edges. (The trivial O(n²) algorithm is to just test
  every set against every smaller set, and throw it out if it's a superset.)
  Since loops in our hypergraphs are presumed to be fairly rare, it would not
  seem worth it to do full minimization.

  Instead, we pick the low-hanging fruit only: Every _simple_ edge is trivial
  to test against. We just collect the simple edges into a mask, and any
  (complex) hyperedge that overlaps with that bitmap can immediately be
  discarded. Even more, since we don't have to pick min(S) but can pick
  something arbitrary, we can let {R2,R3} (which gets R2 as its representative
  node) subsume {R1,R2}, even though it's not an actual subset, by pretending
  we picked R2 as the representative node for the latter! This is similar to
  what Moerkotte describes in his “Building Query Compilers” document,
  which seems to contain a slightly extended version of the DPhyp paper
  (under a different name). We could have collected all the simple edges in a
  separate pass first, but the microbenchmarks show that the added loop overhead
  isn't worth it.

  Note that we also keep E↓'(S,X), the set of interesting hypernodes; we
  bitwise-or it into “full_neighborhood”. This is useful later when searching
  for edges to connect the connected subgraph and its complement; we know that
  only edges into “full_neighborhood” can connect the two.

  This function accounts for roughly 20–70% of the total DPhyp running time,
  depending on the shape of the graph (~40% average across the microbenchmarks).
  It is fairly big to inline, but it helps speed significantly, probably due
  to the large amount of parameters to be passed back and forth.
 */// 返回 subgraph 的直连邻居，不含间接邻居，也不包含本身的任何元素。这函数参数中的 just_grown_by ⊆ subgraph。
inline NodeMap FindNeighborhood(int depth, const Hypergraph &g, NodeMap subgraph, 
                                NodeMap forbidden, NodeMap just_grown_by,
                                NeighborhoodCache *cache,
                                NodeMap *full_neighborhood_arg) {
  HYPERGRAPH_PRINTF(
      "%sFindNeighborhood(subgraph:%s, forbidden:%s, just_grown_by:%s, full_neighborhood_arg:%s),     &cache:%p, cache.m_taboo_bit:%s\n",
      std::string(depth * 4, ' ').c_str(), PrintSet(subgraph).c_str(), PrintSet(forbidden).c_str(), PrintSet(just_grown_by).c_str(), 
      PrintSet(*full_neighborhood_arg).c_str(), (void*)cache, PrintSet(cache->m_taboo_bit).c_str());

  assert(IsSubset(just_grown_by, subgraph));

  NodeMap full_neighborhood =
      *full_neighborhood_arg;  // Keep us out of aliasing trouble.
  NodeMap neighborhood = 0;

  HYPERGRAPH_PRINTF(
      "%s FindNeighborhood     the last cache:{m_last_just_grown_by:%s, m_last_full_neighborhood:%s, m_last_neighborhood:%s, m_taboo_bit:%s},      &cache:%p\n",
       std::string(depth * 4, ' ').c_str(), PrintSet(cache->m_last_just_grown_by).c_str(), 
       PrintSet(cache->m_last_full_neighborhood).c_str(), PrintSet(cache->m_last_neighborhood).c_str(), PrintSet(cache->m_taboo_bit).c_str(), (void*)cache);

  NodeMap to_search =
      cache->InitSearch(just_grown_by, &neighborhood, &full_neighborhood);

  HYPERGRAPH_PRINTF("%s FindNeighborhood     to_search:%s, cache.m_taboo_bit:%s,      &cache:%p\n",
                  std::string(depth * 4, ' ').c_str(), PrintSet(to_search).c_str(), PrintSet(cache->m_taboo_bit).c_str(), (void*)cache);
  assert(IsSubset(neighborhood, full_neighborhood));

  for (size_t node_idx : BitsSetIn(to_search)) {
    // Simple edges.
    // NOTE: This node's simple neighborhood will be added lazily to
    // full_neighborhood below. Forbidden nodes will also be removed below.
    neighborhood |= g.nodes[node_idx].simple_neighborhood;

    // Now go through the complex edges and see which ones point out of the
    // subgraph.
    for (size_t edge_idx : g.nodes[node_idx].complex_edges) {
      const Hyperedge e = g.edges[edge_idx];

      if (IsSubset(e.left, subgraph) &&
          !Overlaps(e.right, subgraph | forbidden)) {
        // e.right is an interesting hypernode (part of E↓'(S,X)).
        full_neighborhood |= e.right;
        if (!Overlaps(e.right, neighborhood)) {
          // e.right is also not subsumed by another edge (ie., it is part of
          // E↓(S,X)), so add a “representative node” for it to the
          // neighborhood.
          //
          // Is is possible to do the Overlaps() test above branch-free by using
          // -int64_t(e.right & neighborhood) >> 63 as a mask (assuming we do
          // not have more than 63 tables) but it seems to do better on some
          // tests and worse on others, so it's not worth it.
          neighborhood |= IsolateLowestBit(e.right);
        }
      }
    }
  }

  neighborhood &= ~(subgraph | forbidden);
  full_neighborhood |= neighborhood;

  HYPERGRAPH_PRINTF(
      "%s FindNeighborhood     Neighborhood of %s (calculated on %s) with forbidden %s = %s,     &cache:%p, cache.m_taboo_bit:%s\n",
      std::string(depth * 4, ' ').c_str(), PrintSet(subgraph).c_str(), PrintSet(just_grown_by).c_str(),
      PrintSet(forbidden).c_str(), PrintSet(neighborhood).c_str(), (void*)cache, PrintSet(cache->m_taboo_bit).c_str());

  cache->Store(depth + 1, just_grown_by, neighborhood, full_neighborhood);

  *full_neighborhood_arg = full_neighborhood;
  return neighborhood;
}

// Given a subgraph of g, enumerate all possible complements that do
// not include anything from the exclusion subset. Works by looking
// at every possible node of the _neighborhood_ of the given subgraph
// (see FindNeighborhood()); these are then used as seeds for growing
// the complement graph.
//
// Called EmitCsg() in the DPhyp paper.
// 这个函数的总体流程：把 neighborhood 的每个节点当 complement 的种子，先计算 complement = seed 的初始值，然后调用 ExpandComplement 从 seed 开始扩展 complement
// subgraph 是连通的才会调用这个函数
// 返回 false 表示可以继续，返回 true 表示需要终止
template <class Receiver>
[[nodiscard]] bool EnumerateComplementsTo(
    int depth, const Hypergraph &g, size_t lowest_node_idx, NodeMap subgraph,
    NodeMap full_neighborhood, NodeMap neighborhood, Receiver *receiver) {
  HYPERGRAPH_PRINTF("%sEnumerateComplementsTo(lowest_node_idx:%ld, subgraph:%s, full_neighborhood:%s, neighborhood:%s)\n",
                    std::string(depth * 4, ' ').c_str(), lowest_node_idx, PrintSet(subgraph).c_str(), PrintSet(full_neighborhood).c_str(), PrintSet(neighborhood).c_str());

  PrintDynamicTable(lowest_node_idx, /*subgraph*/subgraph, /*forbidden*/0, /*Neighborhood*/neighborhood, /*Complement*/0, /*FunctionName*/"EnumerateComplementsTo");

  NodeMap forbidden = TablesBetween(0, lowest_node_idx);

  neighborhood &= ~subgraph;  // 这个处理会不会多余，neighborhood 跟 subgraph 怎么会有交集？在 ExpandSubgraph 中会把 forbidden 添加到 subgraph 中，但这个影响是什么？

  // Similar to EnumerateAllConnectedPartitions(), we start at seed nodes
  // counting _backwards_, so that we consider larger and larger potential
  // graphs. This is critical for the property that we want to enumerate smaller
  // subsets before larger ones.
  NeighborhoodCache* cache = new NeighborhoodCache(neighborhood);

  HYPERGRAPH_PRINTF("%s EnumerateComplementsTo     init cache(neighborhood:%s), cache.m_taboo_bit:%s,     &cache:%p\n",
                  std::string(depth * 4, ' ').c_str(), PrintSet(neighborhood).c_str(), PrintSet(cache->m_taboo_bit).c_str(), (void*)cache);

  for (size_t seed_idx : BitsSetInDescending(neighborhood)) { // 后序遍历 neighborhood 的所有 bits。为什么是遍历直接邻居，而不是 （subgraph ∪ forbidden）在 g 中的补集。此时 forbidden ∩ subgraph = ∅
    // First consider a complement consisting solely of the seed node;
    // see if we can find an edge (or multiple ones) connecting it
    // to the given subgraph.
    NodeMap seed = TableBitmap(seed_idx);
    // Complement 为单点时，可以用下面的办法搜索连接谓词。如果 Complement 非单点，则需要用到 TryConnecting 函数来搜索
    if (Overlaps(g.nodes[seed_idx].simple_neighborhood, subgraph)) {  // 如果该种子节点跟 subgraph 之间有简单边
      for (size_t edge_idx : g.nodes[seed_idx].simple_edges) {  // 遍历该种子节点的所有简单边
        const Hyperedge e = g.edges[edge_idx];
        assert(e.left == seed);
        if (Overlaps(e.right, subgraph)) {  // 如果简单边的右边跟 subgraph 相连
          // 如果 FoundSubgraphPair 返回 false 则表示可以找到 subgraph 跟 seed node 之间的连接谓词。 这里相当动态规划中的保存子集的计算结果。
          // 这里 edge_idx / 2 的原因，是因为每个连接谓词都会生成对两条边(比如R1.id=R2.id会生成{R1,R2}和{R2,R1})，所以除以 2 后可以找到对应的连接谓词
          if (receiver->FoundSubgraphPair(subgraph, seed, edge_idx / 2)) {
            return true;
          } else {            
            HYPERGRAPH_PRINTF("Complete EnumerateComplementsTo csg-cmp-pair: {%s}-{%s}\n",
                        PrintSet(subgraph).c_str(), PrintSet(seed).c_str());
          }        
        }
      }
    }
    for (size_t edge_idx : g.nodes[seed_idx].complex_edges) { // 遍历该种子的超边
      const Hyperedge e = g.edges[edge_idx];
      if (e.left == seed && IsSubset(e.right, subgraph)) {  // 如果 seed node 的超点 跟 subgraph 相连
        if (receiver->FoundSubgraphPair(subgraph, seed, edge_idx / 2)) {  // 保存动态规划中的子集的计算结果
          return true;
        } else {            
            HYPERGRAPH_PRINTF("Complete EnumerateComplementsTo csg-cmp-pair: {%s}-{%s}\n",
                        PrintSet(subgraph).c_str(), PrintSet(seed).c_str());
        }
      }
    }

    // Grow the complement candidate along the neighborhoods to create
    // a larger, connected complement. Note that we do this even if the
    // the seed complement wasn't connected to our subgraph, since it
    // might be connected as we add more nodes.
    //
    // Note that the extension of the forbidden set is required to avoid
    // enumerating the same set twice; consider e.g. if you have a clique
    // R1-R2-R3 and want to find complements to {R1} (ie., {R2,R3} is the
    // neighborhood). When considering the seed {R3}, you don't want to be able
    // to grow it into R2, since the {R2,R3} combination will be seen later when
    // using {R2} as the seed. This is analogous to what we do in
    // EnumerateAllConnectedPartitions(), and the whole reason for iterating
    // backwards, but the DPhyp paper misses this. The “Building Query
    // Compilers” document, however, seems to have corrected it.
    // 在《Building Query Compilers》 这本书的第78页有处理 new_forbidden 的伪代码
    NodeMap new_forbidden =
        // 为了防止遍历重复的 complement，需要把 neighborhood 集合中的 seed_idx 之前的子集合屏蔽，也就是把 neighborhood 的前半部分添加进 new_forbidden        
        forbidden | subgraph | (neighborhood & TablesBetween(0, seed_idx));
    NodeMap new_full_neighborhood = 0;  // Unused; see comment on TryConnecting.
    // 计算 seed 的直接邻居。
    NodeMap new_neighborhood = FindNeighborhood(depth + 1, g, /*subgraph*/seed, /*forbidden*/new_forbidden, /*just_grown_by*/seed,
                                                cache, /**full_neighborhood_arg*/&new_full_neighborhood);
    // 每次进入 ExpandComplement函数 之前，都会把需要用到的参数先计算好，再调用该ExpandComplement函数。 了解这个对我们分析打印日志信息很有帮助。
    if (ExpandComplement(depth + 1, g, /*lowest_node_idx*/lowest_node_idx, /*subgraph*/subgraph, /*subgraph_full_neighborhood*/full_neighborhood, /*complement*/seed,
                         /*neighborhood*/new_neighborhood, /*forbidden*/new_forbidden, receiver)) {
      return true;
    }
  }
  return false;
}

// Given a subgraph of g, grow it recursively along the neighborhood.
// (The subgraph is not necessarily connected, but we hope it eventually
// will be, or it won't be of much use to us.) If the subgraph is connected,
// use it as base for enumerating a complement graph before growing it.
//
// Called EnumerateCsgRec() in the paper.
// 沿着 neighborhood 扩展 subgraph，如果新的 grown_subgraph 连通，则调用 EnumerateComplementsTo。然后继续扩展 subgraph
// 总共有两个地方调用 ExpandSubgraph，一个是 EnumerateAllConnectedPartitions， 另外一个是 ExpandSubgraph 递归。
// EnumerateAllConnectedPartitions 调用 ExpandSubgraph 时，subgraph ⊆ forbidden。
// ExpandSubgraph 递归时，subgraph ∩ forbidden = ∅。 
// 返回 false 表示可以继续，返回 true 表示需要终止
template <class Receiver>
[[nodiscard]] bool ExpandSubgraph(int depth, const Hypergraph &g, size_t lowest_node_idx,
                                  NodeMap subgraph, NodeMap full_neighborhood,
                                  NodeMap neighborhood, NodeMap forbidden,
                                  Receiver *receiver) {
  HYPERGRAPH_PRINTF(
      "%sExpandSubgraph(lowest_node_idx:%ld, subgraph:%s, full_neighborhood:%s, neighborhood:%s, forbidden:%s)\n",
      std::string(depth * 4, ' ').c_str(), lowest_node_idx, PrintSet(subgraph).c_str(), PrintSet(full_neighborhood).c_str(), PrintSet(neighborhood).c_str(),
      PrintSet(forbidden).c_str());

  PrintDynamicTable(lowest_node_idx, /*subgraph*/subgraph, /*forbidden*/forbidden, /*Neighborhood*/neighborhood, /*Complement*/0, /*FunctionName*/"ExpandSubgraph");

  // Given a neighborhood, try growing our subgraph by all possible
  // combinations of included/excluded (except the one where all are
  // excluded).
  NeighborhoodCache* cache = new NeighborhoodCache(neighborhood);
  HYPERGRAPH_PRINTF("%s ExpandSubgraph     init cache(neighborhood:%s), cache.m_taboo_bit:%s,     &cache:%p\n",
                  std::string(depth * 4, ' ').c_str(), PrintSet(neighborhood).c_str(), PrintSet(cache->m_taboo_bit).c_str(), (void*)cache);
  // 枚举 neighborhood 的所有非空子集，包含 neighborhood 本身
  for (NodeMap grow_by : NonzeroSubsetsOf(neighborhood)) {
    HYPERGRAPH_PRINTF(
        "%s ExpandSubgraph     ->EnumerateComplementsTo     grown_subgraph={%s+%s}, grow_by:%s is subset of neighborhood:%s, [connected=%d]\n",
        std::string(depth * 4, ' ').c_str(), PrintSet(subgraph).c_str(), PrintSet(grow_by).c_str(),  PrintSet(grow_by).c_str(),
        PrintSet(neighborhood).c_str(), receiver->HasSeen(subgraph | grow_by));

    // See if the new subgraph is connected. The candidate subgraphs that are
    // connected will previously have been seen as csg-cmp-pairs, and thus, we
    // can ask the receiver!
    NodeMap grown_subgraph = subgraph | grow_by;
    // 为什么沿着 subgraph 的 neighborhood 扩展，还要判断 grown_subgraph 是否连通
    // 我来分析两种情况
    // 1、如果 subgraph 跟 grow_by 之间存在简单边，那么 grown_subgraph 肯定连通，则下面的判断显得多余
    // 2、如果 subgraph 跟 grow_by 之间是超边连接，是需要判断的。原因是：假设 {R2,R3}-{R4,R5} 是个超边（可以推出在数据结构表示中 R4 属于 {R2,R3} 的邻居），
    //    我们现在开始处理 grown_subgraph = {R2,R3,R4}，这个时候 grown_subgraph 未必连通,所以我们需要从 receiver 中查看它是否连通
    if (receiver->HasSeen(grown_subgraph)) {
      // Find the neighborhood of the new subgraph.
      NodeMap new_full_neighborhood = full_neighborhood;
      NodeMap new_neighborhood =
          FindNeighborhood(depth + 1, g, /*subgraph*/subgraph | grow_by, /*forbidden*/forbidden, /*just_grow_by*/grow_by, cache,
                           /**full_neighborhood_arg*/&new_full_neighborhood);

      // EnumerateComplementsTo() resets the forbidden set, since nodes that
      // were forbidden under this subgraph may very well be part of the
      // complement. However, this also means that the neighborhood we just
      // computed may be incomplete; it just looks at recently-added nodes,
      // but there are older nodes that may have neighbors that we added to
      // the forbidden set (X) instead of the subgraph itself (S). However,
      // this is also the only time we add to the forbidden set, so we know
      // exactly which nodes they are! Thus, simply add our forbidden set
      // to the neighborhood for purposes of computing the complement.
      //
      // This behavior is tested in the SmallStar unit test.

      // 因为 EnumerateComplementsTo 中会重置 forbidden，这样会导致 (subgraph 中的 forbidden) 会成为 (complement 的一部分)
      // 所以我们会把 forbidden 添加到 new_neighborhood ，然后才交给 EnumerateComplementsTo 处理 new_neighborhood
      // 那怎么证明这种做法不影响 EnumerateComplementsTo 正确性？现在我们只能保证在 EnumerateComplementsTo 中的 forbidden 的正确性，
      // 还不能保证 EnumerateComplementsTo 中的 complement 的正确性(因为在 EnumerateComplementsTo 中，complement 需要遍历现在的 new_neighborhood 获取，明显影响到了 complement)。
      // 重新阅读 EnumerateComplementsTo，会发现它的 neighborhood 只参与了 seed 的遍历、和 new_forbidden 的赋值，这两种情况都不会影响代码的正确性、完备性。

      // 对应的测试用例 unittest/gunit/dphyp-t.cc 中的 TEST(DPhypTest, SmallStar)。测试命令（路径需要根据环境进行调整）：
      // cmake --build /data/mysql-server-8.2.0/build --config Debug --target dphyp-t -j 6
      // /data/mysql-server-8.2.0/build/runtime_output_directory/dphyp-t --gtest_filter=DPhypTest.SmallStar

      // 上面分析的过程中，可能会有人好奇 forbidden 除了 TablesBetween(0, lowest_node_idx)，还有哪些场景其它 node 也会被加入 forbidden。
      // 整个算法只有两个地方调用 ExpandSubgraph：
      // 一个是 EnumerateAllConnectedPartitions 中，它的 forbidden 包含 subgraph，也就是 subgraph ⊆ forbidden。 
      // 另外一个是 ExpandSubgraph 本身的递归中，它会把 subgraph 从 forbidden 中剔除，也就是 subgraph ∩ forbidden = ∅
      new_neighborhood |= forbidden & ~TablesBetween(0, lowest_node_idx);

      // This node's neighborhood is also part of the new neighborhood
      // it's just not added to the forbidden set yet, so we missed it in
      // the previous calculation).
      new_neighborhood |= neighborhood;
      if (EnumerateComplementsTo(depth + 1, g, /*lowest_node_idx*/lowest_node_idx, /*subgraph*/grown_subgraph,
                                 /*full_neighborhood*/new_full_neighborhood, /*neighborhood*/new_neighborhood,
                                 receiver)) {
        return true;
      }
    }
  }
  

  // Now try to grow all the grown subgraphs into larger, connected subgraphs.
  // Note that we do this even if the grown subgraph isn't connected, since it
  // might be connected as we add more nodes.
  //
  // We need to do this after EnumerateComplementsTo() has run on all of them
  // (in turn, generating csg-cmp-pairs and calling FoundSubgraphPair()),
  // to guarantee that we will see any smaller subgraphs before larger ones.
  // 如果 grow_by 都是 简单边的点集合，那么 grown_subgraph 肯定是连通的, 否则 grown_subgraph 未必连通
  // 不管连不连通，我们都需要继续扩展 subgraph，因为扩展后可能会连通
  for (NodeMap grow_by : NonzeroSubsetsOf(neighborhood)) {
                      
    HYPERGRAPH_PRINTF(
        "%s ExpandSubgraph     ->ExpandSubgraph     grown_subgraph={%s+%s}, grow_by:%s is subset of neighborhood:%s, [connected=%d]\n",
        std::string(depth * 4, ' ').c_str(), PrintSet(subgraph).c_str(), PrintSet(grow_by).c_str(),  PrintSet(grow_by).c_str(),
        PrintSet(neighborhood).c_str(), receiver->HasSeen(subgraph | grow_by));

    NodeMap grown_subgraph = subgraph | grow_by;

    // Recursive calls are not allowed to add any of the nodes from
    // our current neighborhood, since we're already trying all
    // combinations of those ourselves.
    // 我们目标是快速搜索所有的 csg-cmp-pairs，因为在当前的循环体会组合 neighborhood 所有子集到 csg，所以进入下一轮的递归之前需要把 neighborhood 添加到 new_forbidden。
    // 这里的 & ~grown_subgraph 等价把 grown_subgraph 剔除。为什么要剔除呢？

    NodeMap new_forbidden = (forbidden | neighborhood) & ~grown_subgraph;
    assert(!IsSubset(grown_subgraph, new_forbidden));

    // Find the neighborhood of the new subgraph.
    NodeMap new_full_neighborhood = full_neighborhood;
    NodeMap new_neighborhood =  // 剔除上面的 grown_subgraph 不会影响这里的 new_neighborhood
        FindNeighborhood(depth + 1, g, /*subgraph*/subgraph | grow_by, /*forbidden*/new_forbidden, /*just_grow_by*/grow_by, cache,
                         /**full_neighborhood_arg*/&new_full_neighborhood);
    if (ExpandSubgraph(depth + 1, g, /*lowest_node_idx*/lowest_node_idx, /*subgraph*/grown_subgraph,
                       /*full_neighborhood*/new_full_neighborhood, /*neighborhood*/new_neighborhood, /*forbidden*/new_forbidden,
                       receiver)) {
      return true;
    }
  }
  return false;
}

// Given a connected subgraph and a connected complement, see if they are
// connected through some edge, and if so, which edge. (They may be connected
// through multiple edges if there are loops in the graph.)
//
// In order to reduce the amount of searching for a connecting edge, we can use
// the information about the subgraph's full neighborhood that we've been
// connecting earlier. (This helps ~20% on the chain benchmark, and more on the
// hypercycle benchmark.) The edge must touch something that's immediately
// reachable from the subgraph (pretty much by definition), so we don't need to
// look in all the nodes in the complement; those not in the subgraph's full
// neighborhood cannot contain such edges.
//
// We could probably have kept full neighborhoods for both the subgraph and the
// complement, and picked the one with fewest nodes to study, but it doesn't
// seem to be worth it.
template <class Receiver>
// 返回 false 表示可以继续，返回 true 表示需要终止
// 扩展 complement 时，则判断新的 complement 跟 subgraph 是否连通，如果新增的集合是单点，逻辑在 EnumerateComplementsTo 中，如果新增的集合是多个点，则判断逻辑在在这里
[[nodiscard]] bool TryConnecting(int depth, const Hypergraph &g, NodeMap subgraph,
                                 NodeMap subgraph_full_neighborhood,
                                 NodeMap complement, Receiver *receiver) {
  HYPERGRAPH_PRINTF(
      "%sTryConnecting(subgraph:%s, subgraph_full_neighborhood:%s, complement:%s)\n",
      std::string(depth * 4, ' ').c_str(), PrintSet(subgraph).c_str(), PrintSet(subgraph_full_neighborhood).c_str(), PrintSet(complement).c_str());

  for (size_t node_idx : BitsSetIn(complement & subgraph_full_neighborhood)) {  // 利用 subgraph_full_neighborhood 可以提升搜索效率
    // complement 连通，并不代表找到 csg-cmp-pairs,还是需要先从超图上判断是否连通，然后再到receiver中确认是否满足冲突规则规则
    // Simple edges.
    if (Overlaps(g.nodes[node_idx].simple_neighborhood, subgraph)) {  // 判断是否有简单边连接，逻辑跟 EnumerateComplementsTo 的简单边判断完全一样
      for (size_t edge_idx : g.nodes[node_idx].simple_edges) {
        // The tests are really IsSubset(), but Overlaps() is equivalent
        // here, and slightly faster.
        const Hyperedge e = g.edges[edge_idx];
        if (Overlaps(e.right, subgraph) && Overlaps(e.left, complement)) {
          if (receiver->FoundSubgraphPair(subgraph, complement, edge_idx / 2)) {
            return true;
          }
        }
      }
    }

    // Complex edges.
    NodeMap node = TableBitmap(node_idx);
    for (size_t edge_idx : g.nodes[node_idx].complex_edges) {   // 判断是否有超边连接，逻辑跟 EnumerateComplementsTo 的超边判断基本一样，不一样的地方是需要判断 IsolateLowestBit(e.left) == node
      const Hyperedge e = g.edges[edge_idx];

      // NOTE: We call IsolateLowestBit() so that we only see the edge once.
      if (IsolateLowestBit(e.left) == node && IsSubset(e.left, complement) &&
          IsSubset(e.right, subgraph)) {
        if (receiver->FoundSubgraphPair(subgraph, complement, edge_idx / 2)) {
          return true;
        }
      }
    }
  }
  return false;
}

// Very similar to ExpandSubgraph: Given a connected subgraph of g and
// another subgraph of g (its complement; not necessarily connected),
// grow the complement recursively along the neighborhood. The former
// subgraph stays unchanged through the recursion, while the second
// is grown. If the complement at any point gets connected, see if we
// can find a connection between the connected subgraph and complement;
// if so, they form a so-called csg-cmp-pair. We tell the receiver about the
// csg-cmp-pair, not only because it is the entire goal of the algorithm,
// but because it will allow us to remember for later that the csg-cmp-pair
// is connected. (This is used for connectivity testing, both in
// ExpandSubgraph() and ExpandComplement().)
//
// Called EnumerateCmpRec() in the paper.
// 沿着 neighborhood 扩展 complement，如果不连通，一直扩展下去，肯定会连通，因为如果是超点，那么扩展的方向不会错
// 参数中的 complement 会包含上次递归的值，所以可以用来判断 csg-cmp-pair
// 返回 false 表示可以继续，返回 true 表示需要终止
template <class Receiver>
[[nodiscard]] bool ExpandComplement(int depth, const Hypergraph &g, size_t lowest_node_idx,
                                    NodeMap subgraph,
                                    NodeMap subgraph_full_neighborhood,
                                    NodeMap complement, NodeMap neighborhood,
                                    NodeMap forbidden, Receiver *receiver) {
  HYPERGRAPH_PRINTF(
      "%s ExpandComplement(lowest_node_idx:%ld, subgraph:%s, subgraph_full_neighborhood:%s, complement:%s, neighborhood:%s, forbidden:%s)\n",
      std::string(depth * 4, ' ').c_str(), lowest_node_idx, PrintSet(subgraph).c_str(), PrintSet(subgraph_full_neighborhood).c_str(),
      PrintSet(complement).c_str(), PrintSet(neighborhood).c_str(), PrintSet(forbidden).c_str());

  PrintDynamicTable(lowest_node_idx, /*subgraph*/subgraph, /*forbidden*/forbidden, /*Neighborhood*/neighborhood, /*Complement*/complement, /*FunctionName*/"ExpandComplement");

  assert(IsSubset(subgraph, forbidden));
  assert(!IsSubset(complement, forbidden));

  // Given a neighborhood, try growing our subgraph by all possible
  // combinations of included/excluded (except the one where all are
  // excluded).
  //
  // The only difference from ExpandSubgraph() here is that when we
  // find a connected complement (and thus have two disjoint, connected
  // subgraphs), we don't need to recurse to find a third subgraph;
  // we can just check whether they are connected, and if so, tell
  // the receiver.
  // 按{v1}，{v2}，{v1，v2}，{v3}，{v1，3}，{v2，v3}，{v1，v2，v3} …… 从小到大的顺序枚举 neighborhood 的所有非空子集，包含 neighborhood 本身
  for (NodeMap grow_by : NonzeroSubsetsOf(neighborhood)) {
    NodeMap grown_complement = complement | grow_by;
    // 如果之前处理过 grown_complement，则保存中间的计算结果
    // 比如之前处理过 {R2,R3}-{R4,R5,R6},现在开始处理{R2,R3,R4}-{R5,R6}，这个时候要计算哪个路径更优并保存
    if (receiver->HasSeen(grown_complement)) {//判断grown_complement是否连通，只需要到receiver中去判断，不需要像在 EnumerateComplementsTo 中通过超图的计算来判断，因为更大的lowest_node_idx中肯定会计算过
      if (TryConnecting(depth + 1, g, subgraph, subgraph_full_neighborhood, // 尝试连接 subgraph 和 grown_complement
                        grown_complement, receiver)) {
        return true;
      } else {
        HYPERGRAPH_PRINTF("Complete  ExpandComplement csg-cmp-pair: {%s}-{%s}\n",
                  PrintSet(subgraph).c_str(), PrintSet(grown_complement).c_str());
      }
    }
  }

  // Same logic as in ExpandSubgraph:
  //
  // Try to grow all the grown complements into larger, connected complements.
  // Note that we do this even if the grown complement isn't connected, since it
  // might be connected as we add more nodes.
  //
  // We need to do this after FoundSubgraphPair() has run on all of them,
  // to guarantee that we will see any smaller subgraphs before larger ones.
  NeighborhoodCache* cache = new NeighborhoodCache(neighborhood);
  HYPERGRAPH_PRINTF("%s ExpandComplement     init cache(neighborhood:%s), cache.m_taboo_bit:%s,     &cache:%p\n",
                  std::string(depth * 4, ' ').c_str(), PrintSet(neighborhood).c_str(), PrintSet(cache->m_taboo_bit).c_str(), (void*)cache);

  for (NodeMap grow_by : NonzeroSubsetsOf(neighborhood)) {
    HYPERGRAPH_PRINTF(
        "%s ExpandComplement     ->sExpandComplement     grown_complement={%s+%s}, grow_by:%s is subset of neighborhood:%s\n",
        std::string(depth * 4, ' ').c_str(), PrintSet(complement).c_str(), PrintSet(grow_by).c_str(),  PrintSet(grow_by).c_str(),
        PrintSet(neighborhood).c_str());

    NodeMap grown_complement = complement | grow_by;

    // Recursive calls are not allowed to add any of the nodes from
    // our current neighborhood, since we're already trying all
    // combinations of those ourselves.
    // 比如现在的 complement={R5} neighborhood = {R3 R6 R8 R9} 在循环体体，第I次循环 grow_by = {R3 R6 R8} 第j次循环 grow_by = {R6 R8 R9}，
    // 那么以后不管第I次循环 和 第j次循环 迭代多少次， 扩展的出来的 complement 都不一样，因为 i 有 R3, j 永远不会有 R3(因为j禁止了R3)。但每个循环体中覆盖所有，所以满足完备、正确
    NodeMap new_forbidden = (forbidden | neighborhood) & ~grown_complement;
    assert(!IsSubset(grown_complement, new_forbidden));

    // Find the neighborhood of the new complement.
    NodeMap new_full_neighborhood = 0;  // Unused; see comment on TryConnecting.
    NodeMap new_neighborhood =
        FindNeighborhood(depth + 1, g, /*subgraph*/complement | grow_by, /*forbidden*/new_forbidden, /*just_grown_by*/grow_by,
                         cache, /**full_neighborhood_arg*/&new_full_neighborhood);
    if (ExpandComplement(depth + 1, g, /*lowest_node_idx*/lowest_node_idx, /*subgraph*/subgraph,
                         /*subgraph_full_neighborhood*/subgraph_full_neighborhood, /*complement*/grown_complement,
                         /*neighborhood*/new_neighborhood, /*forbidden*/new_forbidden, receiver)) {
      return true;
    }
  }
  
  return false;
}

// Consider increasing subsets of the graph, backwards; first only the
// last node (say, R6), then {R5,R6} with R5 as the seed, then
// {R4,R5,R6} with R4 as the seed, and so on. From the single-node
// seed, we grow the connected subgraph recursively into new connected
// subgraphs; when we such a new subgraph (the paper calls it a csg),
// we do two things with it:
//
//   1. Keep growing it into new and even larger subgraphs.
//   2. Look for _another_, separate subgraph (the paper calls it a
//      complement, or cmp) that can be connected to our subgraph.
//      If we find one such pair (a csg-cmp-pair), that's what the
//      algorithm fundamentally is looking for.
//
// Called Solve() in the DPhyp paper.(void*)&cache
//
// If at any point receiver->FoundSingleNode() or receiver->FoundSubgraphPair()
// returns true, the algorithm will abort, and this function also return true.
template <class Receiver>
bool EnumerateAllConnectedPartitions(const Hypergraph &g, Receiver *receiver) {
  PrintDynamicTableHeader();
  PrintDynamicTableLine();
  for (int seed_idx = g.nodes.size() - 1; seed_idx >= 0; --seed_idx) {
    if (receiver->FoundSingleNode(seed_idx)) {
      return true;
    }

    NodeMap seed = TableBitmap(seed_idx);
    HYPERGRAPH_PRINTF("\n\n EnumerateAllConnectedPartitions     Starting main iteration at node %s\n",
                      PrintSet(seed).c_str());

    // HYPERGRAPH_PRINTF("Starting main iteration at node %s\n",
    //                   PrintSet(seed).c_str());

    NodeMap forbidden = TablesBetween(0, seed_idx);
    NodeMap full_neighborhood = 0;
    NeighborhoodCache* cache = new NeighborhoodCache(0);
    HYPERGRAPH_PRINTF("%s EnumerateAllConnectedPartitions     init cache(neighborhood:%s), cache.m_taboo_bit:%s,     &cache:%p\n",
                  std::string(0, ' ').c_str(), PrintSet(0).c_str(), PrintSet(cache->m_taboo_bit).c_str(), (void*)cache);

    NodeMap neighborhood =
        FindNeighborhood(1, g, /*subgraph*/seed, /*forbidden*/forbidden, /*just_grown_by*/seed, cache, /**full_neighborhood_arg*/&full_neighborhood);  // 第一次计算 neighborhood，由于之前没缓存，所以这行里的 just_grown_by = subgraph
    // 分析之前需要了解：
    // 后续遇到的函数FoundSubgraphPair 返回 false 则表示可以找到 subgraph 跟 seed node 之间的连接谓词
    if (EnumerateComplementsTo(1, g, /*lowest_node_idx*/seed_idx, /*subgraph*/seed, /*full_neighborhood*/full_neighborhood,
                               /*neighborhood*/neighborhood, receiver)) {
      return true;
    }
    if (ExpandSubgraph(1, g, /*lowest_node_idx*/seed_idx, /*subgraph*/seed, /*full_neighborhood*/full_neighborhood, /*neighborhood*/neighborhood, // 进入 ExpandSubgraph 内部之前，先计算好  forbidden
                       /*forbidden*/forbidden | seed, receiver)) {
      return true;
    }
  }
  PrintDynamicTableLine();
  //printf("%s", PrintDottyHypergraph(receiver->m_graph).c_str());
  return false;
}

}  // namespace hypergraph

#endif  // SUBGRAPH_ENUMERATION_H