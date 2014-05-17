package ru.getsk.common.algorithm;

/**************************************************************************
 * File: Dijkstra.java
 * Based on Dijkstra implementation of Keith Schwarz (htiek@cs.stanford.edu)
 * Modified by Sergey Kanukov (s.kanukov@gmail.com)
 *
 * An implementation of Dijkstra's single-source shortest path algorithm.
 * The algorithm takes as input a directed graph with non-negative edge
 * costs and a source node, then computes the shortest path from that node
 * to each other node in the graph.
 *
 * The algorithm works by maintaining a priority queue of nodes whose
 * priorities are the lengths of some path from the source node to the
 * node in question.  At each step, the algortihm dequeues a node from
 * this priority queue, records that node as being at the indicated
 * distance from the source, and then updates the priorities of all nodes
 * in the graph by considering all outgoing edges from the recently-
 * dequeued node to those nodes.
 *
 * In the course of this algorithm, the code makes up to |E| calls to
 * decrease-key on the heap (since in the worst case every edge from every
 * node will yield a shorter path to some node than before) and |V| calls
 * to dequeue-min (since each node is removed from the prioritiy queue
 * at most once).  Using a Fibonacci heap, this gives a very good runtime
 * guarantee of O(|E| + |V| lg |V|).
 *
 * This implementation relies on the existence of a FibonacciHeap class, also
 * from the Archive of Interesting Code.  You can find it online at
 *
 *         http://keithschwarz.com/interesting/code/?dir=fibonacci-heap
 */

import java.util.*;

import com.keithschwarz.FibonacciHeap;
import com.keithschwarz.DirectedGraph;
import org.apache.commons.lang3.tuple.*;

final class GraphPathNode<T> {
    private T mNode = null;
    private final List<T> mPath = new ArrayList<T>();

    public GraphPathNode(T node) {
        setNode(node);
        mPath.add(node);
    }

    public T getNode() {
        return mNode;
    }

    public void setNode(T node) {
        mNode = node;
    }

    public List<T> getPath() {
        return mPath;
    }

    public void prependPath(List<T> path) {
        mPath.clear();
        mPath.addAll(path);
        mPath.add(mNode);
    }
}

final class GraphPath<T> {
    private final List<GraphPathNode<T>> mNodes = new ArrayList<GraphPathNode<T>>();

    public void addNode(GraphPathNode<T> node) {
        mNodes.add(node);
    }

    public GraphPathNode<T> getNode(T value) {
        for (GraphPathNode<T> gn : mNodes) {
            if (gn.getNode().equals(value)) {
                return gn;
            }
        }
        return null;
    }
}

public final class Dijkstra {
    /**
     * Given a directed, weighted graph G and a source node s, produces the
     * distances from s to each other node in the graph.  If any nodes in
     * the graph are unreachable from s, they will be reported at distance
     * +infinity.
     *
     * @param graph The graph upon which to run Dijkstra's algorithm.
     * @param source The source node in the graph.
     * @return A map from nodes in the graph to their distances from the source.
     */
    public static <T> Map<T, Double> getShortestDistances(DirectedGraph<T> graph, T source) {
        /* Create a Fibonacci heap storing the distances of unvisited nodes
         * from the source node.
         */
        FibonacciHeap<T> pq = new FibonacciHeap<T>();

        /* The Fibonacci heap uses an internal representation that hands back
         * Entry objects for every stored element.  This map associates each
         * node in the graph with its corresponding Entry.
         */
        Map<T, FibonacciHeap.Entry<T>> entries = new HashMap<T, FibonacciHeap.Entry<T>>();

        /* Maintain a map from nodes to their distances.  Whenever we expand a
         * node for the first time, we'll put it in here.
         */
        Map<T, Double> result = new HashMap<T, Double>();

        /* Add each node to the Fibonacci heap at distance +infinity since
         * initially all nodes are unreachable.
         */
        for (T node : graph) {
            entries.put(node, pq.enqueue(node, Double.POSITIVE_INFINITY));
        }

        /* Update the source so that it's at distance 0.0 from itself; after
         * all, we can get there with a path of length zero!
         */
        pq.decreaseKey(entries.get(source), 0.0);

        /* Keep processing the queue until no nodes remain. */
        while (!pq.isEmpty()) {
            /* Grab the current node.  The algorithm guarantees that we now
             * have the shortest distance to it.
             */
            FibonacciHeap.Entry<T> curr = pq.dequeueMin();

            /* Store this in the result table. */
            result.put(curr.getValue(), curr.getPriority());

            /* Update the priorities of all of its edges. */
            for (Map.Entry<T, Double> arc : graph.edgesFrom(curr.getValue()).entrySet()) {
                /* If we already know the shortest path from the source to
                 * this node, don't add the edge.
                 */
                if (result.containsKey(arc.getKey())) continue;

                /* Compute the cost of the path from the source to this node,
                 * which is the cost of this node plus the cost of this edge.
                 */
                double pathCost = curr.getPriority() + arc.getValue();

                /* If the length of the best-known path from the source to
                 * this node is longer than this potential path cost, update
                 * the cost of the shortest path.
                 */
                FibonacciHeap.Entry<T> dest = entries.get(arc.getKey());
                if (pathCost < dest.getPriority()) {
                    pq.decreaseKey(dest, pathCost);
                }
            }
        }

        /* Finally, report the distances we've found. */
        return result;
    }

    /**
     * Given a directed, weighted graph G and a source node s, produces the
     * paths from s to each other node in the graph.  If any nodes in
     * the graph are unreachable from s, they will be reported at distance
     * +infinity.
     *
     * @param graph The graph upon which to run Dijkstra's algorithm.
     * @param source The source node in the graph.
     * @return A map from nodes in the graph to pair of nodes paths and distances from the source.
     */
    public static <T> Map<T, Pair<List<T>, Double>> getShortestPaths(DirectedGraph<T> graph, T source) {
        /* Create a Fibonacci heap storing the distances of unvisited nodes
         * from the source node.
         */
        FibonacciHeap<T> pq = new FibonacciHeap<T>();

        /* The Fibonacci heap uses an internal representation that hands back
         * Entry objects for every stored element.  This map associates each
         * node in the graph with its corresponding Entry.
         */
        Map<T, FibonacciHeap.Entry<T>> entries = new HashMap<T, FibonacciHeap.Entry<T>>();

        /* Maintain a map from nodes to their distances.  Whenever we expand a
         * node for the first time, we'll put it in here.
         */
        Map<T, Pair<List<T>, Double>> result = new HashMap<T, Pair<List<T>, Double>>();

        GraphPath<T> nodesGraph = new GraphPath<T>();

        /* Add each node to the Fibonacci heap at distance +infinity since
         * initially all nodes are unreachable.
         */
        for (T node : graph) {
            entries.put(node, pq.enqueue(node, Double.POSITIVE_INFINITY));
            nodesGraph.addNode(new GraphPathNode<T>(node));
        }

        /* Update the source so that it's at distance 0.0 from itself; after
         * all, we can get there with a path of length zero!
         */
        pq.decreaseKey(entries.get(source), 0.0);

        /* Keep processing the queue until no nodes remain. */
        while (!pq.isEmpty()) {
            /* Grab the current node.  The algorithm guarantees that we now
             * have the shortest distance to it.
             */
            FibonacciHeap.Entry<T> curr = pq.dequeueMin();
            GraphPathNode<T> curGraphNode = nodesGraph.getNode(curr.getValue());

            /* Store this in the result table. */
            result.put(curr.getValue(), new ImmutablePair<List<T>, Double>(curGraphNode.getPath(), curr.getPriority()));

            /* Update the priorities of all of its edges. */
            for (Map.Entry<T, Double> arc : graph.edgesFrom(curr.getValue()).entrySet()) {
                /* If we already know the shortest path from the source to
                 * this node, don't add the edge.
                 */
                if (result.containsKey(arc.getKey())) continue;

                /* Compute the cost of the path from the source to this node,
                 * which is the cost of this node plus the cost of this edge.
                 */
                double pathCost = curr.getPriority() + arc.getValue();

                /* If the length of the best-known path from the source to
                 * this node is longer than this potential path cost, update
                 * the cost of the shortest path.
                 */
                FibonacciHeap.Entry<T> dest = entries.get(arc.getKey());
                if (pathCost < dest.getPriority()) {
                    pq.decreaseKey(dest, pathCost);

                    GraphPathNode<T> destGraphNode = nodesGraph.getNode(dest.getValue());
                    if (curGraphNode != null && destGraphNode != null) {
                        destGraphNode.prependPath(curGraphNode.getPath());
                    }
                }
            }
        }

        /* Finally, report the result we've found. */
        return result;
    }

    /**
     * Given a directed, weighted graph G and source and destination nodes, produces the
     * path from source to destination. If any nodes in
     * the graph are unreachable from s, they will be reported at distance
     * +infinity.
     *
     * @param graph The graph upon which to run Dijkstra's algorithm.
     * @param source The source node in the graph.
     * @param destinstion The destination node in the graph.
     * @return A pair of nodes path and distance from the source.
     */
    public static <T> Pair<List<T>, Double> getShortestPath(DirectedGraph<T> graph, T source, T destinstion) {
    /* Create a Fibonacci heap storing the distances of unvisited nodes
         * from the source node.
         */
        FibonacciHeap<T> pq = new FibonacciHeap<T>();

        /* The Fibonacci heap uses an internal representation that hands back
         * Entry objects for every stored element.  This map associates each
         * node in the graph with its corresponding Entry.
         */
        Map<T, FibonacciHeap.Entry<T>> entries = new HashMap<T, FibonacciHeap.Entry<T>>();

        /* Maintain a map from nodes to their distances.  Whenever we expand a
         * node for the first time, we'll put it in here.
         */
        Map<T, Pair<List<T>, Double>> result = new HashMap<T, Pair<List<T>, Double>>();

        GraphPath<T> nodesGraph = new GraphPath<T>();

        /* Add each node to the Fibonacci heap at distance +infinity since
         * initially all nodes are unreachable.
         */
        for (T node : graph) {
            entries.put(node, pq.enqueue(node, Double.POSITIVE_INFINITY));
            nodesGraph.addNode(new GraphPathNode<T>(node));
        }

        /* Update the source so that it's at distance 0.0 from itself; after
         * all, we can get there with a path of length zero!
         */
        pq.decreaseKey(entries.get(source), 0.0);

        /* Keep processing the queue until no nodes remain. */
        while (!pq.isEmpty()) {
            /* Grab the current node.  The algorithm guarantees that we now
             * have the shortest distance to it.
             */
            FibonacciHeap.Entry<T> curr = pq.dequeueMin();
            GraphPathNode<T> curGraphNode = nodesGraph.getNode(curr.getValue());

            /* Store this in the result table. */
            ImmutablePair<List<T>, Double> p = new ImmutablePair<List<T>, Double>(curGraphNode.getPath(), curr.getPriority());
            if (curr.getValue().equals(destinstion)) {
                if (Double.isInfinite(p.getRight())) {
                    return null;
                }
                return p;
            }

            result.put(curr.getValue(), p);

            /* Update the priorities of all of its edges. */
            for (Map.Entry<T, Double> arc : graph.edgesFrom(curr.getValue()).entrySet()) {
                /* If we already know the shortest path from the source to
                 * this node, don't add the edge.
                 */
                if (result.containsKey(arc.getKey())) continue;

                /* Compute the cost of the path from the source to this node,
                 * which is the cost of this node plus the cost of this edge.
                 */
                double pathCost = curr.getPriority() + arc.getValue();

                /* If the length of the best-known path from the source to
                 * this node is longer than this potential path cost, update
                 * the cost of the shortest path.
                 */
                FibonacciHeap.Entry<T> destEntry = entries.get(arc.getKey());
                if (pathCost < destEntry.getPriority()) {
                    pq.decreaseKey(destEntry, pathCost);

                    GraphPathNode<T> destGraphNode = nodesGraph.getNode(destEntry.getValue());
                    if (curGraphNode != null && destGraphNode != null) {
                        destGraphNode.prependPath(curGraphNode.getPath());
                    }
                }
            }
        }

        return null;
    }
}