package ru.getsk.common.algorithm;

/**************************************************************************
 * File: AStar.java
 * Author: Sergey Kanukov (s.kanukov@gmail.com)
 * Based on AStar implementation of JavaDeveloper
 * (http://codereview.stackexchange.com/questions/38376/a-search-algorithm)
 */

import java.util.*;

import com.keithschwarz.FibonacciHeap;
import com.keithschwarz.DirectedGraph;
import org.apache.commons.lang3.tuple.*;

/**
 * NodeData stores all information of the node needed by the AStar algorithm.
 * This information includes the value of g, h, and f.
 * However, the value of all 3 variables are dependent on source and destination,
 * thus obtains at runtime.
 *
 * @param <T>
 */
final class NodeData<T> {
    private final T mNode;
    private double mG;  // g is distance from the source
    private double mF;  // f = g + h

    public NodeData(T nodeId) {
        mNode = nodeId;
        mG = Double.POSITIVE_INFINITY;
    }

    public T getNode() {
        return mNode;
    }

    public double getG() {
        return mG;
    }

    public void setG(double g) {
        mG = g;
    }

    public double calcF(double h) {
        return mF = h + mG;
    }

    public double getF() {
        return mF;
    }
}

// Comparator used to sort elements in PriorityQueue
class NodeDataComparator<T> implements Comparator<NodeData<T>> {
    public int compare(NodeData<T> nodeFirst, NodeData<T> nodeSecond) {
        if (nodeFirst.getF() > nodeSecond.getF()) return 1;
        if (nodeSecond.getF() > nodeFirst.getF()) return -1;
        return 0;
    }
}

public class AStar {
    /**
     * Given a directed, weighted graph G and source and destination nodes, produces the
     * path from source to destination using AStar algorithm based on PriorityQueue.
     * If can not find path produces null.
     *
     * @param graph The graph upon which to run AStar algorithm.
     * @param source The source node in the graph.
     * @param destination The destination node in the graph.
     * @return A pair of nodes path and distance from the source, null if path can not be found.
     */
    public static <T> Pair<List<T>, Double> getShortestPathPQ(DirectedGraph<T> graph, T source, T destination,
            AStarHeuristic heuristic) {
        final Map<T, NodeData<T>> nodesDataMap = new HashMap<T, NodeData<T>>();
        /**
         * http://stackoverflow.com/questions/20344041/why-does-priority-queue-has-default-initial-capacity-of-11
         */
        final Queue<NodeData<T>> openQueue = new PriorityQueue<NodeData<T>>(11, new NodeDataComparator<T>());

        NodeData<T> sourceNodeData = new NodeData<T>(source);
        sourceNodeData.setG(0);
        sourceNodeData.calcF(heuristic.getWeight(source, destination));
        nodesDataMap.put(source, sourceNodeData);
        openQueue.add(sourceNodeData);

        final Map<T, T> path = new HashMap<T, T>();
        final Set<NodeData<T>> closedList = new HashSet<NodeData<T>>();

        while (!openQueue.isEmpty()) {
            final NodeData<T> nodeData = openQueue.poll();

            if (nodeData.getNode().equals(destination)) {
                return new ImmutablePair<List<T>, Double>(reconstructPath(path, destination), nodeData.getG());
            }

            closedList.add(nodeData);

            for (Map.Entry<T, Double> neighborEntry : graph.edgesFrom(nodeData.getNode()).entrySet()) {
                final T entryNode = neighborEntry.getKey();
                NodeData<T> neighbor;
                if (nodesDataMap.containsKey(entryNode)) {
                    neighbor = nodesDataMap.get(entryNode);
                } else {
                    neighbor = new NodeData<T>(entryNode);
                    nodesDataMap.put(entryNode, neighbor);
                }

                if (closedList.contains(neighbor)) continue;

                double distanceBetweenTwoNodes = neighborEntry.getValue();
                double tentativeG = distanceBetweenTwoNodes + nodeData.getG();

                if (tentativeG < neighbor.getG()) {
                    neighbor.setG(tentativeG);
                    neighbor.calcF(heuristic.getWeight(neighbor.getNode(), destination));

                    path.put(neighbor.getNode(), nodeData.getNode());
                    // Java PriorityQueue does not provide correct way to reassign element priority.
                    // Force update queue.
                    if (openQueue.contains(neighbor)) {
                        openQueue.remove(neighbor);
                    }
                    openQueue.add(neighbor);
                }
            }
        }

        return null;
    }

    /**
     * Given a directed, weighted graph G and source and destination nodes, produces the
     * path from source to destination using AStar algorithm based on Fibonacci heap.
     * If can not find path produces null.
     *
     * @param graph The graph upon which to run AStar algorithm.
     * @param source The source node in the graph.
     * @param destination The destination node in the graph.
     * @return A pair of nodes path and distance from the source, null if path can not be found.
     */
    public static <T> Pair<List<T>, Double> getShortestPath(DirectedGraph<T> graph, T source, T destination,
                                                            AStarHeuristic heuristic) {
        final Map<T, NodeData<T>> nodesDataMap = new HashMap<T, NodeData<T>>();

        /* Create a Fibonacci heap storing the distances of unvisited nodes
         * from the source node.
         */
        final FibonacciHeap<NodeData<T>> openQueue = new FibonacciHeap<NodeData<T>>();

        /* The Fibonacci heap uses an internal representation that hands back
         * Entry objects for every stored element.  This map associates each
         * node in the graph with its corresponding Entry.
         */
        final Map<NodeData<T>, FibonacciHeap.Entry<NodeData<T>>> heapEntries = new HashMap<NodeData<T>, FibonacciHeap.Entry<NodeData<T>>>();

        NodeData<T> sourceNodeData = new NodeData<T>(source);
        sourceNodeData.setG(0);
        double f = sourceNodeData.calcF(heuristic.getWeight(source, destination));
        nodesDataMap.put(source, sourceNodeData);
        heapEntries.put(sourceNodeData, openQueue.enqueue(sourceNodeData, f));

        final Map<T, T> path = new HashMap<T, T>();
        final Set<NodeData<T>> closedList = new HashSet<NodeData<T>>();

        while (!openQueue.isEmpty()) {
            final NodeData<T> nodeData = openQueue.dequeueMin().getValue();

            if (nodeData.getNode().equals(destination)) {
                return new ImmutablePair<List<T>, Double>(reconstructPath(path, destination), nodeData.getG());
            }

            closedList.add(nodeData);

            for (Map.Entry<T, Double> neighborEntry : graph.edgesFrom(nodeData.getNode()).entrySet()) {
                final T entryNode = neighborEntry.getKey();
                NodeData<T> neighbor;
                if (nodesDataMap.containsKey(entryNode)) {
                    neighbor = nodesDataMap.get(entryNode);
                } else {
                    neighbor = new NodeData<T>(entryNode);
                    nodesDataMap.put(entryNode, neighbor);
                }

                if (closedList.contains(neighbor)) continue;

                double distanceBetweenTwoNodes = neighborEntry.getValue();
                double tentativeG = distanceBetweenTwoNodes + nodeData.getG();

                if (tentativeG < neighbor.getG()) {
                    neighbor.setG(tentativeG);
                    f = neighbor.calcF(heuristic.getWeight(neighbor.getNode(), destination));

                    path.put(neighbor.getNode(), nodeData.getNode());
                    if (!heapEntries.containsKey(neighbor)) {
                        heapEntries.put(neighbor, openQueue.enqueue(neighbor, f));
                    } else {
                        openQueue.decreaseKey(heapEntries.get(neighbor), f);
                    }
                }
            }
        }

        return null;
    }

    private static <T> List<T> reconstructPath(Map<T, T> path, T destination) {
        assert path != null;
        assert destination != null;

        final List<T> pathList = new ArrayList<T>();
        pathList.add(destination);
        while (path.containsKey(destination)) {
            destination = path.get(destination);
            pathList.add(destination);
        }
        Collections.reverse(pathList);
        return pathList;
    }
}
