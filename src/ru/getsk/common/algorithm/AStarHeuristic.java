package ru.getsk.common.algorithm;

/**
 * Defines the heuristic interface for finding the potential best weight from
 * source to destination nodes in graph.
 */
public interface AStarHeuristic {
    <T> double getWeight(T source, T destination);
}
