import ru.getsk.common.algorithm.AStar;
import ru.getsk.common.algorithm.AStarHeuristic;
import ru.getsk.common.algorithm.Dijkstra;
import ru.getsk.common.helper.RandomHelper;
import com.keithschwarz.DirectedGraph;
import org.apache.commons.lang3.tuple.Pair;
import java.util.List;

class Point {
    public long Id;
    public float X;
    public float Y;

    public Point(long id, float x, float y) {
        Id = id;
        X = x;
        Y = y;
    }
}

public class Main {

    public static void main(String[] args) {
        DirectedGraph<Point> g = new DirectedGraph<Point>();

        int matrixSize = 50;
        long pointId = 0;

        // Add some nodes to graph
        Point[][] pointMatrix = new Point[matrixSize][matrixSize];
        for (int i = 0; i < pointMatrix.length; i++) {
            for (int j = 0; j < pointMatrix[i].length; j++) {
                pointMatrix[i][j] = new Point(pointId++, RandomHelper.randInt(1, pointMatrix.length),
                        RandomHelper.randInt(1, pointMatrix[i].length));
                g.addNode(pointMatrix[i][j]);
            }
        }

        // Connect nodes
        for (int i = 0; i < pointMatrix.length - 1; i++) {
            for (int j = 0; j < pointMatrix[i].length; j++) {
                for (int k = 0; k < pointMatrix[i + 1].length; k++) {
                    g.addEdge(pointMatrix[i][j], pointMatrix[i + 1][k],
                            euclideanDistance(pointMatrix[i][j], pointMatrix[i + 1][k]));
                }
            }
        }

        // Randomly get source and destination nodes
        Point source = pointMatrix[RandomHelper.randInt(0, 2)][RandomHelper.randInt(0, 2)];
        Point dest = pointMatrix[RandomHelper.randInt(matrixSize - 3, matrixSize - 1)]
                [RandomHelper.randInt(matrixSize - 3, matrixSize - 1)];

        // Define heuristic for AStar algorithm
        AStarHeuristic heuristic = new AStarHeuristic() {
            @Override
            public <T> double getWeight(T source, T destination) {
                return euclideanDistance((Point)source, (Point)destination);
            }
        };
        
        dijkstra(g, source, dest);

        aStarPQ(g, source, dest, heuristic);
        aStarFH(g, source, dest, heuristic);
    }

    private static double manhattanDistance(Point src, Point dest) {
        return Math.abs(src.X - dest.X) + Math.abs(src.Y - dest.Y);
    }

    private static double euclideanDistance(Point src, Point dest) {
        return Math.sqrt((src.X - dest.X) * (src.X - dest.X) + (src.Y - dest.Y) * (src.Y - dest.Y));
    }

    private static void aStarPQ(DirectedGraph<Point> g, Point source, Point dest, AStarHeuristic heuristic) {
        long before = System.currentTimeMillis();
        Pair<List<Point>, Double> aStarPQResult = AStar.getShortestPathPQ(g, source, dest, heuristic);
        long now = System.currentTimeMillis();

        if (aStarPQResult != null) {
            System.out.println("***** AStar using Priority queue *****");
            for (Point point : aStarPQResult.getLeft()) {
                System.out.print(point.Id + "; ");
            }
            System.out.println();
            System.out.println("Weight: " + aStarPQResult.getRight().toString());
            System.out.println("Time: " + (now - before));
            System.out.println();
        }
    }

    private static void aStarFH(DirectedGraph<Point> g, Point source, Point dest, AStarHeuristic heuristic) {
        long before = System.currentTimeMillis();
        Pair<List<Point>, Double> aStarFHResult = AStar.getShortestPath(g, source, dest, heuristic);
        long now = System.currentTimeMillis();

        if (aStarFHResult != null) {
            System.out.println("***** AStar using Fibonacci heap *****");
            for (Point point : aStarFHResult.getLeft()) {
                System.out.print(point.Id + "; ");
            }
            System.out.println();
            System.out.println("Weight: " + aStarFHResult.getRight().toString());
            System.out.println("Time: " + (now - before));
            System.out.println();
        }
    }

    private static void dijkstra(DirectedGraph<Point> g, Point source, Point dest) {
        long before = System.currentTimeMillis();
        Pair<List<Point>, Double> dijkstraResult = Dijkstra.getShortestPath(g, source, dest);
        long now = System.currentTimeMillis();

        System.out.println("***** Dijkstra *****");

        if (dijkstraResult != null) {
            for (Point point : dijkstraResult.getLeft()) {
                System.out.print(point.Id + "; ");
            }
            System.out.println();
            System.out.println("Weight: " + dijkstraResult.getRight().toString());
            System.out.println("Time: " + (now - before));
            System.out.println();
        }
    }
}
