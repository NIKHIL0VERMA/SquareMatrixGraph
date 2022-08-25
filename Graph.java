package SquareMatrixGraph;

import java.util.*;

/**
 * Graph class use to create int graph.
 * 
 * @author Nikhil Verma
 * @version 1.0
 * @since 23-08-2022
 */
public class Graph{
    private ArrayList<ArrayList<Node>> adjArrayList = new ArrayList<>();
    /**
     * Used to Structure out the data in graph
     * @param edges ArrayList of edges to be added to graph.
     */
    protected void Structure(ArrayList<Edge> edges){
        int n = 0;
        for(Edge e: edges){
            n = Integer.max(n, Integer.max(e.src, e.dest));
        }
        for(int i = 0; i <= n; i++){
            adjArrayList.add(i, new ArrayList<>());
        }
        for(Edge e: edges){
            adjArrayList.get(e.src).add(new Node(e.dest,e.cost));
        }
    }

    /**
     * Used to create graph with data.
     * Consider a 3X3 matrix Graph.
     * <p> 1--2--3
     * <p> 4--5--6
     * <p> 7--8--9
     * @param N Use to define number of Rows and Column to be used to create Square Matrix Graph.
     * @param Up Cost to go in Up, Example 4 to 1.
     * @param Forward Cost to go in Forward, Exampe 1 to 2.
     * @param Back Cost to go in Back, Example 2 to 1.
     * @param Down Cost to go in Down, Example 1 to 4.
     */
    public Graph(int N, int Up, int Back, int Down, int Forward){
        int k = 1;
        ArrayList<Edge> edges = new ArrayList<>();
        for(int i = 1; i <= N; i++){
            for(int j = 1; j <= N; j++){
                if(j != 1){
                    edges.add(new Edge(k, k-1, Back)); // West
                }
                if(k != i*N){
                    edges.add(new Edge(k, k+1, Forward)); // East
                }
                if(i != 1){
                    edges.add(new Edge(k, k-N, Up)); // North
                }
                if(i != N){
                    edges.add(new Edge(k, k+N, Down)); // South
                }
                k++;
            }
        }
        Structure(edges);
    }

    /**
     * Function to change cost of operation between source and destination.
     * 
     * @param graph Instance of Graph.
     * @param src Source from where we reach to destination.
     * @param dest Destination where we reach from source.
     * @param newCost Cost to be changed between source and Destination.
     */
    public static void changeCost(Graph graph, int src, int dest, int newCost){
        int index = findIndex(graph, src, dest);
        if(index != -1){
            graph.adjArrayList.get(src).set(index, new Node(dest, newCost));
        }else{
            System.out.println("Can't find destination");
        }
    }

    /**
     * Function used to remove Destination from source list.
     * 
     * @param graph Instance of Graph.
     * @param src Dource to find list of possible destinations.
     * @param dest Destination to be removed.
     */
    public static void removeDest(Graph graph, int src, int dest){
        int index = findIndex(graph, src, dest);
        if(index != -1){
            graph.adjArrayList.get(src).remove(index);
        }else{
            System.out.println("Can't find destination");
        }
    }

    /**
     * Funtion used to find index of destination in source list.
     * 
     * @param graph Instance of Graph.
     * @param src Source to get it's destination list.
     * @param dest Destination whose index is to be find.
     * @return The index of Destination in Source's Destination list.
     */
    private static int findIndex(Graph graph, int src, int dest){
        int index = 0;
        graph.adjArrayList.get(src);
        for(Node node: graph.adjArrayList.get(src)){
            if(node.dest == dest){
                return index;
            }
            index++;
        }
        return -1;
    }

    /**
     * Funtion used to get the Graph in 2D Array
     * 
     * @param graph Instance of Graph.
     * @return Graph in 2D Array.
     */
    public static int[][] to2DArray(Graph graph){
        int src = 1;
        int n = graph.adjArrayList.size();
        int[][] Array2D = new int[n-1][n-1];
        while(src < n){
            for(Node node: graph.adjArrayList.get(src)){
                Array2D[src-1][node.dest-1] = node.cost;
            }
            src++;
        }
        return Array2D;
    }

    /**
     * Function used to get ArrayList of ArrayList of Node.
     * 
     * @param graph Instance of Graph.
     * @return Graph in ArrayList<ArrayList<Node>>.
     */
    public static ArrayList<ArrayList<Node>> toArrayLists(Graph graph){
        return graph.adjArrayList;
    }

    /**
     * Function to find shortest path to all other destinations from source.
     * 
     * @param graph Instance of Graph.
     * @param src Source of search.
     * @return The Array of shortest distance with destination as index.
     */
    public static int[] shortestPathToAll(Graph graph, int src){
        return dijkstra(graph.adjArrayList, src);
    }

    /**
     * Function to find shortest path between any two point on Graph.
     * 
     * @param graph Instance of Graph.
     * @param src Source of stating to calculation distance.
     * @param dest Destination to reach with shortest distance.
     * @return The Shortest distance between source and destination.
     */
    public static int shortestPathBetween(Graph graph, int src, int dest){
        return dijkstra(graph.adjArrayList, src)[dest];
    }

    /**
     * Dijkstra Algorith used to calculate the distance between source and all other destinations.
     * 
     * @param graph Instance of Graph.
     * @param src Source of starting calculation.
     * @return Array of shortest distance between source and destinations.
     */
    private static int[] dijkstra(ArrayList<ArrayList<Node>> graph, int src){
        int[] distance = new int[graph.size()];
        for (int i = 0; i < graph.size(); i++){
            distance[i] = Integer.MAX_VALUE; // Setting distance between all to max.
        }
        distance[src] = 0; // Distance between source to source is 0.

        PriorityQueue<Node> pq = new PriorityQueue<>((v1, v2) -> v1.cost - v2.cost);
        pq.add(new Node(src, 0));
 
        while (pq.size() > 0) {
            Node current = pq.poll();
            for (Node n : graph.get(current.dest)) {
                if (distance[current.dest] + n.cost < distance[n.dest]) {
                    distance[n.dest] = n.cost + distance[current.dest];
                    pq.add(new Node(n.dest, distance[n.dest]));
                }
            }
        }
        return distance;
    }

    /**
     * Print graph on Console serial wise with cost of movement.
     * @param graph pass instance of graph class.
     */
    public static void PrintGraph(Graph graph){
        int src = 0;
        int n = graph.adjArrayList.size();
        while(src < n){
            for(Node node: graph.adjArrayList.get(src)){
                System.out.printf("%d ----> %s\t", src, node);
            }
            System.out.println();
            src++;
        }
    }
}

/**
 * Node class use to define destination with cost of movement.
 * Used to add destination from Source.
 * @param dest use to define Destination.
 * @param cost use to define cost to reach Destination.
  * 
 * @author Nikhil Verma
 * @version 1.0
 * @since 23-08-2022
 */
class Node{
    int dest, cost;
    /**
     * @param dest use to define Destination.
     * @param cost use to define cost to reach Destination.
     */
    public Node(int dest, int cost){
        this.dest = dest;
        this.cost = cost;
    }

    /**
     * @return Stirng containing destination with cost in curly brackets.
     */
    @Override
    public String toString(){
        return this.dest + " (" + this.cost + ")";
    }
}

/**
 * Edge class use to define edges of graph.
 * @param src use to define source from where it's start.
 * @param dest use to define destination where we can go from Source.
 * @param cost use to define the cost of movment from Source to Destination.
 * 
 * @author Nikhil Verma
 * @version 1.0
 * @since 23-08-2022
 */
class Edge {
    int src, dest, cost;
    /**
     * @param src use to define source from where it's start.
     * @param dest use to define destination where we can go from Source.
     * @param cost use to define the cost of movment from Source to Destination.
     */
    public Edge(int src, int dest, int cost){
        this.src = src;
        this.dest = dest;
        this.cost = cost;
    }
}