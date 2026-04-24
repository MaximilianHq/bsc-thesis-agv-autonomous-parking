package com.mycompany.agvparking;

import java.util.ArrayList;
import java.util.LinkedList;
import java.util.List;

public class OptPlan {

    private List<Vertex> nodes;
    private List<Edge> edges;
    private DataStore ds;

    public OptPlan(DataStore ds) {
        this.ds = ds;
    }

    private void buildGraph(boolean isLoaded) {
        nodes = new ArrayList<Vertex>();
        edges = new ArrayList<Edge>();

        for (int i = 0; i < ds.rows; i++) {
            for (int j = 0; j < ds.columns; j++) {
                Vertex location = new Vertex("" + (i * ds.columns + j), "Nod #" + (i * ds.columns + j));
                nodes.add(location);
            }
        }

        int mainRow = (int) (ds.LocationY[0] / ds.gridsize);
        double diagCost = isLoaded ? 2.1 : 1.4;

        for (int i = 0; i < ds.rows; i++) {
            for (int j = 0; j < ds.columns; j++) {
                int currentNode = i * ds.columns + j;

                // --- HÖGER & VÄNSTER (Med Highway-rabatt) ---
                if (j < ds.columns - 1) {
                    double costR = (i == mainRow) ? 0.5 : 1.0;
                    if (ds.ObstacleMatrix[j + 1][i] != 0) {
                        costR = 1000.0;
                    }
                    edges.add(new Edge("r", nodes.get(currentNode), nodes.get(i * ds.columns + j + 1), costR));
                }
                if (j > 0) {
                    double costL = (i == mainRow) ? 0.5 : 1.0;
                    if (ds.ObstacleMatrix[j - 1][i] != 0) {
                        costL = 1000.0;
                    }
                    edges.add(new Edge("l", nodes.get(currentNode), nodes.get(i * ds.columns + j - 1), costL));
                }

                // --- UPP & NER ---
                if (i < ds.rows - 1) {
                    double costD = (ds.ObstacleMatrix[j][i + 1] != 0) ? 1000.0 : 1.0;
                    edges.add(new Edge("d", nodes.get(currentNode), nodes.get((i + 1) * ds.columns + j), costD));
                }
                if (i > 0) {
                    double costU = (ds.ObstacleMatrix[j][i - 1] != 0) ? 1000.0 : 1.0;
                    edges.add(new Edge("u", nodes.get(currentNode), nodes.get((i - 1) * ds.columns + j), costU));
                }

                // --- DIAGONALER ---
                if (j < ds.columns - 1 && i < ds.rows - 1) {
                    double costDR = diagCost;
                    if (ds.ObstacleMatrix[j + 1][i + 1] != 0 || ds.ObstacleMatrix[j + 1][i] != 0 || ds.ObstacleMatrix[j][i + 1] != 0) {
                        costDR = 1000.0;
                    }
                    edges.add(new Edge("dr", nodes.get(currentNode), nodes.get((i + 1) * ds.columns + j + 1), costDR));
                }
                if (j > 0 && i > 0) {
                    double costUL = diagCost;
                    if (ds.ObstacleMatrix[j - 1][i - 1] != 0 || ds.ObstacleMatrix[j - 1][i] != 0 || ds.ObstacleMatrix[j][i - 1] != 0) {
                        costUL = 1000.0;
                    }
                    edges.add(new Edge("ul", nodes.get(currentNode), nodes.get((i - 1) * ds.columns + j - 1), costUL));
                }
                if (j < ds.columns - 1 && i > 0) {
                    double costUR = diagCost;
                    if (ds.ObstacleMatrix[j + 1][i - 1] != 0 || ds.ObstacleMatrix[j + 1][i] != 0 || ds.ObstacleMatrix[j][i - 1] != 0) {
                        costUR = 1000.0;
                    }
                    edges.add(new Edge("ur", nodes.get(currentNode), nodes.get((i - 1) * ds.columns + j + 1), costUR));
                }
                if (j > 0 && i < ds.rows - 1) {
                    double costDL = diagCost;
                    if (ds.ObstacleMatrix[j - 1][i + 1] != 0 || ds.ObstacleMatrix[j - 1][i] != 0 || ds.ObstacleMatrix[j][i + 1] != 0) {
                        costDL = 1000.0;
                    }
                    edges.add(new Edge("dl", nodes.get(currentNode), nodes.get((i + 1) * ds.columns + j - 1), costDL));
                }
            }
        }
    }

    public void createPlan(int startNodeId, int endNodeId) {
        buildGraph(true);
        Graph graph = new Graph(nodes, edges);
        DijkstraAlgorithm dijkstra = new DijkstraAlgorithm(graph);
        LinkedList<Vertex> path = null;
        if (startNodeId >= 0 && startNodeId < nodes.size() && endNodeId >= 0 && endNodeId < nodes.size()) {
            dijkstra.execute(nodes.get(startNodeId));
            path = dijkstra.getPath(nodes.get(endNodeId));
            ds.currentPath = path;
        }
    }

    public void createReturnPlan(int currentNodeId, int targetStartNodeId) {
        buildGraph(false);
        Graph graph = new Graph(nodes, edges);
        DijkstraAlgorithm dijkstra = new DijkstraAlgorithm(graph);
        dijkstra.execute(nodes.get(currentNodeId));
        ds.currentPath = dijkstra.getPath(nodes.get(targetStartNodeId));
    }
}
