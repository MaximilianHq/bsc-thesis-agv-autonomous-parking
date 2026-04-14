/*
 * OptPlan.java
 */
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
    

    // ÄNDRAD METOD: Tar nu startNode och endNode som parametrar
    public void createPlan(int startNodeId, int endNodeId) {
        nodes = new ArrayList<Vertex>();
        edges = new ArrayList<Edge>();

        // --- 1. Skapa Noder ---
        for (int i = 0; i < ds.rows; i++) {
            for (int j = 0; j < ds.columns; j++) {
                Vertex location = new Vertex("" + (i * ds.columns + j), "Nod #" + (i * ds.columns + j));
                nodes.add(location);
            }
        }

        double cost = 1;

        // --- 2. Skapa Kanter (Edges)  ---
        for (int i = 0; i < ds.rows; i++) {
            for (int j = 0; j < ds.columns - 1; j++) {
                cost = 1;
                if (ds.ObstacleMatrix[j + 1][i] == 1 || ds.ObstacleMatrix[j + 1][i] == 2) cost = 1000; // Hinder

                // Horisontella
                edges.add(new Edge("r", nodes.get(i * ds.columns + j), nodes.get(i * ds.columns + j + 1), cost));
                edges.add(new Edge("l", nodes.get(i * ds.columns + j + 1), nodes.get(i * ds.columns + j), cost));

                // Diagonala
                if (i < ds.rows - 1) {
                    cost = 1.4;
                    if (ds.ObstacleMatrix[j + 1][i + 1] == 1 || ds.ObstacleMatrix[j + 1][i + 1] == 2) cost = 1000;
                    edges.add(new Edge("dr", nodes.get(i * ds.columns + j), nodes.get((i + 1) * ds.columns + j + 1), cost));
                    edges.add(new Edge("ul", nodes.get((i + 1) * ds.columns + j + 1), nodes.get(i * ds.columns + j), cost));
                }
                if (i > 0) {
                    cost = 1.4;
                    if (ds.ObstacleMatrix[j + 1][i - 1] == 1 || ds.ObstacleMatrix[j + 1][i - 1] == 2) cost = 1000;
                    edges.add(new Edge("ur", nodes.get(i * ds.columns + j), nodes.get((i - 1) * ds.columns + j + 1), cost));
                    edges.add(new Edge("dl", nodes.get((i - 1) * ds.columns + j + 1), nodes.get(i * ds.columns + j), cost));
                }
            }
        }

        // --- 3. Skapa Vertikala Kanter ---
        for (int i = 0; i < ds.rows - 1; i++) {
            for (int j = 0; j < ds.columns; j++) {
                cost = 1;
                if (ds.ObstacleMatrix[j][i + 1] == 1 || ds.ObstacleMatrix[j][i + 1] == 2) cost = 1000;
                
                edges.add(new Edge("d", nodes.get(i * ds.columns + j), nodes.get((i + 1) * ds.columns + j), cost));
                edges.add(new Edge("u", nodes.get((i + 1) * ds.columns + j), nodes.get(i * ds.columns + j), cost));
            }
        }

        // --- 4. Kör Dijkstra ---
        Graph graph = new Graph(nodes, edges);
        DijkstraAlgorithm dijkstra = new DijkstraAlgorithm(graph);
        
        LinkedList<Vertex> path = null;

        if (startNodeId >= 0 && startNodeId < nodes.size() && endNodeId >= 0 && endNodeId < nodes.size()) {
            dijkstra.execute(nodes.get(startNodeId));
            path = dijkstra.getPath(nodes.get(endNodeId));
            
            ds.currentPath = path; // Spara till DataStore

            if (path != null) {
                System.out.println("Rutt hittad: " + path.size() + " steg.");
                // Rita ut vägen i matrisen
                for (Vertex v : path) {
                    int id = Integer.parseInt(v.getId());
                    int ipos = id / ds.columns; // Rad (Y)
                    int jpos = id - (ipos * ds.columns); // Kolumn (X)
                    // Markera som väg (3) om det inte är start/mål (för snyggare grafik)
                    //ds.ObstacleMatrix[jpos][ipos] = 3;
                }
            } else {
                System.out.println("Ingen väg kunde hittas.");
            }
        }
    }
}