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

   /* 
    public void createPlan() {
        nodes = new ArrayList<Vertex>();
        edges = new ArrayList<Edge>();

        // --- 1. Skapa Noder (Grid network) ---
        for (int i = 0; i < ds.rows; i++) {      // i = rad (y)
            for (int j = 0; j < ds.columns; j++) { // j = kolumn (x)
                Vertex location = new Vertex("" + (i * ds.columns + j), "Nod #" + (i * ds.columns + j));
                nodes.add(location);
            }
        }

        System.out.println("columns: " + ds.columns);
        System.out.println("rows: " + ds.rows);

        double cost = 1;

        // --- 2. Skapa Horisontella och Diagonala kanter ---
        for (int i = 0; i < ds.rows; i++) {
            for (int j = 0; j < ds.columns - 1; j++) {

                // Add horizontal links in both directions
                cost = 1;
                // OBS: Matrisen är [x][y], så vi kollar [j+1][i]
                if (ds.ObstacleMatrix[j + 1][i] != 0) {
                    cost = 1000;
                }

                // Kant åt höger
                Edge dirA = new Edge("r" + nodes.get(i * ds.columns + j),
                        nodes.get(i * ds.columns + j), nodes.get(i * ds.columns + j + 1), cost);
                edges.add(dirA);

                // Kant åt vänster
                Edge dirB = new Edge("l" + nodes.get(i * ds.columns + j),
                        nodes.get(i * ds.columns + j + 1), nodes.get(i * ds.columns + j), cost);
                edges.add(dirB);

                // --- Diagonaler ---
                if (i < ds.rows - 1) {
                    cost = 1.4;
                    // Kollar hinder diagonalt (x+1, y+1)
                    if (ds.ObstacleMatrix[j + 1][i + 1] != 0) {
                        cost = 1000;
                    }
                    Edge dirC = new Edge("1d" + nodes.get(i * ds.columns + j),
                            nodes.get(i * ds.columns + j), nodes.get((i + 1) * ds.columns + j + 1), cost);
                    edges.add(dirC);

                    Edge dirD = new Edge("2d" + nodes.get(i * ds.columns + j),
                            nodes.get((i + 1) * ds.columns + j + 1), nodes.get(i * ds.columns + j), cost);
                    edges.add(dirD);
                }

                if (i > 0) {
                    cost = 1.4;
                    // Kollar hinder diagonalt uppåt (x+1, y-1)
                    if (ds.ObstacleMatrix[j + 1][i - 1] != 0) {
                        cost = 1000;
                    }
                    Edge dirE = new Edge("3d" + nodes.get(i * ds.columns + j),
                            nodes.get(i * ds.columns + j), nodes.get((i - 1) * ds.columns + j + 1), cost);
                    edges.add(dirE);

                    Edge dirF = new Edge("4d" + nodes.get(i * ds.columns + j),
                            nodes.get((i - 1) * ds.columns + j + 1), nodes.get(i * ds.columns + j), cost);
                    edges.add(dirF);
                }
            }
        }

        // --- 3. Skapa Vertikala kanter ---
        for (int i = 0; i < ds.rows - 1; i++) {
            for (int j = 0; j < ds.columns; j++) {
                // Add vertical links in both directions
                cost = 1;
                // Kollar hinder under (x, y+1)
                if (ds.ObstacleMatrix[j][i + 1] != 0) {
                    cost = 1000;
                }

                // Kant neråt
                Edge dirC = new Edge("d" + nodes.get(i * ds.columns + j),
                        nodes.get(i * ds.columns + j), nodes.get((i + 1) * ds.columns + j), cost);
                edges.add(dirC);

                // Kant uppåt
                Edge dirD = new Edge("u" + nodes.get(i * ds.columns + j),
                        nodes.get((i + 1) * ds.columns + j), nodes.get(i * ds.columns + j), cost);
                edges.add(dirD);
            }
        }

        // --- 4. Kör Dijkstra ---
        Graph graph = new Graph(nodes, edges);
        DijkstraAlgorithm dijkstra = new DijkstraAlgorithm(graph);

        // Convert from Location coordinates to node id
        int x = (int) (ds.LocationX[0] / ds.gridsize);
        int y = (int) (ds.LocationY[0] / ds.gridsize);
        int startnode = (int) (y * ds.columns) + x;

        x = (int) (ds.LocationX[1] / ds.gridsize);
        y = (int) (ds.LocationY[1] / ds.gridsize);
        int endnode = (int) (y * ds.columns) + x;

        System.out.println("Start: " + startnode + " End: " + endnode);

        // HÄR INITIERAR VI PATH
        LinkedList<Vertex> path = null;

        // Compute shortest path
        if (startnode >= 0 && startnode < nodes.size() && endnode >= 0 && endnode < nodes.size()) {
            dijkstra.execute(nodes.get(startnode));

            // Nu tilldelar vi värdet till variabeln vi skapade utanför
            path = dijkstra.getPath(nodes.get(endnode));

            if (path != null) {
                System.out.println("Rutt beräknad! Antal steg: " + path.size());
                
                // --- VIKTIGT: Skicka vägen till DataStore så roboten kan läsa den ---
                ds.currentPath = path;
                // -------------------------------------------------------------------
                
            } else {
                System.out.println("Ingen väg funnen.");
            }
        } else {
            System.out.println("Start- eller slutnod ligger utanför kartan.");
        }

        // Kollar om path finns innan vi försöker rita ut den
        if (path != null && path.size() > 0) {
            
            // Hämta kostnaden (OBS: getShortestDistance måste vara public i DijkstraAlgorithm)
            double mincost = dijkstra.getShortestDistance(nodes.get(endnode));
            System.out.println("The minimum path cost is: " + mincost);

            // Get shortest path
            for (int i = 0; i < path.size(); i++) {
                int nodeId = Integer.parseInt(path.get(i).getId());
                
                int ipos = nodeId / ds.columns; // Detta blir Raden (Y)
                int jpos = nodeId - (ipos * ds.columns); // Detta blir Kolumnen (X)
                
                System.out.println("Node: " + nodeId + ", Grid postition: (" + jpos + ", " + ipos + ")");
                
                // Uppdatera matrisen så vägen ritas ut i svart (3)
                ds.ObstacleMatrix[jpos][ipos] = 3; 
            }
        }
    }
}
*/
/*
 * OptPlan.java
 */
//package com.mycompany.autopark;
//
//import java.util.ArrayList;
//import java.util.LinkedList;
//import java.util.List;
//
//public class OptPlan {
//
//    private List<Vertex> nodes;
//    private List<Edge> edges;
//    private DataStore ds;
//
//    public OptPlan(DataStore ds) {
//        this.ds = ds;
//    }
//
    // NY METOD: Rensa bort den gamla vägen (svarta rutor)
    public void clearPath() {
//        for (int i = 0; i < ds.rows; i++) {
//            for (int j = 0; j < ds.columns; j++) {
//                // Om rutan är markerad som väg (3), gör den till golv (0) igen
//                if (ds.ObstacleMatrix[j][i] == 3) {
//                    ds.ObstacleMatrix[j][i] = 0;
//                }
//            }
//        }
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
                
                // Horisontella
                cost = 1;
                // Använder != 0 för att täcka in alla typer av hinder
                if (ds.ObstacleMatrix[j + 1][i] != 0) cost = 1000; 
                edges.add(new Edge("r", nodes.get(i * ds.columns + j), nodes.get(i * ds.columns + j + 1), cost));
                edges.add(new Edge("l", nodes.get(i * ds.columns + j + 1), nodes.get(i * ds.columns + j), cost));

                // Diagonala (Ner-Höger & Upp-Vänster)
                if (i < ds.rows - 1) {
                    cost = 1.4;
                    // LÖSNING CORNER CLIPPING: Kolla målnoden OCH de två intilliggande rutorna!
                    if (ds.ObstacleMatrix[j + 1][i + 1] != 0 || 
                        ds.ObstacleMatrix[j + 1][i] != 0 || 
                        ds.ObstacleMatrix[j][i + 1] != 0) {
                        cost = 1000;
                    }
                    edges.add(new Edge("dr", nodes.get(i * ds.columns + j), nodes.get((i + 1) * ds.columns + j + 1), cost));
                    edges.add(new Edge("ul", nodes.get((i + 1) * ds.columns + j + 1), nodes.get(i * ds.columns + j), cost));
                }
                
                // Diagonala (Upp-Höger & Ner-Vänster)
                if (i > 0) {
                    cost = 1.4;
                    // LÖSNING CORNER CLIPPING: Kolla målnoden OCH de två intilliggande rutorna!
                    if (ds.ObstacleMatrix[j + 1][i - 1] != 0 || 
                        ds.ObstacleMatrix[j + 1][i] != 0 || 
                        ds.ObstacleMatrix[j][i - 1] != 0) {
                        cost = 1000;
                    }
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