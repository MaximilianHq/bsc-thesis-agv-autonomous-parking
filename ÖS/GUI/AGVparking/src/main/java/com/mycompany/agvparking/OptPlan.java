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

    // Vi behåller denna så att ControlUI inte klagar, men den gör ingenting!
    public void setTarget(int x, int y) {
        // Borttaget: Det var denna som orsakade den gigantiska omvägen!
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

                // --- HÖGER ---
                if (j < ds.columns - 1) {
                    int nx = j + 1;
                    int ny = i;
                    double costR = (ny == mainRow) ? 0.5 : 1.0;
                    int val = ds.ObstacleMatrix[nx][ny];
                    if (val >= 2) {
                        costR = 100000.0;
                    } else if (val == 1) {
                        costR = 1000.0;
                    }
                    edges.add(new Edge("r", nodes.get(currentNode), nodes.get(ny * ds.columns + nx), costR));
                }

                // --- VÄNSTER ---
                if (j > 0) {
                    int nx = j - 1;
                    int ny = i;
                    double costL = (ny == mainRow) ? 0.5 : 1.0;
                    int val = ds.ObstacleMatrix[nx][ny];
                    if (val >= 2) {
                        costL = 100000.0;
                    } else if (val == 1) {
                        costL = 1000.0;
                    }
                    edges.add(new Edge("l", nodes.get(currentNode), nodes.get(ny * ds.columns + nx), costL));
                }

                // --- NER ---
                if (i < ds.rows - 1) {
                    int nx = j;
                    int ny = i + 1;
                    double costD = 1.0;
                    int val = ds.ObstacleMatrix[nx][ny];
                    if (val >= 2) {
                        costD = 100000.0;
                    } else if (val == 1) {
                        costD = 1000.0;
                    }
                    edges.add(new Edge("d", nodes.get(currentNode), nodes.get(ny * ds.columns + nx), costD));
                }

                // --- UPP ---
                if (i > 0) {
                    int nx = j;
                    int ny = i - 1;
                    double costU = 1.0;
                    int val = ds.ObstacleMatrix[nx][ny];
                    if (val >= 2) {
                        costU = 100000.0;
                    } else if (val == 1) {
                        costU = 1000.0;
                    }
                    edges.add(new Edge("u", nodes.get(currentNode), nodes.get(ny * ds.columns + nx), costU));
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
        LinkedList<Vertex> path = null;
        dijkstra.execute(nodes.get(currentNodeId));
        path = dijkstra.getPath(nodes.get(targetStartNodeId));
        ds.currentPath = path;

        if (path != null) {
            ds.currentPath = path;

            // Skapa körinstruktionerna för resan hem ---
            RouteOptimizer optimizer = new RouteOptimizer(ds);
            optimizer.compressPath(path, false);

            //LÄgger till rotation på slutnoden
            //Räkna ut differensen mellan nuvarande rotation och önskad
            int diff = ds.currentRotation - ds.startRotation;

            // Normalisera vinkeln till spannet -180 till 180 grader
            while (diff > 180) {
                diff -= 360;
            }
            while (diff <= -180) {
                diff += 360;
            }

            // 3. Lägg till rotationsinstruktioner i kön
            if (Math.abs(diff) > 2) { // En liten tolerans på 2 grader för att slippa mikro-justeringar

                // --- NYTT: Hämta ut X och Y för mål-noden så att Stop-and-wait fungerar ---
                int targetX = targetStartNodeId % ds.columns;
                int targetY = targetStartNodeId / ds.columns;
                // --------------------------------------------------------------------------

                //Hämta maneuver och hastighet
                int maneuver = (diff > 0) ? InstructionsStore.TURNING_RIGHT : InstructionsStore.TURNING_LEFT;
                int velocity = InstructionsStore.getTargetVelocity(maneuver, false);
                if (diff > 0) {
                    // Positiv diff betyder att vi roterar höger
                    // Använder fullständig konstruktor: maneuver, velocity, steps, targetX, targetY, monitorPosition
            
                    ds.instructionQueue.add(new AgvInstruction(maneuver, velocity, Math.abs(diff), targetX, targetY, false));
                    System.out.println("Avslutar med högerrotation: " + Math.abs(diff) + " grader.");
                } else {
                    // Negativ diff betyder att vi roterar vänster
                    ds.instructionQueue.add(new AgvInstruction(maneuver, velocity, Math.abs(diff), targetX, targetY, false));
                    System.out.println("Avslutar med vänsterrotation: " + Math.abs(diff) + " grader.");
                }
            } else {
                System.out.println("Ingen avslutande rotation krävs (redan vänd mot " + ds.startRotation + "°).");
            }

        } else {
            System.out.println("Ingen väg kunde hittas tillbaka till startnoden.");
        }
    }
}
