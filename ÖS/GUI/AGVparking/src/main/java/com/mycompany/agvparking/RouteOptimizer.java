package com.mycompany.agvparking;

import java.util.List;

public class RouteOptimizer {

    private DataStore ds;

    public RouteOptimizer(DataStore ds) {
        this.ds = ds;
    }

    public void compressPath(List<Vertex> path, boolean isLoaded) {
        if (path == null || path.size() < 2) {
            System.out.println("Ingen giltig rutt att optimera.");
            return;
        }

        System.out.println("\n--- Startar optimering av rutt ---");

        int currentDx = 0;
        int currentDy = 0;
        int stepCount = 0;
        boolean firstMove = true;

        // Loopa igenom vägen fram till näst sista noden
        for (int i = 0; i < path.size() - 1; i++) {

            // Hämta nuvarande och nästa nod
            Vertex currentVertex = path.get(i);
            Vertex nextVertex = path.get(i + 1);

            // Gör om id till int
            int currentId = Integer.parseInt(currentVertex.getId());
            int nextId = Integer.parseInt(nextVertex.getId());

            // Räkna ut X och Y för båda noderna
            int currentX = currentId % ds.columns;
            int currentY = currentId / ds.columns;

            int nextX = nextId % ds.columns;
            int nextY = nextId / ds.columns;

            // Räkna ut skillnaden (delta) mellan dem
            int dx = nextX - currentX;
            int dy = nextY - currentY;

            // Om detta är det allra första steget i rutten
            if (firstMove) {
                currentDx = dx;
                currentDy = dy;
                stepCount = 1;
                firstMove = false;
            } else {
                    // Skriv ut instruktion för raksträckan vi precis kört
                    printInstruction(currentDx, currentDy, stepCount, currentX, currentY,isLoaded);

                    // --- NYTT: IDENTIFIERA SVÄNG OCH LÄGG TILL CURVED TRAJECTORY ---
                    // Kryssprodukten avslöjar om vi svänger höger eller vänster
                    int crossProduct = currentDx * dy - currentDy * dx;
 
                    
                    if (crossProduct > 0) { // HÖGERSVÄNG
                        int maneuver = isLoaded ? InstructionsStore.CURVED_TRAJECTORY_RIGHT : InstructionsStore.TURNING_RIGHT;
                        //Hastighet
                    int velocity = InstructionsStore.getTargetVelocity(maneuver,isLoaded);
                        if (isLoaded) {
                            System.out.println("Körinstruktion: Kurvad bana Höger (90 grader)");
                            ds.instructionQueue.add(new AgvInstruction(maneuver, velocity, 0, currentX, currentY, false));
                        } else {
                            System.out.println("Körinstruktion: Rotera Höger på stället");
                            ds.instructionQueue.add(new AgvInstruction(maneuver, velocity, 0, currentX, currentY, false));
                        }
                    } else if (crossProduct < 0) { // VÄNSTERSVÄNG
                        int maneuver = isLoaded ? InstructionsStore.CURVED_TRAJECTORY_LEFT : InstructionsStore.TURNING_LEFT;
                        //Hastighet
                    int velocity = InstructionsStore.getTargetVelocity(maneuver,isLoaded);
                        if (isLoaded) {
                            System.out.println("Körinstruktion: Kurvad bana Vänster (90 grader)");
                            ds.instructionQueue.add(new AgvInstruction(maneuver, velocity, 0, currentX, currentY, false));
                        } else {
                            System.out.println("Körinstruktion: Rotera Vänster på stället");
                            ds.instructionQueue.add(new AgvInstruction(maneuver, velocity, 0, currentX, currentY, false));
                        }
                    }

                    // ... och börja räkna på den nya riktningen
                    currentDx = dx;
                    currentDy = dy;
                    stepCount = 1;
                }
        }

        // När loopen är klar måste vi skriva ut den sista rörelsen som vi räknat på
        if (!firstMove) {
            // Räkna ut slutmålet för den allra sista sträckan ---
            Vertex lastNode = path.get(path.size() - 1);
            int lastId = Integer.parseInt(lastNode.getId());
            int finalX = lastId % ds.columns;
            int finalY = lastId / ds.columns;
            // Skriv ut instruktion
            printInstruction(currentDx, currentDy, stepCount, finalX, finalY, isLoaded);
        }

        System.out.println("--- Optimering klar ---\n");
    }

    // Hjälpmetod för att tolka dx och dy till de 8 möjliga manövrarna
    private void printInstruction(int dx, int dy, int steps, int targetX, int targetY, boolean isLoaded) {
       int direction = InstructionsStore.getDirectionFromDelta(dx,dy);
       
       //Hastighet
       int velocity = InstructionsStore.getTargetVelocity(direction, isLoaded);
       
        String directionText = InstructionsStore.getInstructionText(direction);

        // Här skriver vi ut det i konsolen. Längre fram är det kanske här 
        // vi skapar ett meddelande och skickar det via Bluetooth
        System.out.println("Körinstruktion: Åk " + directionText + " i " + steps + " steg.");

        // Lägg till instruktionen i brevlådan för Bluetooth
        
        boolean shouldMonitor = true; 
        if (direction >= 5) { 
            shouldMonitor = false; 
        ds.instructionQueue.add(new AgvInstruction(direction, velocity, steps, targetX, targetY, shouldMonitor)); 
        } 
        if (direction > 0 && direction <= 4) { 
            ds.instructionQueue.add(new AgvInstruction(direction, velocity, steps, targetX, targetY, shouldMonitor)); 
        } 
        
            }
}
