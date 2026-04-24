package com.mycompany.agvparking;

import java.util.List;

public class RouteOptimizer {

    private DataStore ds;

    public RouteOptimizer(DataStore ds) {
        this.ds = ds;
    }

    public void compressPath(List<Vertex> path) {
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
                // Om vi fortsätter i exakt samma riktning
                if (dx == currentDx && dy == currentDy) {
                    stepCount++;
                } else {
                    // Skriv ut instruktion
                    printInstruction(currentDx, currentDy, stepCount, currentX, currentY);

                    // ... och börja räkna på den nya riktningen
                    currentDx = dx;
                    currentDy = dy;
                    stepCount = 1;
                }
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
            printInstruction(currentDx, currentDy, stepCount, finalX, finalY);
        }

        System.out.println("--- Optimering klar ---\n");
    }

    // Hjälpmetod för att tolka dx och dy till de 8 möjliga manövrarna
    private void printInstruction(int dx, int dy, int steps, int targetX, int targetY) {
        String directionText = "";
        int direction = 0;

        // OBS: Y ökar nedåt på skärmen, så dy = -1 betyder Uppåt.
        if (dx == 0 && dy == -1) {
            directionText = "Uppåt (Rakt fram)";
            direction = 1;
        } else if (dx == 0 && dy == 1) {
            directionText = "Nedåt (Bakåt)";
            direction = 2;
        } else if (dx == 1 && dy == 0) {
            directionText = "Höger (Sidled)";
            direction = 3;
        } else if (dx == -1 && dy == 0) {
            directionText = "Vänster (Sidled)";
            direction = 4;
        } else if (dx == 1 && dy == -1) {
            directionText = "Diagonalt Upp-Höger";
            direction = 5;
        } else if (dx == -1 && dy == -1) {
            directionText = "Diagonalt Upp-Vänster";
            direction = 6;
        } else if (dx == 1 && dy == 1) {
            directionText = "Diagonalt Ner-Höger";
            direction = 7;
        } else if (dx == -1 && dy == 1) {
            directionText = "Diagonalt Ner-Vänster";
            direction = 8;
        } else {
            directionText = "Okänd riktning (" + dx + "," + dy + ")";
        }

        // Här skriver vi ut det i konsolen. Längre fram är det kanske här 
        // vi skapar ett meddelande och skickar det via Bluetooth
        System.out.println("Körinstruktion: Åk " + directionText + " i " + steps + " steg.");
        
        // Lägg till instruktionen i brevlådan för Bluetooth
        if (ds.instructionQueue != null) {
            // Skapar ny AgvInstruction(manöver, hastighet, steg, mål-X, mål-Y)
            ds.instructionQueue.add(new AgvInstruction(direction, 100, steps, targetX, targetY));
        }
    }
}
