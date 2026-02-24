package com.mycompany.autopark;

import java.util.Collections;
import java.util.List;
import java.util.Random;

public class GuiUpdate implements Runnable {

    private int sleepTime;
    private ControlUI cui;
    private DataStore ds;
    private OptPlan op; 

    public GuiUpdate(DataStore ds, ControlUI cui) {
        this.cui = cui;
        this.ds = ds;
        this.op = new OptPlan(ds); 
        this.sleepTime = 100; 
    }

    @Override
    public void run() {
        try {
            cui.appendStatus("GuiUpdate startar... Väntar på RobotRead.\n");
            while (ds.updateUiflag == false) {
                Thread.sleep(500);
            }
            cui.appendStatus("Startar ruttplanering för alla punkter.\n");

            // Hämta startpunkten (den röda pricken, index 0)
            int startX = (int) (ds.LocationX[0] / ds.gridsize);
            int startY = (int) (ds.LocationY[0] / ds.gridsize);
            int startNodeId = (startY * ds.columns) + startX;

            // Loopa igenom alla destinationer (de gröna prickarna, index 1 och framåt)
            for (int destIndex = 1; destIndex < ds.locations; destIndex++) {
                
                cui.appendStatus("Planerar rutt till destination " + destIndex + "\n");

                // --- 1. RÄKNA UT MÅL-NOD ---
                int destX = (int) (ds.LocationX[destIndex] / ds.gridsize);
                int destY = (int) (ds.LocationY[destIndex] / ds.gridsize);
                
                // Hacket: Flytta målet utanför parkeringsrutan (7 rutor ner) så vi inte krockar
                
                int endNodeId = (destY * ds.columns) + destX;

                // --- 2. SKAPA RUTT OCH RITA UPP ---
                op.createPlan(startNodeId, endNodeId);
                cui.repaint(); // Visa den svarta vägen

                // --- 3. ÅK TILL MÅLET ---
                if (ds.currentPath != null) {
                    moveRobotAlongPath(ds.currentPath);
                    
                    if (ds.isStopped) break;
                    
                    cui.appendStatus("Framme vid mål " + destIndex + ". Åker tillbaka...\n");
                    Thread.sleep(500); // Liten paus vid målet
                    
                    if (ds.isStopped) break;    

                    // --- 4. ÅK TILLBAKA (SAMMA VÄG) ---
                    // Vänd på listan för att åka baklänges
                    Collections.reverse(ds.currentPath);
                    moveRobotAlongPath(ds.currentPath);
                    
                    if (ds.isStopped) break;
                }

                // --- 5. RENSA VÄGEN ---
                op.clearPath();
                cui.repaint(); // Nu försvinner den svarta vägen
                Thread.sleep(500); // Kort paus innan nästa rutt visas
            }

            cui.appendStatus("Alla punkter besökta. Klar!\n");

        } catch (InterruptedException exception) {
            cui.appendStatus("GuiUpdate avbruten.\n");
        }
    }

    // Hjälpmetod för att flytta roboten längs en lista av noder
    private void moveRobotAlongPath(List<Vertex> path) throws InterruptedException {
        for (Vertex v : path) {
            
            // Om användare avbryter körningen helt
            if (ds.isStopped) {
                cui.appendStatus("Körningen avbröts helt.\n");
                return; // Avbryter metoden och slutar åka
            }

            // Om användaren pausat körningen
            while (ds.isPaused) {
                Thread.sleep(200); // Vänta lite och kolla igen
                
                // Om användaren klickar "Nej" medan vi är pausade
                if (ds.isStopped) {
                    cui.appendStatus("Körningen avbröts helt.\n");
                    return; 
                }
            }
            // ---------------------------------------------
            
            int nodeId = Integer.parseInt(v.getId());
            int col = nodeId % ds.columns; // X
            int row = nodeId / ds.columns; // Y

            // Uppdatera robotens position
            ds.robotX = col * ds.gridsize + (ds.gridsize / 2.0);
            ds.robotY = row * ds.gridsize + (ds.gridsize / 2.0);

            cui.repaint(); // Rita om roboten på nya platsen
            Thread.sleep(50); // Hastighet på roboten
        }
    }
}