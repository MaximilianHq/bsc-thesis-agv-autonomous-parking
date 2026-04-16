package com.mycompany.agvparking; 

public class RobotRead implements Runnable { 

    private ControlUI cui; 
    private DataStore ds; 

    public RobotRead(DataStore ds, ControlUI cui) { 
        this.cui = cui; 
        this.ds = ds; 
    } 

    @Override 
    public void run() { 
        try { 
            cui.appendStatus("RobotRead väntar på kommando...\n"); 
            
            while(!ds.isStopped) { 
                
                // Kolla om vi är igång och har en rutt laddad i ControlUI 
                if (!ds.isPaused && cui.fullDemoPath != null && !cui.fullDemoPath.isEmpty()) { 
                    
                    // KÖR ETT STEG I TAGET (Hämtar magin från updateRobotPosition!) 
                    while (ds.demoStep < cui.fullDemoPath.size()) { 
                        
                        while (ds.isPaused) { 
                            Thread.sleep(500); 
                        } 
                        
                        if (ds.isStopped) break; 
                        
                        // Låt ControlUI klippa ut rätt rutt, flytta pricken och färga rutan röd 
                        cui.updateRobotPosition(); 
                        ds.updateUiflag = true; 
                        
                        // Hastigheten (200 ms per steg) 
                        Thread.sleep(200); 
                        
                        // Gå vidare till nästa steg 
                        ds.demoStep++; 
                    } 
                    
                    // Tillbaka vid startpunkt 
                    if (!ds.isStopped) { 
                        cui.appendStatus("Robot är tillbaka vid startpunkten.\n"); 
                        
                        // Hämta nästa bil! 
                        boolean hasMore = cui.triggerNextRealMission(); 
                        
                        if (!hasMore) { 
                            cui.fullDemoPath = null; 
                            cui.appendStatus("AGV är klar med alla uppdrag i Autoläget \n"); 
                            ds.isPaused = true; 
                        } 
                    } 
                } else { 
                    // Väntar på att Start-knappen ska tryckas 
                    Thread.sleep(500); 
                } 
            } 
        } catch (InterruptedException exception) { 
            cui.appendStatus("RobotRead tråden avbröts.\n"); 
        } 
    } 
} 


// Gammal kod 

//package com.mycompany.agvparking; 
//
//import java.util.Random;
//
//public class RobotRead implements Runnable {
//
//    private int sleepTime;
//    private static Random generator = new Random();
//    private ControlUI cui;
//    private DataStore ds;
//
//    public RobotRead(DataStore ds, ControlUI cui) {
//        this.cui = cui;
//        this.ds = ds;
//        sleepTime = generator.nextInt(20000);
//    }
//
////    @Override
////    public void run() {
////        try {
////            cui.appendStatus("RobotRead kommer att köra i " + sleepTime + " millisekunder.\n"); 
////            int i = 1;
////            while (i <= 20) {
////                Thread.sleep(sleepTime / 20); // senare användas för att välja hur ofta vi ska ta info från AGV
////                // cui.appendStatus("Jag är tråd RobotRead! För " + i + ":te gången.\n"); // ändra till typ position
////                if (i == 10){
////                    ds.updateUiflag = true;
////                }
////                i++;
////            }
////        } catch (InterruptedException exception) {
////        }
////        // cui.appendStatus("RobotRead är nu klar!\n"); // om AGVn utfört alla instruktioner?
////    }
//
//    @Override 
//    public void run() { 
//        try { 
//            cui.appendStatus("RobotRead väntar på kommando...\n"); 
//            
//            while(!ds.isStopped) { 
//                
//                // Kolla om vi är igång och har en rutt laddad i ControlUI
//                if (!ds.isPaused && cui.fullDemoPath != null && !cui.fullDemoPath.isEmpty()) { 
//                    
//                    // Kör så länge vi har steg kvar på hela tur-och-retur-resan
//                    while (ds.demoStep < cui.fullDemoPath.size()) { 
//                        
//                        while (ds.isPaused) { 
//                            Thread.sleep(500); 
//                        } 
//                        
//                        if (ds.isStopped) break; 
//                        
//                        // HÄR ÄR MAGIN: Låt ControlUI uppdatera positionen, 
//                        // klippa ut rätt rutt, och färga parkeringsrutan röd!
//                        cui.updateRobotPosition();
//                        ds.updateUiflag = true; 
//                        
//                        Thread.sleep(200); // Tiden det tar att köra ett steg
//                        
//                        // Gå vidare till nästa steg
//                        ds.demoStep++;
//                    } 
//                    
//                    // Tillbaka vid startpunkt 
//                    if (!ds.isStopped) { 
//                        cui.appendStatus("Robot är tillbaka vid startpunkten.\n"); 
//                        
//                        // Ladda nästa bil!
//                        boolean hasMore = cui.triggerNextRealMission(); 
//                        
//                        if (!hasMore) { 
//                            cui.fullDemoPath = null; 
//                            cui.appendStatus("AGV är klar med alla uppdrag i Autoläget \n"); 
//                            ds.isPaused = true; 
//                        } 
//                    } 
//                } else { 
//                    Thread.sleep(500); 
//                } 
//            } 
//        } catch (InterruptedException exception) {
//            cui.appendStatus("RobotRead tråden avbröts.\n"); 
//        } 
//    } 
//    
////    @Override 
////    public void run() { 
////        try { 
////            cui.appendStatus("RobotRead väntar på kommando...\n"); 
////            
////            while(!ds.isStopped) { 
////                
////                if (!ds.isPaused && ds.currentPath != null && !ds.currentPath.isEmpty()) { 
////                    for (int i = 0; i < ds.currentPath.size(); i++) { 
////                        
////                        while (ds.isPaused) { 
////                            Thread.sleep(500); 
////                        } 
////                        
////                        if (ds.isStopped) break; 
////                        
////                        Vertex v = ds.currentPath.get(i); 
////                        int nodeId = Integer.parseInt(v.getId()); 
////                        
////                        ds.robotX = (nodeId % ds.columns) * ds.gridsize + (ds.gridsize / 2.0); 
////                        ds.robotY = (nodeId / ds.columns) * ds.gridsize + (ds.gridsize / 2.0); 
////                        
////                        for (int j = 1; j < ds.locations; j++) { 
////                            int targetX = (int) (ds.LocationX[j] / ds.gridsize); 
////                            int targetY = (int) (ds.LocationY[j] / ds.gridsize); 
////
////                            if ((int) (ds.robotX / ds.gridsize) == targetX && (int) (ds.robotY / ds.gridsize) == targetY) { 
////                                ds.markAreaAsVisited(ds.robotX, ds.robotY); 
////                            } 
////                        } 
////                        
////                        ds.updateUiflag = true; 
////                        
////                        Thread.sleep(200); 
////                    } 
////                    // Tillbaka vid startpunkt 
////                    if (!ds.isStopped) { 
////                        cui.appendStatus("Robot är tillbaka vid startpunkten.\n"); 
////                        
////                        boolean hasMore = cui.triggerNextRealMission(); 
////                        
////                        if (!hasMore) { 
////                            ds.currentPath = null; 
////                            cui.appendStatus("AGV är klar med alla uppdrag i Autoläget \n"); 
////                            ds.isPaused = true; 
////                        } 
////                    } 
////                } else { 
////                    Thread.sleep(500); 
////                } 
////            } 
////        } catch (InterruptedException exception) {
////            cui.appendStatus("RobotRead tråden avbröts.\n"); 
////        } 
////    } 
//} 
