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
                if (!ds.isPaused && cui.masterDemoPath != null && !cui.masterDemoPath.isEmpty()) { 
                    
                    // KÖR ETT STEG I TAGET (Hämtar magin från updateRobotPosition!) 
                    while (ds.demoStep < cui.masterDemoPath.size()) { 
                        
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
                            cui.masterDemoPath = null; 
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


