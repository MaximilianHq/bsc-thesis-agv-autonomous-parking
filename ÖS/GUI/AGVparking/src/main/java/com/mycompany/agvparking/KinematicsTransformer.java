package com.mycompany.agvparking;

import java.util.ArrayList;
import java.util.List;

public class KinematicsTransformer {
    private DataStore ds;

    public KinematicsTransformer(DataStore ds) {
        this.ds = ds;
    }

    public List<RobotState> generateRotationOnSpot(RobotState lastState, double targetAngleRad) {
        List<RobotState> maneuver = new ArrayList<>();
        double L = ds.agvOffset * ds.gridsize;
        double currentAngle = lastState.angle;
        
        // Räkna ut kortaste vägen för rotationen
        double diff = targetAngleRad - currentAngle;
        while (diff > Math.PI) diff -= 2 * Math.PI;
        while (diff < -Math.PI) diff += 2 * Math.PI;

        // Om vi redan står rätt (mindre än 1 grads diff), gör ingenting
        if (Math.abs(diff) < 0.02) return maneuver;

        int steps = 40; // Samma antal steg som i hörnsvängar
        for (int i = 1; i <= steps; i++) {
            double tempAngle = currentAngle + (diff * i / steps);
            // AGV:n står kvar på samma punkt, men bakaxeln (axleX/Y) svingar runt den
            maneuver.add(new RobotState(lastState.agvX, lastState.agvY, 
                                        lastState.agvX - L * Math.cos(tempAngle), 
                                        lastState.agvY - L * Math.sin(tempAngle), 
                                        tempAngle, false));
        }
        return maneuver;
    }
    // --- TRANSPORTSTRÄCKAN (På väg till mål eller bas) ---
    public List<RobotState> transformPath(List<Vertex> path, boolean loaded, RobotState startState) {
        List<RobotState> detailedPath = new ArrayList<>();
        if (path == null || path.size() < 2) return detailedPath;

        double L = ds.agvOffset * ds.gridsize; 
        
        double currentHeading;
        
        // gridX och gridY är punkterna på den svarta linjen
        double gridX = (Integer.parseInt(path.get(0).getId()) % ds.columns) * ds.gridsize + (ds.gridsize / 2.0);
        double gridY = (Integer.parseInt(path.get(0).getId()) / ds.columns) * ds.gridsize + (ds.gridsize / 2.0);

        // Säkerställ att vi fortsätter exakt där vi slutade
        if (startState != null) {
            currentHeading = startState.angle;
            if (loaded) {
                gridX = startState.axleX;
                gridY = startState.axleY;
            } else {
                gridX = startState.agvX;
                gridY = startState.agvY;
            }
        } else {
            double nextGridX = (Integer.parseInt(path.get(1).getId()) % ds.columns) * ds.gridsize + (ds.gridsize / 2.0);
            double nextGridY = (Integer.parseInt(path.get(1).getId()) / ds.columns) * ds.gridsize + (ds.gridsize / 2.0);
            currentHeading = Math.atan2(nextGridY - gridY, nextGridX - gridX);
        }

// --- STARTPOSITIONEN ---
        if (loaded && startState == null) {
            // NYTT: "Inkörning" från utsidan!
            // AGV:n står på startpunkten och bakaxeln är utanför (till vänster).
            // AGV:n drar fram ekipaget tills bakaxeln hamnar på startnoden.
            int pullSteps = Math.max(10, (int)(L * 1.5)); 
            double startAxleX = gridX - L * Math.cos(currentHeading);
            double startAxleY = gridY - L * Math.sin(currentHeading);
            
            for (int i = 0; i <= pullSteps; i++) {
                double ax = startAxleX + (gridX - startAxleX) * ((double)i / pullSteps);
                double ay = startAxleY + (gridY - startAxleY) * ((double)i / pullSteps);
                detailedPath.add(new RobotState(ax + L * Math.cos(currentHeading), ay + L * Math.sin(currentHeading), ax, ay, currentHeading, true));
            }
        } else if (loaded && startState != null) {
            // Bakaxeln på rutnätet, AGV:n hamnar längre fram
            detailedPath.add(new RobotState(gridX + L * Math.cos(currentHeading), gridY + L * Math.sin(currentHeading), gridX, gridY, currentHeading, true));
        } else {
            // AGV:n på rutnätet, dragstången släpar bakom
            detailedPath.add(new RobotState(gridX, gridY, gridX - L * Math.cos(currentHeading), gridY - L * Math.sin(currentHeading), currentHeading, false));
        }

        // --- LOOPA IGENOM RUTTEN ---
        for (int i = 1; i < path.size(); i++) {
            double nextGridX = (Integer.parseInt(path.get(i).getId()) % ds.columns) * ds.gridsize + (ds.gridsize / 2.0);
            double nextGridY = (Integer.parseInt(path.get(i).getId()) / ds.columns) * ds.gridsize + (ds.gridsize / 2.0);

            double dist = Math.hypot(nextGridX - gridX, nextGridY - gridY);
            if (dist < 0.1) continue; 

            double targetHeading = Math.atan2(nextGridY - gridY, nextGridX - gridX);
            double diff = targetHeading - currentHeading;
            while (diff > Math.PI) diff -= 2 * Math.PI;
            while (diff < -Math.PI) diff += 2 * Math.PI;

            // --- SVÄNGAR I HÖRN ---
            if (Math.abs(diff) > 0.1) {
                int rotationSteps = 40; 
                for (int s = 1; s <= rotationSteps; s++) {
                    double tempAngle = currentHeading + ((double)s / rotationSteps * diff);
                    if (loaded) {
                        // Bakaxeln står STILLA på svarta hörn-punkten. AGV:n svingar ut i en stor BLÅ båge.
                        detailedPath.add(new RobotState(gridX + L * Math.cos(tempAngle), gridY + L * Math.sin(tempAngle), gridX, gridY, tempAngle, true));
                    } else {
                        // AGV:n står stilla på svarta punkten och roterar.
                        detailedPath.add(new RobotState(gridX, gridY, gridX - L * Math.cos(tempAngle), gridY - L * Math.sin(tempAngle), tempAngle, false));
                    }
                }
            }
            currentHeading = targetHeading;

            // --- KÖR RAKT FRAM ---
            int driveSteps = Math.max(10, (int)(dist * 1.5)); 
            for (int s = 1; s <= driveSteps; s++) {
                double t = (double) s / driveSteps;
                double currX = gridX + t * (nextGridX - gridX);
                double currY = gridY + t * (nextGridY - gridY);
                
                if (loaded) {
                    // Bakaxeln (currX/currY) ligger på svarta linjen!
                    detailedPath.add(new RobotState(currX + L * Math.cos(currentHeading), currY + L * Math.sin(currentHeading), currX, currY, currentHeading, true));
                } else {
                    // AGV:n (currX/currY) ligger på svarta linjen!
                    detailedPath.add(new RobotState(currX, currY, currX - L * Math.cos(currentHeading), currY - L * Math.sin(currentHeading), currentHeading, false));
                }
            }
            gridX = nextGridX;
            gridY = nextGridY;
        }
        return detailedPath;
    }

    // ========================================================================
    // PARKERINGSMANÖVRARNA (Oförändrade, exakt som du ville ha dem)
    // ========================================================================

    // MANÖVER 5-9: LÅGA RUTOR
    public List<RobotState> generateParkingManeuver(RobotState startState, int destX_grid, int destY_grid) {
        List<RobotState> maneuver = new ArrayList<>();
        double L = ds.agvOffset * ds.gridsize; 
        
        double axleX = startState.axleX;
        double axleY = startState.axleY;
        double angle = startState.angle;
        
        for(int i = 0; i < 50; i++) maneuver.add(new RobotState(axleX + L*Math.cos(angle), axleY + L*Math.sin(angle), axleX, axleY, angle, true));
        
        double targetAngle = -Math.PI / 2;
        double angleDiff = targetAngle - angle;
        while(angleDiff > Math.PI) angleDiff -= 2*Math.PI;
        while(angleDiff < -Math.PI) angleDiff += 2*Math.PI;
        
        int steps = 100; 
        for(int i = 1; i <= steps; i++) {
            double stepAngle = angle + (angleDiff * i / steps);
            double agvX = axleX + L * Math.cos(stepAngle); 
            double agvY = axleY + L * Math.sin(stepAngle);
            maneuver.add(new RobotState(agvX, agvY, axleX, axleY, stepAngle, true));
        }
        angle = targetAngle; 
        
        double backDist = 150.0;
        steps = 150;
        for(int i = 1; i <= steps; i++) {
            axleX -= (backDist / steps) * Math.cos(angle);
            axleY -= (backDist / steps) * Math.sin(angle);
            double agvX = axleX + L * Math.cos(angle);
            double agvY = axleY + L * Math.sin(angle);
            maneuver.add(new RobotState(agvX, agvY, axleX, axleY, angle, true));
        }
        
        for(int i = 0; i < 50; i++) maneuver.add(new RobotState(axleX + L*Math.cos(angle), axleY + L*Math.sin(angle), axleX, axleY, angle, false));
        
        double agvX = axleX + L*Math.cos(angle);
        double agvY = axleY + L*Math.sin(angle);
        double forwardDist = 75.0;
        steps = 75;
        for(int i = 1; i <= steps; i++) {
            agvX += (forwardDist / steps) * Math.cos(angle);
            agvY += (forwardDist / steps) * Math.sin(angle);
            maneuver.add(new RobotState(agvX, agvY, agvX - L*Math.cos(angle), agvY - L*Math.sin(angle), angle, false));
        } 
        for(int i = 0; i < 50; i++) maneuver.add(new RobotState(agvX, agvY, agvX - L*Math.cos(angle), agvY - L*Math.sin(angle), angle, false)); 
        return maneuver; 
    }
    
    // MANÖVER RUTA 1 
    public List<RobotState> generateComplexParkingManeuver(RobotState startState, int destX_grid, int destY_grid) {
        List<RobotState> maneuver = new ArrayList<>();
        double L = ds.agvOffset * ds.gridsize; 
        
        // Bakaxeln står redan på den gröna pricken
        double axleX = startState.axleX;
        double axleY = startState.axleY;
        double angle = startState.angle;
        
        for(int i = 0; i < 50; i++) maneuver.add(new RobotState(axleX + L*Math.cos(angle), axleY + L*Math.sin(angle), axleX, axleY, angle, true));

        // 1. Rotera 180 grader VÄNSTER runt BAKAXELN (-Math.PI = vänster i GUI)
        double angleDiff = -Math.PI; 
        int steps = 150; 
        for(int i = 1; i <= steps; i++) {
            double stepAngle = angle + (angleDiff * i / steps);
            double agvX = axleX + L * Math.cos(stepAngle); 
            double agvY = axleY + L * Math.sin(stepAngle);
            maneuver.add(new RobotState(agvX, agvY, axleX, axleY, stepAngle, true));
        }
        angle += angleDiff;
        
        while(angle > Math.PI) angle -= 2*Math.PI;
        while(angle < -Math.PI) angle += 2*Math.PI;
        
        // 2. Backa 80 cm neråt i rutan
        double backDist = 80.0;
        steps = 80;
        for(int i = 1; i <= steps; i++) {
            axleX -= (backDist / steps) * Math.cos(angle);
            axleY -= (backDist / steps) * Math.sin(angle);
            double agvX = axleX + L * Math.cos(angle);
            double agvY = axleY + L * Math.sin(angle);
            maneuver.add(new RobotState(agvX, agvY, axleX, axleY, angle, true));
        }

        // 3. Sväng 90 grader HÖGER runt BAKAXELN (Fronten blir riktad vänster)
        angleDiff = Math.PI / 2; // +Math.PI/2 = höger i GUI
        steps = 80;
        for(int i = 1; i <= steps; i++) {
            double stepAngle = angle + (angleDiff * i / steps);
            // ÄNDRING: AGV:n svingas nu runt bakaxeln (som ligger stilla!)
            double agvX = axleX + L * Math.cos(stepAngle);
            double agvY = axleY + L * Math.sin(stepAngle);
            maneuver.add(new RobotState(agvX, agvY, axleX, axleY, stepAngle, true));
        }
        angle += angleDiff;

        while(angle > Math.PI) angle -= 2*Math.PI;
        while(angle < -Math.PI) angle += 2*Math.PI;

        // Skapa den slutgiltiga agvX/agvY för nästa steg
        double finalAgvX = axleX + L * Math.cos(angle); 
        double finalAgvY = axleY + L * Math.sin(angle);

        // 4. Lasta av och pausa
        for(int i = 0; i < 50; i++) maneuver.add(new RobotState(finalAgvX, finalAgvY, axleX, axleY, angle, false));

        // 5. Kör 30 cm framåt
        double forwardDist = 30.0;
        steps = 30;
        // 5. Kör 25 cm framåt
        double forwardDist = 25.0;
        steps = 25;
        for(int i = 1; i <= steps; i++) {
            finalAgvX += (forwardDist / steps) * Math.cos(angle);
            finalAgvY += (forwardDist / steps) * Math.sin(angle);
            maneuver.add(new RobotState(finalAgvX, finalAgvY, finalAgvX - L*Math.cos(angle), finalAgvY - L*Math.sin(angle), angle, false));
        }

        // 6. Kör 40 cm i sidled åt vänster (neråt i bild)
        double sideDist = 40.0;
        steps = 40;
        double sideAngle = angle - (Math.PI / 2); 
        
        for(int i = 1; i <= steps; i++) {
            finalAgvX += (sideDist / steps) * Math.cos(sideAngle);
            finalAgvY += (sideDist / steps) * Math.sin(sideAngle);
            maneuver.add(new RobotState(finalAgvX, finalAgvY, finalAgvX - L*Math.cos(angle), finalAgvY - L*Math.sin(angle), angle, false));
        } 
        for(int i = 0; i < 50; i++) maneuver.add(new RobotState(finalAgvX, finalAgvY, finalAgvX - L*Math.cos(angle), finalAgvY - L*Math.sin(angle), angle, false)); 
        return maneuver;
    } 
    
        // MANÖVER RUTA 2-4 
    public List<RobotState> generateTopParkingManeuver(RobotState startState, int destX_grid, int destY_grid) {
        List<RobotState> maneuver = new ArrayList<>();
        double L = ds.agvOffset * ds.gridsize; 
        
        // Bakaxeln står redan på den gröna pricken
        double axleX = startState.axleX;
        double axleY = startState.axleY;
        double angle = startState.angle;
        
        for(int i = 0; i < 50; i++) maneuver.add(new RobotState(axleX + L*Math.cos(angle), axleY + L*Math.sin(angle), axleX, axleY, angle, true));

        // 1. Rotera 180 grader Höger runt BAKAXELN (Math.PI = höger i GUI)
        double angleDiff = Math.PI; 
        int steps = 150; 
        for(int i = 1; i <= steps; i++) {
            double stepAngle = angle + (angleDiff * i / steps);
            double agvX = axleX + L * Math.cos(stepAngle); 
            double agvY = axleY + L * Math.sin(stepAngle);
            maneuver.add(new RobotState(agvX, agvY, axleX, axleY, stepAngle, true));
        }
        angle += angleDiff;
        
        while(angle > Math.PI) angle -= 2*Math.PI;
        while(angle < -Math.PI) angle += 2*Math.PI;
        
        // 2. Backa 80 cm in i rutan
        double backDist = 80.0;
        steps = 80;
        for(int i = 1; i <= steps; i++) {
            axleX -= (backDist / steps) * Math.cos(angle);
            axleY -= (backDist / steps) * Math.sin(angle);
            double agvX = axleX + L * Math.cos(angle);
            double agvY = axleY + L * Math.sin(angle);
            maneuver.add(new RobotState(agvX, agvY, axleX, axleY, angle, true));
        }

        // 3. Sväng 90 grader VÄNSTER runt BAKAXELN (Fronten blir riktad höger)
        angleDiff = -Math.PI / 2; // +Math.PI/2 = höger i GUI
        steps = 80;
        for(int i = 1; i <= steps; i++) {
            double stepAngle = angle + (angleDiff * i / steps);
            // ÄNDRING: AGV:n svingas nu runt bakaxeln (som ligger stilla!)
            double agvX = axleX + L * Math.cos(stepAngle);
            double agvY = axleY + L * Math.sin(stepAngle);
            maneuver.add(new RobotState(agvX, agvY, axleX, axleY, stepAngle, true));
        }
        angle += angleDiff;

        while(angle > Math.PI) angle -= 2*Math.PI;
        while(angle < -Math.PI) angle += 2*Math.PI;

        // Skapa den slutgiltiga agvX/agvY för nästa steg
        double finalAgvX = axleX + L * Math.cos(angle); 
        double finalAgvY = axleY + L * Math.sin(angle);

        // 4. Lasta av och pausa
        for(int i = 0; i < 50; i++) maneuver.add(new RobotState(finalAgvX, finalAgvY, axleX, axleY, angle, false));

        // 5. Kör 20 cm framåt // Oklart vad som krävs för att inte träffa väggen med AGVn 
        double forwardDist = 20.0;
        steps = 20;
        // 5. Kör 15 cm framåt // Oklart vad som krävs för att inte träffa väggen med AGVn 
        double forwardDist = 15.0;
        steps = 15;
        for(int i = 1; i <= steps; i++) {
            finalAgvX += (forwardDist / steps) * Math.cos(angle);
            finalAgvY += (forwardDist / steps) * Math.sin(angle);
            maneuver.add(new RobotState(finalAgvX, finalAgvY, finalAgvX - L*Math.cos(angle), finalAgvY - L*Math.sin(angle), angle, false));
        }

        // 6. Kör 40 cm i sidled åt höger (neråt i bild)
        double sideDist = -40.0;
        steps = 40;
        double sideAngle = angle - (Math.PI / 2); 
        
        for(int i = 1; i <= steps; i++) {
            finalAgvX += (sideDist / steps) * Math.cos(sideAngle);
            finalAgvY += (sideDist / steps) * Math.sin(sideAngle);
            maneuver.add(new RobotState(finalAgvX, finalAgvY, finalAgvX - L*Math.cos(angle), finalAgvY - L*Math.sin(angle), angle, false));
        } 
        
        for(int i = 0; i < 50; i++) maneuver.add(new RobotState(finalAgvX, finalAgvY, finalAgvX - L*Math.cos(angle), finalAgvY - L*Math.sin(angle), angle, false)); 
        return maneuver;
    } 
    
    // MANÖVER FÖR RUTA 10 
    public List<RobotState> generateSpot10Maneuver(RobotState startState) {
        List<RobotState> maneuver = new ArrayList<>();
        double L = ds.agvOffset * ds.gridsize; 
        
        // Bakaxeln står redan på pricken
        double axleX = startState.axleX;
        double axleY = startState.axleY;
        double angle = startState.angle;
        
        // 0. Paus vid ankomst
        for(int i = 0; i < 50; i++) maneuver.add(new RobotState(axleX + L*Math.cos(angle), axleY + L*Math.sin(angle), axleX, axleY, angle, true));

        // 1. Rotera 180 grader åt VÄNSTER runt bakaxeln (-Math.PI = vänster i GUI)
        double angleDiff = -Math.PI; 
        int steps = 150; 
        for(int i = 1; i <= steps; i++) {
            double stepAngle = angle + (angleDiff * i / steps);
            double agvX = axleX + L * Math.cos(stepAngle); 
            double agvY = axleY + L * Math.sin(stepAngle);
            maneuver.add(new RobotState(agvX, agvY, axleX, axleY, stepAngle, true));
        }
        angle += angleDiff;
        while(angle > Math.PI) angle -= 2*Math.PI;
        while(angle < -Math.PI) angle += 2*Math.PI;

        // 2. Backa 50 cm
        double backDist = 50.0;
        steps = 50; 
        for(int i = 1; i <= steps; i++) {
            axleX -= (backDist / steps) * Math.cos(angle);
            axleY -= (backDist / steps) * Math.sin(angle);
            double agvX = axleX + L * Math.cos(angle);
            double agvY = axleY + L * Math.sin(angle);
            maneuver.add(new RobotState(agvX, agvY, axleX, axleY, angle, true));
        }

        // 3. Rotera 90 grader HÖGER runt bakaxeln (+Math.PI/2 = höger)
        angleDiff = Math.PI / 2;
        steps = 80;
        for(int i = 1; i <= steps; i++) {
            double stepAngle = angle + (angleDiff * i / steps);
            double agvX = axleX + L * Math.cos(stepAngle);
            double agvY = axleY + L * Math.sin(stepAngle);
            maneuver.add(new RobotState(agvX, agvY, axleX, axleY, stepAngle, true));
        }
        angle += angleDiff;
        while(angle > Math.PI) angle -= 2*Math.PI;
        while(angle < -Math.PI) angle += 2*Math.PI;

        // 4. Backa 150 cm
        backDist = 150.0;
        steps = 150; // Lite fler steg för att backa långsamt och snyggt
        for(int i = 1; i <= steps; i++) {
            axleX -= (backDist / steps) * Math.cos(angle);
            axleY -= (backDist / steps) * Math.sin(angle);
            double agvX = axleX + L * Math.cos(angle);
            double agvY = axleY + L * Math.sin(angle);
            maneuver.add(new RobotState(agvX, agvY, axleX, axleY, angle, true));
        }

        // Räkna ut AGV:ns position för de sista stegen
        double finalAgvX = axleX + L * Math.cos(angle);
        double finalAgvY = axleY + L * Math.sin(angle);

        // 5. Lasta av bilen och pausa
        for(int i = 0; i < 50; i++) maneuver.add(new RobotState(finalAgvX, finalAgvY, axleX, axleY, angle, false));

        // 6. Kör 60 cm framåt
        double forwardDist = 75.0;
        steps = 75;
        for(int i = 1; i <= steps; i++) {
            finalAgvX += (forwardDist / steps) * Math.cos(angle);
            finalAgvY += (forwardDist / steps) * Math.sin(angle);
            maneuver.add(new RobotState(finalAgvX, finalAgvY, finalAgvX - L*Math.cos(angle), finalAgvY - L*Math.sin(angle), angle, false));
        } 
        return maneuver;
    }     
}