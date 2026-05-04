package com.mycompany.agvparking;

import java.util.ArrayList;
import java.util.List;

public class KinematicsTransformer {
    private DataStore ds;

    public KinematicsTransformer(DataStore ds) {
        this.ds = ds;
    }

    public List<RobotState> transformPath(List<Vertex> path, boolean loaded) {
        List<RobotState> detailedPath = new ArrayList<>();
        if (path == null || path.size() < 2) return detailedPath;

        double L = ds.agvOffset * ds.gridsize;

        // --- 1. LÖSNING PÅ START-GLITCHEN ---
        // Räkna ut vinkeln mot det allra första målet
        int id0 = Integer.parseInt(path.get(0).getId());
        int id1 = Integer.parseInt(path.get(1).getId());
        double cx0 = (id0 % ds.columns) * ds.gridsize + (ds.gridsize / 2.0);
        double cy0 = (id0 / ds.columns) * ds.gridsize + (ds.gridsize / 2.0);
        double nx0 = (id1 % ds.columns) * ds.gridsize + (ds.gridsize / 2.0);
        double ny0 = (id1 / ds.columns) * ds.gridsize + (ds.gridsize / 2.0);
        double currentHeading = Math.atan2(ny0 - cy0, nx0 - cx0);

        // Lägg in startpositionen så att ekipaget är färdig-roterat redan innan det rullar!
        if (loaded) {
            detailedPath.add(new RobotState(cx0 + L * Math.cos(currentHeading), cy0 + L * Math.sin(currentHeading), cx0, cy0, currentHeading, true));
        } else {
            detailedPath.add(new RobotState(cx0, cy0, cx0, cy0, currentHeading, false));
        }

        // --- 2. BYGG RESTEN AV RUTTEN (Både raksträckor och svängar) ---
        for (int i = 0; i < path.size() - 1; i++) {
            Vertex curr = path.get(i);
            Vertex next = path.get(i + 1);

            double cx = (Integer.parseInt(curr.getId()) % ds.columns) * ds.gridsize + (ds.gridsize / 2.0);
            double cy = (Integer.parseInt(curr.getId()) / ds.columns) * ds.gridsize + (ds.gridsize / 2.0);
            double nx = (Integer.parseInt(next.getId()) % ds.columns) * ds.gridsize + (ds.gridsize / 2.0);
            double ny = (Integer.parseInt(next.getId()) / ds.columns) * ds.gridsize + (ds.gridsize / 2.0);

            double targetHeading = Math.atan2(ny - cy, nx - cx);

            double diff = targetHeading - currentHeading;
            while (diff > Math.PI) diff -= 2 * Math.PI;
            while (diff < -Math.PI) diff += 2 * Math.PI;

            // Om vi svänger (skillnaden i vinkel är större än lite brus)
            if (Math.abs(diff) > 0.1) {
                int rotationSteps = 15; // Animationens mjukhet
                for (int s = 1; s <= rotationSteps; s++) {
                    double t = (double) s / rotationSteps;
                    double tempAngle = currentHeading + (t * diff);
                    
                    if (loaded) {
                        // BAKAXELN FRYSER. AGV:n svingar runt cx, cy. (PIVOT-MANÖVERN)
                        double rx = cx + L * Math.cos(tempAngle);
                        double ry = cy + L * Math.sin(tempAngle);
                        detailedPath.add(new RobotState(rx, ry, cx, cy, tempAngle, true));
                    } else {
                        // Tom AGV snurrar bara runt sin egen axel på noden
                        detailedPath.add(new RobotState(cx, cy, cx, cy, tempAngle, false));
                    }
                }
            }
            currentHeading = targetHeading;

            // RAKSTRÄCKA
            int driveSteps = 10; 
            for (int s = 1; s <= driveSteps; s++) {
                double t = (double) s / driveSteps;
                double currentAxleX = cx + t * (nx - cx);
                double currentAxleY = cy + t * (ny - cy);
                
                if (loaded) {
                    double rx = currentAxleX + L * Math.cos(currentHeading);
                    double ry = currentAxleY + L * Math.sin(currentHeading);
                    detailedPath.add(new RobotState(rx, ry, currentAxleX, currentAxleY, currentHeading, true));
                } else {
                    detailedPath.add(new RobotState(currentAxleX, currentAxleY, currentAxleX, currentAxleY, currentHeading, false));
                }
            }
        }
        return detailedPath;
    }
}