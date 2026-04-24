package com.mycompany.agvparking;

import java.io.File;
import java.util.List;
import java.util.Scanner;
import java.util.LinkedList;
import java.util.Queue;

/**
 *
 * @author kts grupp 2
 */
public class DataStore {

    String fileName = null;
    double[] LocationX;
    double[] LocationY;
    int[][] ObstacleMatrix;
    boolean DataAvail;
    int xsize, ysize, gridsize;
    int locations;

    public int demoStep = 0; // Håller koll på var i listan vi är
    public double agvAngle = 0.0;

    
    boolean updateUiflag;
    public volatile boolean isPaused = true;
    public volatile boolean isStopped = false;

    // AGV koordinater
    double robotX;
    double robotY;

    // Parkeringsplatser, H: horisontella, V: vertikala
    double[] HLocationX;
    double[] HLocationY;
    double[] VLocationX;
    double[] VLocationY;
    int Hspaces;
    int Vspaces;

    int rows;
    int columns;
    public List<Vertex> currentPath;
    
    public Queue<AgvInstruction> instructionQueue;
    double robotAngle;

    public DataStore() {
        // Initialize the datastore with fixed size arrays for storing the network data
        LocationX = new double[20];
        LocationY = new double[20];
        ObstacleMatrix = new int[541][361];
        DataAvail = false;
        xsize = 540;
        ysize = 360;
        gridsize = 10;
        updateUiflag = false;

        // Parkeringsplatser, H: horisontella, V: vertikala    
        HLocationX = new double[30];
        HLocationY = new double[30];
        VLocationX = new double[30];
        VLocationY = new double[30];

        columns = xsize / gridsize;
        rows = ysize / gridsize;

        currentPath = null;
        
        instructionQueue = new LinkedList<>();

    }

    public void setFileName(String newFileName) {
        this.fileName = newFileName;
    }

    public String getFileName() {
        return fileName;
    }

    public void readCoords() {
        String line;

        if (fileName == null) {
            System.err.println("No file name set. Data read aborted.");
            return;
        }
        try {
            File file = new File(fileName);
            Scanner scanner = new Scanner(file, "iso-8859-1");
            String[] sline;

            line = scanner.nextLine();
            locations = Integer.parseInt(line.trim());
            System.out.println("No locations to be read: " + locations);

            // Read all points as, x, y
            for (int i = 0; i < locations; i++) {
                line = scanner.nextLine();
                //split space separated data on line
                sline = line.split(", ");
                LocationX[i] = Double.parseDouble(sline[0].trim());
                LocationY[i] = Double.parseDouble(sline[1].trim());
            }

            line = scanner.nextLine(); // Behövs tydligen en för varje lol
            Hspaces = Integer.parseInt(line.trim());
            System.out.println("No parkingspaces to be read: " + Hspaces);

            // Read all points as, x, y
            for (int i = 0; i < Hspaces; i++) {
                line = scanner.nextLine();
                //split space separated data on line
                sline = line.split(", ");
                HLocationX[i] = Double.parseDouble(sline[0].trim());
                HLocationY[i] = Double.parseDouble(sline[1].trim());
            }

            line = scanner.nextLine();
            Vspaces = Integer.parseInt(line.trim());
            System.out.println("No parkingspaces to be read: " + Vspaces);

            // Read all points as, x, y
            for (int i = 0; i < Vspaces; i++) {
                line = scanner.nextLine();
                //split space separated data on line
                sline = line.split(", ");
                VLocationX[i] = Double.parseDouble(sline[0].trim());
                VLocationY[i] = Double.parseDouble(sline[1].trim());
            }

            DataAvail = true;
            // Debug printout
            System.out.println("Location A: " + LocationX[0] + " " + LocationY[0]);
            System.out.println("Location B: " + LocationX[1] + " " + LocationY[1]);

        } catch (Exception e) {
            // TODO Auto-generated catch block
            e.printStackTrace();
        }
        robotX = LocationX[0];
        robotY = LocationY[0];
    }

    public void markAreaAsVisited(double xCoord, double yCoord) {
    // ökar marginalen lite (gridsize * 4 istället för 2) så att den 
    // garanterat hittar rutan även om roboten inte stannar exakt i mitten.
    double tolerance = gridsize * 4; 

    // 1. Kolla om vi står på en Horisontell ruta
    for (int k = 0; k < Hspaces; k++) {
        double dist = Math.sqrt(Math.pow(xCoord - HLocationX[k], 2) + Math.pow(yCoord - HLocationY[k], 2));
        
        if (dist < tolerance) {
            int xpos = (int) (HLocationX[k] / gridsize);
            int ypos = (int) (HLocationY[k] / gridsize);

            for (int i = 0; i < (int) (110 / gridsize); i++) {
                for (int j = 0; j < (int) (60 / gridsize); j++) {
                    if (xpos + i < columns && ypos + j < rows) {
                        ObstacleMatrix[xpos + i][ypos + j] = 2;
                    }
                }
            }
            return;
        }
    }

    // Kolla om vi står på en Vertikal ruta
    for (int k = 0; k < Vspaces; k++) {
        double dist = Math.sqrt(Math.pow(xCoord - VLocationX[k], 2) + Math.pow(yCoord - VLocationY[k], 2));
        
        if (dist < tolerance) {
            int xpos = (int) (VLocationX[k] / gridsize);
            int ypos = (int) (VLocationY[k] / gridsize);

            for (int i = 0; i < (int) (60 / gridsize); i++) {
                for (int j = 0; j <= (int) (110 / gridsize); j++) {
                    if (xpos + i < columns && ypos + j < rows) {
                        ObstacleMatrix[xpos + i][ypos + j] = 2;
                    }
                }
            }
            return;
        }
    }
}
    // Återställer alla besökta (röda) rutor till vanliga (grå/svarta) rutor
    public void clearVisitedAreas() {
        for (int i = 0; i < columns; i++) {
            for (int j = 0; j < rows; j++) {
                // Om värdet är 2 (besökt mål), ändra tillbaka till 1 (vanligt hinder/parkering)
                if (ObstacleMatrix[i][j] == 2) {
                    ObstacleMatrix[i][j] = 1;
                }
            }
        }
    }
}

