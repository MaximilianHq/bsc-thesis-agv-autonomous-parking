package com.mycompany.agvparking;

import java.io.File;
import java.util.List;
import java.util.Scanner;

/**
 *
 * @author clary35
 */
public class DataStore {

    String fileName = null;
    double[] LocationX;
    double[] LocationY;
    int[][] ObstacleMatrix;
    boolean DataAvail;
    int xsize, ysize, gridsize;
    int locations;

    /* Del 5 och 6
        *
        *
        *
     */
    // Boolean för att pausa GuiUpdate
    boolean updateUiflag;
    public volatile boolean isPaused = false;
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

}
