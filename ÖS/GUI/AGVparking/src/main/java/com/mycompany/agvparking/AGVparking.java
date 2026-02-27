/*
 * Click nbfs://nbhost/SystemFileSystem/Templates/Licenses/license-default.txt to change this license
 * Click nbfs://nbhost/SystemFileSystem/Templates/Classes/Class.java to edit this template
 */
package com.mycompany.agvparking;

/**
 *
 * @author fredr
 */


public class AGVparking {

    DataStore ds;
    ControlUI cui;
    GuiUpdate gui;
    RobotRead rr;

    AGVparking() {

        /*
         * Initialize the DataStore call where all "global" data will be stored
         */
        ds = new DataStore();

        /*
         * This sets the file path and read network text file. Adjust for your needs.
         */
        ds.setFileName("C:\\Users\\fredr\\OneDrive - Linköpings universitet\\TNK132\\TNK132\\GUI\\Lab2\\Autopark-Lab2\\loc.txt");
        ds.readCoords();
        /*
         * Initialize and show the GUI. The constructor gets access to the DataStore
         */
        cui = new ControlUI(ds);
        cui.setVisible(true);
        cui.showStatus();

        /* Initialize the objects rr and gui
         */
        rr = new RobotRead(ds, cui);
        gui = new GuiUpdate(ds, cui);

        // 3. Skapa TRÅDARNA (Threads) och ge dem objekten (som är Runnable)
        Thread t1 = new Thread(rr);
        Thread t2 = new Thread(gui);

        // 4. Starta trådarna
        t1.start();
        t2.start();
        
        
        OptPlan op = new OptPlan(ds);
        //op.createPlan();
        //cui.repaint();

    }

    public static void main(String[] args) {

        /* This is the "main" method what gets called when the application starts
         * All that is done here is to make an instance of the RobotControl class,
         * and thereby, call the RobotControl constructor.
         */
        AGVparking x = new AGVparking();
    }
}
