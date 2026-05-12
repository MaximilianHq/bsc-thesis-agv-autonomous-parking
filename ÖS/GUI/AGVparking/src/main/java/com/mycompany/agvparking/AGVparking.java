/*
 * Click nbfs://nbhost/SystemFileSystem/Templates/Licenses/license-default.txt to change this license
 * Click nbfs://nbhost/SystemFileSystem/Templates/Classes/Class.java to edit this template
 */
package com.mycompany.agvparking;

/**
 *
 * @author KTS - G2 
 */


public class AGVparking {

    DataStore ds;
    ControlUI cui;
    GuiUpdate gui;
    RobotRead rr;

    AGVparking() {

        // Initialize the DataStore call where all "global" data will be stored 
        ds = new DataStore();

        // This sets the file path and read network text file. Adjust for your needs. 
        
        // ds.setFileName("C:\\Users\\fredr\\OneDrive - Linköpings universitet\\TNK132\\TNK132\\GUI\\Lab2\\Autopark-Lab2\\loc.txt");
        ds.setFileName("C:\\Users\\andre\\OneDrive - Linköpings universitet\\GitHub\\Kandidatprojekt\\Kanditatprojekt-2026\\ÖS\\loc_ny.txt");
        ds.setFileName("C:\\Users\\andre\\OneDrive - Linköpings universitet\\GitHub\\Kandidatprojekt\\Kanditatprojekt-2026\\ÖS\\loc_ny.txt"); 
        ds.readCoords(); 

        Object[] options = {"Demo-läge", "Drift-läge"}; 
        int choice = javax.swing.JOptionPane.showOptionDialog( 
        null, 
        "Välj startläge för Autopark:", 
        "Välj startläge för Autopark:",
        "Startinställningar", 
        javax.swing.JOptionPane.YES_NO_OPTION, 
        javax.swing.JOptionPane.QUESTION_MESSAGE, 
        null, 
        options, 
        options[0]); 
        
        boolean isDemo = (choice == 0); 
        
        
        // Initialize and show the GUI. The constructor gets access to the DataStore         
        cui = new ControlUI(ds, isDemo); 
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
        // t2.start(); 
        
        
        OptPlan op = new OptPlan(ds);
        //op.createPlan();
        //cui.repaint();

    }

    public static void main(String[] args) { 
    try { 
            // Ladda det mörka temat som bas 
            com.formdev.flatlaf.FlatDarkLaf.setup(); 

    } catch (Exception ex) {
            System.err.println("Kunde inte starta FlatLaf. " + ex.getMessage());
        } 
        /* This is the "main" method what gets called when the application starts
         * All that is done here is to make an instance of the RobotControl class,
         * and thereby, call the RobotControl constructor.
         */
        AGVparking x = new AGVparking();
    }
}
