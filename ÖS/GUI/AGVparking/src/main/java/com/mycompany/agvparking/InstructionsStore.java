/*
 * Click nbfs://nbhost/SystemFileSystem/Templates/Licenses/license-default.txt to change this license
 * Click nbfs://nbhost/SystemFileSystem/Templates/Classes/Class.java to edit this template
 */
package com.mycompany.agvparking;

/**
 *
 * @author kts
 * 
 * NOTE!
 * VI BEHÖVER SE ÖVER värden för instruktioner 1-8 alt. 1-12 då 9=0x09 -> error! För nu: 99
 */
public class InstructionsStore {
    // Instruktioner 1-4
    public static final int MOVE_UP = 1;
    public static final int MOVE_DOWN = 2;
    public static final int MOVE_RIGHT = 3;
    public static final int MOVE_LEFT = 4;
    
    // Instruktioner 5-8. Dessa ska endast kunna användas då AGV ej är lastad
    public static final int MOVE_DIAG_UP_RIGHT = 5;
    public static final int MOVE_DIAG_UP_LEFT = 6;
    public static final int MOVE_DIAG_DOWN_RIGHT = 7;
    public static final int MOVE_DIAG_DOWN_LEFT = 99;
    
    // Instruktioner 9-12 med höger / vänster
    public static final int TURNING_RIGHT = 0x08;
    public static final int TURNING_LEFT = 0x09;
    public static final int CURVED_TRAJECTORY_RIGHT = 0x0A;
    public static final int CURVED_TRAJECTORY_LEFT = 0x0B;
    public static final int LATERAL_ARC_RIGHT = 0x0C;
    public static final int LATERAL_ARC_LEFT = 0x0D;
    
    
    // Ytterligare instruktioner (FYLL PÅ VID BEHOV!)
    public static final int START_PARKING = 13;
    public static final int STOP = 14;

    /**
     * Översätter dx och dy till en rörelseinstruktion.
     */
    public static int getDirectionFromDelta(int dx, int dy) {
        if (dx == 0 && dy == -1) return MOVE_UP;
        if (dx == 0 && dy == 1)  return MOVE_DOWN;
        if (dx == 1 && dy == 0)  return MOVE_RIGHT;
        if (dx == -1 && dy == 0) return MOVE_LEFT;
        if (dx == 1 && dy == -1) return MOVE_DIAG_UP_RIGHT;
        if (dx == -1 && dy == -1) return MOVE_DIAG_UP_LEFT;
        if (dx == 1 && dy == 1)  return MOVE_DIAG_DOWN_RIGHT;
        if (dx == -1 && dy == 1) return MOVE_DIAG_DOWN_LEFT;
        return 0; // Okänd
    }

    /**
     * Returnerar en textbeskrivning för debug-utskrifter.
     */
    public static String getInstructionText(int instructionCode) {
        switch (instructionCode) {
            case MOVE_UP: return "Uppåt (Rakt fram)";
            case MOVE_DOWN: return "Nedåt (Bakåt)";
            case MOVE_RIGHT: return "Höger (Sidled)";
            case MOVE_LEFT: return "Vänster (Sidled)";
            case MOVE_DIAG_UP_RIGHT: return "Diagonalt Upp-Höger";
            case MOVE_DIAG_UP_LEFT: return "Diagonalt Upp-Vänster";
            case MOVE_DIAG_DOWN_RIGHT: return "Diagonalt Ner-Höger";
            case MOVE_DIAG_DOWN_LEFT: return "Diagonalt Ner-Vänster";
            case TURNING_LEFT: return "Rotera Vänster";
            case TURNING_RIGHT: return "Rotera Höger";
            case CURVED_TRAJECTORY_LEFT: return "Kurvad bana Vänster";
            case CURVED_TRAJECTORY_RIGHT: return "Kurvad bana Höger";
            default: return "Okänd instruktion";
        }
    }
    
}
