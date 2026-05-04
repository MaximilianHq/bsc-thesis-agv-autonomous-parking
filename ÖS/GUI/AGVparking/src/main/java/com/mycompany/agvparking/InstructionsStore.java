/*
 * Click nbfs://nbhost/SystemFileSystem/Templates/Licenses/license-default.txt to change this license
 * Click nbfs://nbhost/SystemFileSystem/Templates/Classes/Class.java to edit this template
 */
package com.mycompany.agvparking;

/**
 *
 * @author hanna
 */
public class InstructionsStore {
    // Befintliga rörelseinstruktioner
    public static final int MOVE_UP = 1;
    public static final int MOVE_DOWN = 2;
    public static final int MOVE_RIGHT = 3;
    public static final int MOVE_LEFT = 4;
    public static final int MOVE_DIAG_UP_RIGHT = 5;
    public static final int MOVE_DIAG_UP_LEFT = 6;
    public static final int MOVE_DIAG_DOWN_RIGHT = 7;
    public static final int MOVE_DIAG_DOWN_LEFT = 8;
    
    // Nya instruktioner 
    public static final int ROTATE_LEFT = 9;
    public static final int ROTATE_RIGHT = 10;
    public static final int CURVED_TRAJECTORY_LEFT = 11;
    public static final int CURVED_TRAJECTORY_RIGHT = 12;
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
            case ROTATE_LEFT: return "Rotera Vänster";
            case ROTATE_RIGHT: return "Rotera Höger";
            case CURVED_TRAJECTORY_LEFT: return "Kurvad bana Vänster";
            case CURVED_TRAJECTORY_RIGHT: return "Kurvad bana Höger";
            default: return "Okänd instruktion";
        }
    }
    
}
