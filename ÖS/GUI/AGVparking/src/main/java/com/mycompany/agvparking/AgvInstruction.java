/*
 * Click nbfs://nbhost/SystemFileSystem/Templates/Licenses/license-default.txt to change this license
 * Click nbfs://nbhost/SystemFileSystem/Templates/Classes/Class.java to edit this template
 */
package com.mycompany.agvparking;

/**
 *
 * @author fredr
 */

public class AgvInstruction {
    public int maneuver;
    public int velocity; // Kanske ska ändras till angleAGV
    public int steps;
    public int targetX;
    public int targetY;
    public int rotation;
    public boolean monitorPosition; 

    public AgvInstruction(int maneuver, int velocity, int steps, int targetX, int targetY, boolean monitorPosition) {
        this.maneuver = maneuver;
        this.velocity = velocity;
        this.steps = steps; 


    public AgvInstruction(int maneuver, int velocity, int steps, int targetX, int targetY) {
        this.maneuver = maneuver;
        this.velocity = velocity;
        this.steps = steps;
        this.targetX = targetX;
        this.targetY = targetY;
       
    }
    
    public AgvInstruction(int maneuver, int rotation)
    {
        this.maneuver = maneuver; 
        this.rotation = rotation;//Behövs något mer göras för denna?
    }
}
