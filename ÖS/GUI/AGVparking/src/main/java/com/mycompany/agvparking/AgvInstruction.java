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
    public int direction;
    public int steps;

    public AgvInstruction(int direction, int steps) {
        this.direction = direction;
        this.steps = steps;
    }
}