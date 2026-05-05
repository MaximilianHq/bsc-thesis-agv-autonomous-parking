package com.mycompany.agvparking;

public class RobotState {
    public double agvX, agvY;     // Den blå prickens position
    public double axleX, axleY;   // Den magenta prickens position
    public double angle;          // AGV:ns rotationsvinkel
    public boolean isLoaded;      // Är bilen påhakad just nu?

    public RobotState(double agvX, double agvY, double axleX, double axleY, double angle, boolean isLoaded) {
        this.agvX = agvX;
        this.agvY = agvY;
        this.axleX = axleX;
        this.axleY = axleY;
        this.angle = angle;
        this.isLoaded = isLoaded;
    }
}