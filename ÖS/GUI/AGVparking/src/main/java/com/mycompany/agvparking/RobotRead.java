package com.mycompany.agvparking;

import java.util.Random;

public class RobotRead implements Runnable {

    private int sleepTime;
    private static Random generator = new Random();
    private ControlUI cui;
    private DataStore ds;

    public RobotRead(DataStore ds, ControlUI cui) {
        this.cui = cui;
        this.ds = ds;
        sleepTime = generator.nextInt(20000);
    }

    @Override
    public void run() {
        try {
            cui.appendStatus("RobotRead kommer att köra i " + sleepTime + " millisekunder.\n");
            int i = 1;
            while (i <= 20) {
                Thread.sleep(sleepTime / 20);
                cui.appendStatus("Jag är tråd RobotRead! För " + i + ":te gången.\n");
                if (i == 10){
                    ds.updateUiflag = true;
                }
                i++;
            }
        } catch (InterruptedException exception) {
        }
        cui.appendStatus("RobotRead är nu klar!\n");
    }
}
