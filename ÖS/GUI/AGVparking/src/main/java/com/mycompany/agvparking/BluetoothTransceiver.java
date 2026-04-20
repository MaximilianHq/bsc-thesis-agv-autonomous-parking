package com.mycompany.agvparking;

import java.io.*;
import javax.microedition.io.*;

public class BluetoothTransceiver implements Runnable {

    private DataStore ds;
    private ControlUI cui;
    private OutputStream bluetooth_ut;
    private InputStream bluetooth_in;
    
    private int nuvarandeSekvens = 0; // Sekvensnummer enligt protokoll [cite: 19, 88]

    public BluetoothTransceiver(DataStore ds, ControlUI cui) {
        this.ds = ds;
        this.cui = cui;
    }

    @Override
    public void run() {
        try {
            cui.appendStatus("Bluetooth: Försöker ansluta till AGV...\n");
            // ANPASSNING: Se till att adressen matchar din AGV!
            StreamConnection anslutning = (StreamConnection) Connector.open("btspp://3c71bfcf083e:1");
            cui.appendStatus("Bluetooth: Ansluten och redo!\n");

            bluetooth_ut = anslutning.openOutputStream();
            bluetooth_in = anslutning.openInputStream();

            // 1. Starta mottagartråden (Lyssnar på AGV:ns $P-meddelanden)
            startaMottagarTrad();

            // 2. Sändarloop: Håller koll på kön och väntar på rätt position
            while (!ds.isStopped) {
                
                if (ds.isPaused) {
                    Thread.sleep(500);
                    continue;
                }

                // Finns det en ny instruktion i brevlådan?
                if (ds.instructionQueue != null && !ds.instructionQueue.isEmpty()) {
                    AgvInstruction instruktion = ds.instructionQueue.poll();
                    
                    // Skapa Körkommando enligt protokoll: $ D M V S C \n [cite: 19]
                    // Checksumman beräknas på Typ + Data (D + M + V) [cite: 94]
                    byte[] dataForCrc = new byte[] { 'D', (byte) instruktion.maneuver, (byte) instruktion.velocity };
                    byte checksum = beraknaChecksumma(dataForCrc);
                    byte seq = (byte) nuvarandeSekvens;

                    byte[] paket = new byte[] { '$', 'D', (byte) instruktion.maneuver, (byte) instruktion.velocity, seq, checksum, '\n' };

                    // Skicka till AGV
                    bluetooth_ut.write(paket);
                    bluetooth_ut.flush();
                    
                    cui.logSent("M:" + instruktion.maneuver + " (Mål: " + instruktion.targetX + "," + instruktion.targetY + ") Seq:" + nuvarandeSekvens);
                    nuvarandeSekvens = (nuvarandeSekvens + 1) % 256;

                    // --- POSITIONS-LÅS (Stop and Wait för koordinater) ---
                    // Vi pausar här och väntar på att AGV:n skickar en position som matchar målet
                    boolean framme = false;
                    while (!framme && !ds.isStopped && !ds.isPaused) {
                        
                        // Räkna ut AGV:ns nuvarande position i rutnätet (baserat på inkommande P-meddelanden)
                        int currentGridX = (int) (ds.robotX / ds.gridsize);
                        int currentGridY = (int) (ds.robotY / ds.gridsize);
                        
                        if (currentGridX == instruktion.targetX && currentGridY == instruktion.targetY) {
                            framme = true;
                            cui.logSent("Mål nått! Plockar nästa instruktion...");
                        } else {
                            Thread.sleep(100); // Vänta 100ms innan vi kollar koordinaten igen
                        }
                    }
                } else {
                    Thread.sleep(200); // Ingen instruktion i kön, vila lite
                }
            }
            anslutning.close();
            
        } catch (Exception e) {
            cui.appendStatus("Bluetooth-fel: " + e.getMessage() + "\n");
        }
    }

    private void startaMottagarTrad() {
        Thread t = new Thread(new Runnable() {
            public void run() {
                try {
                    ByteArrayOutputStream buffer = new ByteArrayOutputStream();
                    int b;
                    boolean start = false;

                    while (!ds.isStopped && (b = bluetooth_in.read()) != -1) {
                        if (b == '$') { start = true; buffer.reset(); }
                        if (start) {
                            buffer.write(b);
                            if (b == '\n') {
                                hanteraInkommandeMeddelande(buffer.toByteArray());
                                start = false;
                            }
                        }
                    }
                } catch (Exception e) {}
            }
        });
        t.start();
    }

    private void hanteraInkommandeMeddelande(byte[] data) {
        if (data == null || data.length < 4) return;
        
        byte typ = data[1];
        
        // Enligt protokoll (sid 7): Positionsuppdatering börjar med 'P' [cite: 71]
        if (typ == 'P' && data.length == 8) {
            int x = data[2] & 0xFF;
            int y = data[3] & 0xFF;
            int seq = data[5] & 0xFF;

            // VIKTIGT: Uppdatera DataStore så att sändar-loopen ser att vi rör oss!
            ds.robotX = x * ds.gridsize + (ds.gridsize / 2.0);
            ds.robotY = y * ds.gridsize + (ds.gridsize / 2.0);
            ds.updateUiflag = true;
            
            cui.logReceived("AGV Position: (" + x + "," + y + ")");
            cui.repaint(); // Uppdatera gula pricken på kartan direkt
            
            skickaACK((byte) 1, seq); // Bekräfta mottagning [cite: 19, 82]
        }
    }

    private void skickaACK(byte status, int seq) {
        try {
            byte[] dataForCrc = new byte[] { 'A', status };
            byte crc = beraknaChecksumma(dataForCrc);
            byte[] paket = new byte[] { '$', 'A', status, (byte)seq, crc, '\n' };
            bluetooth_ut.write(paket);
            bluetooth_ut.flush();
        } catch (Exception e) {}
    }

    private byte beraknaChecksumma(byte[] data) {
        byte crc = 0;
        for (byte b : data) { crc ^= b; } // XOR-checksumma enligt protokoll [cite: 94]
        return crc;
    }
}