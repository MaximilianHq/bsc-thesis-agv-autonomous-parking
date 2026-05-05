package com.mycompany.agvparking;

import java.io.*;
import javax.microedition.io.*;

public class BluetoothTransceiver implements Runnable {

    private DataStore ds;
    private ControlUI cui;
    private OutputStream bluetooth_ut;
    private InputStream bluetooth_in;
    
    private int nuvarandeSekvens = 0; // Sekvensnummer enligt protokoll

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

                // Hämta nästa instruktion som RouteOptimizer (eller UI) lagt i kön
                if (ds.instructionQueue != null && !ds.instructionQueue.isEmpty()) {
                    AgvInstruction instruktion = ds.instructionQueue.poll();
                    
                    byte typ = (byte) instruktion.type;
                    byte seq = (byte) nuvarandeSekvens;
                    
                    byte[] dataBytes;
                    byte[] paket;
                    String loggText = "";

                    // --- BYGG PAKET DYNAMISKT BEROENDE PÅ TYP ---
                    if (typ == 'D') { // Körkommando
                        byte maneuver = (byte) instruktion.maneuver;
                        byte velocity = (byte) instruktion.velocity;
                        dataBytes = new byte[] { maneuver, velocity };
                        byte checksum = beraknaChecksumma(typ, dataBytes, seq);
                        paket = new byte[] { '$', typ, maneuver, velocity, seq, checksum, '\n' };
                        
                        loggText = "KÖR: " + InstructionsStore.getInstructionText(instruktion.maneuver) + 
                                   " [Mål: " + instruktion.targetX + "," + instruktion.targetY + "]";
                                   
                    } else if (typ == 'K') { // Enskilt kommando
                        byte instrByte = (byte) instruktion.instructionByte;
                        dataBytes = new byte[] { instrByte };
                        byte checksum = beraknaChecksumma(typ, dataBytes, seq);
                        paket = new byte[] { '$', typ, instrByte, seq, checksum, '\n' };
                        
                        loggText = "ENSKILT KMD: Inst (" + instrByte + ")";
                        
                    } else if (typ == 'X') { // Stanna
                        dataBytes = new byte[] {}; // Ingen data för Stanna
                        byte checksum = beraknaChecksumma(typ, dataBytes, seq);
                        paket = new byte[] { '$', typ, seq, checksum, '\n' };
                        
                        loggText = "STANNA (X-Kommando)";
                        
                    } else {
                        // Säkerhet om okänd typ skulle skickas från kön
                        cui.appendStatus("Varning: Okänd meddelandetyp '" + (char)typ + "' ignorerades.\n");
                        continue;
                    }

                    // Skicka till AGV
                    bluetooth_ut.write(paket);
                    bluetooth_ut.flush();
                    
                    cui.logSent(loggText + " Seq:" + nuvarandeSekvens);
                    nuvarandeSekvens = (nuvarandeSekvens + 1) % 256;

                    // --- STOP-AND-WAIT LOGIK ---
                    if (typ == 'D') {
                        // Vi pausar här och väntar på att AGV:n skickar en position som matchar målet
                        boolean framme = false;
                        while (!framme && !ds.isStopped && !ds.isPaused) {
                            
                            int currentGridX = (int) (ds.robotX / ds.gridsize);
                            int currentGridY = (int) (ds.robotY / ds.gridsize);
                            
                            if (currentGridX == instruktion.targetX && currentGridY == instruktion.targetY) {
                                framme = true;
                            } else {
                                Thread.sleep(100); 
                            }
                        }
                    } else if (typ == 'K') {
                        // Om vi skickar ett K-kommando (t.ex. släpp bil), vänta lite innan vi skickar nästa
                        Thread.sleep(500); 
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
        
        // Positionsuppdatering börjar med 'P' [cite: 67]
        if (typ == 'P' && data.length >= 8) {
            int x = data[3] & 0xFF; // X-koordinat är på index 3 [cite: 67]
            int y = data[4] & 0xFF; // Y-koordinat är på index 4 [cite: 67]
            int seq = data[6] & 0xFF; // Sekvensnummer ligger på index 6 [cite: 67]

            // Uppdatera DataStore så att sändar-loopen ser att vi rör oss
            ds.robotX = x * ds.gridsize + (ds.gridsize / 2.0);
            ds.robotY = y * ds.gridsize + (ds.gridsize / 2.0);
            ds.updateUiflag = true;
            
            cui.repaint(); // Uppdatera GUI direkt
            
            skickaACK((byte) 1, seq); // Bekräfta mottagning
        }
        // Lägg in hantering av 'H' (Hinder) eller 'A' (ACK) här i framtiden om nödvändigt [cite: 67]
    }

    private void skickaACK(byte status, int seq) {
        try {
            byte typ = 'A';
            byte[] data = { status };
            byte crc = beraknaChecksumma(typ, data, (byte)seq);
            byte[] paket = new byte[] { '$', typ, status, (byte)seq, crc, '\n' };
            bluetooth_ut.write(paket);
            bluetooth_ut.flush();
        } catch (Exception e) {}
    }

    /**
     * Beräknar checksumma enligt kommunikationsprotokollet.
     * Checksumman beräknas genom en XOR av meddelandetyp, all data, och sekvensnummer. [cite: 91, 101]
     */
    private byte beraknaChecksumma(byte typ, byte[] data, byte seq) {
        byte crc = 0;
        crc ^= typ;
        for (byte b : data) { 
            crc ^= b; 
        }
        crc ^= seq; 
        return crc;
    }
    
    /* Metod för nödstopp!
    Denna metod ska ta sig förbi alla andra väntande Bluetooth-strömmar. Om det till exempel
    finns meddelanden kvar i en rutt som behöver skickas kommer denna ta sig förbi dessa.
    */
    
    public void skickaStoppKommando() {
        try {
            if (bluetooth_ut != null) {
                byte typ = 'X';
                byte seq = (byte) nuvarandeSekvens;
                byte[] dataBytes = new byte[] {}; // 'X' har noll databytes 
                
                byte checksum = beraknaChecksumma(typ, dataBytes, seq);
                byte[] paket = new byte[] { '$', typ, seq, checksum, '\n' };
                
                // synchronized blockerar andra trådar (t.ex. sändarloopen) från att 
                // skriva till bluetooth_ut exakt samtidigt, vilket förhindrar korrupta paket.
                synchronized(bluetooth_ut) {
                    bluetooth_ut.write(paket);
                    bluetooth_ut.flush();
                }
                
                cui.logSent("EXPRESS: STANNA (X-Kommando) Seq:" + nuvarandeSekvens);
                nuvarandeSekvens = (nuvarandeSekvens + 1) % 256;
                
                // Töm även kön i Java så att den gamla, nu ogiltiga rutten försvinner!
                if (ds.instructionQueue != null) {
                    ds.instructionQueue.clear();
                }
            }
        } catch (Exception e) {
            cui.appendStatus("OBS! Kunde inte skicka nödstopp: " + e.getMessage() + "\n");
        }
    }
    
}   
