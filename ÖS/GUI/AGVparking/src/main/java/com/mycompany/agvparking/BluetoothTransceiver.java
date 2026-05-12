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

                    // --- STOP-AND-WAIT LOGIK (Closed-Loop med Tolerans) ---
                    if (typ == 'D') {
                        // 1. Definiera hur nära som är "framme" (Toleransradie)
                        double toleranceDistance = ds.gridsize * 1.5; // STOR FELMARGINAL! Ändra efter tester
                        boolean framme = false;
                        
                        while (!framme && !ds.isStopped && !ds.isPaused) {
                            
                            // Räkna ut målkoordinaten (mitten på mål-rutan)
                            double targetX_cm = (instruktion.targetX * ds.gridsize) + (ds.gridsize / 2.0);
                            double targetY_cm = (instruktion.targetY * ds.gridsize) + (ds.gridsize / 2.0);
                            
                            // Använd Pythagoras sats för att kolla det exakta avståndet i cm
                            double dx = targetX_cm - ds.robotX;
                            double dy = targetY_cm - ds.robotY;
                            double avståndTillMål = Math.sqrt((dx * dx) + (dy * dy));
                            
                            if (avståndTillMål <= toleranceDistance) {
                                // Vi är tillräckligt nära! Släpp loopen och skicka nästa kommando.
                                framme = true;
                                cui.appendStatus("Delmål nått (Avvikelse: " + String.format("%.1f", avståndTillMål) + " enheter)\n");
                            } else {
                                // Vi är inte framme än, vänta på nästa $P-uppdatering
                                Thread.sleep(100); 
                            }
                        }
                    } else if (typ == 'K') {
                        Thread.sleep(500); 
                    }
                    // --------------------------------------------------------
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
        
// --- POSITIONSDATA ($P) ---
        if (typ == 'P' && data.length >= 11) { 
            
            // 1. Läs in X (i millimeter)
            int xHigh = data[2] & 0xFF;
            int xLow = data[3] & 0xFF;
            short x_mm = (short) ((xHigh << 8) | xLow); // cast till short fångar minustecken!
            
            // 2. Läs in Y (i millimeter)
            int yHigh = data[4] & 0xFF;
            int yLow = data[5] & 0xFF;
            short y_mm = (short) ((yHigh << 8) | yLow);
            
            cui.updatePositionDisplay(); 
            cui.repaint(); // Uppdatera GUI direkt
            // 3. Läs in Vinkeln (Theta i grader * 100)
            int thetaHigh = data[6] & 0xFF;
            int thetaLow = data[7] & 0xFF;
            short thetaRå = (short) ((thetaHigh << 8) | thetaLow);
            
            // 4. Läs in Status (data[8]) och Seq (data[9])
            
            
            // ---------- HAR INGET FÖR DETTA ÄNNU! ----------
            int statusByte = data[8] & 0xFF;
            // -----------------------------------------------
            int seq = data[9] & 0xFF; 

            // --- VINKELKONVERTERING ---
            // Återskapa decimalerna: 3141 blir 31.41 grader
            double thetaGrader = thetaRå / 100.0;
            
            // Konvertera till radianer för Javas Math-funktioner
            double thetaRadianer = thetaGrader * (Math.PI / 180.0);
            
            // AGV = Positivt motsols. Java GUI = Positivt medsols.
            // Vi sätter ett minustecken för att översätta det rätt till skärmen!
            ds.robotAngle = -thetaRadianer; 

            // --- SKALNING OCH POSITIONERING ---
            // Din karta är 3500 mm (350 cm) delat på antalet rader (ds.rows)
            double cell_size_mm = 3500.0 / ds.rows; 
            
            ds.robotX = (x_mm / cell_size_mm) * ds.gridsize;
            ds.robotY = (y_mm / cell_size_mm) * ds.gridsize;
            
            // --- RÄKNA UT BAKAXELN PÅ BILEN ---
            double pixelOffset = ds.agvOffset * ds.gridsize;
            ds.axleX = ds.robotX - pixelOffset * Math.cos(ds.robotAngle);
            ds.axleY = ds.robotY - pixelOffset * Math.sin(ds.robotAngle);
            
            ds.updateUiflag = true;
            cui.repaint(); 
            
            skickaACK((byte) 1, seq); // Bekräfta mottagning
        }
        
        // --- HINDERDETEKTERING ($H) ---
        // Format antaget: $ H X Y Seq CRC \n (Minst 8 bytes)
        else if (typ == 'H' && data.length >= 8) {
            int hinderX = data[3] & 0xFF; 
            int hinderY = data[4] & 0xFF;
            int seq = data[6] & 0xFF;
            
            cui.appendStatus("LARM: AGV upptäckte ett hinder på (" + hinderX + ", " + hinderY + ")!\n");
            
            // 1. Skicka ACK tillbaka direkt så AGV vet att vi mottagit varningen
            skickaACK((byte) 1, seq);
            
            // 2. Tvinga AGV att stanna omedelbart och rensa gamla rutten från kön
            skickaStoppKommando();
            
            // 3. Lägg in hindret i vår karta så Dijkstra kan se det
            if (hinderX >= 0 && hinderX < ds.columns && hinderY >= 0 && hinderY < ds.rows) {
                // Addera till listan av hinder
                ds.ObstacleMatrix[hinderX][hinderY] = 1; 
            }
            
            // 4. Be ControlUI att räkna om rutten runt hindret.
            // (SwingUtilities används för att inte krascha GUI:t från en bakgrundstråd)
            javax.swing.SwingUtilities.invokeLater(new Runnable() {
                public void run() {
                    cui.recalculateCurrentMission();
                }
            });
        }
        // --- FELMEDDELANDE FRÅN AGV ($E) ---
        else if (typ == 'E' && data.length >= 6) { 
            int seq_ref = data[2] & 0xFF; // Vilket kommando från oss som misslyckades
            int seq = data[3] & 0xFF;     // AGV:ns sekvensnummer för felmeddelandet
            
            cui.appendStatus("KRITISKT LARM: AGV rapporterar fel! Refererar till vår sekvens: " + seq_ref + ".\n");
            
            // 1. Skicka ACK för att bekräfta att vi sett felet
            skickaACK((byte) 1, seq);
            
            // 2. Tvinga systemet att stanna
            skickaStoppKommando();
            
            // 3. Spärra vidare körning i Java-programmet
            ds.isStopped = true; 
            ds.isPaused = false; 
            
            cui.appendStatus("Systemet är fryst. Målbufferten i AGV:n kan vara tom. Starta om körningen.\n");
        }
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
