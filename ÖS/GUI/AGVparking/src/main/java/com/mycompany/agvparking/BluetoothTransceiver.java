package com.mycompany.agvparking; // Se till att namnet matchar ditt paket!

import java.io.*;
import javax.microedition.io.*;

public class BluetoothTransceiver implements Runnable {

    private DataStore ds;
    private ControlUI cui;
    private OutputStream bluetooth_ut;
    private InputStream bluetooth_in;
    
    private int nuvarandeSekvens = 0; // Håller koll på sekvensnummer (0-255)

    public BluetoothTransceiver(DataStore ds, ControlUI cui) {
        this.ds = ds;
        this.cui = cui;
    }

    @Override
    public void run() {
        try {
            cui.appendStatus("Bluetooth: Ansluter till AGV...\n");
            // OBS: Byt ut adressen till er riktiga AGV-adress!
            StreamConnection anslutning = (StreamConnection) Connector.open("btspp://3c71bfcf083e:1");
            cui.appendStatus("Bluetooth: Ansluten!\n");

            bluetooth_ut = anslutning.openOutputStream();
            bluetooth_in = anslutning.openInputStream();

            // 1. Starta mottagartråden (Lyssnar på AGVn)
            startaMottagarTrad();

            // 2. Huvudloop (Sänder till AGVn)
            while (!ds.isStopped) {
                
                // Om användaren tryckt på STOP
                if (ds.isPaused) {
                    // Om vi behöver skicka Stanna ($XSC\n) direkt
                    // skickaStannaKommando(); 
                    Thread.sleep(500);
                    continue;
                }

                // Finns det instruktioner att skicka?
                if (ds.instructionQueue != null && !ds.instructionQueue.isEmpty()) {
                    AgvInstruction instruktion = ds.instructionQueue.poll(); // Plocka ut och ta bort från kön
                    
                    // Skapa datan för checksumman: [Typ, Manöver, Hastighet] -> ['D', M, V]
                    byte[] dataForChecksumma = new byte[] {
                        'D', 
                        (byte) instruktion.maneuver, 
                        (byte) instruktion.velocity
                    };
                    
                    byte checksum = beraknaChecksumma(dataForChecksumma);
                    byte sekvensByte = (byte) nuvarandeSekvens;

                    // Bygg hela paketet: $ D M V S C \n
                    byte[] komplettPaket = new byte[] {
                        '$',
                        'D',
                        (byte) instruktion.maneuver,
                        (byte) instruktion.velocity,
                        sekvensByte,
                        checksum,
                        '\n'
                    };

                    bluetooth_ut.write(komplettPaket);
                    bluetooth_ut.flush();
                    
                    cui.logSent("Körkommando M:" + instruktion.maneuver + " V:" + instruktion.velocity + " Seq:" + nuvarandeSekvens);
                    
                    // Stega sekvensnumret (0-255)
                    nuvarandeSekvens = (nuvarandeSekvens + 1) % 256;
                    
                    // Här borde man egentligen lägga in en Stop-And-Wait logik 
                    // som väntar på ACK innan vi loopar och skickar nästa meddelande!
                    Thread.sleep(1000); // Tillfällig paus
                } else {
                    Thread.sleep(200); // Ingen data att skicka, vänta lite
                }
            }
            
            anslutning.close();
            cui.appendStatus("Bluetooth: Frånkopplad.\n");
            
        } catch (Exception e) {
            cui.appendStatus("Bluetooth Fel: " + e.getMessage() + "\n");
        }
    }

    private void startaMottagarTrad() {
        Thread mottagarTrad = new Thread(new Runnable() {
            public void run() {
                try {
                    ByteArrayOutputStream mottagarBuffer = new ByteArrayOutputStream();
                    int inkommandeByte;
                    boolean startHittad = false;

                    while (!ds.isStopped && (inkommandeByte = bluetooth_in.read()) != -1) {
                        if (inkommandeByte == '$') {
                            startHittad = true;
                            mottagarBuffer.reset();
                        }
                        
                        if (startHittad) {
                            mottagarBuffer.write(inkommandeByte);
                            if (inkommandeByte == '\n') {
                                byte[] fardigtMeddelande = mottagarBuffer.toByteArray();
                                hanteraInkommandeMeddelande(fardigtMeddelande);
                                startHittad = false;
                            }
                        }
                    }
                } catch (Exception e) {
                    System.out.println("Mottagartråden avslutades.");
                }
            }
        });
        mottagarTrad.start();
    }

    private void hanteraInkommandeMeddelande(byte[] data) {
        if (data == null || data.length < 4) return;
        
        byte meddelandeTyp = data[1];
        byte mottagenChecksumma = data[data.length - 2]; 

        // Enligt protokoll (sid 9): Checksumman beräknas på Typ + Data (exklusive Sekvens och \n)
        int dataLangdForCrc = data.length - 4; 
        byte[] dataForChecksumma = new byte[dataLangdForCrc];
        System.arraycopy(data, 1, dataForChecksumma, 0, dataLangdForCrc);

        if (beraknaChecksumma(dataForChecksumma) != mottagenChecksumma) {
            int felSekvens = data[data.length - 3] & 0xFF;
            cui.logReceived("FEL CRC från AGV! Seq: " + felSekvens);
            skickaACK((byte) 3, felSekvens); // 3 = Checksumma icke godkänd
            return;
        }

        switch (meddelandeTyp) {
            case 'P': // ÄNDRAD FRÅN 'M' TILL 'P' enligt protokoll
                if (data.length == 8) { 
                    int posX = data[2] & 0xFF;
                    int posY = data[3] & 0xFF;
                    int status = data[4] & 0xFF;
                    int sekvensnummer = data[5] & 0xFF;
                    
                    cui.logReceived("Position: X" + posX + " Y" + posY + " Stat:" + status);
                    
                    // Uppdatera GUI med AGVns riktiga position här!
                    // ds.robotX = posX; ds.robotY = posY; ds.updateUiflag = true;
                    
                    skickaACK((byte) 1, sekvensnummer);
                } 
                break;

            case 'H':
                if (data.length == 7) { 
                    int posX = data[2] & 0xFF;
                    int posY = data[3] & 0xFF;
                    int sekvensnummer = data[4] & 0xFF;
                    
                    cui.logReceived("Hinder upptäckt: X" + posX + " Y" + posY);
                    skickaACK((byte) 1, sekvensnummer);
                } 
                break;

            case 'A':
                if (data.length == 6) { 
                    int statusSvar = data[2] & 0xFF;
                    int sekvensnummer = data[3] & 0xFF;
                    
                    cui.logReceived("ACK (" + statusSvar + ") för Seq: " + sekvensnummer);
                    // Här borde vi flagga att "Meddelandet kom fram, okej att skicka nästa i kön!"
                } 
                break;
        }
    }

    private void skickaACK(byte status, int sekvensnummer) {
        try {
            byte sekvensByte = (byte) sekvensnummer;
            byte[] dataForChecksumma = new byte[] { 'A', status }; // Typ + Data (Enligt sid 9)
            byte checksum = beraknaChecksumma(dataForChecksumma);
            
            byte[] ackPaket = new byte[] { '$', 'A', status, sekvensByte, checksum, '\n' };
            
            bluetooth_ut.write(ackPaket);
            bluetooth_ut.flush(); 
            
        } catch (Exception e) {
            cui.appendStatus("Kunde inte skicka svarsmeddelande.\n");
        }
    }

    private byte beraknaChecksumma(byte[] data) {
        byte crc = 0;
        for (byte b : data) {
            crc ^= b; 
        }
        return crc;
    }
}