package com.mycompany.bluetoothtransceiver;

import java.io.*;
import javax.microedition.io.*;
import javax.bluetooth.*;

public class BluetoothTransceiver {

    static OutputStream bluetooth_ut; 

    public static void main(String args[]) {
        try {
            // 1. Upprätta anslutning
            System.out.println("Ansluter till AGV...");
            StreamConnection anslutning = (StreamConnection) Connector.open("btspp://3c71bfcf083e:1");
            System.out.println("Ansluten!");

            // 2. Förbered strömmar
            bluetooth_ut = anslutning.openOutputStream();
            final InputStream bluetooth_in = anslutning.openInputStream(); 
            BufferedReader tangentbord = new BufferedReader(new InputStreamReader(System.in));

            // --- NYTT: STARTA EN BAKGRUNDSTRÅD FÖR MOTTAGNING ---
            // Denna tråd lyssnar hela tiden efter svar, utan att blockera tangentbordet
            Thread mottagarTrad = new Thread(new Runnable() {
                public void run() {
                    try {
                        ByteArrayOutputStream mottagarBuffer = new ByteArrayOutputStream();
                        int inkommandeByte;
                        boolean startHittad = false;

                        // Läs kontinuerligt från AGV:n
                        while ((inkommandeByte = bluetooth_in.read()) != -1) {
                            if (inkommandeByte == '$') {
                                startHittad = true;
                                mottagarBuffer.reset(); // Rensa gammalt skräp
                            }
                            
                            if (startHittad) {
                                mottagarBuffer.write(inkommandeByte);
                                
                                if (inkommandeByte == '\n') {
                                    // Hela paketet är inläst! Skicka till tolkaren
                                    byte[] fardigtMeddelande = mottagarBuffer.toByteArray();
                                    hanteraInkommandeMeddelande(fardigtMeddelande);
                                    startHittad = false; // Återställ inför nästa meddelande
                                }
                            }
                        }
                    } catch (Exception e) {
                        System.out.println("Mottagartråden avslutades: " + e.getMessage());
                    }
                }
            });
            // Starta tråden i bakgrunden!
            mottagarTrad.start(); 


            // 3. HUVUDLOOP (BARA FÖR ATT SÄNDA)
            while (true) {
                //System.out.println("\nSkriv testdata (t.ex. 'M,10,20,1,5' eller 'H,15,30,2') eller 'exit' för att avsluta:");
                String meddelande_ut = tangentbord.readLine();
                
                if (meddelande_ut != null && !meddelande_ut.isEmpty()) {
                    if (meddelande_ut.equalsIgnoreCase("exit")) break;

                    try {
                        String[] delar = meddelande_ut.split(",");
                        byte[] dataForChecksumma = new byte[delar.length];
                        dataForChecksumma[0] = (byte) delar[0].trim().charAt(0);
                        
                        for (int i = 1; i < delar.length; i++) {
                            dataForChecksumma[i] = (byte) Integer.parseInt(delar[i].trim());
                        }

                        byte checksum = beraknaChecksumma(dataForChecksumma);
                        System.out.println("Skickar binärt paket av typen '" + (char)dataForChecksumma[0] + "'...");

                        bluetooth_ut.write('$');
                        bluetooth_ut.write(dataForChecksumma);
                        bluetooth_ut.write(checksum);
                        bluetooth_ut.write('\n');
                        bluetooth_ut.flush();
                        
                    } catch (Exception e) {
                        System.out.println("Fel format! Använd: Bokstav,Siffra,Siffra...");
                    }
                }
            }
            
            anslutning.close();
        } catch (Exception e) {
            System.out.println("Ett fel uppstod i huvudprogrammet: " + e.toString());
        }
    }

    // --- METOD FÖR ATT TOLKA INKOMMANDE MEDDELANDEN ---
    public static void hanteraInkommandeMeddelande(byte[] data) {
        if (data == null || data.length < 4) return;
        
        if (data[0] != '$') {
            System.out.println("Ogiltigt meddelande: Saknar startbyte ($).");
            return;
        }

        byte meddelandeTyp = data[1];
        byte mottagenChecksumma = data[data.length - 2]; 

        int dataLangd = data.length - 3; 
        byte[] dataForChecksumma = new byte[dataLangd];
        System.arraycopy(data, 1, dataForChecksumma, 0, dataLangd);

        if (beraknaChecksumma(dataForChecksumma) != mottagenChecksumma) {
            System.out.println("Felaktig checksumma mottagen! Kastar paketet.");
            int felSekvens = (dataLangd > 1) ? (data[data.length - 3] & 0xFF) : 0;
            skickaACK(bluetooth_ut, (byte) 3, felSekvens); 
            return;
        }

        switch (meddelandeTyp) {
            case 'M':
                if (data.length == 8) { 
                    int posX = data[2] & 0xFF;
                    int posY = data[3] & 0xFF;
                    int status = data[4] & 0xFF;
                    int sekvensnummer = data[5] & 0xFF;
                    
                    System.out.println("\n\n--- [M] POSITION UPPDATERAD FRÅN AGV ---");
                    System.out.println("Koordinater : X=" + posX + ", Y=" + posY);
                    System.out.println("Status      : " + status);
                    System.out.println("Sekvens     : " + sekvensnummer);
                    System.out.print("> "); // Snygg detalj för att visa att du fortfarande kan skriva
                    
                    skickaACK(bluetooth_ut, (byte) 1, sekvensnummer);
                } 
                break;

            case 'H':
                if (data.length == 7) { 
                    int posX = data[2] & 0xFF;
                    int posY = data[3] & 0xFF;
                    int sekvensnummer = data[4] & 0xFF;
                    
                    System.out.println("\n\n--- [H] HINDER UPPTÄCKT AV AGV ---");
                    System.out.println("Plats       : X=" + posX + ", Y=" + posY);
                    System.out.println("Sekvens     : " + sekvensnummer);
                    System.out.print("> ");
                    
                    skickaACK(bluetooth_ut, (byte) 1, sekvensnummer);
                } 
                break;

            case 'A':
                if (data.length == 6) { 
                    int statusSvar = data[2] & 0xFF;
                    int sekvensnummer = data[3] & 0xFF;
                    
                    System.out.println("\n\n--- [A] KVITTENS MOTTAGEN ---");
                    if (statusSvar == 1) {
                        System.out.println("AGV godkände meddelande (Sekvens " + sekvensnummer + ")");
                    } else {
                        System.out.println("AGV nekade med NACK kod " + statusSvar + " (Sekvens " + sekvensnummer + ")");
                    }
                    System.out.print("> ");
                } 
                break;

            default:
                System.out.println("\nOkänd meddelandetyp mottagen: " + (char)meddelandeTyp);
                System.out.print("> ");
                break;
        }
    }

    // --- METOD FÖR ATT SKICKA SVAR (ACK/NACK) ---
    public static void skickaACK(OutputStream ut, byte status, int sekvensnummer) {
        try {
            byte sekvensByte = (byte) sekvensnummer;
            byte[] dataForChecksumma = new byte[] { 'A', status, sekvensByte };
            byte checksum = beraknaChecksumma(dataForChecksumma);
            
            byte[] ackPaket = new byte[] { '$', 'A', status, sekvensByte, checksum, '\n' };
            
            ut.write(ackPaket);
            ut.flush(); 
            
        } catch (Exception e) {
            System.out.println("Kunde inte skicka svarsmeddelande: " + e.getMessage());
        }
    }

    public static byte beraknaChecksumma(byte[] data) {
        byte crc = 0;
        for (byte b : data) {
            crc ^= b; 
        }
        return crc;
    }
}