package com.mycompany.bluetoothtransceiver;

import java.io.*;
import javax.microedition.io.*;

public class BluetoothTransceiver {

    // Gör ut-strömmen statisk så att metoder som skickaACK också kommer åt den
    static OutputStream bluetooth_ut; 

    public static void main(String args[]) {
        try {
            // 1. Upprätta anslutning
            System.out.println("Ansluter till AGV...");
            StreamConnection anslutning = (StreamConnection) Connector.open("btspp://3c71bfcf083e:1");
            System.out.println("Ansluten!");

            // 2. Förbered strömmar (rena data-strömmar för bytes)
            bluetooth_ut = anslutning.openOutputStream();
            InputStream bluetooth_in = anslutning.openInputStream(); 
            BufferedReader tangentbord = new BufferedReader(new InputStreamReader(System.in));

            // 3. Huvudloop (Ping-pong testning)
            while (true) {
                // --- SÄNDA DATA (TEST) ---
                System.out.println("\nSkriv testdata (t.ex. 'M123') eller tryck Enter för att vänta på mottagning:");
                String meddelande_ut = tangentbord.readLine();
                
                if (meddelande_ut != null && !meddelande_ut.isEmpty()) {
                    if (meddelande_ut.equalsIgnoreCase("exit")) break;

                    // Bygg ett testpaket (För testning från tangentbord)
                    byte typeByte = (byte) meddelande_ut.charAt(0);
                    byte[] dataBytes = meddelande_ut.substring(1).getBytes();

                    // 3A. Skapa en temporär array (Typ + Data) för att använda vår checksumme-funktion
                    byte[] dataForChecksumma = new byte[1 + dataBytes.length];
                    dataForChecksumma[0] = typeByte;
                    System.arraycopy(dataBytes, 0, dataForChecksumma, 1, dataBytes.length);

                    // 3B. BERÄKNA CHECKSUMMA VID SÄNDNING
                    byte checksum = beraknaChecksumma(dataForChecksumma);
                    System.out.println("Skickar paket med checksum: " + checksum);

                    // 3C. Skicka iväg paketet
                    bluetooth_ut.write('$');
                    bluetooth_ut.write(typeByte);
                    bluetooth_ut.write(dataBytes);
                    bluetooth_ut.write(checksum);
                    bluetooth_ut.write('\n');
                    bluetooth_ut.flush();
                }

                // --- TA EMOT DATA (BYTE FÖR BYTE) ---
                ByteArrayOutputStream mottagarBuffer = new ByteArrayOutputStream();
                int inkommandeByte;
                boolean startHittad = false;

                // Läs en byte i taget. Avbryt loopen när vi hittar '\n'
                while ((inkommandeByte = bluetooth_in.read()) != -1) {
                    if (inkommandeByte == '$') {
                        startHittad = true;
                        mottagarBuffer.reset(); // Rensa gammalt skräp
                    }
                    
                    if (startHittad) {
                        mottagarBuffer.write(inkommandeByte);
                        
                        if (inkommandeByte == '\n') {
                            break; // Hela meddelandet är nu inläst!
                        }
                    }
                }

                byte[] fardigtMeddelande = mottagarBuffer.toByteArray();
                
                // Tolka och konvertera meddelandet till variabler 
                hanteraInkommandeMeddelande(fardigtMeddelande);
            }

            // Stäng anslutningen snyggt om loopen bryts
            anslutning.close();
        } catch (Exception e) {
            System.out.print("Ett fel uppstod: " + e.toString());
        }
    }

    // --- METOD FÖR ATT TOLKA INKOMMANDE MEDDELANDEN ---
    public static void hanteraInkommandeMeddelande(byte[] data) {
        if (data == null || data.length < 4) return;
        
        if (data[0] != '$') {
            System.out.println("Ogiltigt meddelande, saknar startbyte.");
            return;
        }

        byte meddelandeTyp = data[1];
        byte mottagenChecksumma = data[data.length - 2]; // Näst sista byten är alltid C

        // 1. BERÄKNA CHECKSUMMA VID MOTTAGNING FÖR ATT VALIDERA
        // Vi plockar ut all data mellan $ och C (dvs Typ, Positioner, Status, Sekvens)
        int dataLangd = data.length - 3; 
        byte[] dataForChecksumma = new byte[dataLangd];
        System.arraycopy(data, 1, dataForChecksumma, 0, dataLangd);

        if (beraknaChecksumma(dataForChecksumma) != mottagenChecksumma) {
            System.out.println("Felaktig checksumma mottagen! Kastar paketet.");
            
            // Frivilligt: Här hade ni kunnat svara med ett NACK för fel checksumma
            // skickaACK(bluetooth_ut, (byte) 3, data[data.length - 3]); 
            return;
        }

        // 2. TOLKA MEDDELANDET (Nu vet vi att det är helt och felfritt)
        switch (meddelandeTyp) {
            case 'M':
                if (data.length == 8) { // Protokoll: $, M, XX, YY, L, S, C, \n
                    int posX = data[2] & 0xFF;
                    int posY = data[3] & 0xFF;
                    int status = data[4] & 0xFF;
                    int sekvensnummer = data[5] & 0xFF;
                    
                    System.out.println("--- POSITION UPPDATERAD ---");
                    System.out.println("X: " + posX + ", Y: " + posY);
                    System.out.println("Status: " + status + ", Sekvens: " + sekvensnummer);
                    
                    // SKICKA ACK! (Status 1 = Godkänt)
                    skickaACK(bluetooth_ut, (byte) 1, sekvensnummer);
                }   break;
            case 'H':
                if (data.length == 7) { // Protokoll: $, H, XX, YY, S, C, \n
                    int posX = data[2] & 0xFF;
                    int posY = data[3] & 0xFF;
                    int sekvensnummer = data[4] & 0xFF;
                    
                    System.out.println("--- Akta HINDER!!! ---");
                    System.out.println("Hinder vid X: " + posX + ", Y: " + posY);
                    
                    // SKICKA ACK! (Status 1 = Godkänt)
                    skickaACK(bluetooth_ut, (byte) 1, sekvensnummer);
                }   break;
            case 'A':
                // Om AGV:n skickar en ACK/NACK till oss
                if (data.length == 6) { // Protokoll: $, A, B, S, C, \n
                    int statusB = data[2] & 0xFF;
                    int sekvensnummer = data[3] & 0xFF;
                    System.out.println("Mottog svarsmeddelande från AGV! Statuskod: " + statusB + " för sekvens: " + sekvensnummer);
                }   break;
            default:
                System.out.println("Okänd meddelandetyp: " + (char)meddelandeTyp);
                break;
        }
    }

    // --- METOD FÖR ATT SKICKA SVAR (ACK/NACK) ---
    public static void skickaACK(OutputStream ut, byte status, int sekvensnummer) {
        try {
            byte sekvensByte = (byte) sekvensnummer;
            
            // 1. Skapa array för de bytes som ska ingå i checksumman
            byte[] dataForChecksumma = new byte[] { 'A', status, sekvensByte };
            
            // 2. BERÄKNA CHECKSUMMA IGEN (För svarsmeddelandet)
            byte checksum = beraknaChecksumma(dataForChecksumma);
            
            // 3. Bygg det kompletta paketet
            byte[] ackPaket = new byte[] {
                '$', 
                'A', 
                status, 
                sekvensByte, 
                checksum, 
                '\n'
            };
            
            // 4. Skicka iväg det
            ut.write(ackPaket);
            ut.flush(); 
            
            if (status == 1) {
                System.out.println("-> Skickade ACK för sekvens: " + sekvensnummer);
            } else {
                System.out.println("-> Skickade NACK (Felkod " + status + ") för sekvens: " + sekvensnummer);
            }
            
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