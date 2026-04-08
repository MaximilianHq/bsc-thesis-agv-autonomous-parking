package com.mycompany.bluetoothtransceiver;

import java.io.*;
import javax.microedition.io.*;
import javax.bluetooth.*;

public class BluetoothTransceiver {

    public static void main(String args[]) {
        try {
            // 1. Försöker upprätta en anslutning till mottagaren (Servern)
            StreamConnection anslutning = (StreamConnection) Connector.open("btspp://3c71bfcf083e:1");

            // 2. Förbereder strömmar (ÄNDRING: Ändrat till InputStream för att hantera bytes)
            PrintStream bluetooth_ut = new PrintStream(anslutning.openOutputStream());
            InputStream bluetooth_in = anslutning.openInputStream(); 
            BufferedReader tangentbord = new BufferedReader(new InputStreamReader(System.in));

            // 3. Startar en oändlig loop för kontinuerlig kommunikation
            while (true) {
                // Läs en rad text som användaren skriver in via tangentbordet
                String meddelande_ut = tangentbord.readLine();
                
                if (meddelande_ut == null || meddelande_ut.isEmpty()) {
                    break;
                }

                String meddelande_ut_type = meddelande_ut.substring(0, 1);
                byte typeByte = (byte) meddelande_ut.charAt(0);
                String dataStrang = meddelande_ut.substring(1);
                byte[] dataBytes = dataStrang.getBytes();

                Packet pkt = new Packet(typeByte, dataBytes);
                byte checksum = csum(pkt);
                System.out.println("Beräknad checksum: " + checksum);

                // Skickar data till AGV
                bluetooth_ut.print("$" + meddelande_ut); 
                bluetooth_ut.write(checksum);
                bluetooth_ut.print("\n");

                // --- NYTT SÄTT ATT LÄSA MOTTAGET MEDDELANDE BYTE FÖR BYTE ---
                ByteArrayOutputStream mottagarBuffer = new ByteArrayOutputStream();
                int inkommandeByte;
                boolean startHittad = false;

                // Läs en byte i taget. Avbryt loopen när vi hittar '\n'
                while ((inkommandeByte = bluetooth_in.read()) != -1) {
                    if (inkommandeByte == '$') {
                        startHittad = true;
                        mottagarBuffer.reset(); // Rensa ifall det fanns gammalt skräp
                    }
                    
                    if (startHittad) {
                        mottagarBuffer.write(inkommandeByte);
                        
                        if (inkommandeByte == '\n') {
                            break; // Hela meddelandet är nu inläst! Hoppa ur while-loopen.
                        }
                    }
                }

                // Konvertera bufferten till en faktisk byte-array
                byte[] fardigtMeddelande = mottagarBuffer.toByteArray();
                
                System.out.println("Mottog paket med längd: " + fardigtMeddelande.length + " bytes");

                // Tolka och konvertera meddelandet till variabler 
                hanteraInkommandeMeddelande(fardigtMeddelande);
            }

            // Stäng anslutningen snyggt om loopen bryts
            anslutning.close();
        } catch (Exception e) {
            System.out.print(e.toString());
        }
    }

    // Metod för att beräkna checksum (csum)
    public static byte csum(Packet pkt) {
        byte crc = 0;
        crc ^= pkt.type;  // XOR med typen
        for (int i = 0; i < pkt.dataLen; i++) {
            crc ^= pkt.data[i];  // XOR med varje byte i data
        }
        return crc;
    }

    // Packet-klass som representerar en packet
    public static class Packet {
        public byte type;    // Tänk på att byte motsvarar uint8_t
        public byte[] data;  // data arrayen är samma som pkt.data
        public int dataLen;  // data_len motsvaras av längden på data-arrayen

        public Packet(byte type, byte[] data) {
            this.type = type;
            this.data = data;
            this.dataLen = data.length;
        }
    }

    // --- ÄNDRAD METOD FÖR ATT HANTERA BYTE-ARRAY ISTÄLLET FÖR STRING ---
    public static void hanteraInkommandeMeddelande(byte[] data) {
        // Kontrollera att meddelandet har en rimlig minimilängd (ex: $, Typ, C, \n)
        if (data == null || data.length < 4) {
            System.out.println("Ogiltigt eller för kort meddelande.");
            return;
        }
        
        // Kontrollera att det börjar med $
        if (data[0] != '$') {
            System.out.println("Ogiltigt meddelande, saknar startbyte.");
            return;
        }

        byte meddelandeTyp = data[1]; // Andra byten är typen (M, H etc)

        // '& 0xFF' används för att göra om Javas signed byte (-128 till 127) till 0-255.
        if (meddelandeTyp == 'M') {
            // Kontrollera längden utifrån Tabell 2: $, M, XX, YY, L, S, C, \n (8 bytes)
            if (data.length >= 8) {
                int posX = data[2] & 0xFF;
                int posY = data[3] & 0xFF;
                int status = data[4] & 0xFF;
                int sekvensnummer = data[5] & 0xFF;
                int checksum = data[6] & 0xFF; // Om du vill skriva ut den som positiv siffra

                System.out.println("--- POSITION UPPDATERAD ---");
                System.out.println("X: " + posX + ", Y: " + posY);
                System.out.println("Status: " + status + ", Sekvensnummer: " + sekvensnummer);
                System.out.println("Checksum: " + checksum);
                
                // uppdateraKarta(posX, posY, status); 
            } else {
                System.out.println("Saknar parametrar för meddelandetyp Positionsuppdatering.");
            }
        } 
        else if (meddelandeTyp == 'H') {
            // Kontrollera längden utifrån Tabell 2: $, H, XX, YY, S, C, \n (7 bytes)
            if (data.length >= 7) {
                int posX = data[2] & 0xFF;
                int posY = data[3] & 0xFF;
                // Status för Hinder fanns inte med i tabell 2, men låt oss anta att den finns om du lade in den
                // Om er struct ser annorlunda ut, justera indexen nedan:
                int sekvensnummer = data[4] & 0xFF;

                System.out.println("--- Akta HINDER!!! ---");
                System.out.println("Hinder vid X: " + posX + ", Y: " + posY);

                // hanteraHinder(posX, posY); 
            } else {
                System.out.println("Saknar parametrar för meddelandetyp Hinderdetektion.");
            }
        } 
        else {
            System.out.println("Okänd meddelandetyp: " + (char)meddelandeTyp);
        }
    }
}