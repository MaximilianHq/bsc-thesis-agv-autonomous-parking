///*
// * Click nbfs://nbhost/SystemFileSystem/Templates/Licenses/license-default.txt to change this license
// */
//
//package com.mycompany.bluetoothtransceiver;
//
///**
// * Interaktiv klient för tvåvägskommunikation (chatt-klient).
// * Denna kod körs hos sändaren och ansluter till en server (t.ex. BluetoothMirror).
// * Den läser in text från tangentbordet, skickar det, och väntar därefter på 
// * att få ett svar (ACK) tillbaka från servern innan nästa meddelande kan skickas.
// * Följande kod kräver att Linux körs som OS hos både sändare och mottagare.
// * * @author kts
// */
//
//import java.io.*;
//import javax.microedition.io.*;
//import javax.bluetooth.*;
//
//public class BluetoothTransceiver {
//
//    public static void main(String args[]) {
//        try { 
//            // 1. Försöker upprätta en anslutning till mottagaren (Servern)
//            // Byt ut MAC-adressen till mottagarens aktuella adress och kanal
//            StreamConnection anslutning = (StreamConnection) Connector.open("btspp://3c71bfcf083e:1"); 
//            
//            // 2. Förbereder strömmar för kommunikation:
//            // bluetooth_ut: För att skicka text TILL mottagaren
//            PrintStream bluetooth_ut = new PrintStream(anslutning.openOutputStream());
//            
//            // bluetooth_in: För att läsa inkommande svar FRÅN mottagaren
//            BufferedReader bluetooth_in = new BufferedReader(new InputStreamReader(anslutning.openInputStream()));
//
//            // tangentbord: För att läsa det användaren skriver lokalt i konsolen
//            BufferedReader tangentbord = new BufferedReader(new InputStreamReader(System.in));
//            
//            // 3. Startar en oändlig loop för kontinuerlig kommunikation
//            while (true) { 
//                // Läs en rad text som användaren skriver in via tangentbordet
//                String meddelande_ut = tangentbord.readLine();
//                
//                // Om användaren avbryter (t.ex. Ctrl+C), bryt loopen
//                if (meddelande_ut == null) {
//                    break;
//                }
//                
//                // Skicka tangentbordsinmatningen över Bluetooth till mottagaren
//                bluetooth_ut.println(meddelande_ut); 
//                
//                // Blockerar (väntar) tills mottagaren skickar ett svar tillbaka
//                String meddelande_in = bluetooth_in.readLine(); 
//                
//                // Skriv ut svaret från mottagaren
//                System.out.println("Mottaget: " + meddelande_in); 
//            }
//            
//            // Stäng anslutningen snyggt om loopen bryts
//            anslutning.close(); 
//        } catch (Exception e) {
//            System.out.print(e.toString());
//        }
//    }
//}
package com.mycompany.bluetoothtransceiver;

import java.io.*;
import javax.microedition.io.*;
import javax.bluetooth.*;

public class BluetoothTransceiver {

    public static void main(String args[]) {
        try {
            // 1. Försöker upprätta en anslutning till mottagaren (Servern)
            // Byt ut MAC-adressen till mottagarens aktuella adress och kanal
            StreamConnection anslutning = (StreamConnection) Connector.open("btspp://3c71bfcf083e:1");

            // 2. Förbereder strömmar för kommunikation:
            // bluetooth_ut: För att skicka text TILL mottagaren
            PrintStream bluetooth_ut = new PrintStream(anslutning.openOutputStream());

            // bluetooth_in: För att läsa inkommande svar FRÅN mottagaren
            BufferedReader bluetooth_in = new BufferedReader(new InputStreamReader(anslutning.openInputStream()));

            // tangentbord: För att läsa det användaren skriver lokalt i konsolen
            BufferedReader tangentbord = new BufferedReader(new InputStreamReader(System.in));

            // 3. Startar en oändlig loop för kontinuerlig kommunikation
            while (true) {
                // Läs en rad text som användaren skriver in via tangentbordet
                String meddelande_ut = tangentbord.readLine();
                String meddelande_ut_type = meddelande_ut.substring(0, 1);

                // Om användaren avbryter (t.ex. Ctrl+C), bryt loopen
                // Om användaren avbryter (t.ex. Ctrl+C) eller skickar tom sträng
                if (meddelande_ut == null || meddelande_ut.isEmpty()) {
                    break;
                }

                // 1. Plocka ut FÖRSTA tecknet som en ensam 'byte' till type
                byte typeByte = (byte) meddelande_ut.charAt(0);

                // 2. Klipp ut RESTEN av meddelandet (från index 1) till data-arrayen
                String dataStrang = meddelande_ut.substring(1);
                byte[] dataBytes = dataStrang.getBytes();

                // 3. Nu matchar argumenten konstruktorn perfekt: (byte, byte[])
                Packet pkt = new Packet(typeByte, dataBytes);

                // Beräkna checksumma med csum-metoden
                byte checksum = csum(pkt);
                System.out.println("Beräknad checksum: " + checksum);

                // Skicka tangentbordsinmatningen över Bluetooth till mottagaren
                // Obs: La till ett mellanslag eller separator om det behövs innan checksumman?
                // 1. Skicka inledningen och meddelandet som vanlig text
                bluetooth_ut.print("$" + meddelande_ut); 

                // 2. Skicka checksumman som ETT ENSAMT RÅTT MATEMATISKT VÄRDE (1 byte)
                bluetooth_ut.write(checksum);

                // 3. Skicka radbrytningen ("newline") som text för att avsluta paketet
                bluetooth_ut.print("\n");

                // Blockera (väntar) tills mottagaren skickar ett svar tillbaka
                String meddelande_in = bluetooth_in.readLine();

                // Skriv ut svaret från mottagaren
                System.out.println("Mottagen rådata: " + meddelande_in);

                //Tolka och konvertera meddelandet till variabler 
                hanteraInkommandeMeddelande(meddelande_in);
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

        // Konstruktor för Packet-klassen
        public Packet(byte type, byte[] data) {
            this.type = type;
            this.data = data;
            this.dataLen = data.length;
        }
    }

    public static void hanteraInkommandeMeddelande(String meddelande) {
        // Kollar så meddelandet inte är tomt och börjat med "$" 
        if (meddelande == null || !meddelande.trim().startsWith("$")) {
            System.out.println("Ogiltigt meddelande, saknar startbyte: " + meddelande);
            return;
        }
        // Dela upp meddelandet i en array baserat på mellanslag 
        // Exempel: "$ M 10 20 1 2 255" blir ["$", "M", "10", "20", "1", "2", "255"] 
        String[] delar = meddelande.trim().split("\\s+");

        if (delar.length < 2) {
            System.out.println("För kort meddelande");
            return;
        }

        String meddelandeTyp = delar[1]; // M eller H 

        try {
            // Positionsuppdatering 
            if (meddelandeTyp.equals("M")) {
                if (delar.length >= 7) { // Säkerställ att alla parametrar finns 

                    int posX = Integer.parseInt(delar[2]);
                    int posY = Integer.parseInt(delar[3]);
                    int status = Integer.parseInt(delar[4]);
                    int sekvensnummer = Integer.parseInt(delar[5]);
                    int checksum = Integer.parseInt(delar[6]);

                    // Tveksamt om det här behövs? 
                    System.out.println("--- POSITION UPPDATERAD ---");
                    System.out.println("X: " + posX + ", Y: " + posY);
                    System.out.println("Status: " + status + ", Sekvensnummer: " + sekvensnummer);
                    System.out.println("Checksum: " + checksum);

                    // Här ska variablerna skickas till övriga delar av programmet/ÖS!! 
                    // t.ex uppdateraKarta(posX, posY, status); 
                } else {
                    System.out.println("Saknar parametrar för meddelandetyp Positionsuppdatering.");
                }
            } // Hinderdetektering 
            else if (meddelandeTyp.equals("H")) {
                if (delar.length >= 5) {
                    int posX = Integer.parseInt(delar[2]);
                    int posY = Integer.parseInt(delar[3]);
                    int status = Integer.parseInt(delar[4]);

                    System.out.println("--- Akta HINDER!!! ---");
                    System.out.println("Hinder vid X: " + posX + ", Y: " + posY);
                    System.out.println("Status: " + status);

                    // Här ska variablerna skickas till övriga delar av programmet/ÖS!! 
                    // t.ex hanteraHinder(posX, posY); 
                } else {
                    System.out.println("Saknar parametrar för meddelandetyp Hinderdetektion.");
                }
            } // Okänd meddelandetyp 
            else {
                System.out.println("Okänd meddelandetyp: " + meddelandeTyp);
            }
        } catch (NumberFormatException e) {
            System.out.println("Kunde inte konvertera data till siffror: " + e.getMessage());
        }
    }
}
