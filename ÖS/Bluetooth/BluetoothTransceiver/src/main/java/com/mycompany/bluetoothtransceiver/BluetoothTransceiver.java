/*
 * Click nbfs://nbhost/SystemFileSystem/Templates/Licenses/license-default.txt to change this license
 */

package com.mycompany.bluetoothtransceiver;

/**
 * Interaktiv klient för tvåvägskommunikation (chatt-klient).
 * Denna kod körs hos sändaren och ansluter till en server (t.ex. BluetoothMirror).
 * Den läser in text från tangentbordet, skickar det, och väntar därefter på 
 * att få ett svar (ACK) tillbaka från servern innan nästa meddelande kan skickas.
 * Följande kod kräver att Linux körs som OS hos både sändare och mottagare.
 * * @author kts
 */

import java.io.*;
import javax.microedition.io.*;
import javax.bluetooth.*;

public class BluetoothTransceiver {

    public static void main(String args[]) {
        try { 
            // 1. Försöker upprätta en anslutning till mottagaren (Servern)
            // Byt ut MAC-adressen till mottagarens aktuella adress och kanal
            StreamConnection anslutning = (StreamConnection) Connector.open("btspp://d4d252ed2d43:1"); 
            
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
                
                // Om användaren avbryter (t.ex. Ctrl+C), bryt loopen
                if (meddelande_ut == null) {
                    break;
                }
                
                // Skicka tangentbordsinmatningen över Bluetooth till mottagaren
                bluetooth_ut.println(meddelande_ut); 
                
                // Blockerar (väntar) tills mottagaren skickar ett svar tillbaka
                String meddelande_in = bluetooth_in.readLine(); 
                
                // Skriv ut svaret från mottagaren
                System.out.println("Mottaget: " + meddelande_in);
            }
            
            // Stäng anslutningen snyggt om loopen bryts
            anslutning.close(); 
        } catch (Exception e) {
            System.out.print(e.toString());
        }
    }
}