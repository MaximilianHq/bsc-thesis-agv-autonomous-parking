/*
 * Click nbfs://nbhost/SystemFileSystem/Templates/Licenses/license-default.txt to change this license
 */

package com.mycompany.bluetoothmirror;

/**
 * Eko-server för kontinuerlig tvåvägskommunikation.
 * Denna kod körs i mottagardatorn (Servern). 
 * Den lyssnar efter anslutningar, tar emot meddelanden och skickar OMEDELBART 
 * tillbaka samma meddelande (som ett eko/ACK) till avsändaren.
 * Används i kombination med BluetoothTransceiver.
 * Kräver Linux hos både sändare och mottagare.
 *
 * @author kts
 */

import java.io.*; 
import javax.microedition.io.*; 
import javax.bluetooth.*; 

public class BluetoothMirror { 

    public static void main(String args[]) { 
        try { 
            // 1. Försök starta upp en Bluetooth-tjänst lokalt (väntar på att klienter ska ansluta)
            // Använder SPP (Serial Port Profile) UUID 0x1101
            StreamConnectionNotifier service = (StreamConnectionNotifier) Connector.open("btspp://localhost:" + new UUID(0x1101).toString() 
                    + ";name=TNK132-spegel-Grupp_2"); 
            
            // Blockerar och väntar tills en sändare (Transceiver) ansluter
            StreamConnection anslutning = (StreamConnection) service.acceptAndOpen(); 
            System.out.println("Anslutning upprättad"); 

            // 2. Förbered kommunikationsströmmar:
            // bluetooth_ut: För att skicka TILLBAKA data till sändaren
            PrintStream bluetooth_ut = new PrintStream(anslutning.openOutputStream()); 
            // bluetooth_in: För att läsa INKOMMANDE data från sändaren
            BufferedReader bluetooth_in = new BufferedReader(new InputStreamReader(anslutning.openInputStream())); 
            
            // 3. Oändlig loop för att hålla anslutningen öppen och kontinuerligt processa data 
            while (true) { 
                System.out.println("Väntar på instruktion..."); 
                
                // Läser det inkommande meddelandet från sändaren 
                String Instruktion = bluetooth_in.readLine(); 
                Int instruktions_Langd = bluetooth_in.readLine(); // Dessa två behöver delas så instruktionen och tiden(?) skiljs 
                
                // Om klienten kopplat ifrån (skickar null), bryt loopen och stäng ner 
                if (Instruktion == null) { 
                    break; 
                } 
                if (Instruktion == Stopp) { 
                    // Execute [0, 0, 0, 0, aktuell nod, tid/celler] 
                    bluetooth_ut.println("Stoppades. Vid position: " + AGV_Position); 
                } 
                
                // Skicka tillbaka exakt samma meddelande (ett eko) som en "Ack/Bekräftelse" 
                bluetooth_ut.println(Instruktion); // Detta kanske är onödigt om vi inte vill lägga så höga prestandakrav på AGV:n 
                
                // Skriv ut i vår lokala konsol vad vi precis tog emot och returnerade 
                // Troligtvis onödigt, finns ingen display på AGV:n? 
                System.out.println("Tog emot instruktion: " + Instruktion); // Eller positionsdata 
                // bluetooth_ut.println("AGV:ns position: " + AGV_Position); AGV_Position behöver definieras/hämtas på något sätt. Från fyrerna? ED? 
                
                /* 
                Här behövs någon form av execute så AGV:n utför instruktionen "Instruktion" under 
                if (Instruktion == Instruktion_Forward) { 
                // Utför den mottagna instruktionen 
                } 
                */ 
                
            } 
            
            // Stäng anslutningen om klienten kopplar ifrån 
            anslutning.close(); 
        } catch (IOException e) { 
            System.out.print(e.toString()); 
        } 
    } 
} 
