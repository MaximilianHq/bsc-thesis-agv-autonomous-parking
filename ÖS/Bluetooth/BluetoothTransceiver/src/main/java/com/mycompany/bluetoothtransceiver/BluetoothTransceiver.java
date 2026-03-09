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
            StreamConnection anslutning = (StreamConnection) Connector.open("btspp://d4d252ed2d43:1"); // MAC-address och kanal ändras till AGV:n 
            
            // 2. Förbereder strömmar för kommunikation: 
            // bluetooth_ut: För att skicka text TILL mottagaren 
            PrintStream bluetooth_ut = new PrintStream(anslutning.openOutputStream()); 
            
            // bluetooth_in: För att läsa inkommande svar FRÅN mottagaren 
            BufferedReader bluetooth_in = new BufferedReader(new InputStreamReader(anslutning.openInputStream())); 
            
            // 3. Startar en oändlig loop för kontinuerlig kommunikation 
            while (true) { // tangentbord: För att läsa det användaren skriver lokalt i konsolen 
            BufferedReader tangentbord = new BufferedReader(new InputStreamReader(System.in)); 
            // Den här ska bytas ut mot att "tangentbord" ska vara den instruktion som krävs för att följa linjen 
            // som tagits fram med vår variant av Dijkstras algoritm. Det Fredrik har gjort! 
            
            int Instruktion_ut = new Dijkstras_Algoritm(value_for_an_Instruction); 
            // Dessa är samma tanke! Inte löst hur det ska göras än! 
            vector Instruktion_ut = [x1, x2, x3, x4, Node_for_new_Instruction, Instruktion_Langd]; 
            // = [vänster fram, höger fram, vänster bak, höger bak, Nod där ny rörelse ska påbörjas, tid/celler]; 
            // x1, x2, x3, x4 = [-1, 0 , 1] = [Back, Stilla, Fram] 
                
                // Läs en rad text som användaren skriver in via tangentbordet 
                // Tas bort? 
                String meddelande_ut = tangentbord.readLine(); 
                

        /* 
                if (Instruktion_ut == Instruktion_Forward) { 
                // Instruktion för vänster framhjul kör framåt, höger framhjul kör framåt, vänster bakhjul kör framåt & höger bakhjul kör framåt. 
                } 
                else if (Instruktion_ut == Instruktion_Reverse) { 
                // Instruktion för vänster framhjul kör bakåt, höger framhjul kör bakåt, vänster bakhjul kör bakåt & höger bakhjul kör bakåt. 
                } 
                else if (Instruktion_ut == Instruktion_Left) { 
                // Instruktion för vänster framhjul kör framåt, höger framhjul kör bakåt, vänster bakhjul kör bakåt & höger bakhjul kör framåt. 
                } 
                else if (Instruktion_ut == Instruktion_Right) { 
                // Instruktion för vänster framhjul kör bakåt, höger framhjul kör framåt, vänster bakhjul kör framåt & höger bakhjul kör bakåt. 
                } 
                else if (Instruktion_ut == Instruktion_Forward_Left) { 
                // Instruktion för vänster framhjul kör framåt, höger framhjul är still, vänster bakhjul är still & höger bakhjul kör framåt. 
                } 
                else if (Instruktion_ut == Instruktion_Forward_Right) { 
                // Instruktion för vänster framhjul är still, höger framhjul kör framåt, vänster bakhjul kör framåt & höger bakhjul är still. 
                } 
                else if (Instruktion_ut == Instruktion_Reverse_Left) { 
                // Instruktion för vänster framhjul är still, höger framhjul kör bakåt, vänster bakhjul kör bakåt & höger bakhjul är still. 
                } 
                else if (Instruktion_ut == Instruktion_Reverse_Right) { 
                // Instruktion för vänster framhjul kör bakåt, höger framhjul är still, vänster bakhjul är still & höger bakhjul kör bakåt. 
                } 
        } 
          */       
                // Om användaren avbryter (t.ex. Ctrl+C), bryt loopen 
                if (Instruktion == null) { 
                    break; 
                } 
                
                // Skicka tangentbordsinmatningen över Bluetooth till mottagaren 
                bluetooth_ut.println(Instruktion_ut); 
                
                // Blockerar (väntar) tills mottagaren skickar ett svar tillbaka 
                String Position_in = bluetooth_in.readLine(); 

                vector Stopp = [0, 0, 0, 0, 

                // int eller double eller vector Position_in = bluetooth.in.readLine(); 
                // if (Position_in != inom +-1 cell från linjen) { 
                // bluetooth_ut.println(Stopp); 
                // Här behöver det nog delas upp på något vis så det är tydligt vad som läses in till vilken variabel. 

            } 
                
                // Skriv ut svaret från mottagaren 
                System.out.println("Fick tillbaka position: " + position_in); 
            } 
            
            // Stäng anslutningen om loopen bryts 
            anslutning.close(); 
        } catch (Exception e) {
            System.out.print(e.toString());
        }
    }
}
