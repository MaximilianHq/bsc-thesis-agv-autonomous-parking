/*
 * Click nbfs://nbhost/SystemFileSystem/Templates/Licenses/license-default.txt to change this license
 */

package com.mycompany.bluetoothtransmitter; 

/** * Enkel klient för envägskommunikation över Bluetooth.
 * Denna klass ansluter till en specifik mottagare (via MAC-adress), 
 * skickar ett förbestämt testmeddelande och stänger sedan anslutningen.
 * Används tillsammans med BluetoothReceiver.
 * * @author kts 
 */ 

import java.io.*; 
import javax.microedition.io.*; 
import javax.bluetooth.*; 

public class BluetoothTransmitter {

    public static void main(String args[]) {
        try {
            // Etablerar en anslutning till servern. 
            // btspp:// indikerar Bluetooth Serial Port Profile.
            // Ersätt D4D252ED2D43 med den faktiska mottagarens MAC-adress och :17 med rätt kanal.
            StreamConnection anslutning = (StreamConnection) Connector.open("btspp://D4D252ED2D43:17"); // Hörndator 
            // StreamConnection anslutning = (StreamConnection) Connector.open("btspp://2C7BA0880044:3"); // Andrés laptop 
            
            // Skapar en ut-ström för att skicka data över den upprättade Bluetooth-anslutningen
            PrintStream bluetooth_ut = new PrintStream(anslutning.openOutputStream()); 
            
            // Skickar iväg det hårdkodade textmeddelandet
            bluetooth_ut.println("Test från grupp 2"); 
            
            // Pausar programmet i en halv sekund för att säkerställa att meddelandet hinner 
            // skickas över nätverket innan vi stänger kopplingen.
            Thread.sleep(500); 
            
            // Stänger anslutningen när vi är klara
            anslutning.close();
        } catch (Exception e) {
            System.out.print(e.toString());
        } 
    }
}