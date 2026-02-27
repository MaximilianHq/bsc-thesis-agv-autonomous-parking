/*
 * Click nbfs://nbhost/SystemFileSystem/Templates/Licenses/license-default.txt to change this license
 */

package com.mycompany.bluetoothreceiver;

/**
 * Enkel server för envägskommunikation över Bluetooth.
 * Denna kod körs i mottagardatorn och fungerar som en passiv lyssnare.
 * Den tar emot ETT inkommande meddelande (max 80 bytes) och stänger sedan anslutningen.
 * Används tillsammans med BluetoothTransmitter.
 * * @author kts
 */

import java.io.*;
import javax.microedition.io.*;
import javax.bluetooth.*;

public class BluetoothReceiver {

    public static void main(String args[]) {
        try {
            // Skapar en lokal tjänst som väntar på inkommande anslutningar.
            // localhost indikerar att vi agerar server. UUID (0x1101) är standard för Serial Port Profile (SPP).
            StreamConnectionNotifier service = (StreamConnectionNotifier) Connector.open("btspp://localhost:" + new UUID(0x1101).toString()
                    + ";name=TNK132-test");
            
            // Blockerar och väntar här tills en sändare (t.ex. BluetoothTransmitter) ansluter
            StreamConnection anslutning = (StreamConnection) service.acceptAndOpen();
            
            // Skapar en in-ström för att läsa den data som sändaren skickar
            InputStream bluetooth_in = anslutning.openInputStream();
            
            // Förbereder en buffer (tillfälligt minne) för att hålla max 80 bytes av data
            byte buffer[] = new byte[80];
            
            // Läser in datan till buffern och sparar hur många bytes som faktiskt lästes
            int antal_bytes = bluetooth_in.read(buffer);
            
            // Omvandlar de inlästa byten till en läsbar sträng och skriver ut den
            String mottaget = new String(buffer, 0, antal_bytes);
            System.out.println("\n" + "Mottaget meddelande: " + mottaget);
            
            // Stänger ner anslutningen efter det första mottagna meddelandet
            anslutning.close();
        } catch (IOException e) {
            System.err.print(e.toString());
        }
    }
}