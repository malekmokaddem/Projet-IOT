
#include <stm32l475e_iot01.h>
#include <stm32l475e_iot01_accelero.h>
#include <stm32l475e_iot01_gyro.h>
#include <stm32l475e_iot01_hsensor.h>
#include <stm32l475e_iot01_magneto.h>
#include <stm32l475e_iot01_psensor.h>
#include <stm32l475e_iot01_qspi.h>
#include <stm32l475e_iot01_tsensor.h>





// This example uses an Arduino Uno together with
// a WiFi Shield to connect to shiftr.io.
//
// You can check on your device after a successful
// connection here: https://shiftr.io/try.
//
// by Joël Gähwiler
// https://github.com/256dpi/arduino-mqtt

#include <SPI.h>
#include <WiFiST.h>
#include <WiFiUdpST.h>
#include <MQTTClient.h>

SPIClass SPI_3(PC12, PC11, PC10);
WiFiClass WiFi(&SPI_3, PE0, PE1, PE8, PB13);
int status = WL_IDLE_STATUS;
char ssid[] = "ORANGE_17C9";
//char ssid[] = "iPad";
char pass[] = "BGAYZQ8C";  
//char pass[] = "khaledjelassi";     
WiFiClient net;
MQTTClient client;
unsigned int localPort = 8002;           // local port to listen on
char packetBuffer[255];                  //buffer to hold incoming packet
char  ReplyBuffer[] = "acknowledged";    // a string to send back

WiFiUDP Udp;
unsigned long lastMillis = 0;
float sensor_value_T = 0;
float sensor_value_H = 0;
float sensor_value_P = 0;
String string_MQTT;
int16_t pDataXYZ[3] = {0};
float pGyroDataXYZ[3] = {0};
int16_t topic=0; // envoyer une grandeur par seconde
void setup() {
    BSP_TSENSOR_Init();
    BSP_HSENSOR_Init();
    BSP_PSENSOR_Init();
    BSP_MAGNETO_Init();
    BSP_GYRO_Init();
    BSP_ACCELERO_Init();
    Serial.begin(115200);
    if (WiFi.status() == WL_NO_SHIELD) {
    Serial.println("WiFi module not present");
    // don't continue:
    while (true);
  }
  
// attempt to connect to Wifi network:
    while (status != WL_CONNECTED) {
    Serial.print("Attempting to connect to SSID: ");
    Serial.println(ssid);
    // Connect to WPA/WPA2 network. Change this line if using open or WEP network:
    status = WiFi.begin(ssid, pass);

    // wait 10 seconds for connection:
    delay(10000);
  }
    Serial.println("Connected to wifi");
    printWifiStatus();

    Serial.println("\nStarting connection to server...");
  // if you get a connection, report back via serial:
  
    Serial.println("Received packet From ");

  // Note: Local domain names (e.g. "Computer.local" on OSX) are not supported by Arduino.
  // You need to set the IP address directly.
    client.begin("192.168.1.11", net);
 //   client.begin("172.20.10.3", net);  // client MQTT
    client.onMessage(messageReceived);
    Serial.print("\nconnecting...");   // Mot de passe: always; user: khaled
    while (!client.connect("arduino", "khaled", "always")) {
    Serial.print(".");
    delay(1000);
  }

    Serial.println("\n connected!");

    client.subscribe("/Reponse_Serveur");  // Ceci est un essai de souscription au topic /hello
  // client.unsubscribe("/hello");
}

void loop() {
   
    client.loop();

  if (!client.connected()) {
     client.disconnect();
   
    
    // wait 10 seconds for connection:
      
     client.begin("192.168.1.11", net);
     client.connect("STM32-B-L475-IOT01A2", "khaled", "always");
     delay(1000);
     string_MQTT =  String(sensor_value_H, 1);  
     client.publish("/test","Hello");
     topic=-1;
    
  }
    
  // publish a message roughly every second.
  if (millis() - lastMillis > 1000) {
     topic++;
     Serial.print("\n *************TEMPERATURE*************** ");
     sensor_value_T = BSP_TSENSOR_ReadTemp();
     Serial.print("\n TEMPERATURE = ");
     Serial.print(sensor_value_T);
     Serial.print("\n *************************************** ");
     Serial.print("\n *************HUMIDITY****************** ");
     sensor_value_H = BSP_HSENSOR_ReadHumidity();
     Serial.print("\n Humidite = ");
     Serial.print(sensor_value_H);
     Serial.print("\n *************************************** ");
     Serial.print("\n *************PRESSURE****************** ");
     sensor_value_P = BSP_PSENSOR_ReadPressure();
     Serial.print("\n Pression = ");
     Serial.print(sensor_value_P);
     Serial.print("\n *************************************** ");
     Serial.print("\n *************MAGNET******************** ");
     BSP_MAGNETO_GetXYZ(pDataXYZ);
     Serial.print("\n MAGNETO_X........... = ");
     Serial.print(pDataXYZ[0]);
     Serial.print("\n MAGNETO_Y........... = ");
     Serial.print(pDataXYZ[1]);
     Serial.print("\n MAGNETO_Z........... = ");
     Serial.print(pDataXYZ[2]);
     Serial.print("\n *************************************** ");
     Serial.print("\n *************GYRO********************** ");
     BSP_GYRO_GetXYZ(pGyroDataXYZ);
     Serial.print("\n GYRO_X ............. = ");
     Serial.print(pGyroDataXYZ[0]);
     Serial.print("\n GYRO_Y ............. = ");
     Serial.print(pGyroDataXYZ[1]);
     Serial.print("\n GYRO_Z ............. = ");
     Serial.print(pGyroDataXYZ[2]);
      Serial.print("\n ************************************** ");
     Serial.print("\n *************ACCELERATION************** ");
     BSP_ACCELERO_AccGetXYZ(pDataXYZ);
     Serial.print("\n ACCELERO_X ......... = ");
     Serial.print(pDataXYZ[0]);
     Serial.print("\n ACCELERO_Y ......... = ");
     Serial.print(pDataXYZ[1]);
     Serial.print("\n ACCELERO_Z ......... = ");
     Serial.print(pDataXYZ[2]);
     lastMillis = millis();
     if(topic==1) {
     string_MQTT =  String(sensor_value_T, 1);  
     client.publish("/temperature",string_MQTT);
     }
     if(topic==2) {
     string_MQTT =  String(sensor_value_H, 1);  
     client.publish("/Humidite",string_MQTT);
     }
     if(topic==3) {
     string_MQTT =  String(sensor_value_P, 1);  
     client.publish("/Pression",string_MQTT);
     topic=0;
     }
  }
}

void messageReceived(String &topic, String &payload) {
     Serial.print("\n -------------------------------------- ");
     Serial.println("\n\n incoming: " + topic + " - " + payload);
     Serial.print("\n\n -------------------------------------- ");
}
void printWifiStatus() {
  // print the SSID of the network you're attached to:
    Serial.print("SSID: ");
    Serial.println(WiFi.SSID());

  // print your WiFi device's IP address:
    IPAddress ip = WiFi.localIP();
    Serial.print("IP Address: ");
    Serial.println(ip);

  // print the received signal strength:
    long rssi = WiFi.RSSI();
    Serial.print("signal strength (RSSI):");
    Serial.print(rssi);
    Serial.println(" dBm");
}
