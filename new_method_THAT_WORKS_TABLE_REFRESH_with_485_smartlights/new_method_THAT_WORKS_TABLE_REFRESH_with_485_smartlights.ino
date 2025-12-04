#include <WiFi.h>
#include <WebServer.h>
#include <ModbusMaster.h>
#include <ESPmDNS.h>
#include <NetworkUdp.h>
#include <ArduinoOTA.h>
//#include <ESPAsyncWebServer.h>

int photocellInputPin = 10;     // the cell and 10K pulldown are connected to earth
int photocellReading;     // the analog reading from the sensor divider
int fenceLightsPin = 9;          // connect PWM to pin 9 (PWM pin)
int garageLightsPin = 46;        // connect PWM to pin 46 (PWM pin)

int brightness;

int onTime;  // For ambient light delay
int offTime;  // For ambient light delay

int dayLevel = 25;  // photocell percentage reading SETTING for lights OFF
int nightLevel = 15; // photocell percentage reading SETTING for lights ON
float lightDelayTime = 10;  //Adjust this for light delay in minutes MUST BE A WHOLE NUMBER (no decimal places)

// Below converts minutes to secconds BUT taking into account the 2 seccond program cycle (hence multiply by 30 and not 60)
float correctDelayTime = lightDelayTime * 30;

ModbusMaster node;

/*
A note about which ESP pins to use to read data from Renogy: 
- The original code used different pins for a different ESP32 board.
The one I was first using was a wroom 30 pin so 16 and 17.
Then I used a ESP32-S3 with 44 pins so used pin 17 (U1TXD) and pin 18 (U1RXD)
other ESP's will be different.
It is important to get these correct!
*/

#define RXD2 18  // For RS232 AND RS485
#define TXD2 17  // For RS232 AND RS485

// Newer Renogy Rover chargers have switched to RS485 coms now, anoying but I got round it with extra code
// So this version of my code can deal with both now, but need to use 2 extra pins for transmission for RS485.
// You need a MAX485 to TTL pcb, cheap as chips on Amazon but only use the 3.3V supply for ESP32, works fine here.
// Personally, my Renogy is the older model with RS232, but works fine with a RS232 to TTL Amazon board.
// Again, I altered the code to work with RS485 and a RS485/ttl converter for a m8, confirmed it works fine on his charger!
// So this code will work on both now without changes.

#define MAX485_DE 5  // For RS485 Only
#define MAX485_RE_NEG 4  // For RS485 Only



/*
Define some colours for webpage data cells to have background colours
on condition of the state of values (good/green, average/yellow, poor/red).
*/

#define bacGreen "GreenYellow"
#define bacYellow "Yellow"
#define bacRed "Salmon"

/*
String some names for the value colours.
*/
String batVoltCol;
String batChargeCur;
String batTemperature;
String conTemperature;
String solPanVol;
String solPanCur;
String solPanWat;
String sysLoadVol;
String sysLoadCur;
String maxBatVolToday;
String minBatVolToday;
String maxBatChargeToday;
String maxBatDischargeToday;
String espTemp;
String fenceLightsCond;

int ESP_temp_celsius;

/*
Number of registers to check. I think all Renogy controllers have 35
data registers (not all of which are used) and 17 info registers.
*/
const uint32_t num_data_registers = 35;
const uint32_t num_info_registers = 17;

// if you don't have a charge controller to test with, can set this to true to get non 0 voltage readings
bool simulator_mode = false;


// A struct to hold the controller data
struct Controller_data {
  
  uint8_t battery_soc;               // percent
  float battery_voltage;             // volts
  float battery_charging_amps;       // amps
  uint8_t battery_temperature;       // celcius
  uint8_t controller_temperature;    // celcius
  float load_voltage;                // volts
  float load_amps;                   // amps
  uint8_t load_watts;                // watts
  float solar_panel_voltage;         // volts
  float solar_panel_amps;            // amps
  uint8_t solar_panel_watts;         // watts
  float min_battery_voltage_today;   // volts
  float max_battery_voltage_today;   // volts
  float max_charging_amps_today;     // amps
  float max_discharging_amps_today;  // amps
  uint8_t max_charge_watts_today;    // watts
  uint8_t max_discharge_watts_today; // watts
  uint8_t charge_amphours_today;     // amp hours
  uint8_t discharge_amphours_today;  // amp hours
  uint8_t charge_watthours_today;    // watt hours
  uint8_t discharge_watthours_today; // watt hours
  uint8_t controller_uptime_days;    // days
  uint8_t total_battery_overcharges; // count
  uint8_t total_battery_fullcharges; // count

  // convenience values
  float battery_temperatureF;        // fahrenheit
  float controller_temperatureF;     // fahrenheit
  float battery_charging_watts;      // watts. necessary? Does it ever differ from solar_panel_watts?
  long last_update_time;             // millis() of last update time
  bool controller_connected;         // bool if we successfully read data from the controller
};
Controller_data renogy_data;


// A struct to hold the controller info params
struct Controller_info {
  
  uint8_t voltage_rating;            // volts
  uint8_t amp_rating;                // amps
  uint8_t discharge_amp_rating;      // amps
  uint8_t type;
  uint8_t controller_name;
  char software_version[40];
  char hardware_version[40];
  char serial_number[40];
  uint8_t modbus_address;  

  float wattage_rating;
  long last_update_time;           // millis() of last update time
};
Controller_info renogy_info;

/*Put your SSID & Password for your Wifi router*/
const char* ssid = "************";   // Enter SSID here
const char* password = "**********";  // Enter Password here

/*
I prefer a static IP so my router is configured to issue DHCP IP addresses after
192.168.1.126, this prevents confusion of the ESP32 changing it's IP address randomly
*/
IPAddress staticIP(192, 168, 1, 125);  // Personally best to use a static IP but will have to set router dns going to start from 192.168.1.126
IPAddress gateway(192, 168, 1, 2);   // Replace this with your gateway IP Addess, mine is odd, yours may be 192.168.1.1
IPAddress subnet(255, 255, 255, 0);  // Replace this with your Subnet Mask
IPAddress dns(192, 168, 1, 2);   // Replace this with your DNS, or better still, use google dns 8.8.8.8 or 8.8.4.4

/*
Normally webserver port will be 80, but I already use this for a home webserver.
I am also using port 8080 for another home server so for the ESP I am using
another alternative of port 591, if you not hosting stuff at home, then USE PORT 80 TO KEEP IT SIMPLE.
to keep it simple
*/
WebServer server(591);


// Below for RS485 Only, for newer Renogy controllers
void preTransmission()
{
  digitalWrite(MAX485_RE_NEG, 1);
  digitalWrite(MAX485_DE, 1);
}

void postTransmission()
{
  digitalWrite(MAX485_RE_NEG, 0);
  digitalWrite(MAX485_DE, 0);
}

void setup() {

 // Sometimes the ANOYING on-board PCB RGB LED is lit bright for no reason at all
 // SO force OFF the anoying RGB LED to stop the thing lighting up!

neopixelWrite(RGB_BUILTIN, 0, 0, 0);

// Below for RS485
  pinMode(MAX485_RE_NEG, OUTPUT);
  pinMode(MAX485_DE, OUTPUT);
  // Init in receive mode
  digitalWrite(MAX485_RE_NEG, 0);
  digitalWrite(MAX485_DE, 0);


  pinMode(fenceLightsPin, OUTPUT);


  // create serial interface for serial monitor (debugging).
  Serial.begin(9600);

  // create a second serial interface for modbus 232 or 485 coms with Renogy
  Serial2.begin(9600, SERIAL_8N1, RXD2, TXD2);  // was 8n1

  // ORIGINAL NOTES. my Renogy Wanderer has an (slave) address of 255! Not in docs??? 
  // Do all Renogy charge controllers use this address?
  int modbus_address = 255; 
  node.begin(modbus_address, Serial2); 

  Serial.println("Connecting to ");
  Serial.println(ssid);

  //connect to your local wi-fi network
  WiFi.config(staticIP, gateway, subnet);
  WiFi.begin(ssid, password);

  //check wi-fi is connected to wi-fi network
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.print(".");
  }
  Serial.println("");
  Serial.println("WiFi connected..!");
  Serial.print("Got IP: ");
  Serial.println(WiFi.localIP());

  server.on("/", handle_OnConnect);
  server.onNotFound(handle_NotFound);

  server.begin();
  Serial.println("HTTP server started");

  ArduinoOTA
    .onStart([]() {
      String type;
      if (ArduinoOTA.getCommand() == U_FLASH) {
        type = "sketch";
      } else {  // U_SPIFFS
        type = "filesystem";
      }

      // NOTE: if updating SPIFFS this would be the place to unmount SPIFFS using SPIFFS.end()
      Serial.println("Start updating " + type);
    })
    .onEnd([]() {
      Serial.println("\nEnd");
    })
    .onProgress([](unsigned int progress, unsigned int total) {
      Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
    })
    .onError([](ota_error_t error) {
      Serial.printf("Error[%u]: ", error);
      if (error == OTA_AUTH_ERROR) {
        Serial.println("Auth Failed");
      } else if (error == OTA_BEGIN_ERROR) {
        Serial.println("Begin Failed");
      } else if (error == OTA_CONNECT_ERROR) {
        Serial.println("Connect Failed");
      } else if (error == OTA_RECEIVE_ERROR) {
        Serial.println("Receive Failed");
      } else if (error == OTA_END_ERROR) {
        Serial.println("End Failed");
      }
    });

  ArduinoOTA.begin();

  Serial.println("Ready");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());


    delay(1000);


// For RS485 Only
  node.preTransmission(preTransmission);
  node.postTransmission(postTransmission);
}


void loop() {


  ArduinoOTA.handle(); // Added this so I can update the ESP32 via wifi, saves me going down the garden into the shed with a laptop and USB cable.

  renogycomplete(); // Original code by Wrybread, credit to him for producing and sharing, I corrected a few things though such as decimal points etc.

  calcColours(); // Visual colours added to represent Good/OK/Bad, values are a guess and may need improving.

  otherControls(); // This is an optional extra for controlling lights in my garden, you can remove this and the code at the end of this sketch.

  server.handleClient(); // To make the website work.

}

void handle_OnConnect() {

  server.send(200, "text/html", createHTML());
}

void handle_NotFound() {
  server.send(404, "text/plain", "Not found");
}

String createHTML() {
  String str = "<!DOCTYPE html> <html>";
  str += "<head><meta http-equiv=refresh content=60><meta name=\"viewport\" content=\"width=device-width, initial-scale=1.0, user-scalable=no\">";
  str += "<style>";
  str += "body {font-family: Arial, sans-serif; font-size: 14px; color: #444; text-align: center;}";
  str += "</style>";
  str += "</head>";
  str += "<body>";
  str += "<body style=background-color:#1f567c;>";
  str += "<h3><font color=white><b><u>GARDEN SOLAR SYSTEM STATUS</u></b></h3>"; //Main Heading on webpage, edit this to change title.
//  Below adds a weather widget for where I live.
  str += "<div id=\"2018fa56235db034114db995c92b926c\" class=\"ww-informers-box-854753\"><p class=\"ww-informers-box-854754\"><a href=\"https://world-weather.info/forecast/united_kingdom/york/14days/\">Weather tomorrow</a><br><a href=\"https://world-weather.info/widget/\">world-weather.info/widget/</a></p></div><script async type=\"text/javascript\" charset=\"utf-8\" src=\"https://world-weather.info/wwinformer.php?userid=2018fa56235db034114db995c92b926c\"></script><style>.ww-informers-box-854754{-webkit-animation-name:ww-informers54;animation-name:ww-informers54;-webkit-animation-duration:1.5s;animation-duration:1.5s;white-space:nowrap;overflow:hidden;-o-text-overflow:ellipsis;text-overflow:ellipsis;font-size:12px;font-family:Arial;line-height:18px;text-align:center}@-webkit-keyframes ww-informers54{0%,80%{opacity:0}100%{opacity:1}}@keyframes ww-informers54{0%,80%{opacity:0}100%{opacity:1}}</style>";
  str += "<style>table, th, td {border: 1px solid white;}</style>";
  str += "<font color=black>";
  str += "<table width=310px, cellpadding=0, align=center>";
  str += "<h3><tr><td bgcolor=Lavender><b>WIFI Signal = " + String(WiFi.RSSI()) + "dBm</b></td></tr></h3>";
  str += "<style>meter {width: 300px;height: 12px;}</style>";
  str += "<table width=310px, cellpadding=0, align=center>";
  str += "<tr><td bgcolor=Lavender><meter min=-85 max=-30 value=" + String(WiFi.RSSI()) + "></meter></td></tr>";
  str += "<table width=95%, cellpadding=0, align=center>";
  str += "<h5><tr><td bgcolor=Lavender><b>Battery Voltage Now</td><td bgcolor=" + String(batVoltCol) + "><b>" + String(renogy_data.battery_voltage) + "V</td></tr></h5>";
  str += "<h5><tr><td bgcolor=Lavender><b>Battery Level Now</td><td bgcolor=" + String(batVoltCol) + "><b>" + String(renogy_data.battery_soc) + "%</td></tr></h5>";
  str += "<h5><tr><td bgcolor=Lavender><b>Battery Charging Current Now</td><td bgcolor=" + String(batChargeCur) + "><b>" + String(renogy_data.battery_charging_amps) + "A</td></tr></h5>";
  str += "<h5><tr><td bgcolor=Lavender><b>Battery Temperature Now</td><td bgcolor=" + String(batTemperature) + "><b>" + String(renogy_data.battery_temperature) + "'C</td></tr></h5>";
  str += "<h5><tr><td bgcolor=Lavender><b>Renogy Controller Temperature Now</td><td bgcolor=" + String(conTemperature) + "><b>" + String(renogy_data.controller_temperature) + "'C</td></tr></h5>";
  str += "<h5><tr><td bgcolor=Lavender><b>ESP32 Microcontroller Temperature Now</td><td bgcolor=" + String(espTemp) + "><b>" + String(ESP_temp_celsius) + "'C</td></tr></h5>";
  str += "<h5><tr><td bgcolor=Lavender><b>Solar Panel Voltage Now</td><td bgcolor=" + String(solPanVol) + "><b>" + String(renogy_data.solar_panel_voltage) + "V</td></tr></h5>";
  str += "<h5><tr><td bgcolor=Lavender><b>Solar Panel Current Now</td><td bgcolor=" + String(solPanCur) + "><b>" + String(renogy_data.solar_panel_amps) + "A</td></tr></h5>";
  str += "<h5><tr><td bgcolor=Lavender><b>Solar Panel Wattage Now</td><td bgcolor=" + String(solPanWat) + "><b>" + String(renogy_data.solar_panel_watts) + "W</td></tr></h5>";
  str += "<h5><tr><td bgcolor=Lavender><b>System Load Voltage Now</td><td bgcolor=" + String(sysLoadVol) + "><b>" + String(renogy_data.load_voltage) + "V</td></tr></h5>";
  str += "<h5><tr><td bgcolor=Lavender><b>System load Current Now</td><td bgcolor=" + String(sysLoadCur) + "><b>" + String(renogy_data.load_amps) + "A</td></tr></h5>";
  str += "<h5><tr><td bgcolor=Lavender><b>Battery Max Voltage Today</td><td bgcolor=" + String(maxBatVolToday) + "><b>" + String(renogy_data.max_battery_voltage_today) + "V</td></tr></h5>";
  str += "<h5><tr><td bgcolor=Lavender><b>Battery Min Voltage Today</td><td bgcolor=" + String(minBatVolToday) + "><b>" + String(renogy_data.min_battery_voltage_today) + "V</td></tr></h5>";
  str += "<h5><tr><td bgcolor=Lavender><b>Battery Max Charging Today</td><td bgcolor=" + String(maxBatChargeToday) + "><b>" + String(renogy_data.max_charging_amps_today) + "A</td></tr></h5>";
  str += "<h5><tr><td bgcolor=Lavender><b>Battery Max Discharging Today</td><td bgcolor=" + String(maxBatDischargeToday) + "><b>" + String(renogy_data.max_discharging_amps_today) + "A</td></tr></h5>";
  str += "<h5><tr><td bgcolor=Lavender><b>Battery Charge Amps/Hour Today</td><td bgcolor=Lavender><b>" + String(renogy_data.charge_amphours_today) + "A/h</td></tr></h5>";
  str += "<h5><tr><td bgcolor=Lavender><b>Battery Discharge Amps/Hour Today</td><td bgcolor=Lavender><b>" + String(renogy_data.discharge_amphours_today) + "A/h</td></tr></h5>";
  str += "</table>";
  str += "<h3><b><font color=white>Photocell Signal = " + String(photocellReading) + " %</b></h3>"; //temporary measure to get an idea on bits = whatever light level.
  str += "<h3><b><font color=white>Fence Lights Are " + String(fenceLightsCond) +"</b></h3>"; //temp measure to see if lights are ON.
  str += "<h3><b><font color=white>Brightness = " + String(brightness) +"</b></h3>"; //temp measure to see light level increasing.
  str += "</div>";
  str += "</body>";
  str += "</html>";
  return str;
}

void renogycomplete()
{
  static uint32_t i;
  i++;
  
  // set word 0 of TX buffer to least-significant word of counter (bits 15..0)
  node.setTransmitBuffer(0, lowWord(i));  
  // set word 1 of TX buffer to most-significant word of counter (bits 31..16)
  node.setTransmitBuffer(1, highWord(i));

  renogy_read_data_registers();
  renogy_read_info_registers();

  Serial.println("Battery voltage: " + String(renogy_data.battery_voltage));
  Serial.println("WORKING");
  Serial.println("    ");
  Serial.println("---");


  // turn the load on for 10 seconds
  //renogy_control_load(1)
  //delay(10000);
  //renogy_control_load(0)
  
    
  delay(2000); 

}

void renogy_read_data_registers() 
{
  uint8_t j, result;
  uint16_t data_registers[num_data_registers];
  char buffer1[40], buffer2[40];
  uint8_t raw_data;

  // prints data about each read to the console
  bool print_data=0; 
  
  result = node.readHoldingRegisters(0x100, num_data_registers);
  if (result == node.ku8MBSuccess)
  {
    if (print_data) Serial.println("Successfully read the data registers!");
    renogy_data.controller_connected = true;
    for (j = 0; j < num_data_registers; j++)
    {
      data_registers[j] = node.getResponseBuffer(j);
      if (print_data) Serial.println(data_registers[j]);
    }

    renogy_data.battery_soc = data_registers[0]; 
    renogy_data.battery_voltage = data_registers[1] * 0.1; // will it crash if data_registers[1] doesn't exist?
    renogy_data.battery_charging_amps = data_registers[2] * 0.01;

    renogy_data.battery_charging_watts = renogy_data.battery_voltage * renogy_data.battery_charging_amps;
    
    //0x103 returns two bytes, one for battery and one for controller temp in c
    uint16_t raw_data = data_registers[3]; // eg 5913
    renogy_data.controller_temperature = raw_data/256;
    renogy_data.battery_temperature = raw_data%256; 
    // for convenience, fahrenheit versions of the temperatures
    renogy_data.controller_temperatureF = (renogy_data.controller_temperature * 1.8)+32;
    renogy_data.battery_temperatureF = (renogy_data.battery_temperature * 1.8)+32;

    renogy_data.load_voltage = data_registers[4] * .1;
    renogy_data.load_amps = data_registers[5] * .01;
    renogy_data.load_watts = data_registers[6];
    renogy_data.solar_panel_voltage = data_registers[7] * .1;
    renogy_data.solar_panel_amps = data_registers[8] * .01;
    renogy_data.solar_panel_watts = data_registers[9];
     //Register 0x10A - Turn on load, write register, unsupported in wanderer - 10
    renogy_data.min_battery_voltage_today = data_registers[11] * .1;
    renogy_data.max_battery_voltage_today = data_registers[12] * .1; 
    renogy_data.max_charging_amps_today = data_registers[13] * .01;
    renogy_data.max_discharging_amps_today = data_registers[14] * 0.01;
    renogy_data.max_charge_watts_today = data_registers[15];
    renogy_data.max_discharge_watts_today = data_registers[16];
    renogy_data.charge_amphours_today = data_registers[17];
    renogy_data.discharge_amphours_today = data_registers[18];
    renogy_data.charge_watthours_today = data_registers[19];
    renogy_data.discharge_watthours_today = data_registers[20];
    renogy_data.controller_uptime_days = data_registers[21];
    renogy_data.total_battery_overcharges = data_registers[22];
    renogy_data.total_battery_fullcharges = data_registers[23];
    renogy_data.last_update_time = millis();

    // Add these registers:
    //Registers 0x118 to 0x119- Total Charging Amp-Hours - 24/25    
    //Registers 0x11A to 0x11B- Total Discharging Amp-Hours - 26/27    
    //Registers 0x11C to 0x11D- Total Cumulative power generation (kWH) - 28/29    
    //Registers 0x11E to 0x11F- Total Cumulative power consumption (kWH) - 30/31    
    //Register 0x120 - Load Status, Load Brightness, Charging State - 32    
    //Registers 0x121 to 0x122 - Controller fault codes - 33/34

    if (print_data) Serial.println("---");
  }
  else 
  {
    if (result == 0xE2) 
    {
    Serial.println("Timed out reading the data registers!");
    }
    else 
    {
      Serial.print("Failed to read the data registers... ");
      Serial.println(result, HEX); // E2 is timeout
    }
    // Reset some values if we don't get a reading
    renogy_data.controller_connected = false;
    renogy_data.battery_voltage = 0; 
    renogy_data.battery_charging_amps = 0;
    renogy_data.battery_soc = 0;
    renogy_data.battery_charging_amps = 0;
    renogy_data.controller_temperature = 0;
    renogy_data.battery_temperature = 0;    
    renogy_data.solar_panel_amps = 0;
    renogy_data.solar_panel_watts = 0;
    renogy_data.battery_charging_watts = 0;
    if (simulator_mode) {
      renogy_data.battery_voltage = 13.99;    
      renogy_data.battery_soc = 55; 
    }
  }


}

void renogy_read_info_registers() 
{
  uint8_t j, result;
  uint16_t info_registers[num_info_registers];
  char buffer1[40], buffer2[40];
  uint8_t raw_data;

  // prints data about the read to the console
  bool print_data=0;
  
  result = node.readHoldingRegisters(0x00A, num_info_registers);
  if (result == node.ku8MBSuccess)
  {
    if (print_data) Serial.println("Successfully read the info registers!");
    for (j = 0; j < num_info_registers; j++)
    {
      info_registers[j] = node.getResponseBuffer(j);
      if (print_data) Serial.println(info_registers[j]);
    }

    // read and process each value
    //Register 0x0A - Controller voltage and Current Rating - 0
    // Not sure if this is correct. I get the correct amp rating for my Wanderer 30 (30 amps), but I get a voltage rating of 0 (should be 12v)
    raw_data = info_registers[0]; 
    renogy_info.voltage_rating = raw_data/256; 
    renogy_info.amp_rating = raw_data%256;
    renogy_info.wattage_rating = renogy_info.voltage_rating * renogy_info.amp_rating;
    //Serial.println("raw ratings = " + String(raw_data));
    //Serial.println("Voltage rating: " + String(renogy_info.voltage_rating));
    //Serial.println("amp rating: " + String(renogy_info.amp_rating));


    //Register 0x0B - Controller discharge current and type - 1
    raw_data = info_registers[1]; 
    renogy_info.discharge_amp_rating = raw_data/256; // not sure if this should be /256 or /100
    renogy_info.type = raw_data%256; // not sure if this should be /256 or /100

    //Registers 0x0C to 0x13 - Product Model String - 2-9
    // Here's how the nodeJS project handled this:
    /*
    let modelString = '';
    for (let i = 0; i <= 7; i++) {  
        rawData[i+2].toString(16).match(/.{1,2}/g).forEach( x => {
            modelString += String.fromCharCode(parseInt(x, 16));
        });
    }
    this.controllerModel = modelString.replace(' ','');
    */

    //Registers 0x014 to 0x015 - Software Version - 10-11
    itoa(info_registers[10],buffer1,10); 
    itoa(info_registers[11],buffer2,10);
    strcat(buffer1, buffer2); // should put a divider between the two strings?
    strcpy(renogy_info.software_version, buffer1); 
    //Serial.println("Software version: " + String(renogy_info.software_version));

    //Registers 0x016 to 0x017 - Hardware Version - 12-13
    itoa(info_registers[12],buffer1,10); 
    itoa(info_registers[13],buffer2,10);
    strcat(buffer1, buffer2); // should put a divider between the two strings?
    strcpy(renogy_info.hardware_version, buffer1);
    //Serial.println("Hardware version: " + String(renogy_info.hardware_version));

    //Registers 0x018 to 0x019 - Product Serial Number - 14-15
    // I don't think this is correct... Doesn't match serial number printed on my controller
    itoa(info_registers[14],buffer1,10); 
    itoa(info_registers[15],buffer2,10);
    strcat(buffer1, buffer2); // should put a divider between the two strings?
    strcpy(renogy_info.serial_number, buffer1);
    //Serial.println("Serial number: " + String(renogy_info.serial_number)); // (I don't think this is correct)

    renogy_info.modbus_address = info_registers[16];
    renogy_info.last_update_time = millis();
  
    if (print_data) Serial.println("---");
  }
  else
  {
    if (result == 0xE2) 
    {
      Serial.println("Timed out reading the info registers!");
    }
    else 
    {
      Serial.print("Failed to read the info registers... ");
      Serial.println(result, HEX); // E2 is timeout
    }
    // anything else to do if we fail to read the info reisters?
  }
}


// control the load pins on Renogy charge controllers that have them
void renogy_control_load(bool state) {
  if (state==1) node.writeSingleRegister(0x010A, 1);  // turn on load
  else node.writeSingleRegister(0x010A, 0);  // turn off load
}

void calcColours()

//Read certain values and assign a colour for cell background on condition.
// Below are standard HTML colours
// Good = "GreenYellow", Average = "Yellow", Poor = "Salmon"
// I chose these colours as they don't 'over shadow' the text numbers on the website.
{
     // Battery voltage (V) and state of charge (%)

    if (renogy_data.battery_voltage > 12.7)
    {
      batVoltCol = "GreenYellow";
    }
    else if ((renogy_data.battery_voltage <=12.7) && (renogy_data.battery_voltage >= 12))
    {
      batVoltCol = "Yellow";
    }
    else
    {
      batVoltCol = "Salmon";
    }
 

     // Battery charge current (A)

    if (renogy_data.battery_charging_amps > 1)
    {
      batChargeCur = "GreenYellow";
    }
    else if ((renogy_data.battery_charging_amps <=1) && (renogy_data.battery_charging_amps >= 0.05))
    {
      batChargeCur = "Yellow";
    }
    else
    {
      batChargeCur = "Salmon";
    }


     // Battery temperature (C)

    if (renogy_data.battery_temperature > 10)
    {
      batTemperature = "GreenYellow";
    }
    else if ((renogy_data.battery_temperature <=10) && (renogy_data.battery_temperature >= 0))
    {
      batTemperature = "Yellow";
    }
    else
    {
      batTemperature = "Salmon";
    }

     // Controller temperature (C)

    if (renogy_data.controller_temperature > 10)
    {
      conTemperature = "GreenYellow";
    }
    else if ((renogy_data.controller_temperature <=10) && (renogy_data.controller_temperature >= 0))
    {
      conTemperature = "Yellow";
    }
    else
    {
      conTemperature = "Salmon";
    }


     // Solar panel voltage (V)

    if (renogy_data.solar_panel_voltage > 15)
    {
      solPanVol = "GreenYellow";
    }
    else if ((renogy_data.solar_panel_voltage <=15) && (renogy_data.solar_panel_voltage >= 13))
    {
      solPanVol = "Yellow";
    }
    else
    {
      solPanVol = "Salmon";
    }


     // Solar panel current (A) I BASED THIS ON MYSELF HAVING 2 PANELS, ADJUST VALUES TO YOUR SETUP

    if (renogy_data.solar_panel_amps > 5)
    {
      solPanCur = "GreenYellow";
    }
    else if ((renogy_data.solar_panel_amps <=5) && (renogy_data.solar_panel_amps >= 0.01))
    {
      solPanCur = "Yellow";
    }
    else
    {
      solPanCur = "Salmon";
    }


     // Solar panel wattage (W) I BASED THIS ON MYSELF HAVING 2 PANELS, ADJUST VALUES TO YOUR SETUP

    if (renogy_data.solar_panel_watts > 25)
    {
      solPanWat = "GreenYellow";
    }
    else if ((renogy_data.solar_panel_watts <=25) && (renogy_data.solar_panel_watts >= 1))
    {
      solPanWat = "Yellow";
    }
    else
    {
      solPanWat = "Salmon";
    }


     // System load voltage (V)

    if (renogy_data.load_voltage > 12.5)
    {
      sysLoadVol = "GreenYellow";
    }
    else if ((renogy_data.load_voltage <=12.5) && (renogy_data.load_voltage >= 11.9))
    {
      sysLoadVol = "Yellow";
    }
    else
    {
      sysLoadVol = "Salmon";
    }


     // System load current (A)

    if (renogy_data.load_amps > 15)
    {
      sysLoadCur = "Salmon";
    }
    else if ((renogy_data.load_amps <=15) && (renogy_data.load_amps >= 8))
    {
      sysLoadCur = "Yellow";
    }
    else
    {
      sysLoadCur = "GreenYellow";
    }


     // Battery max voltage today (V)

    if (renogy_data.max_battery_voltage_today > 12.5)
    {
      maxBatVolToday = "GreenYellow";
    }
    else if ((renogy_data.max_battery_voltage_today <=12.5) && (renogy_data.max_battery_voltage_today >= 11.9))
    {
      maxBatVolToday = "Yellow";
    }
    else
    {
      maxBatVolToday = "Salmon";
    }


     // Battery min voltage today (V)

    if (renogy_data.min_battery_voltage_today > 12.1)
    {
      minBatVolToday = "GreenYellow";
    }
    else if ((renogy_data.min_battery_voltage_today <=12.1) && (renogy_data.min_battery_voltage_today >= 11))
    {
      minBatVolToday = "Yellow";
    }
    else
    {
      minBatVolToday = "Salmon";
    }


     // Battery max charge today (A)

    if (renogy_data.max_charging_amps_today > 15)
    {
      maxBatChargeToday = "Salmon";
    }
    else if ((renogy_data.max_charging_amps_today <=15) && (renogy_data.max_charging_amps_today >= 10))
    {
      maxBatChargeToday = "Yellow";
    }
    else
    {
      maxBatChargeToday = "GreenYellow";
    }


     // Battery max discharge today (A)

    if (renogy_data.max_discharging_amps_today > 14)
    {
      maxBatDischargeToday = "Salmon";
    }
    else if ((renogy_data.max_discharging_amps_today <=14) && (renogy_data.max_discharging_amps_today >= 10))
    {
      maxBatDischargeToday = "Yellow";
    }
    else
    {
      maxBatDischargeToday = "GreenYellow";
    }


     // ESP Internal temperature (C)

    if (ESP_temp_celsius > 60)
    {
      espTemp = "Salmon";
    }
    else if ((ESP_temp_celsius <=60) && (ESP_temp_celsius >= 0))
    {
      espTemp = "GreenYellow";
    }
    else if (ESP_temp_celsius < 0)
    {
      espTemp = "Salmon";
    }


}

void otherControls()

{

/*

Here we do other things such as measure daylight level
and switch on lights etc etc.

Also other random features I want to add will go here.

REMEMBER THIS IS AN ESP32, MAX ANALOGUE VOLTAGE = 3.3V!!!!
So supply photocell (or any other sensor inputs) with on-board 3.3V regulator only!!!!

*/

// Control fence lights depending of reading of photocell

  photocellReading = analogRead(photocellInputPin) / 40.95;

// Below we count up time to ensure ambient light settlement

  if (photocellReading > dayLevel)

    {
      onTime = 0;
      offTime = offTime +1;
    }

    else if (photocellReading < nightLevel)

    {
      onTime = onTime +1;
      offTime = 0;
    }


// Below is to stop the ONTIME and OFFTIME figure becoming too big

    if (onTime >= correctDelayTime)

    {
    onTime = correctDelayTime;
    }

    else if (offTime >= correctDelayTime)

    {
    offTime = correctDelayTime;
    }

// Ramp up brightness over time, and not exceeding max level of 255.

    if (brightness >= 254)

    {
      brightness = 254;
    }

// Below, control the night lights depending on ambient light and time delay for ambient settlement

   if ((photocellReading > dayLevel) && (offTime >= correctDelayTime))
    {
      brightness = 0;
      analogWrite(fenceLightsPin, brightness); //Fence lights OFF
      fenceLightsCond = "OFF";  // For display on webpage
      analogWrite(garageLightsPin, brightness); //Garage lights OFF

    }
    else if ((photocellReading < nightLevel) && (onTime >= correctDelayTime))
    {
      brightness = brightness + 1; //increase brightness over program cycle
      analogWrite(fenceLightsPin, brightness); //Fence lights ON
      fenceLightsCond = "ON";  // For display on webpage
      analogWrite(garageLightsPin, brightness); //Garage lights ON
    }

    // Below, when ESP is flashed/updated, you have to wait for the light delay to expire
    // So below shows CALCULATING just so you know on the site that the delay is working
    // Also, the transision to day/night, night/day will show CALCULATING while day/night/day is being processed!
    // Light to Dark and Dark to Light isn't linear, hysteresis (fluctuation) and all that comes in, so time delays needed.

    else
    
    {
    fenceLightsCond = "CALCULATING...";
    }

// The ESP32 has an internal temperature sensor, we may as well use it and display temperature on the webpage
  ESP_temp_celsius = temperatureRead();

}




















