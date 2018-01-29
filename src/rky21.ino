#define TFT
#include "ArduCAM.h"
#include "IO.h"
#if defined TFT
  #include "tft.h"
#endif
#include "Adafruit_10DOF_IMU.h"
#include "HttpClient.h"

//SYSTEM_THREAD(ENABLED);
// -------------------
// Version Information
#define NAME "RKY-21"
#define AUTEUR "e-Coucou"
#define VERSION_MAJ 0
#define VERSION_MIN 07
#define RELEASE "jan. 2018"
#define CREATE "dec. 2017"

TCPClient client;

#define SERVER_ADDRESS "192.168.1.33"
#define SERVER_TCP_PORT 5550

#define TX_BUFFER_MAX 2048
uint8_t buffer[TX_BUFFER_MAX + 1];
int tx_buffer_index = 0;

struct stParam {
  uint8_t version, resolution;
  unsigned long timeout;
};
stParam Param = {1,0,60000};

#define WAIT 10
#define PI 3.1415926
#define toDEG(rad) ((rad)/PI*180.0)
#if defined TFT
//--------------------------------------------- TFT  -------
//--                                            -----
// init variable ST7735 : afficheur TFT 1.8" w/SD Card
#define dc      D2 
#define cs      D4 
#define rst     D5 
#define sclk    A3
#define sid     A5
ST7735 tft = ST7735(cs, dc, rst); //l'affichage en mode MISO
#endif
//--------------------------------------------- Capteurs  -------
//--                                            -------
Adafruit_BMP085_Unified       bmp   = Adafruit_BMP085_Unified(18001);
Adafruit_LSM303_Accel_Unified accel = Adafruit_LSM303_Accel_Unified(30301);
Adafruit_LSM303_Mag_Unified   mag   = Adafruit_LSM303_Mag_Unified(30302);
Adafruit_L3GD20_Unified       gyro  = Adafruit_L3GD20_Unified(20);
sensor_t sensor;
//--------------------------------------------- ArduCAM  -------
//--                                            -------
// set pin A2 as the slave select for the ArduCAM shield
const int SPI_CS = A2;

ArduCAM myCAM(OV5642, SPI_CS);
int WebCde(String Cde);
bool serial_on;

Power Power;

String Mois[12] = {"JAN", "FEV", "MAR", "AVR", "MAI", "JUN", "JUI", "AOU", "SEP", "OCT", "NOV", "DEC"};

HttpClient http;  
http_header_t headers[] = {  
    { "Content-Type", "application/json" },  
    { "Accept" , "application/json" },
    { NULL, NULL }   
 };  
http_request_t request;  
http_response_t response;  

void setup()
{
  Particle.publish("status", "by e-Coucou 2017");
  Time.zone(+1);

  uint8_t vid,pid;
  uint8_t temp;

  Wire.setSpeed(CLOCK_SPEED_100KHZ);
  Wire.begin();

  Serial.begin(115200);
  Serial.println("Rky - Video"+String::format(" [%s - %s]",NAME,RELEASE));
  Serial.println("ArduCAM Start!");
  Serial.println("Version "+String::format("%d.%d", VERSION_MAJ,VERSION_MIN)+" - Ready/success");
  Serial.println("by eCoucou 2017");
  // Start WebCommande
  bool success = Particle.function("Cde",WebCde);
  Serial.println(String::format("WebCommande : Cde start= %s",success ? "Ok" : "Nok"));
  // set the SPI_CS as an output:
  pinMode(SPI_CS, OUTPUT);
  // read EEPROM
  uint8_t vEE = EEPROM.read(0);
  if (vEE != 0xFF) {
    EEPROM.get(1,Param);
    EEPROM.write(0,Param.version);
    Serial.println(String::format("Lecture EEprom v%X",Param.version));
  } else {
    EEPROM.put(1,Param);
    EEPROM.write(0,Param.version);
    Serial.println("EEprom initialisation");
  }

  // initialize SPI:
  SPI.begin();

  Serial.println("Configuration Camera");

  while(1) {
    Serial.println("Verification de la camera...");

    //Check if the ArduCAM SPI bus is OK
    myCAM.write_reg(ARDUCHIP_TEST1, 0xAA);
    temp = myCAM.read_reg(ARDUCHIP_TEST1);
    if(temp != 0xAA)
    {
      Serial.println("SPI interface Error!");
      Serial.println("myCam.read_reg => " + String(temp));
      delay(5000);
    }
    else {
      break;
    }
  }
while(1){
  //Check if the camera module type is OV5642
  myCAM.rdSensorReg16_8(OV5642_CHIPID_HIGH, &vid);
  myCAM.rdSensorReg16_8(OV5642_CHIPID_LOW, &pid);
  if ((vid != 0x56) || (pid != 0x42)){
    Serial.println(F("module OV5642 introuvable!"));
    Particle.publish("status", "Not found, camera says " + String::format("%X:%X", vid, pid));
    delay(5000);
    continue;
  }
  else {
    Serial.println("module OV5642 OK! "+ String::format("[%X:%X]", vid, pid));
    Particle.publish("status", "OV5642 detected: " + String::format("%x:%x", vid, pid));
    break;
  }
}
Serial.println("Camera prête ...");
    noInterrupts();
    //myCAM.write_reg(ARDUCHIP_MODE, 0x01);		 	//Switch to CAM

    //Change MCU mode
//    myCAM.set_format(JPEG);
    myCAM.set_format(JPEG);
    delay(WAIT);

  Serial.println("Initialisation de la Camera ...");
    myCAM.InitCAM();
    delay(WAIT);

  Serial.println("Camera reglages supplementaires ...");

    myCAM.set_bit(ARDUCHIP_TIM, VSYNC_LEVEL_MASK);
//    myCAM.write_reg(ARDUCHIP_TIM, VSYNC_LEVEL_MASK);
    delay(WAIT);

interrupts();
delay(1000);
  Serial.println("Camera OK.");
  Serial.println("Init TFT Screen");
#if defined TFT
  init_tft();
  copyright();

  delay(3000);
#endif
    Serial.println("Connexion au server serveur TCP");
    delay(1000);

    client.connect(SERVER_ADDRESS, SERVER_TCP_PORT);
    serial_on = Serial.isConnected();
  // les capteurs ...
    bmp.begin();
    gyro.begin();
    accel.begin();
    mag.begin();
	
    aff_Trame();
} // -end setup
//--------------------------------------------------------------------------------------------- LOOP -- //
volatile unsigned long now, start = 0, count =0, iteration=0;
volatile int update = 0;
bool tft_update=true;
char szMessage[30];
uint16_t menu = 0x0001, mode = 0x0001;
float cell,status;
int bytesRead = 0;
sensors_event_t event;
char isPicture = 0;
void loop()
{
  now = millis(); count++;
  if ((now-start)>Param.timeout) {
    #if defined TFT
      aff_Click();
    #endif
    serial_on = Serial.isConnected();
    isPicture = take_picture();
    start = millis();
    tft_update = true;
    update = 0;
    aff_Trame();
  } else if ( (now-start)/1000 > update ) {
      update = (now-start)/1000;
      #if defined TFT
      aff_Heure();
      tft.setTextColor(ST7735_YELLOW,ST7735_BLACK);
      tft.setCursor(145,24);
      tft.setTextSize(1);
      tft.println(String::format("%2d",Param.timeout/1000-update));
//    bmp.getSensor(&sensor);
  /* Display the pressure sensor results (barometric pressure is measure in hPa) */
      tft.setTextColor(ST7735_WHITE,ST7735_BLACK);
  bmp.getEvent(&event);
  if (event.pressure)
  {
    /* Display ambient temperature in C */
    float temperature;
    bmp.getTemperature(&temperature);
      tft.setCursor(1,5);tft.println(String::format("I %4.1f",temperature));
      tft.setCursor(12,14);tft.println("C");
      tft.setCursor(41,5);tft.println(String::format("I %5.0f",event.pressure));
      tft.setCursor(49,14);tft.println("hPa");
      tft.setCursor(81,5);tft.println(String::format("%5.0f",bmp.pressureToAltitude(SENSORS_PRESSURE_SEALEVELHPA,event.pressure,temperature)));
      tft.setCursor(90,14);tft.println("m");
  }
  gyro.getEvent(&event);
  sprintf(szMessage,"(%5.1f,%5.1f,%5.1f)",toDEG(event.gyro.x),toDEG(event.gyro.y),toDEG(event.gyro.z));  
  tft.setCursor(1,90);
  tft.println(szMessage);
  accel.getEvent(&event);
  sprintf(szMessage,"(%5.0f,%5.0f,%5.0f)",event.acceleration.x*100.0,event.acceleration.y*100.0,event.acceleration.z*100.0);  
  tft.setCursor(1,100);
  tft.println(szMessage);
  mag.getEvent(&event);
  sprintf(szMessage,"(%5.0f,%5.0f,%5.0f)",event.magnetic.x*100.0,event.magnetic.y*100.0,event.magnetic.z*100.0);  
  tft.setCursor(1,80);
  tft.println(szMessage);
#endif
  }
}
//-------------------------------------------------------------------------------------------- end LOOP //
char take_picture() {
  if (!client.connected()) {
    if (!client.connect(SERVER_ADDRESS, SERVER_TCP_PORT)) {
      delay(2000);
      return -1;
    }
  }
  if (serial_on) Serial.println("Taking a picture...");
  noInterrupts();
  delay(WAIT);
  switch (Param.resolution) {
    case 0 : 
      myCAM.OV5642_set_JPEG_size(OV5642_320x240);
      break;
    case 1 :
      myCAM.OV5642_set_JPEG_size(OV5642_640x480);
      break;
    case 2:
      myCAM.OV5642_set_JPEG_size(OV5642_1280x960);
      break;
    case 3:
      myCAM.OV5640_set_JPEG_size(OV5642_1600x1200); //not Work ! idem à 1280x960
      break;
    case 4:
      myCAM.OV5642_set_JPEG_size(OV5642_2592x1944);
      break;
  }
  delay(WAIT);
  myCAM.write_reg(ARDUCHIP_FRAMES,0x00); delay(WAIT);
  myCAM.flush_fifo(); delay(WAIT);
  myCAM.clear_fifo_flag(); delay(WAIT);
  myCAM.start_capture(); delay(WAIT);
  unsigned long start_time = millis(), yet = millis();
//
//  wait for the photo to be done
//
  while(!myCAM.get_bit(ARDUCHIP_TRIG , CAP_DONE_MASK)) {
    yet = millis();
    if ((yet-start_time) > 30000) {
      break;
    }
    Particle.process();
  }
  delay(WAIT);
  int length = myCAM.read_fifo_length();
  if (serial_on) Serial.println("Image size is " + String(length));
  uint8_t temp = 0xff, temp_last = 0;
  bytesRead = 0;
  delay(WAIT);
  if(myCAM.get_bit(ARDUCHIP_TRIG, CAP_DONE_MASK))
  {
    delay(WAIT);
    iteration++;
    if (serial_on) Serial.println(F("Capture Ok !"));
      SPI.beginTransaction(SPISettings(4*MHZ, MSBFIRST, SPI_MODE0));
      myCAM.CS_LOW(); //by ep
      delay(WAIT);
      myCAM.set_fifo_burst(); delay(WAIT);
        //#if !(defined (OV5642_MINI_5MP_PLUS) ||(defined (ARDUCAM_SHIELD_V2) && defined (OV5642_CAM)))
//        SPI.transfer(0xFF);
//delay(WAIT);
        //#endif
      tx_buffer_index = 0;
      temp = 0;
//      Particle.process();
      while( (temp != 0xD9) | (temp_last != 0xFF) )
      {
        temp_last = temp;
        temp = SPI.transfer(0x00);
        bytesRead++;
        buffer[tx_buffer_index++] = temp;
        if (tx_buffer_index >= TX_BUFFER_MAX) {
          client.write(buffer, tx_buffer_index);
          tx_buffer_index = 0;
          Particle.process();
        }
        if (bytesRead > 2048000) {
          // failsafe
          break;
        }
      }
      if (tx_buffer_index != 0) {
        client.write(buffer, tx_buffer_index);
      }
      //Clear the capture done flag
      myCAM.CS_HIGH(); //by ep
      SPI.endTransaction();
      myCAM.clear_fifo_flag();
      if (serial_on) Serial.println(String::format("Photo [%d] : %d octets, resolution = %X",iteration,bytesRead,Param.resolution));
  }
  interrupts();
  return 1;
} // - end Picture
//------------------------------------------------------------------ Web Commande ------
//--                                                                 ------------
int WebCde(String  Cde) {
    
    unsigned char commande=0;
    char szMess[30];

    int index = Cde.indexOf('=');
    char inputStr[64];
    Cde.toCharArray(inputStr,64);
    char *p = strtok(inputStr," ,.-=");
    p = strtok(NULL, " ,.-=");
    int arg[10];
    String arg_s[10];
    int i=0;
    serial_on = Serial.isConnected();
    while (p != NULL)
    {
        arg[i++] = atoi(p);// & 0xFF;
        p = strtok (NULL, " ,.-");
    }
    
    if (Cde.startsWith("coul"))  commande = 0xA0;
    if (Cde.startsWith("tempo"))  commande = 0xA1;
    if (Cde.startsWith("resol"))  commande = 0xB0;
    if (Cde.startsWith("repet"))  commande = 0xB1;
    switch (commande) {
        case 0xA0: // color,w,g,r,b
            commande  = arg[0] & 0xFF;
//            neo_color = (arg[0] << 24) + (arg[1] << 16) + (arg[2] << 8) + arg[3];
            sprintf(szMess,"Change la couleur wgrb : %X:%X:%X:%X",arg[0],arg[1],arg[2],arg[3]);
            break;
        case 0xA1: // tempo,s -> a convertir en ms
            commande  = arg[0] & 0xFF;
//            timeout = max( arg[0] * 1000, 30000); // pas moins de 30s
            sprintf(szMess,"Temporisation : %d en s",arg[0]);
            break;
        case 0xB0: // resolution,d -> 0 (320) à 4 (12)
            commande  = arg[0] & 0xFF;
            Param.resolution = min(max(arg[0],0),4); // résolution entre 0 et 4 de la plus basse à la plus haute
            sprintf(szMess,"Resolution de l'image du -0 au ++4 : %d",arg[0]);
            break;
        case 0xB1: // tempo,s -> s en seconde avec mini à 2s et maxi à ...
            commande  = arg[0] & 0xFF;
            Param.timeout = max( arg[0], 2)*1000; // pas moins de 2s
            sprintf(szMess,"Repetition photo toutes les %d secondes",arg[0]);
            break;
        default:
            break;
    }
    if (serial_on) {
      Serial.println("WebCde : reception WebCommande -> ");
      Serial.println(szMess);
    }
    Param.version++;
    EEPROM.put(1,Param);
    EEPROM.write(0,Param.version);
    Serial.println("EEprom update");
    return commande;
} // end WebCde

#if defined TFT
void init_tft() {
//    digitalWrite(POWER,HIGH);
//    delay(30);
    tft.initR(INITR_BLACKTAB);   // initialize a ST7735S chip, black tab
    tft.setRotation(0x03);
}
void copyright() {
    tft.fillScreen(ST7735_BLACK);// 19 19 70 - ST7735_BLACK);
    tft.setCursor(0, 0);
//    tft.setTextColor(tft.Color565(0xAC,0xEE,0xEE),tft.Color565(0x19,0x19,0x70));//,ST7735_BLACK);
    tft.setTextColor(tft.Color565(0xAC,0xEE,0xEE),ST7735_BLACK);
    tft.setTextWrap(true);
    tft.println("Welcome @HOME. \n(c) e-Coucou 2017\n\nDemarrage du SYSTEME\nby rky ...");
    // copyright
    char szMess[50];
    tft.setTextSize(1);
    tft.setTextColor(0xEEEE,ST7735_BLACK);
    tft.setCursor(0, 80);
    sprintf(szMess,"(c)Rky %d.%d %02d:%02d %s",VERSION_MAJ,VERSION_MIN,Time.hour(),Time.minute(),WiFi.ready() ? "Wifi" : " - - ");
    tft.print(szMess);
}
void aff_Heure() {
    char szMess[20];
    int heure = int(Time.hour());
    int minute = int(Time.minute());
    int seconde = int(Time.second());
    sprintf(szMess,"%2d:%s%d",heure,minute>9 ? "":"0",minute);
//    tft.fillRect(0,0,tft.width(),tft.height(),ST7735_BLACK);
//    tft.setTextColor(tft.Color565(0xAF,0xEE,0xEE));
//    tft.fillScreen(ST7735_BLACK);
    tft.setTextColor(ST7735_WHITE,ST7735_BLACK);
    tft.setCursor(8,43);
    tft.setTextSize(3);
    tft.println(szMess);
    tft.setCursor(105,43);
    tft.setTextSize(1);
    tft.println(String::format("%s%d",seconde>9 ? "":"0", seconde));
    tft_update = false;
}
void aff_Date() {
    char szMess[20];
    int jour = int(Time.day());
    int mois = int(Time.month());
    int annee = int(Time.year());
    sprintf(szMess,"%2d/%2d/%4d",jour,mois,annee);
//    tft.fillRect(0,0,tft.width(),tft.height(),ST7735_BLACK);
//    tft.setTextColor(tft.Color565(0xAF,0xEE,0xEE));
//    tft.fillScreen(ST7735_BLACK);
    tft.setTextColor(ST7735_WHITE,ST7735_BLACK);
    tft.setTextSize(1);
    tft.setCursor(137,37);tft.println(String::format("%2d",jour));
    tft.setCursor(133,47);tft.println(Mois[mois-1]);
    tft.setCursor(130,57);tft.println(String::format("%2d",annee));
//    tft.println(szMess);
//    tft_update = false;
}
void aff_Click() {
    char szMess[20];
    sprintf(szMess,"CLICK");
//    tft.fillRect(0,0,tft.width(),tft.height(),ST7735_BLACK);
//    tft.setTextColor(tft.Color565(0xAF,0xEE,0xEE));
    tft.fillScreen(ST7735_RED);
    tft.setTextColor(ST7735_WHITE);
    tft.setCursor(10,50);
    tft.setTextSize(4);
    tft.println(szMess);
    tft_update = false;
}
void aff_Trame() {
  tft.fillScreen(ST7735_BLACK);
  tft.drawFastHLine(0,33,160,ST7735_WHITE);
  tft.drawFastHLine(0,73,160,ST7735_WHITE);
  tft.drawFastVLine(40,0,33,ST7735_WHITE);
  tft.drawFastVLine(80,0,33,ST7735_WHITE);
  tft.drawFastVLine(120,0,120,ST7735_WHITE);
  tft.setTextColor(ST7735_BLUE,ST7735_BLACK);
  tft.setTextSize(1);
  tft.setCursor(1,121);
  tft.println(String::format("%s %d.%d (%s)",AUTEUR,VERSION_MAJ,VERSION_MIN,RELEASE));
  tft.setTextColor(ST7735_WHITE,ST7735_BLACK);
  tft.setCursor(125,1);
  tft.println(String::format("%5.1f",bytesRead/1024.0));
  tft.setCursor(135,24);
  tft.println(String::format("%c",isPicture ? 'V' : 'x'));
  aff_Date();
  getRequest();
  getBatterie();
}
#endif

void getBatterie() {
  cell = Power.getVCell();
  status = Power.getSoC();
  tft.setCursor(127,88);
  tft.setTextSize(1);
  tft.setTextColor(status>50.0 ? ST7735_GREEN : ST7735_RED,ST7735_BLACK);
  tft.println(String::format("%c %2.0f%%",0x0b+int(status/25.0+0.4),status));
  tft.setCursor(127,98);
  tft.setTextColor(ST7735_WHITE,ST7735_BLACK);
  tft.println(String::format("%3.1f V",cell));

}
void getRequest() {  
   
  // serveur Accuweather
  request.hostname = "dataservice.accuweather.com"; //IPAddress(192,168,1,169); 
  request.hostname = "apidev.accuweather.com"; //from internet
  request.port = 80;
  request.path = "/currentconditions/v1/623.json?language=en&details=false&apikey=hoArfRosT1215"; //OU3bvqL9RzlrtSqXAJwg93E1Tlo3grVS";  
  request.path = "/currentconditions/v1/623.json?language=fr-fr&details=true&apikey=hoArfRosT1215"; //from inetnet
  request.body = "";
   
  http.get(request, response, headers);
  tft.setCursor(129,78);
  tft.setTextColor(ST7735_WHITE,ST7735_BLACK);
  tft.setTextSize(1);
  tft.println(response.status);
//  Serial.println(request.url);
  Serial.println(response.status);
  Serial.println(response.body);  
  String key1 = "WeatherText";
  String Meteo = KeyJson(key1 , response.body);
  String Jour = KeyJson("IsDayTime" , response.body);
  String jsonTemp = KeyJson("Temperature" , response.body);
  String Temperature = KeyJson("Value",jsonTemp);
  String Humidite = KeyJson("RelativeHumidity",response.body);
  jsonTemp = KeyJson("Wind" , response.body);
  Serial.println(jsonTemp);
  String Vent = KeyJson("Degrees", jsonTemp);
  tft.setCursor(1,112);
  tft.setTextColor(ST7735_YELLOW,ST7735_BLACK);
  tft.setTextSize(1);
  tft.println(Meteo+" "+Jour +" "+Humidite);
  tft.setCursor(5,22);tft.println(Temperature);
 }  

String KeyJson(const String& k, const String& j){
  int keyStartsAt = j.indexOf(k);
  Serial.println( keyStartsAt );
  int keyEndsAt = keyStartsAt + k.length(); // inludes double quote
  Serial.println( keyEndsAt );
  int colonPosition = j.indexOf(":", keyEndsAt);
  int valueEndsAt = j.indexOf(",", colonPosition);
  String val = j.substring(colonPosition + 1, valueEndsAt);
  val.trim();
  Serial.println( val );
  return val;
}
