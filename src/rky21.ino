#define TFT
#include "ArduCAM.h"
#include "IO.h"
#if defined TFT
  #include "tft.h"
#endif
#include "Adafruit_10DOF_IMU.h"
//SYSTEM_THREAD(ENABLED);
// -------------------
// Version Information
#define NAME "RKY-21"
#define VERSION_MAJ 0
#define VERSION_MIN 06
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
sensor_t sensor;
//--------------------------------------------- ArduCAM  -------
//--                                            -------
// set pin A2 as the slave select for the ArduCAM shield
const int SPI_CS = A2;

ArduCAM myCAM(OV5642, SPI_CS);
int WebCde(String Cde);
bool serial_on;

Power Power;

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

void loop()
{
  now = millis(); count++;
  if ((now-start)>Param.timeout) {
#if defined TFT
    aff_Click();
#endif
    serial_on = Serial.isConnected();
    take_picture();
    start = millis();
    tft_update = true;
    update = 0;
  } else if ( (now-start)/1000 > update ) {
      update = (now-start)/1000;
#if defined TFT
      aff_Heure();
      sprintf(szMessage,"%2d",Param.timeout/1000-update);
      tft.setTextColor(ST7735_WHITE);
      tft.setCursor(140,110);
      tft.setTextSize(1);
      tft.println(szMessage);
      cell = Power.getVCell();
      status = Power.getSoC();
      sprintf(szMessage,"%4.1f - %4.2f",status,cell);
      tft.setCursor(1,110);
      tft.println(szMessage);
      sprintf(szMessage,"%c %c %c : %5.1f ko",0x0A,0x02,0x01,bytesRead/1024.0);
      tft.setTextSize(1);
      tft.setCursor(1,1);
      tft.println(szMessage);
//    bmp.getSensor(&sensor);
  /* Display the pressure sensor results (barometric pressure is measure in hPa) */
  bmp.getEvent(&event);
  if (event.pressure)
  {
    /* Display atmospheric pressure in hPa */
    Serial.print(F("PRESS "));
    Serial.print(event.pressure);
    Serial.print(F(" hPa, "));
    /* Display ambient temperature in C */
    float temperature;
    bmp.getTemperature(&temperature);
    Serial.print(temperature);
    Serial.print(F(" C, "));
    /* Then convert the atmospheric pressure, SLP and temp to altitude    */
    /* Update this next line with the current SLP for better results      */
    float seaLevelPressure = SENSORS_PRESSURE_SEALEVELHPA;
    Serial.print(bmp.pressureToAltitude(seaLevelPressure,
                                        event.pressure,
                                        temperature)); 
    Serial.println(F(" m"));

      sprintf(szMessage,"temperature : %5.1f °C",temperature);
      tft.setTextSize(1);
      tft.setCursor(1,15);
      tft.println(szMessage);
  }
#endif
  }
}
//-------------------------------------------------------------------------------------------- end LOOP //
void take_picture() {
  if (!client.connected()) {
    if (!client.connect(SERVER_ADDRESS, SERVER_TCP_PORT)) {
      delay(2000);
      return;
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
    sprintf(szMess,"%2d:%s%d",heure,minute>9 ? "":"0",minute);
//    tft.fillRect(0,0,tft.width(),tft.height(),ST7735_BLACK);
//    tft.setTextColor(tft.Color565(0xAF,0xEE,0xEE));
    tft.fillScreen(ST7735_BLACK);
    tft.setTextColor(ST7735_WHITE);
    tft.setCursor(20,50);
    tft.setTextSize(4);
    tft.println(szMess);
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
    tft.fillScreen(ST7735_BLACK);
    tft.setTextColor(ST7735_WHITE);
    tft.setCursor(10,50);
    tft.setTextSize(2);
    tft.println(szMess);
    tft_update = false;
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
#endif

