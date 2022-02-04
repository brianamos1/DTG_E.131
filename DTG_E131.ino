//DTG E131 
//running e131 through OctoWS2811 powered by fastled on a custom designed
//32 pinout board called DuoTrigesimalWS2811
//
//The following varibles should be edited to match the hardware: 
//  NUM_LEDS_PER_STRIP - this should be the maximum number of leds on the longest string, 1 universe is 170 leds (can be changed below), multiples of this number make life easier. 
//  NUM_STRIPS - if using less than 32, put this at the number you are using
//  DMX_UNIVERSE - starting universe
//  UNIVERSE_COUNT = number of universes, should be #strips * universes per strip
//  IPAddress - static address only, DHCP not enabled at present
//  pinList - only list the pins used, only allows NUM_STRIPS worth of pins to be used.  Should be listed in sequential universe count order. 

#include <OctoWS2811.h>
#include <FastLED.h>
#include <NativeEthernet.h>
#include <Arduino.h>

//debug mode adds some serial feedback during testing. Remove this line for production code
//#define DEBUG 1

 
//Compile Time DEFINES 
#define ETHERNET_BUFFER 636 //540 is artnet leave at 636 for e1.31
#define NUM_LEDS_PER_STRIP 680 //170 per universe //680 default
#define NUM_STRIPS 32 //make sure pin list is accurate
#define DMX_SUBNET 0

int unsigned DMX_UNIVERSE = 1; //**Start** universe 1, 9, 17, 25, 33, 41
int unsigned UNIVERSE_COUNT = 128; //How Many Universes 8, 8, 8, 4, 8, 8
int unsigned UNIVERSE_LAST = DMX_UNIVERSE + UNIVERSE_COUNT - 1; // List the last universe typically its sequencially from start but does not have to. 8, 16, 24, 28, 32, 40
int unsigned CHANNEL_COUNT = 510; //max channels per dmx packet
byte unsigned LEDS_PER_UNIVERSE = 170; // Max RGB pixels for e1.31 universe//170 default
int unsigned NUM_LEDS  = UNIVERSE_COUNT * LEDS_PER_UNIVERSE; //

//ethernet setup
unsigned char packetBuffer[ETHERNET_BUFFER];
int c = 0;
float fps = 0;
unsigned long currentMillis = 0;
unsigned long previousMillis = 0;

EthernetUDP Udp;
uint8_t mac[6]; //this is determined from the Teensy 4.1 board.  
IPAddress ip(192,168,1,10); //static ip address of board
#define UDP_PORT 5568 //E1.31 UDP Port Number


//set up OctoWS2811 and FastLED for Teensy 4.1
const int numPins = NUM_STRIPS;
//byte pinList[numPins] = {2,3,4,5,6,7,8,9,10,11,12,13,14,16,17,16,27,28,29,30,31,35,36,37,38,39,40,41,42,43,44,45}; //listed sequentially, can be changed to meet hardware needs
//byte pinList[numPins[ = {37,36,35,34,15,14,39,38,19,18,17,16,23,22,21,20,9,8,7,6,13,12,11,10,27,26,25,24,32,31,30,28} //pinout order for the DTG WS2811 with the adamtech 4xRJ45 Ports
byte pinList[numPins] = {34,35,36,37,38,39,14,15,16,17,18,19,20,21,22,23,6,7,8,9,10,11,12,13,24,25,26,27,28,30,31,32}; //pinout order for DTG WS2811

const int ledsPerStrip = NUM_LEDS_PER_STRIP; //this should be NUM_LEDS / numPins => VERIFY THIS MATCHES THE HARDWARE!! //should also be multiples of a single universe of leds
CRGB rgbarray[numPins * ledsPerStrip];

  // These buffers need to be large enough for all the pixels.
  // The total number of pixels is "ledsPerStrip * numPins".
  // Each pixel needs 3 bytes, so multiply by 3.  An "int" is
  // 4 bytes, so divide by 4.  The array is created using "int"
  // so the compiler will align it to 32 bit memory.
DMAMEM int displayMemory[ledsPerStrip * numPins * 3 / 4];
int drawingMemory[ledsPerStrip * numPins * 3 / 4];
OctoWS2811 octo(ledsPerStrip, displayMemory, drawingMemory, WS2811_GRB | WS2811_800kHz, numPins, pinList);

template <EOrder RGB_ORDER = RGB,
          uint8_t CHIP = WS2811_800kHz>
class CTeensy4Controller : public CPixelLEDController<RGB_ORDER, 8, 0xFF>
{
    OctoWS2811 *pocto;

public:
    CTeensy4Controller(OctoWS2811 *_pocto)
        : pocto(_pocto){};

    virtual void init() {}
    virtual void showPixels(PixelController<RGB_ORDER, 8, 0xFF> &pixels)
    {

        uint32_t i = 0;
        while (pixels.has(1))
        {
            uint8_t r = pixels.loadAndScale0();
            uint8_t g = pixels.loadAndScale1();
            uint8_t b = pixels.loadAndScale2();
            pocto->setPixel(i++, r, g, b);
            pixels.stepDithering();
            pixels.advanceData();
        }

        pocto->show();
    }
};

CTeensy4Controller<RGB, WS2811_800kHz> *pcontroller;



void setup() {


  Serial.begin(115200);
  delay(10);


  teensyMAC(mac);

  
  static char teensyMac[23];
  sprintf(teensyMac, "MAC: %02x:%02x:%02x:%02x:%02x:%02x", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
  Serial.print("MAC: ");
  Serial.println(teensyMac);
  
  Ethernet.begin(mac, ip);
  

  Serial.print("IP Address: ");
  Serial.println(Ethernet.localIP());
  Serial.print("MAC: ");
  Serial.println(teensyMac);


  
  Udp.begin(UDP_PORT);
  
  octo.begin();
  pcontroller = new CTeensy4Controller<RGB, WS2811_800kHz>(&octo);

  FastLED.addLeds(pcontroller, rgbarray, numPins * ledsPerStrip);
  FastLED.setBrightness(100);
  
  initTest();
  Serial.println("Test Sequence Complete");
} //end setup

static inline void fps2(const int seconds){
  // Create static variables so that the code and variables can
  // all be declared inside a function
  static unsigned long lastMillis;
  static unsigned long frameCount;
  static unsigned int framesPerSecond;
  
  // It is best if we declare millis() only once
  unsigned long now = millis();
  frameCount ++;
  if (now - lastMillis >= seconds * 1000) {
    framesPerSecond = frameCount / seconds;
    
    Serial.print("FPS @ ");
    Serial.println(framesPerSecond);
    frameCount = 0;
    lastMillis = now;
  }

}

static inline void pixelrefresh(const int syncrefresh){
  // Create static variables so that the code and variables can
  // all be declared inside a function 
  static unsigned long frametimestart;
  static unsigned long frametimeend;
  static unsigned long frametimechk;
  static unsigned long frameonce;
  unsigned long now = micros();
 

  //start frame time
  frametimestart = now;
  
  //Serial.println(frametimechk);
   //If we have framed no need to frame again update time to most recent
   if  (syncrefresh == 1){
   frametimeend = frametimestart; 
   frameonce = 1;
   }
   
//If we havent framed this will increment via time and at some point will be true, 
//if so we need to frame to clear out any buffer and the hold off untill 
//we receive our next valid dmx packet. We use the pixel protocol to get a general rule of timing to compare to.

frametimechk = frametimestart - frametimeend;
 // num leds time 30us + 300us reset to simulate the time it would take to write out pixels. 
 //this should help us not loop to fast and risk premature framing and jeopordize ethernet buffer
 if  (frametimechk >= (NUM_LEDS * 30) + 300){
  frametimeend = frametimestart;


    if (frameonce == 1){

    octo.show();

    Serial.println ("Partial framing detected");
    frameonce = 0;  
  }
  
 }
 
}



void sacnDMXReceived(unsigned char* pbuff, int count) {
#ifdef DEBUG
  Serial.print("sacn DMX Received: Subnet=");
  Serial.print(pbuff[113]);
  Serial.print(", Universe=");
  Serial.print(pbuff[114]);
  Serial.print(", Sequence=");
  Serial.println(pbuff[111]);
#endif
  
  static unsigned long uniloopcount;
  if (count > CHANNEL_COUNT) count = CHANNEL_COUNT;
  byte b = pbuff[113]; //DMX Subnet
  if ( b == DMX_SUBNET) {
    b = pbuff[114];  //DMX Universe
    byte s = pbuff[111]; //sequence
    static unsigned long ls; // Last Sequence
    if (s > ls){
    uniloopcount = 0; 
    ls = s;
    }
   //turn framing LED OFF
//   digitalWrite(4, HIGH);
//    Serial.print("UNI ");
//    Serial.println(count );
//    Serial.println(s);
    if ( b >= DMX_UNIVERSE && b <= UNIVERSE_LAST) {
        //Serial.println(b );
      if ( pbuff[125] == 0 ) {  //start code must be 0   
       int ledNumber = (b - DMX_UNIVERSE) * LEDS_PER_UNIVERSE;
         // sACN packets come in seperate RGB but we have to set each led's RGB value together
         // this 'reads ahead' for all 3 colours before moving to the next led.
         //Serial.println("*");
        for (int i = 126;i < 126+count;i = i + 3){
          byte charValueR = pbuff[i];
          byte charValueG = pbuff[i+1];
          byte charValueB = pbuff[i+2];
          octo.setPixel(ledNumber, charValueR,charValueG,charValueB); //RBG GRB
          //Serial.println(ledNumber);
          ledNumber++;
        }
      }
    }
  }

         uniloopcount ++;
         Serial.print("UNILOOP");
         Serial.println(uniloopcount);

        //if (b == UNIVERSE_LAST){
        if (uniloopcount >= UNIVERSE_COUNT){ 
        //Turn Framing LED ON
//        digitalWrite(4, LOW);

        octo.show();
        
        pixelrefresh(1);
        uniloopcount = 0;
        //Frames Per Second Function fps(every_seconds)
        fps2(5);
        }

}


int checkACNHeaders(unsigned char* messagein, int messagelength) {
  //Do some VERY basic checks to see if it's an E1.31 packet.
  //Bytes 4 to 12 of an E1.31 Packet contain "ACN-E1.17"
  //Only checking for the A and the 7 in the right places as well as 0x10 as the header.
  //Technically this is outside of spec and could cause problems but its enough checks for us
  //to determine if the packet should be tossed or used.
  //This improves the speed of packet processing as well as reducing the memory overhead.
  //On an Isolated network this should never be a problem....
  if ( messagein[1] == 0x10 && messagein[4] == 0x41 && messagein[12] == 0x37) {   
      int addresscount = (byte) messagein[123] * 256 + (byte) messagein[124]; // number of values plus start code
      return addresscount -1; //Return how many values are in the packet.
    }
  return 0;
}


void initTest() //runs at board boot to make sure pixels are working
{
  LEDS.clear(); //clear led assignments
//  LEDS.showColor(CRGB(0,0,0)); //turn off all pixels to start
//  delay(500);
  
  LEDS.showColor(CRGB(255, 0, 0)); //turn all pixels on red
  delay(5000);

  LEDS.showColor(CRGB(0, 255, 0)); //turn all pixels on green
  delay(5000);

  LEDS.showColor(CRGB(0, 0, 255)); //turn all pixels on blue
  delay(5000);

  LEDS.showColor(CRGB(0,0,0)); //turn off all pixels to start

//  LEDS.showColor(CRGB(50, 50, 50)); //turn all pixels off
/*
  //process dot trace
     for (int n=0; n < NUM_LEDS_PER_STRIP*NUM_STRIPS; n++){
    octo.setPixel(n,100,0,0);
    octo.setPixel(n-2,0,0,0);
    octo.show();
    delay(10);
  
  }

    for (int n=0; n < NUM_LEDS_PER_STRIP*NUM_STRIPS; n++){
    octo.setPixel(n,0,100,0);
    octo.setPixel(n-2,0,0,0);
    octo.show();
    delay(10);
  
  }

    for (int n=0; n < NUM_LEDS_PER_STRIP*NUM_STRIPS; n++){
    octo.setPixel(n,0,0,100);
    octo.setPixel(n-2,0,0,0);
    octo.show();
    delay(10);
  
  }

*/

}

//define mac address of board
void teensyMAC(uint8_t *mac) {

  static char teensyMac[23];
  
  #if defined(HW_OCOTP_MAC1) && defined(HW_OCOTP_MAC0)
    Serial.println("using HW_OCOTP_MAC* - see https://forum.pjrc.com/threads/57595-Serial-amp-MAC-Address-Teensy-4-0");
    for(uint8_t by=0; by<2; by++) mac[by]=(HW_OCOTP_MAC1 >> ((1-by)*8)) & 0xFF;
    for(uint8_t by=0; by<4; by++) mac[by+2]=(HW_OCOTP_MAC0 >> ((3-by)*8)) & 0xFF;

    #define MAC_OK

  #else
    
    mac[0] = 0x04;
    mac[1] = 0xE9;
    mac[2] = 0xE5;

    uint32_t SN=0;
    __disable_irq();
    
    #if defined(HAS_KINETIS_FLASH_FTFA) || defined(HAS_KINETIS_FLASH_FTFL)
      Serial.println("using FTFL_FSTAT_FTFA - vis teensyID.h - see https://github.com/sstaub/TeensyID/blob/master/TeensyID.h");
      
      FTFL_FSTAT = FTFL_FSTAT_RDCOLERR | FTFL_FSTAT_ACCERR | FTFL_FSTAT_FPVIOL;
      FTFL_FCCOB0 = 0x41;
      FTFL_FCCOB1 = 15;
      FTFL_FSTAT = FTFL_FSTAT_CCIF;
      while (!(FTFL_FSTAT & FTFL_FSTAT_CCIF)) ; // wait
      SN = *(uint32_t *)&FTFL_FCCOB7;

      #define MAC_OK
      
    #elif defined(HAS_KINETIS_FLASH_FTFE)
      Serial.println("using FTFL_FSTAT_FTFE - vis teensyID.h - see https://github.com/sstaub/TeensyID/blob/master/TeensyID.h");
      
      kinetis_hsrun_disable();
      FTFL_FSTAT = FTFL_FSTAT_RDCOLERR | FTFL_FSTAT_ACCERR | FTFL_FSTAT_FPVIOL;
      *(uint32_t *)&FTFL_FCCOB3 = 0x41070000;
      FTFL_FSTAT = FTFL_FSTAT_CCIF;
      while (!(FTFL_FSTAT & FTFL_FSTAT_CCIF)) ; // wait
      SN = *(uint32_t *)&FTFL_FCCOBB;
      kinetis_hsrun_enable();

      #define MAC_OK
      
    #endif
    
    __enable_irq();

    for(uint8_t by=0; by<3; by++) mac[by+3]=(SN >> ((2-by)*8)) & 0xFF;

  #endif

  #ifdef MAC_OK
    sprintf(teensyMac, "MAC: %02x:%02x:%02x:%02x:%02x:%02x", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
    Serial.println(teensyMac);
  #else
    Serial.println("ERROR: could not get MAC");
  #endif
}

void loop() {
   //Process packets
   int packetSize = Udp.parsePacket(); //Read UDP packet count
    
   if(packetSize){
    Serial.println(packetSize);
    Udp.read(packetBuffer,ETHERNET_BUFFER); //read UDP packet
    
    
    int count = checkACNHeaders(packetBuffer, packetSize);
    if (count) {
      
//     Serial.print("packet size first ");
//     Serial.println(packetSize);

     
     

    
     sacnDMXReceived(packetBuffer, count); //process data function
     
    
     
    
    }  



  }

pixelrefresh(0);

}
