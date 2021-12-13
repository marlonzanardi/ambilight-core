/****************************************Informação do Arquivo*************************************************
** Nome do Arquivo:          projeto_smartleds.ino versão: 1.
** Data Ultima Modificação:  05-12-21
** Ultima Versão:            Sim
** Descrição:                Software controlador dos leds SmartLeds.
**------------------------------------------------------------------------------------------------------
** Criado por:          Marlon Zanardi <marlon.zanardi95@hotmail.com>
** Data de Criação:     22-12-18       
********************************************************************************************************/

#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>
#include <BLE2902.h>
#include <FastLED.h>

/* Definição de versão do software e hardware. */
#define V_SFT "1.07"
#define V_HDW 2

void bluetooth_connected_leds();
void inicia_leds();


/*LEDS*/
// --- General Settings
const uint16_t 
  Num_Leds   =  126;         // strip length
const uint8_t
  Brightness =  160;        // maximum brightness
// --- FastLED Setings
#define LED_TYPE     WS2812B  // led strip type for FastLED
#define COLOR_ORDER  GRB      // color order for bitbang
#define PIN_DATA     23        // led data output pin
// #define PIN_CLOCK  7       // led data clock pin (uncomment if you're using a 4-wire LED type)

// --- Serial Settings
const unsigned long
  SerialSpeed    = 115200;  // serial port speed
const uint16_t
  SerialTimeout  = 60;      // time before LEDs are shut off if no data (in seconds), 0 to disable

// --- Optional Settings (uncomment to add)
#define SERIAL_FLUSH          // Serial buffer cleared on LED latch

CRGB leds[Num_Leds];
uint8_t * ledsRaw = (uint8_t *)leds;

CRGBPalette16 currentPalette;
TBlendType    currentBlending;

extern CRGBPalette16 myRedWhiteBluePalette;
extern const TProgmemPalette16 myRedWhiteBluePalette_p PROGMEM;

#define UPDATES_PER_SECOND 100

// A 'magic word' (along with LED count & checksum) precedes each block
// of LED data; this assists the microcontroller in syncing up with the
// host-side software and properly issuing the latch (host I/O is
// likely buffered, making usleep() unreliable for latch). You may see
// an initial glitchy frame or two until the two come into alignment.
// The magic word can be whatever sequence you like, but each character
// should be unique, and frequent pixel values like 0 and 255 are
// avoided -- fewer false positives. The host software will need to
// generate a compatible header: immediately following the magic word
// are three bytes: a 16-bit count of the number of LEDs (high byte
// first) followed by a simple checksum value (high byte XOR low byte
// XOR 0x55). LED data follows, 3 bytes per LED, in order R, G, B,
// where 0 = off and 255 = max brightness.

const uint8_t magic[] = {
  'A','d','a'};
#define MAGICSIZE  sizeof(magic)

// Check values are header byte # - 1, as they are indexed from 0
#define HICHECK    (MAGICSIZE)
#define LOCHECK    (MAGICSIZE + 1)
#define CHECKSUM   (MAGICSIZE + 2)

enum processModes_t {Header, Data} mode = Header;

int16_t c;  // current byte, must support -1 if no data available
uint16_t outPos;  // current byte index in the LED array
uint32_t bytesRemaining;  // count of bytes yet received, set by checksum
unsigned long t, lastByteTime, lastAckTime;  // millisecond timestamps

void headerMode();
void dataMode();
void timeouts();

// Macros initialized
#ifdef SERIAL_FLUSH
  #undef SERIAL_FLUSH
  #define SERIAL_FLUSH while(Serial.available() > 0) { Serial.read(); }
#else
  #define SERIAL_FLUSH
#endif

#ifdef DEBUG_LED
  #define ON  1
  #define OFF 0

  #define D_LED(x) do {digitalWrite(DEBUG_LED, x);} while(0)
#else
  #define D_LED(x)
#endif

#ifdef DEBUG_FPS
  #define D_FPS do {digitalWrite(DEBUG_FPS, HIGH); digitalWrite(DEBUG_FPS, LOW);} while (0)
#else
  #define D_FPS
#endif

#define FASTLED_ESP32_I2S true
#define FASTLED_ALLOW_INTERRUPTS 0
#define FASTLED_INTERRUPT_RETRY_COUNT 1


/*********************************************************************************************************
** Nome da Função:       strsplit
** Descrição:            Faz a separação da string
** Parametro:            Não.
** Valor de retorno:     Não.
*********************************************************************************************************/
char ** strsplit( const char * src, const char * delim )
{
  char * pbuf = NULL;
  char * ptok = NULL;
  int count = 0;
  int srclen = 0;
  char ** pparr = NULL;
  srclen = strlen( src );
  pbuf = (char*) malloc( srclen + 1 );
  if( !pbuf )
    return NULL;
  strcpy( pbuf, src );
  ptok = strtok( pbuf, delim );
  while( ptok )
    {
      pparr = (char**) realloc( pparr, (count+1) * sizeof(char*) );
      *(pparr + count) = strdup(ptok);
      count++;
      ptok = strtok( NULL, delim );
    }
  pparr = (char**) realloc( pparr, (count+1) * sizeof(char*) );
  *(pparr + count) = NULL;
  free(pbuf);
  return pparr;
}

/*********************************************************************************************************
** Nome da Função:       strsplitfree
** Descrição:            Limpa o uso dos ponteiros
** Parametro:            Não.
** Valor de retorno:     Não.
*********************************************************************************************************/
void strsplitfree( char ** strlist )
{
  int i = 0;
  while( strlist[i])
    free( strlist[i++] );
    free( strlist );
}

String get_indice_str(String comando, int indice)
{
  char lista[comando.length()+5];
  //char * array_cmd = NULL; 
  //array_cmd = (char*) malloc( (comando.length() + 1 )* sizeof(char*));
  comando.toCharArray(lista,comando.length()+1);
  char ** pp = NULL;
  pp = strsplit( lista , " " );

  String str_pp = pp[indice];  
  strsplitfree( pp );
  //free(array_cmd);
  return str_pp;
}

/*BLUETOOTH (BLE)*/
#define SERVICE_UUID        "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define CHARACTERISTIC_UUID_TX "beb5483e-36e1-4688-b7f5-ea07361b26a8"
#define CHARACTERISTIC_UUID_RX "beb5483f-36e1-4688-b7f5-ea07361b26a8"
BLECharacteristic *pCharacteristic;
bool bluetooth_connected = false;
bool deviceConnected = false;
int ambilight_mode = 1;
int txValue= 0;
class MyServerCallbacks: public BLEServerCallbacks{
  void onConnect(BLEServer* pServer){
    Serial.println("BLE conectado.");
    deviceConnected = true;
  };

  void onDisconnect(BLEServer* pServer){
    deviceConnected = false;
  }
};
class MyCallbacks: public BLECharacteristicCallbacks{
  void onWrite(BLECharacteristic *pCharacteristic){
    std::string rxValue = pCharacteristic->getValue();
    if (rxValue.length() > 0){
      Serial.println("=======START-RECEIVE=======");
      Serial.print("Received Value: ");
      for (int i=0; i<rxValue.length(); i++){
        Serial.print(rxValue[i]);
      }
      if (rxValue.find("BLUETOOTH_CONNECTED") != -1){
        // Bluetooth conectado.
        bluetooth_connected_leds();
        inicia_leds();
        // Bluetooth conectado.
        bluetooth_connected = 1;
        if (deviceConnected){
          String status_report = "GET_STATUS 1 2 3 4";
          /*pCharacteristic->setValue(status_report);
          pCharacteristic->notify();*/
          Serial.println("BLUETOOTH CONNECTED. ENVIA STATUS REPORT: " + String(status_report));
        }
      }
      if (rxValue.find("SET_COLOR") != -1){
        char cmd[rxValue.length()];
        int i;
        for (i = 0; i < sizeof(rxValue); i++) {
            cmd[i] = rxValue[i];
        }
        String comando = cmd;
        ambilight_mode = 0;
        String r = get_indice_str(comando, 1);
        String g = get_indice_str(comando, 2);
        String b = get_indice_str(comando, 3);
        
        // Bluetooth conectado.
        //bluetooh_connected_leds();
        inicia_leds();
  
        for(int i = 0; i < Num_Leds; i++) 
        {       
          // let's set an led value
          leds[i] = CRGB(r.toInt(),g.toInt(),b.toInt());     
        }
        FastLED.show();
      }
      if (rxValue.find("1") != -1){
        Serial.println("LED ON");
        digitalWrite(LED_BUILTIN,HIGH);
      }
      else if (rxValue.find("0") != -1){
        Serial.println("LED OFF");
        digitalWrite(LED_BUILTIN,LOW);
      }

      Serial.println();
      Serial.println("=======END-RECIVE========");
    }  
  }
};

void inicia_leds()
{  
  for(int i = 0; i < Num_Leds; i++) 
  {       
    // let's set an led value
    leds[i] = CRGB::Black;   
  }
  FastLED.show();  
  // Inicializa o lcd.
  for(int i = 0; i < Num_Leds; i++) 
  {       
    // let's set an led value
    //leds[i] = CRGB::White;   
    //leds[i+1] = CRGB::White;   
    leds[i] = CHSV( random8(), 255, random8()); 
    leds[i+1] = CHSV( random8(), 255, random8());
    leds[i+2] = CHSV( random8(), 255, random8());
    leds[i+3] = CHSV( random8(), 255, random8());
    delay(2);
    if ( i > 3 )
      leds[i-1] = CRGB::Black;  
      leds[i-2] = CRGB::Black;  
      leds[i-3] = CRGB::Black; 
    FastLED.show();
  }
   // Inicializa o lcd.
  for(int i = 0; i < Num_Leds; i++) 
  {       
    // let's set an led value
    leds[i] = CRGB::Black;   
  }
  FastLED.show();  
  // Inicializa o lcd.
  for(int i = 0; i < Num_Leds; i++) 
  {       
    // let's set an led value
    //leds[i] = CRGB::White; 
    //leds[i+1] = CRGB::White;   
    leds[i] = CHSV( random8(), 255, random8()); 
    leds[i+1] = CHSV( random8(), 255, random8());
    leds[i+2] = CHSV( random8(), 255, random8());
    leds[i+3] = CHSV( random8(), 255, random8());
    delay(2);
    if ( i > 3 )
      leds[i-1] = CRGB::Black;  
      leds[i-2] = CRGB::Black;  
      leds[i-3] = CRGB::Black; 
    FastLED.show();
  }
  for(int i = 0; i < Num_Leds; i++) 
  {       
    // let's set an led value
    leds[i] = CRGB::Black;   
  }
  FastLED.show(); 
}

void bluetooth_connected_leds()
{
  for(int i = 0; i < Num_Leds; i++) 
  {       
    // let's set an led value
    leds[i] = CRGB::Black;   
  }
  FastLED.show();  
  delay(100);
  for(int i = 0; i < Num_Leds; i++) 
  {       
    // let's set an led value
    leds[i] = CHSV( random8(), 255, random8());   
  }
  FastLED.show();
  delay(100);
  for(int i = 0; i < Num_Leds; i++) 
  {       
    // let's set an led value
    leds[i] = CRGB::Black;   
  }
  FastLED.show();
  delay(100);
  for(int i = 0; i < Num_Leds; i++) 
  {       
    // let's set an led value
    leds[i] = CHSV( random8(), 255, random8());   
  }
  FastLED.show();
  delay(100);
  for(int i = 0; i < Num_Leds; i++) 
  {       
    // let's set an led value
    leds[i] = CRGB::Black;   
  }
  FastLED.show();
  delay(100);
  for(int i = 0; i < Num_Leds; i++) 
  {       
    // let's set an led value
    leds[i] = CHSV( random8(), 255, random8());   
  }
  FastLED.show();
  delay(100);
  for(int i = 0; i < Num_Leds; i++) 
  {       
    // let's set an led value
    leds[i] = CRGB::Black;   
  }
  FastLED.show();
}

void setup_ambilight()
{
  #ifdef DEBUG_LED
    pinMode(DEBUG_LED, OUTPUT);
    digitalWrite(DEBUG_LED, LOW);
  #endif

  #ifdef DEBUG_FPS
    pinMode(DEBUG_FPS, OUTPUT);
  #endif

  #if defined(PIN_CLOCK) && defined(PIN_DATA)
    FastLED.addLeds<LED_TYPE, PIN_DATA, PIN_CLOCK, COLOR_ORDER>(leds, Num_Leds);
  #elif defined(PIN_DATA)
    FastLED.addLeds<LED_TYPE, PIN_DATA, COLOR_ORDER>(leds, Num_Leds);
  #else
    #error "No LED output pins defined. Check your settings at the top."
  #endif
  
  FastLED.setBrightness(Brightness);

  #ifdef CLEAR_ON_START
    FastLED.show();
  #endif

  // RGB
  currentPalette = RainbowColors_p;
  currentBlending = LINEARBLEND;

  Serial.begin(SerialSpeed);
  Serial.print("Ada\n"); // Send ACK string to host

  lastByteTime = lastAckTime = millis(); // Set initial counters
}

void task_ambilight()
{
  if ( ambilight_mode == 1 )
  {
    t = millis(); // Save current time
  
    // If there is new serial data
    if((c = Serial.read()) >= 0){
      lastByteTime = lastAckTime = t; // Reset timeout counters
  
      switch(mode) {
        case Header:
          headerMode();
          break;
        case Data:
          dataMode();
          break;
      }
    }
    else {
      // No new data
      timeouts();
    }
  }else if( ambilight_mode == 2 )
  {
    //ChangePalettePeriodically();
    
    static uint8_t startIndex = 0;
    startIndex = startIndex + 1; /* motion speed */
    
    FillLEDsFromPaletteColors( startIndex);
    
    FastLED.show();
    FastLED.delay(1 / UPDATES_PER_SECOND);
  }else if ( ambilight_mode == 2 )
  {
    
  }
}

void setup() {
  setup_ambilight();
  configura_ble();  
  inicia_leds();  
  Serial.println("Sistema iniciado com sucesso.");
}

void loop() {
  task_ambilight();
  task_bluetooth();  
}

void configura_ble(){
  pinMode(LED_BUILTIN, OUTPUT);
  BLEDevice::init("SMARTLED");
  BLEServer *pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());  
  BLEService *pService = pServer->createService(SERVICE_UUID);  
  pCharacteristic = pService->createCharacteristic(
                                         CHARACTERISTIC_UUID_TX,
                                         BLECharacteristic::PROPERTY_NOTIFY
                                       );
  pCharacteristic->addDescriptor(new BLE2902());
  BLECharacteristic *pCharacteristic = pService->createCharacteristic(
                                         CHARACTERISTIC_UUID_RX,
                                         BLECharacteristic::PROPERTY_WRITE
                                       );
  pCharacteristic->setCallbacks(new MyCallbacks());  
  pService->start();
  pServer->getAdvertising()->start();
  Serial.println("BLE configurado. Aguardando a conexao do cliente...");
}
bool leds_on = false;
void task_bluetooth(){
  if (deviceConnected){
//    txValue = random(-10,20);
//    char txString[8];
//    dtostrf(txValue,1,2,txString);
//    pCharacteristic->setValue(txString);
//    pCharacteristic->notify();
//    Serial.println("Valor enviado: " + String(txString));
    //delay(500);
    if (leds_on == 0)
    {
      for(int i = 0; i < Num_Leds; i++) 
      {       
        // let's set an led value
        leds[i] = CRGB::Red;   
      }
      FastLED.show(); 
      leds_on = 1;
    }
  }
}

void headerMode(){
  static uint8_t
    headPos,
    hi, lo, chk;

  if(headPos < MAGICSIZE){
    // Check if magic word matches
    if(c == magic[headPos]) {headPos++;}
    else {headPos = 0;}
  }
  else{
    // Magic word matches! Now verify checksum
    switch(headPos){
      case HICHECK:
        hi = c;
        headPos++;
        break;
      case LOCHECK:
        lo = c;
        headPos++;
        break;
      case CHECKSUM:
        chk = c;
        if(chk == (hi ^ lo ^ 0x55)) {
          // Checksum looks valid. Get 16-bit LED count, add 1
          // (# LEDs is always > 0) and multiply by 3 for R,G,B.
          D_LED(ON);
          bytesRemaining = 3L * (256L * (long)hi + (long)lo + 1L);
          outPos = 0;
          memset(leds, 0, Num_Leds * sizeof(struct CRGB));
          mode = Data; // Proceed to latch wait mode
        }
        headPos = 0; // Reset header position regardless of checksum result
        break;
    }
  }
}

void dataMode(){
  // If LED data is not full
  if (outPos < sizeof(leds)){
    ledsRaw[outPos++] = c; // Issue next byte
  }
  bytesRemaining--;
 
  if(bytesRemaining == 0) {
    // End of data -- issue latch:
    mode = Header; // Begin next header search
    FastLED.show();
    D_FPS;
    D_LED(OFF);
    SERIAL_FLUSH;
  }
}

void timeouts(){
  // No data received. If this persists, send an ACK packet
  // to host once every second to alert it to our presence.
  if((t - lastAckTime) >= 1000) {
    Serial.print("Ada");
    //Serial.print(leds_top);
    //Serial.print(" ");
    //Serial.print(leds_bot);
    //Serial.print(" ");
    //Serial.print(leds_left);
    //Serial.print(" ");
    //Serial.print(leds_right);
    Serial.print("\n");
    lastAckTime = t; // Reset counter

    // If no data received for an extended time, turn off all LEDs.
    if(SerialTimeout != 0 && (t - lastByteTime) >= (uint32_t) SerialTimeout * 1000) {
      memset(leds, 0, Num_Leds * sizeof(struct CRGB)); //filling Led array by zeroes
      FastLED.show();
      mode = Header;
      lastByteTime = t; // Reset counter
    }
  }
}

void FillLEDsFromPaletteColors( uint8_t colorIndex)
{
    uint8_t brightness = 255;
    
    for( int i = 0; i < Num_Leds; i++) {
        leds[i] = ColorFromPalette( currentPalette, colorIndex, brightness, currentBlending);
        colorIndex += 3;
    }
}


// There are several different palettes of colors demonstrated here.
//
// FastLED provides several 'preset' palettes: RainbowColors_p, RainbowStripeColors_p,
// OceanColors_p, CloudColors_p, LavaColors_p, ForestColors_p, and PartyColors_p.
//
// Additionally, you can manually define your own color palettes, or you can write
// code that creates color palettes on the fly.  All are shown here.

void ChangePalettePeriodically()
{
    uint8_t secondHand = (millis() / 1000) % 60;
    static uint8_t lastSecond = 99;
    
    if( lastSecond != secondHand) {
        lastSecond = secondHand;
        if( secondHand ==  0)  { currentPalette = RainbowColors_p;         currentBlending = LINEARBLEND; }
        if( secondHand == 10)  { currentPalette = RainbowStripeColors_p;   currentBlending = NOBLEND;  }
        if( secondHand == 15)  { currentPalette = RainbowStripeColors_p;   currentBlending = LINEARBLEND; }
        if( secondHand == 20)  { SetupPurpleAndGreenPalette();             currentBlending = LINEARBLEND; }
        if( secondHand == 25)  { SetupTotallyRandomPalette();              currentBlending = LINEARBLEND; }
        if( secondHand == 30)  { SetupBlackAndWhiteStripedPalette();       currentBlending = NOBLEND; }
        if( secondHand == 35)  { SetupBlackAndWhiteStripedPalette();       currentBlending = LINEARBLEND; }
        if( secondHand == 40)  { currentPalette = CloudColors_p;           currentBlending = LINEARBLEND; }
        if( secondHand == 45)  { currentPalette = PartyColors_p;           currentBlending = LINEARBLEND; }
        if( secondHand == 50)  { currentPalette = myRedWhiteBluePalette_p; currentBlending = NOBLEND;  }
        if( secondHand == 55)  { currentPalette = myRedWhiteBluePalette_p; currentBlending = LINEARBLEND; }
    }
}

// This function fills the palette with totally random colors.
void SetupTotallyRandomPalette()
{
    for( int i = 0; i < 16; i++) {
        currentPalette[i] = CHSV( random8(), 255, random8());
    }
}

// This function sets up a palette of black and white stripes,
// using code.  Since the palette is effectively an array of
// sixteen CRGB colors, the various fill_* functions can be used
// to set them up.
void SetupBlackAndWhiteStripedPalette()
{
    // 'black out' all 16 palette entries...
    fill_solid( currentPalette, 16, CRGB::Black);
    // and set every fourth one to white.
    currentPalette[0] = CRGB::White;
    currentPalette[4] = CRGB::White;
    currentPalette[8] = CRGB::White;
    currentPalette[12] = CRGB::White;
    
}

// This function sets up a palette of purple and green stripes.
void SetupPurpleAndGreenPalette()
{
    CRGB purple = CHSV( HUE_PURPLE, 255, 255);
    CRGB green  = CHSV( HUE_GREEN, 255, 255);
    CRGB black  = CRGB::Black;
    
    currentPalette = CRGBPalette16(
                                   green,  green,  black,  black,
                                   purple, purple, black,  black,
                                   green,  green,  black,  black,
                                   purple, purple, black,  black );
}


// This example shows how to set up a static color palette
// which is stored in PROGMEM (flash), which is almost always more
// plentiful than RAM.  A static PROGMEM palette like this
// takes up 64 bytes of flash.
const TProgmemPalette16 myRedWhiteBluePalette_p PROGMEM =
{
    CRGB::Red,
    CRGB::Gray, // 'white' is too bright compared to red and blue
    CRGB::Blue,
    CRGB::Black,
    
    CRGB::Red,
    CRGB::Gray,
    CRGB::Blue,
    CRGB::Black,
    
    CRGB::Red,
    CRGB::Red,
    CRGB::Gray,
    CRGB::Gray,
    CRGB::Blue,
    CRGB::Blue,
    CRGB::Black,
    CRGB::Black
};
        
