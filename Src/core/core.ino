/****************************************Informação do Arquivo*************************************************
** Nome do Arquivo:          projeto_smartleds.ino versão: 1.
** Data Ultima Modificação:  05-12-21
** Ultima Versão:            Sim
** Descrição:                Software controlador dos leds SmartLeds.
**------------------------------------------------------------------------------------------------------
** Criado por:          Marlon Zanardi <marlon.zanardi95@hotmail.com>
** Data de Criação:     22-12-18       
********************************************************************************************************/

/**********************************************************************************
** Observações:
** - Compilar utilizando versão com suporte a Serial HW com 9bits.
** - Alterar o tamanho do buffer RX de 64 para 128 bytes
**   <Arduino IDE>\hardware\arduino\avr\cores\arduino\HardwareSerial.h
**   "#define SERIAL_RX_BUFFER_SIZE 64" para "#define SERIAL_RX_BUFFER_SIZE 128"
***********************************************************************************/

/* Bibliotecas. */
#include <Ultrasonic.h>
#include <EEPROM.h>
#include <LiquidCrystal.h>
#include <FastLED.h>
#include "Pinagem.h"
#include "Buzzer.h"
#include "Display.h"

/* Definição de versão do software e hardware. */
#define V_SFT "1.06"
#define V_HDW 1

/* Debug Serial. */
//#define DEBUG

/* Constantes */
// Dimensao display.
#define LINHAS 20
#define COLUNAS 4

// Constantes funcao bt_press
#define AVANCAR 1
#define VOLTAR 2
#define SELECIONA 3

// Ativo inativo
#define ATIVO 1
#define INATIVO 0

//Define os pinos para o trigger e echo
#define pino_trigger A8
#define pino_echo A9

// Estados task operacao.
#define INICIALIZACAO 0
#define AGUARDA_INICIALIZACAO 1
#define MENU_PRINCIPAL 2
#define SELECAO_MENU_PRINCIPAL 3
#define FUNCAO_TELA1 4

// Menu principal.
#define QTD_MENU_PRINCIPAL 4

// Constantes operacoes.
#define TELA1 0
#define TELA2 1
#define TELA3 2
#define TELA4 3
#define TELA5 4

// Constantes booleanas
#define TRUE 1
#define FALSE 2

// --- General Settings
const uint16_t 
  Num_Leds   =  126;         // strip length
const uint8_t
  Brightness =  255;        // maximum brightness

// --- FastLED Setings
#define LED_TYPE     WS2812B  // led strip type for FastLED
#define COLOR_ORDER  GRB      // color order for bitbang
#define PIN_DATA     22        // led data output pin
// #define PIN_CLOCK  7       // led data clock pin (uncomment if you're using a 4-wire LED type)

// --- Serial Settings
const unsigned long
  SerialSpeed    = 115200;  // serial port speed
const uint16_t
  SerialTimeout  = 60;      // time before LEDs are shut off if no data (in seconds), 0 to disable

// --- Optional Settings (uncomment to add)
#define SERIAL_FLUSH          // Serial buffer cleared on LED latch
// #define CLEAR_ON_START     // LEDs are cleared on reset

// --- Debug Settings (uncomment to add)
// #define DEBUG_LED 13       // toggles the Arduino's built-in LED on header match
// #define DEBUG_FPS 8        // enables a pulse on LED latch

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

/* Variáveis */
byte leds_top = 0;
byte leds_bot = 0;
byte leds_left = 0;
byte leds_right = 0;
// Bluetooth serial protocol
bool bluetooth_connected = 0;
String comando;
bool stringComplete = false;
// Ambilight
int ambilight_mode = 1;
// Variavel que controla se existe algo barrando o sensor ultrasonico.
byte mao_70 = 0;
//char c;
// Bluetooth conectado.
int bluetooth_conectado = 0;
// Temporizador de Inicialização.
unsigned int tempo_atual_ultrassom = 0 , time_start_ultrassom = millis();
unsigned int tempo_atual_ultrassom_1 = 0 , time_start_ultrassom_1 = millis();
unsigned int tempo_atual_luz = 0 , time_start_luz = millis();
// Controle ultrassom.
int controle_ultrassom = 0;
// Estado linha.
int estado_luz = 0;
int estado_led_tv = 0;
int estado_lamp_trofeu = 0;
// Ultrassom ativo.
int ultrassom_ativo = 0;
// Barra
int contador_barra_progressiva = 0;
// Contador porcentagem.
int contador_porcentagem = 0;
// Contador ativo
int contador_ativo = 0;
// Controle da MS de operacao.
int estado_operacao = 0;
// Posicao do menu principal.
int posicao_menu_principal = 0;
// Tempo de espera.
long tempo_espera = 0;
// Selecao menu principal.
int selecao = 0;
// Contador segundos.
int contador_segundos = 0;
// Contador 10 porcento.
int contador_dez_porcento = 0;
// Em operacao
int em_operacao = 0;

// Temporizadores.
long time_start_mao = millis();
long tempo_inicio_inicializacao = millis(), tempo_atual_inicializacao = millis();
long tempo_inicio_barra_progressiva = millis(), tempo_atual_barra_progressiva = millis();
long tempo_inicio_contador_porcentagem = millis(), tempo_atual_contador_porcentagem = millis();
long tempo_inicio_bt = millis(), tempo_atual_bt = millis();
long tempo_inicio_buzzer = millis(), tempo_atual_buzzer = millis();
long tempo_inicio_lcd = millis(), tempo_atual_lcd = millis();
long tempo_inicio_operacao = millis(), tempo_atual_operacao = millis();
long tempo_inicio_contador_segundos = millis(), tempo_atual_contador_segundos = millis();
long time_start_processamento = millis(), tempo_atual_processamento = millis();

/* Instancias */
// Buzzer
Buzzer buzzer(PINO_BUZZER,DELAY_BUZZER);
// Display LCD.
//Display //display_lcd(LINHAS,COLUNAS);
//Inicializa o sensor nos pinos definidos acima
//Ultrasonic ultrasonic(pino_trigger, pino_echo);

/*********************************************************************************************************
** Nome da Função:       inicia_leds
** Descrição:            Faz os leds piscarem no momento que incia o codigo.
** Parametro:            Não.
** Valor de retorno:     Não.
*********************************************************************************************************/
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

/*********************************************************************************************************
** Nome da Função:       bluetooh_connected_leds
** Descrição:            Faz os leds quando conecta o bluetooth
** Parametro:            Não.
** Valor de retorno:     Não.
*********************************************************************************************************/
void bluetooh_connected_leds()
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

void inicia_eeprom()
{  
  leds_top = EEPROM.read(1);
  leds_bot = EEPROM.read(2);
  leds_left = EEPROM.read(3);
  leds_right = EEPROM.read(4);
}
/*********************************************************************************************************
** Nome da Função:       setup
** Descrição:            Setup principal da main, onde é feita a inicialização das portas seriais/pinos/
                         configurações,etc.
** Parametro:            Não.
** Valor de retorno:     Não.
*********************************************************************************************************/
void setup()
{
  // Iinicia serial principal de comunicação.
  //Serial.begin(115200);
  // Inicializacao da serial do bluetooth
  Serial3.begin(9600); 
  // Faz setup no ambilight
  setup_ambilight();
  // Funcao de inicializacao de pinos e estruturas.
  inicializacao();   
  // Inicializa os leds
  inicia_leds();  
  inicia_eeprom();
  // Debug
  Serial.println("Sistema iniciado com sucesso.");
}

/*********************************************************************************************************
** Nome da Função:       setup_amblight
** Descrição:            Setup ambilight
** Parametro:            Não.
** Valor de retorno:     Não.
*********************************************************************************************************/
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

/*********************************************************************************************************
** Nome da Função:       inicializacao
** Descrição:            Funcao que define valor inicial dos parametros, inicia memoria e le os dados 
                         necessarios.
** Parametro:            Não.
** Valor de retorno:     Não.
*********************************************************************************************************/
void inicializacao()
{
  // Inicialização do display lcd.
  //display_lcd.begin(LINHAS, COLUNAS);
  // Le estado de atividade.
  ultrassom_ativo = EEPROM.read(1);
  // Inicializa pinos.
  inicia_pinos();  
}

/*********************************************************************************************************
** Nome da Função:       inicia_pinos
** Descrição:            Funcao que define os pinos de I/O.
** Parametro:            Não.
** Valor de retorno:     Não.
*********************************************************************************************************/
void inicia_pinos()
{
  // Seta os botoes de interação.
  pinMode (BT_AVANCA, INPUT_PULLUP);
  pinMode (BT_VOLTA, INPUT_PULLUP);
  pinMode (BT_SELECIONA, INPUT_PULLUP);
  // Seta os pinos da ponte H e motores como output.
  pinMode (RELE_LAMP_TROFEU,OUTPUT); 
  pinMode (RELE_LED_TV,OUTPUT); 
  pinMode (RELE_LUZ_QUARTO,OUTPUT); 
  pinMode (IN4,OUTPUT); 
  digitalWrite (RELE_LAMP_TROFEU,HIGH); 
  digitalWrite (RELE_LED_TV,HIGH);
  digitalWrite (RELE_LUZ_QUARTO,HIGH);
  digitalWrite (IN4,HIGH);
  pinMode (LED_VERM,OUTPUT); 
  pinMode (LED_AMAR,OUTPUT); 
  pinMode (LED_VERD,OUTPUT); 
  digitalWrite (LED_VERM,LOW); 
  digitalWrite (LED_AMAR,LOW);
  digitalWrite (LED_VERD,LOW);
}

/*********************************************************************************************************
** Nome da Função:       reinicia_lcd
** Descrição:            Reinicia display lcd.
** Parametro:            Não.
** Valor de retorno:     Não.
*********************************************************************************************************/
void reinicia_lcd()
{
  // A cada 10s faz o lcd reiniciar para evitar bugs.
  tempo_atual_lcd = millis();
  if((tempo_atual_lcd-tempo_inicio_lcd) > 10000)
  {
    // Verifica se esta no menu principal.
    if ( estado_operacao == SELECAO_MENU_PRINCIPAL )
    {
      //Serial.println("REINICIOU LCD");
      // Inicialização do display lcd.
      //display_lcd.begin(LINHAS, COLUNAS); 
      delay(50);       
      estado_operacao = MENU_PRINCIPAL;
    } 
    // Reinicia temporizador.
    tempo_inicio_lcd = millis();       
  }
}

/*********************************************************************************************************
** Nome da Função:       led_processamento
** Descrição:            Tarefa faz o led de processamento piscar.
** Parametro:            Não.
** Valor de retorno:     Não.
*********************************************************************************************************/
void led_processamento()
{
  // A cada 500ms liga e desliga o led.
  tempo_atual_processamento = millis();
  if((tempo_atual_processamento-time_start_processamento) > 500)
  {    
    // Se led estiver ligado.  
    if ( digitalRead(LED_VERD) == HIGH )  
      // Desliga o led.    
      digitalWrite(LED_VERD,LOW);
    // Se desligado.  
    else
      // Liga o led.
      digitalWrite(LED_VERD,HIGH);
   
    // Reseta temporizador.
    time_start_processamento = tempo_atual_processamento;      
  }
}

// Liga luz.
void liga_luz()
{  
  // Se luz esta apagada.
  if ( estado_luz == 0 )
  {
    //Serial.println("Ligando Luz"); 
    digitalWrite(RELE_LUZ_QUARTO,LOW);
    estado_luz = 1;
  }
}

// Desliga luz.
void desliga_luz()
{  
  digitalWrite(RELE_LUZ_QUARTO,HIGH);
  // Se luz esta acesa.
  if ( estado_luz == 1 )
  {
    //Serial.println("Desligando Luz"); 
    digitalWrite(RELE_LUZ_QUARTO,HIGH);
    estado_luz = 0;
  }
}

// Liga led tv
void liga_led_tv()
{  
  // Se luz esta apagada.
  if ( estado_led_tv == 0 )
  {
    //Serial.println("Ligando LED TV");     
    digitalWrite(RELE_LED_TV,LOW);
    estado_led_tv = 1;
  }
}

// Desliga Led tv
void desliga_led_tv()
{  
  // Se luz esta acesa.
  if ( estado_led_tv == 1 )
  {
    //Serial.println("Desligando Luz"); 
    digitalWrite(RELE_LED_TV,HIGH);
    estado_led_tv = 0;
  }
}

// Liga lamp trofeu
void liga_lamp_trofeu()
{  
  // Se luz esta apagada.
  if ( estado_lamp_trofeu == 0 )
  {
    //Serial.println("Ligando Lamp trofeu");  
  digitalWrite(RELE_LAMP_TROFEU,LOW);
    estado_lamp_trofeu = 1;
  }
}

// Desliga lamp trofeu
void desliga_lamp_trofeu()
{  
  // Se luz esta acesa.
  if ( estado_lamp_trofeu == 1 )
  {
    //Serial.println("Desligando Luz");
  digitalWrite(RELE_LAMP_TROFEU,HIGH);
    estado_lamp_trofeu = 0;
  }
}

// Liga led ultrassom.
void liga_led_ultrassom()
{  
  //Serial.println("Ligando Led ultrassom"); 
  ambilight_mode = 0;
}

// Desliga led ultrassom.
void desliga_led_ultrassom()
{  
  //Serial.println("Desligando Led ultrassom"); 
  ambilight_mode = 1;
}

// Tarefa para desligar a luz por tempo de inatividade.
void task_inatividade_luz()
{
  // Verifica se ultrassom esta ativo.
  if ( ultrassom_ativo )
  {
     // Aguarda 200ms.
    tempo_atual_luz = millis();
    if((tempo_atual_luz-time_start_luz) > 600000)
    {
      // Desliga luz.
      desliga_luz();
    }
  }
}

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

/*********************************************************************************************************
** Nome da Função:      serialEvent
** Descrição:     Tafera de interrupção quando a algum evento na serial.
** Parametro:           Não
** Valor de retorno:    Não
*********************************************************************************************************/
void serialEvent() 
{
  // Enquanto exister algo na porta serial.
  if (Serial3.available()) 
  {
    delay(5);
    //Pega o novo byte:
    char inChar = (char)Serial3.read();
    // add it to the inputString:
    if (inChar == ';') {
      stringComplete = true;
    }else
      comando += inChar;    
  }
}

void desliga_leds()
{
  for(int i = 0; i < Num_Leds; i++) 
  {      
    // let's set an led value
    leds[i] = CRGB::Black;          
  }
  FastLED.show();
}

// Tarefa do bluetooth
void task_bluetooth()
{
  serialEvent();

  // Quando chegar algum comando da porta serial.
  if ( stringComplete ) 
  {
    Serial.println("Comando recebido bluetooth: " + (String)comando);
    
    // Quando o bluetooth é conectado.
    if ( comando.startsWith(F("BLUETOOTH_CONNECTED")) )   
    {      
      // Bluetooth conectado.
      bluetooh_connected_leds();
      inicia_leds();
      // Bluetooth conectado.
      bluetooth_connected = 1;
      
      Serial3.println("GET_STATUS 1 2 3 4");
    }

    // Seta a cor.
    if ( comando.startsWith(F("SET_COLOR")) )   
    {
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

      Serial3.println("SET_COLOR 1");
    }

    // Seta o modo do ambilight.
    if ( comando.startsWith(F("SET_MODE")) )   
    {
      String mode = get_indice_str(comando, 1);
      ambilight_mode = mode.toInt();

      if ( ambilight_mode == 0 )
      {        
        // Bluetooth conectado.
        bluetooh_connected_leds();
        inicia_leds();
        desliga_leds();
      }
    }

    // Seta o modo do ambilight.
    if ( comando.startsWith(F("SET_PARAMS")) )   
    {
      String top_str = get_indice_str(comando, 1);
      String bot_str = get_indice_str(comando, 2);
      String left_str = get_indice_str(comando, 3);
      String right_str = get_indice_str(comando, 4);
      leds_top = top_str.toInt();
      leds_bot = bot_str.toInt();
      leds_left = left_str.toInt();
      leds_right = right_str.toInt();
      EEPROM.write(1,leds_top);
      EEPROM.write(2,leds_bot);
      EEPROM.write(3,leds_left);
      EEPROM.write(4,leds_right);      
    }
    

    // Seta o modo do ambilight.
    if ( comando.startsWith(F("LED_ON")) )   
    {
      String led_str = get_indice_str(comando, 1);
      String type_str = get_indice_str(comando, 2);
      String timeout_str =  get_indice_str(comando, 3);
      int num_led = led_str.toInt();
      desliga_leds();
      leds[num_led] = CRGB::White;  
      FastLED.show();
    }   

    // Seta o modo do ambilight.
    if ( comando.startsWith(F("DESLIGA_LEDS")) )   
    {
      desliga_leds();
    }    
    
    // Limpa a string:
    comando = "";
    stringComplete = false;
  }
}

/*
delay(100);
while(1)
{
  desliga_leds();
  delay(500);
  leds[25] = CRGB::White;  
  FastLED.show();
  delay(500);
}*/


/*********************************************************************************************************
** Nome da Função:       task_ambilight
** Descrição:            Task ambilight
** Parametro:            Não.
** Valor de retorno:     Não.
*********************************************************************************************************/
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

/*********************************************************************************************************
** Nome da Função:       loop
** Descrição:            Loop principal do sistema, onde todas tarefas são executadas.
** Parametro:            Não.
** Valor de retorno:     Não.
*********************************************************************************************************/
void loop() 
{    
  // Task abilight
  task_ambilight();
  
  // Tarefa do bluetooth.
  task_bluetooth();

  // Tarefa para desligar a luz por tempo de inatividade.
  //task_inatividade_luz();
  
  // Reinicia o lcd.
  //reinicia_lcd();

  // Máquina de estados principal de interação.
  task_operacao();

  // Tarefa que verifica se é para emitir o efeito sonoro do beep.
  buzzer.task();

  // Tarefa que faz piscar o led para mostrar processamento
  led_processamento();
}

/*********************************************************************************************************
** Nome da Função:       task_operacao
** Descrição:            Tarefa responsavel pela interação do usuario com a selecao do produto, visualização
                         do credito.
** Parametro:            Não.
** Valor de retorno:     Não.
*********************************************************************************************************/
void task_operacao()
{
  // Maquina de estados da operacao.
  switch(estado_operacao)
  {
    // Case principal de venda, com a intereção com o usuario.
    case INICIALIZACAO:
      //display_lcd.limpa_tela();
      //display_lcd.escreve("INICIANDO", CENTRALIZADO, 0, 1);
      //display_lcd.escreve("  AGUARDE    V" +(String)V_SFT, NORMAL, 0, 2);
      // Inicia temporizador.
      tempo_inicio_inicializacao = millis();
      // Continua para proximo estado.
      estado_operacao = AGUARDA_INICIALIZACAO;
      break;  
    // Aguarda inicializacao.
    case AGUARDA_INICIALIZACAO:
      // Aguarda passar 2 segundos.
      tempo_atual_inicializacao = millis();
      if((tempo_atual_inicializacao-tempo_inicio_inicializacao) > 1000 )
      {    
        digitalWrite(LED_VERM,LOW);
        digitalWrite(LED_AMAR,HIGH);
        //display_lcd.escreve(".", NORMAL, 9, 2);
      }
      // Aguarda passar 2 segundos.
      tempo_atual_inicializacao = millis();
      if((tempo_atual_inicializacao-tempo_inicio_inicializacao) > 2000 )
      {    
        digitalWrite(LED_VERM,HIGH);
        digitalWrite(LED_AMAR,LOW);
        //display_lcd.escreve(".", NORMAL, 10, 2);
      }
      // Aguarda passar 2 segundos.
      tempo_atual_inicializacao = millis();
      if((tempo_atual_inicializacao-tempo_inicio_inicializacao) > 3000 )
      {    
        digitalWrite(LED_VERM,HIGH);
        digitalWrite(LED_AMAR,HIGH);
        //display_lcd.escreve(".", NORMAL, 11, 2);
      }
      // Aguarda passar 2 segundos.
      tempo_atual_inicializacao = millis();
      if((tempo_atual_inicializacao-tempo_inicio_inicializacao) > 4000 )
      {    
        digitalWrite(LED_VERM,LOW);
        digitalWrite(LED_AMAR,LOW);
        // Continua para proximo estado.
        estado_operacao = MENU_PRINCIPAL;
      }
      break;
    // Menu principal.
    case MENU_PRINCIPAL:
      // Chama o menu principal.
      menu_principal();
      // Continua para proximo estado.
      estado_operacao = SELECAO_MENU_PRINCIPAL;
      break;  
    // Percorre o menu atraves dos botoes de interação.   
    case SELECAO_MENU_PRINCIPAL: 
      // Pega se algum botao foi pressionado.
      selecao = bt_press();

      if ( selecao != 0 )
      {
        // De acordo com o botao pressionado movimenta o menu.     
        switch(selecao)
        {
          // Se botao avanar pressionado.
          case AVANCAR:
            if ( posicao_menu_principal < QTD_MENU_PRINCIPAL )
              posicao_menu_principal++;
            else
              posicao_menu_principal = 0;
  
            // Volta para mostrar nova selecao.
            estado_operacao = MENU_PRINCIPAL;    
            break;          
          // Se botao voltar pressionado.  
          case VOLTAR:
            if ( posicao_menu_principal > 0 )
              posicao_menu_principal--;
            else
              posicao_menu_principal = QTD_MENU_PRINCIPAL;
  
            // Volta para mostrar nova selecao.
            estado_operacao = MENU_PRINCIPAL;    
            break;
          // Se botao seleciona pressionado.  
          case SELECIONA:
            // Verifica qual opcao foi selecionada.
            switch(posicao_menu_principal)
            {        
              case TELA1: 
                  //Serial.println("SELECINOU TELA1");
                  //display_lcd.limpa_tela();
                  //display_lcd.escreve("INICIANDO", CENTRALIZADO, 0, 1);
                  //display_lcd.escreve("TELA1...", CENTRALIZADO, 0, 2);
                  // Seta o tempo de espera para 2 segundos.
                  seta_tempo_espera(2);
                  // Continua para inicializacao do processo "MODELO".
                  estado_operacao = TELA1;
                  break; 
            }
            break;
        }
      }
  }
}

/*********************************************************************************************************
** Nome da Função:       seta_tempo_espera
** Descrição:            Seta tempo de espera em segundos.
** Parametro:            tempo = tempo em segundos de espera.
** Valor de retorno:     Não.
*********************************************************************************************************/
void seta_tempo_espera(long tempo)
{  
  // Inicia temporizador.
  tempo_inicio_operacao = millis();  
  // Seta o tempo de espera.
  tempo_espera = tempo*1000;  
  //Serial.println("TEMPO ESPERA: " + (String)tempo_espera);  
}

/*********************************************************************************************************
** Nome da Função:       aguarda_tempo_espera
** Descrição:            Aguarda tempo de espera em segundos.
** Parametro:            Não
** Valor de retorno:     1 = sucesso, 0 - aguardar.
*********************************************************************************************************/
int aguarda_tempo_espera()
{
  // Aguarda o tempo para informar a função.
  tempo_atual_operacao = millis();
  if((tempo_atual_operacao-tempo_inicio_operacao) > tempo_espera )
  { 
    return 1;
  }else return 0;
}

/*********************************************************************************************************
** Nome da Função:       menu_principal
** Descrição:            Menu principal de servico, onde contem todos menus.
** Parametro:            Não
** Valor de retorno:     Não.
*********************************************************************************************************/
void menu_principal()
{
 /* switch(posicao_menu_principal)
  {        
     case TELA1:
        //display_lcd.limpa_tela();
        //display_lcd.escreve("BEM VINDO", CENTRALIZADO, 0, 0);
        ////display_lcd.escreve_char_especial(4, 0, 2);   
        if ( ultrassom_ativo )
          //display_lcd.escreve("SENSOR: ATIVO", NORMAL, 0, 2); 
        else
          //display_lcd.escreve("SENSOR: INATIVO", NORMAL, 0, 2);   
        if ( bluetooth_conectado )
          //display_lcd.escreve("BLUETOOTH: ATIVO", NORMAL, 0, 3); 
        else
          //display_lcd.escreve("BLUETOOTH: INATIVO", NORMAL, 0, 3);           
        break; 
     case TELA2:
        //display_lcd.limpa_tela();
        //display_lcd.escreve("TELA: ", CENTRALIZADO, 0, 0);
        //display_lcd.escreve_char_especial(4, 0, 2);   
        //display_lcd.escreve("2", NORMAL, 2, 2); 
        break;  
     case TELA3:
        //display_lcd.limpa_tela();
        //display_lcd.escreve("TELA: ", CENTRALIZADO, 0, 0);
        //display_lcd.escreve_char_especial(4, 0, 2);   
        //display_lcd.escreve("3", NORMAL, 2, 2); 
        break; 
      case TELA4:
        //display_lcd.limpa_tela();
        //display_lcd.escreve("TELA: ", CENTRALIZADO, 0, 0);
        //display_lcd.escreve_char_especial(4, 0, 2);   
        //display_lcd.escreve("4", NORMAL, 2, 2); 
        break;  
      case TELA5:
        //display_lcd.limpa_tela();
        //display_lcd.escreve("TELA: ", CENTRALIZADO, 0, 0);
        //display_lcd.escreve_char_especial(4, 0, 2);   
        //display_lcd.escreve("5", NORMAL, 2, 2); 
        break;   
  }   */
}

/*********************************************************************************************************
** Nome da Função:       bt_press
** Descrição:            Funcao para verificar se algum botao foi pressionado.
** Parametro:            Não
** Valor de retorno:     1 - Avancar. 2 - Voltar. 3 - SELECIONA.
*********************************************************************************************************/
int bt_press()
{
  // A cada 10s faz o lcd reiniciar para evitar bugs.
  tempo_atual_bt = millis();
  if((tempo_atual_bt-tempo_inicio_bt) > 300)
  {
    // Se botao avancar pressionado.
    if(digitalRead(BT_AVANCA))
    {
      // Inicia contador para contagem dos botoes.
      tempo_inicio_bt = millis();
      // Liga o buzzer.
      buzzer.liga();
      return AVANCAR;
    }
    // Se botao voltar pressionado.
    if(digitalRead(BT_VOLTA))
    {
      // Inicia contador para contagem dos botoes.
      tempo_inicio_bt = millis();
      // Liga o buzzer.
      buzzer.liga();
      return VOLTAR;
    }
    // Se botao seleciona pressionado.
    if(digitalRead(BT_SELECIONA))
    {
      // Inicia contador para contagem dos botoes.
      tempo_inicio_bt = millis();
      // Liga o buzzer.
      buzzer.liga();
      return SELECIONA;
    }
  }
  return 0;
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
    Serial.print("Ada ");
    Serial.print(leds_top);
    Serial.print(" ");
    Serial.print(leds_bot);
    Serial.print(" ");
    Serial.print(leds_left);
    Serial.print(" ");
    Serial.print(leds_right);
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
        
