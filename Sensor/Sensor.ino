#include <Arduino.h>
#include <SX126x-RAK4630.h> //http://librarymanager/All#SX126x
#include <SPI.h>

//LoRa
// Function declarations
void OnTxDone(void);
void OnTxTimeout(void);

// Define LoRa parameters
#define RF_FREQUENCY 915000000  // Hz
#define TX_OUTPUT_POWER 22    // dBm
#define LORA_BANDWIDTH 2    // [0: 125 kHz, 1: 250 kHz, 2: 500 kHz, 3: Reserved]
#define LORA_SPREADING_FACTOR 7 // [SF7..SF12]
#define LORA_CODINGRATE 1   // [1: 4/5, 2: 4/6,  3: 4/7,  4: 4/8]
#define LORA_PREAMBLE_LENGTH 8  // Same for Tx and Rx
#define LORA_SYMBOL_TIMEOUT 0 // Symbols
#define LORA_FIX_LENGTH_PAYLOAD_ON false
#define LORA_IQ_INVERSION_ON false
#define RX_TIMEOUT_VALUE 3000
#define TX_TIMEOUT_VALUE 3000

static RadioEvents_t RadioEvents;
static uint8_t TxdBuffer[64];
//LoRa



//pin definitions
#define PIN_DRDY WB_IO3 //Set pin D9 to be the Data Ready Pin
#define PIN_CS WB_IO1 //Chip Select (SS) is set to Pin 10

//internal register values without the R/W bit
#define RM3100_REVID_REG 0x36 // Hexadecimal address for the Revid internal register
#define RM3100_POLL_REG 0x00 // Hexadecimal address for the Poll internal register
#define RM3100_CMM_REG 0x01 // Hexadecimal address for the Continuous Measurement Mode internal register
#define RM3100_STATUS_REG 0x34 // Hexadecimal address for the Status internal register
#define RM3100_CCX1_REG 0x04 // Hexadecimal address for the Cycle Count X1 internal register
#define RM3100_CCX0_REG 0x05 // Hexadecimal address for the Cycle Count X0 internal register

//options
#define initialCC 200 // Set the cycle count
#define singleMode 0 //0 = use continuous measurement mode; 1 = use single measurement mode
#define useDRDYPin 1 //0 = not using DRDYPin ; 1 = using DRDYPin to wait for data

uint8_t revid;
uint16_t cycleCount;
float gain;
long promedio_z;
int limite = 100;
char estado = '0';

void setup() {
  // Initialize LoRa chip.
  lora_rak4630_init();

  // Initialize Serial for debug output
  time_t timeout = millis();
  Serial.begin(115200);
  while (!Serial)
  {
    if ((millis() - timeout) < 5000)
    {
            delay(100);
        }
        else
        {
            break;
        }
  }
  Serial.println("=====================================");
  Serial.println("LoRap2p Tx Test");
  Serial.println("=====================================");

  // Initialize the Radio callbacks
  RadioEvents.TxDone = OnTxDone;
  RadioEvents.RxDone = NULL;
  RadioEvents.TxTimeout = OnTxTimeout;
  RadioEvents.RxTimeout = NULL;
  RadioEvents.RxError = NULL;
  RadioEvents.CadDone = NULL;

  // Initialize the Radio
  Radio.Init(&RadioEvents);

  // Set Radio channel
  Radio.SetChannel(RF_FREQUENCY);

  // Set Radio TX configuration
  Radio.SetTxConfig(MODEM_LORA, TX_OUTPUT_POWER, 0, LORA_BANDWIDTH,
            LORA_SPREADING_FACTOR, LORA_CODINGRATE,
            LORA_PREAMBLE_LENGTH, LORA_FIX_LENGTH_PAYLOAD_ON,
            true, 0, 0, LORA_IQ_INVERSION_ON, TX_TIMEOUT_VALUE);
  send();
  //LoRa
  
  pinMode(PIN_DRDY, INPUT);  
  pinMode(PIN_CS, OUTPUT);
  digitalWrite(PIN_CS, HIGH);
  SPI.begin(); // Initiate the SPI library
  SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE0));  
  Serial.begin(9600); //set baud rate to 9600
  delay(100);

  revid = readReg(RM3100_REVID_REG);
  
  Serial.print("REVID ID = 0x"); //REVID ID should be 0x22
  Serial.println(revid, HEX);

  changeCycleCount(initialCC); //change the cycle count; default = 200 (lower cycle count = higher data rates but lower resolution)

  cycleCount = readReg(RM3100_CCX1_REG);
  cycleCount = (cycleCount << 8) | readReg(RM3100_CCX0_REG);

  Serial.print("Cycle Counts = "); //display cycle count
  Serial.println(cycleCount);

  gain = (0.3671 * (float)cycleCount) + 1.5; //linear equation to calculate the gain from cycle count

  Serial.print("Gain = "); //display gain; default gain should be around 75 for the default cycle count of 200
  Serial.println(gain);

  if (singleMode){
    //set up single measurement mode
    writeReg(RM3100_CMM_REG, 0);
    writeReg(RM3100_POLL_REG, 0x70);
  }
  else{
    // Enable transmission to take continuous measurement with Alarm functions off
    writeReg(RM3100_CMM_REG, 0x79);
  }
  encendido();
  autocalibracion();
}

long leer_datos(){

  long z = 0;
  uint8_t x2,x1,x0,y2,y1,y0,z2,z1,z0;

  //wait until data is ready using 1 of two methods (chosen in options at top of code)
  if(useDRDYPin){ 
    while(digitalRead(PIN_DRDY) == LOW); //check RDRY pin
  }
  else{
    while((readReg(RM3100_STATUS_REG) & 0x80) != 0x80); //read internal status register
  }
  
  //read measurements
  digitalWrite(PIN_CS, LOW);
  delay(100);
  SPI.transfer(0xA4);
  x2 = SPI.transfer(0xA5);
  x1 = SPI.transfer(0xA6);
  x0 = SPI.transfer(0xA7);
  
  y2 = SPI.transfer(0xA8);
  y1 = SPI.transfer(0xA9);
  y0 = SPI.transfer(0xAA);
  
  z2 = SPI.transfer(0xAB);
  z1 = SPI.transfer(0xAC);
  z0 = SPI.transfer(0);
  
  digitalWrite(PIN_CS, HIGH);

  if (z2 & 0x80){
      z = 0xFF;
  }

  //format results into single 32 bit signed value
  z = (z * 256 * 256 * 256) | (int32_t)(z2) * 256 * 256 | (uint16_t)(z1) * 256 | z0;

  return (float)z;   
}

void autocalibracion(){
  estado = '0';
  float sumas = 0.0;
  for (int i = 1; i <= 150; i++) {
    sumas = sumas + (float)leer_datos();
    
    Serial.println(i);
    Serial.println(sumas);
    delay(100);
    if(i == 150){
      promedio_z = sumas / 150.0;
    }
  }
  Serial.print("promedio_z: ");
  Serial.println(promedio_z);
}

void encendido(){
  // tiempo de inicio
  int tiempo = 30000;
  // encender pin testigo
  digitalWrite(LED_BUILTIN, HIGH);
  delay(tiempo);
  // apagar pin testigo
  digitalWrite(LED_BUILTIN, LOW);
  
}

void loop() {
  long x = 0;
  long y = 0;
  long z = 0;
  uint8_t x2,x1,x0,y2,y1,y0,z2,z1,z0;

  //wait until data is ready using 1 of two methods (chosen in options at top of code)
  if(useDRDYPin){ 
    while(digitalRead(PIN_DRDY) == LOW); //check RDRY pin
  }
  else{
    while((readReg(RM3100_STATUS_REG) & 0x80) != 0x80); //read internal status register
  }
  
  //read measurements
  digitalWrite(PIN_CS, LOW);
  delay(100);
  SPI.transfer(0xA4);
  x2 = SPI.transfer(0xA5);
  x1 = SPI.transfer(0xA6);
  x0 = SPI.transfer(0xA7);
  
  y2 = SPI.transfer(0xA8);
  y1 = SPI.transfer(0xA9);
  y0 = SPI.transfer(0xAA);
  
  z2 = SPI.transfer(0xAB);
  z1 = SPI.transfer(0xAC);
  z0 = SPI.transfer(0);
  
  digitalWrite(PIN_CS, HIGH);

  //special bit manipulation since there is not a 24 bit signed int data type
  if (x2 & 0x80){
      x = 0xFF;
  }
  if (y2 & 0x80){
      y = 0xFF;
  }
  if (z2 & 0x80){
      z = 0xFF;
  }

  //format results into single 32 bit signed value
  x = (x * 256 * 256 * 256) | (int32_t)(x2) * 256 * 256 | (uint16_t)(x1) * 256 | x0;
  y = (y * 256 * 256 * 256) | (int32_t)(y2) * 256 * 256 | (uint16_t)(y1) * 256 | y0;
  z = (z * 256 * 256 * 256) | (int32_t)(z2) * 256 * 256 | (uint16_t)(z1) * 256 | z0;

  //calculate magnitude of results
  double uT = sqrt(pow(((float)(x)/gain),2) + pow(((float)(y)/gain),2)+ pow(((float)(z)/gain),2));

  //Promedio z menos z
  long pz = promedio_z - z;
  if(abs((int)pz) > limite){
    estado = '1';
  }else{
    estado = '0';
  }
}

//addr is the 7 bit value of the register's address (without the R/W bit)
uint8_t readReg(uint8_t addr){
  uint8_t data = 0;
  digitalWrite(PIN_CS, LOW);
  delay(100);
  SPI.transfer(addr | 0x80); //OR with 0x80 to make first bit(read/write bit) high for read
  data = SPI.transfer(0);
  digitalWrite(PIN_CS, HIGH);
  return data;
}

//addr is the 7 bit (No r/w bit) value of the internal register's address, data is 8 bit data being written
void writeReg(uint8_t addr, uint8_t data){
  digitalWrite(PIN_CS, LOW); 
  delay(100);
  SPI.transfer(addr & 0x7F); //AND with 0x7F to make first bit(read/write bit) low for write
  SPI.transfer(data);
  digitalWrite(PIN_CS, HIGH);
}

//newCC is the new cycle count value (16 bits) to change the data acquisition
void changeCycleCount(uint16_t newCC){
  uint8_t CCMSB = (newCC & 0xFF00) >> 8; //get the most significant byte
  uint8_t CCLSB = newCC & 0xFF; //get the least significant byte
    
  digitalWrite(PIN_CS, LOW); 
  delay(100);
  SPI.transfer(RM3100_CCX1_REG & 0x7F); //AND with 0x7F to make first bit(read/write bit) low for write
  SPI.transfer(CCMSB);  //write new cycle count to ccx1
  SPI.transfer(CCLSB);  //write new cycle count to ccx0
  SPI.transfer(CCMSB);  //write new cycle count to ccy1
  SPI.transfer(CCLSB);  //write new cycle count to ccy0
  SPI.transfer(CCMSB);  //write new cycle count to ccz1
  SPI.transfer(CCLSB);  //write new cycle count to ccz0
  digitalWrite(PIN_CS, HIGH);
}

void OnTxDone(void)
{
  send();
}

void OnTxTimeout(void)
{
  Serial.println("OnTxTimeout");
}

void send()
{
  TxdBuffer[0] = 'A';
  TxdBuffer[1] = 'A';
  TxdBuffer[2] = 'A';
  TxdBuffer[3] = 'B';
  TxdBuffer[4] = estado;
  Radio.Send(TxdBuffer, 5);
}
