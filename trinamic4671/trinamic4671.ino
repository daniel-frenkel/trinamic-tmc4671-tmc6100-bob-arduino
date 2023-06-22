#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <FastAccelStepper.h>
#include "SimpleFOC.h"
#include "SimpleFOCDrivers.h"
#include "encoders/mt6835/MagneticSensorMT6835.h"

#include "TMC4671_Register.h"
#include "TMC4671_Fields.h"
#include "TMC4671_Constants.h"
#include "TMC4671_Variants.h"

#define PIN_TMC_ENN     26
#define PIN_SPI_SCK     18
#define PIN_SPI_MISO    19
#define PIN_SPI_MOSI    22
#define PIN_SPI_CS_TMC  21
#define PIN_SPI_CS_MAG  5

#define STEP_PIN  15
#define DIR_PIN  14

#define A_pin 32
#define B_pin 33
#define Z_pin 23

MagneticSensorMT6835 sensor = MagneticSensorMT6835(nCS);

FastAccelStepperEngine engine = FastAccelStepperEngine();
FastAccelStepper *stepper = NULL;

int16_t  tgt_torque   = 0;
int16_t  tgt_flux     = 0;
int16_t  tgt_velocity = 0;
uint16_t p_torque     = 400;
uint16_t i_torque     = 20;
uint16_t p_flux       = 50;
uint16_t i_flux       = 10;
uint16_t p_velocity   = 400;
uint16_t i_velocity   = 50;
bool velocity_active  = false;
bool torque_active    = false;

int current_position = 0;
int accel = 10000;
int max_speed = 30000;
int move_to_step = 6000000;
int step_width = 1;

void spi_write(uint8_t reg, uint32_t value) {
  Serial.println("[CPU -> CTRL] " + String(reg, HEX) + ": 0x" + String(value));
  SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE3)); //100000
  digitalWrite(PIN_SPI_CS_TMC, LOW);
  SPI.transfer(reg | 0x80);
  uint8_t value_be[4];
  value_be[0] = (value >> 24) & 0xFF;
  value_be[1] = (value >> 16) & 0xFF;
  value_be[2] = (value >>  8) & 0xFF;
  value_be[3] = (value >>  0) & 0xFF;
  SPI.transfer((uint8_t*) &value_be, 4);
  digitalWrite(PIN_SPI_CS_TMC, HIGH);
  SPI.endTransaction();
  delay(1);
}

uint32_t spi_read(uint8_t reg) {
  SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE3)); //100000
  digitalWrite(PIN_SPI_CS_TMC, LOW);
  SPI.transfer(reg & 0x7F);
  uint8_t value_be[4] = {0,0,0,0};
  SPI.transfer((uint8_t*) &value_be, 4);
  digitalWrite(PIN_SPI_CS_TMC, HIGH);
  SPI.endTransaction();
  return (value_be[0]<<24) | (value_be[1]<<16) | (value_be[2]<<8) | value_be[3];
}

void setup() {
  Serial.begin(115200);
  SPI.begin(PIN_SPI_SCK, PIN_SPI_MOSI, PIN_SPI_MISO, PIN_SPI_CS_MAG);
  //sensor.init();
  digitalWrite(PIN_TMC_ENN, LOW);
  pinMode(PIN_TMC_ENN, OUTPUT);
  pinMode(PIN_SPI_CS_TMC, OUTPUT);
  pinMode(PIN_SPI_CS_MAG, OUTPUT);
  digitalWrite(PIN_SPI_CS_TMC, HIGH);
  digitalWrite(PIN_SPI_CS_MAG, HIGH);


  initHardware(p_torque, i_torque, p_flux, i_flux, p_velocity, i_velocity);

  engine.init();
  stepper = engine.stepperConnectToPin(STEP_PIN);
  stepper->setDirectionPin(DIR_PIN);
  stepper->setEnablePin(PIN_TMC_ENN);
  stepper->setAutoEnable(true);

  stepper->setCurrentPosition(current_position);
  stepper->setAcceleration(accel);
  stepper->setSpeedInHz(max_speed);
  stepper->moveTo(move_to_step);
  
}


void setTorque(int16_t torque, int16_t flux) {
  spi_write(TMC4671_MODE_RAMP_MODE_MOTION, 1);
  spi_write(TMC4671_PID_TORQUE_FLUX_TARGET, (torque << 16) | flux);
  velocity_active = false;
  torque_active = true;
}

void setVelocity(int16_t velocity) {
  spi_write(TMC4671_MODE_RAMP_MODE_MOTION, 2);
  spi_write(TMC4671_PID_VELOCITY_TARGET, velocity);
  velocity_active = true;
  torque_active = false;
}

void setStop() {
  spi_write(TMC4671_MODE_RAMP_MODE_MOTION, 0x00000000);
  velocity_active = false;
  torque_active = false;
}

void setVelocityPi(uint16_t p, uint16_t i) {
  spi_write(TMC4671_PID_VELOCITY_P_VELOCITY_I, (p << 16) | i);
}

void setTorquePi(uint16_t p, uint16_t i) {
  spi_write(TMC4671_PID_TORQUE_P_TORQUE_I, (p << 16) | i);
}

//We are bit shifting using the fileds shift macros
void setFluxPi(uint16_t p, uint16_t i) {
  spi_write(TMC4671_PID_FLUX_P_FLUX_I, (p << TMC4671_PID_FLUX_P_SHIFT) | i << TMC4671_PID_FLUX_I_SHIFT);
}


void setFluxP(uint16_t p){
spi_write(TMC4671_PID_FLUX_P_FLUX_I, p << TMC4671_PID_FLUX_P_SHIFT);
}

void setFluxI(uint16_t i){
spi_write(TMC4671_PID_FLUX_P_FLUX_I, i << TMC4671_PID_FLUX_I_SHIFT);
}

void initHardware(uint16_t torqueP, uint16_t torqueI, uint16_t fluxP, uint16_t fluxI, uint16_t velocityP, uint16_t velocityI) {
  digitalWrite(PIN_TMC_ENN, LOW);

  // Motor type &  PWM configuration
  spi_write(TMC4671_MOTOR_TYPE_N_POLE_PAIRS, 0b100000000000110010); //Set motor type only: Motor type 2 
  spi_write(TMC4671_MOTOR_TYPE_N_POLE_PAIRS, 0b100000000000110010); //set pole pairs only: 50 pole pairs
  spi_write(TMC4671_PWM_POLARITIES, 0x00000000);
  spi_write(TMC4671_PWM_MAXCNT, 0x000003E7);
  spi_write(TMC4671_PWM_BBM_H_BBM_L, 0x00001919);
  spi_write(TMC4671_PWM_SV_CHOP, 0x00000007);
  
  // ADC configuration
  spi_write(TMC4671_ADC_I_SELECT, 0x18000100);
  spi_write(TMC4671_dsADC_MCFG_B_MCFG_A, 0x00100010);
  spi_write(TMC4671_dsADC_MCLK_A, 0x0051EB85);
  spi_write(TMC4671_dsADC_MCLK_B, 0x0051EB85);
  spi_write(TMC4671_dsADC_MDEC_B_MDEC_A, 0x00540054);
  spi_write(TMC4671_ADC_I0_SCALE_OFFSET, 0x010081CB);
  spi_write(TMC4671_ADC_I1_SCALE_OFFSET, 0x010081F9);

  // Digital hall settings
  //spi_write(SPI_DEV_CTRL,  TMC4671_HALL_MODE, 0x00001001);
  //spi_write(SPI_DEV_CTRL,  TMC4671_HALL_PHI_E_PHI_M_OFFSET, 0x1F400000);
  
  // ABN encoder settings
  spi_write(TMC4671_ABN_DECODER_MODE, 0x00000000);
  spi_write(TMC4671_ABN_DECODER_PPR, 0x00000800);
  spi_write(TMC4671_ABN_DECODER_COUNT, 0x00000539);
  spi_write(TMC4671_ABN_DECODER_PHI_E_PHI_M_OFFSET, 0x00000000);
  
  // Limits
  spi_write(TMC4671_PID_TORQUE_FLUX_LIMITS, 2000);
  
  // PI settings
  setTorquePi(torqueP, torqueI);
  setFluxPi(fluxP, fluxI);
  setVelocityPi(velocityP, velocityI);

  // Init encoder (mode 0)
  spi_write(TMC4671_MODE_RAMP_MODE_MOTION, 0x00000008);
  spi_write(TMC4671_ABN_DECODER_PHI_E_PHI_M_OFFSET, 0x00000000);
  spi_write(TMC4671_PHI_E_SELECTION, 0x00000001);
  spi_write(TMC4671_PHI_E_EXT, 0x00000000);
  spi_write(TMC4671_UQ_UD_EXT, 0x000007D0);
  delay(100);

  spi_write(TMC4671_ABN_DECODER_COUNT, 0x00000000);

  // Feedback selection
  spi_write(TMC4671_PHI_E_SELECTION, 5); // 3 is encoder, 5 is hal
  spi_write(TMC4671_VELOCITY_SELECTION, 5);

  //MINE
  //spi_write(
  
  // Stop mode
  setStop();
  digitalWrite(PIN_TMC_ENN, HIGH);
  
}

String serialInput = "";

uint16_t p = 400; // Velocity P
uint16_t i = 50; // Velocity I

void parseSerial() {
  if (serialInput.length() < 1) return;
  char c = serialInput[0];
  serialInput.remove(0,1);
  switch(c) {
    case 'r':
      initHardware(p_torque, i_torque, p_flux, i_flux, p_velocity, i_velocity);
      break;
    case 'v':
      tgt_velocity = serialInput.toInt();
      setVelocity(tgt_velocity);
      break;
    case 't':
      tgt_torque = serialInput.toInt();
      setTorque(tgt_torque, tgt_flux);
      break;
    case 'f':
      tgt_flux = serialInput.toInt();
      setTorque(tgt_torque, tgt_flux);
      break;
    case 'p':
      p_velocity = serialInput.toInt();
      setVelocityPi(p_velocity, i_velocity);
      break;
    case 'i':
      i_velocity = serialInput.toInt();
      setVelocityPi(p_velocity, i_velocity);
      break;
    case 'P':
      p_torque = serialInput.toInt();
      setTorquePi(p_torque, i_torque);
      break;
    case 'I':
      i_torque = serialInput.toInt();
      setTorquePi(p_torque, i_torque);
      break;
    case 'o':
      p_flux = serialInput.toInt();
      setFluxPi(p_flux, i_flux);
      break;
    case 'u':
      i_flux = serialInput.toInt();
      setFluxPi(p_flux, i_flux);
      break;
    case 's':
      setStop();
      break;
    case 'l':
      Serial.println("Torque:   P = " + String(p_torque)   + ", I = " + String(i_torque));
      Serial.println("Flux:     P = " + String(p_flux)     + ", I = " + String(i_flux));
      Serial.println("Velocity: P = " + String(p_velocity) + ", I = " + String(i_velocity));
      break;
  }
  if (velocity_active) {
    Serial.println("Velocity mode, target = " + String(tgt_velocity));
  } else if (torque_active) {
    Serial.println("Torque mode, target = " + String(tgt_torque) + " (flux target = " + String(tgt_flux) + ")");
  } else {
    Serial.println("Stop mode");
  }
  serialInput = "";
}

void checkSerial() {
  while (Serial.available()>0) {
    char c = Serial.read();
    if ((c >= ' ') && (c <= '~')) {
      serialInput += c;
    } else if ((c == '\n') || (c == '\r')) {
      parseSerial();
    } else {
      serialInput = "";
      Serial.println("#Received garbage, buffer cleared");
    }
  }
}

void loop() {
  //checkSerial();
  int TMC_INPUTS_RAW = spi_read(TMC4671_MOTOR_TYPE_N_POLE_PAIRS);
  Serial.println(TMC_INPUTS_RAW);
  delay(1000);
}
