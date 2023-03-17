#include <Adafruit_BLE.h>
#include <Adafruit_BluefruitLE_SPI.h>
#include <SPI.h>
#include <Adafruit_MotorShield.h>
#include "BluefruitConfig.h"

// Create an instance of the BLE SPI class
Adafruit_BluefruitLE_SPI ble(BLUEFRUIT_SPI_CS, BLUEFRUIT_SPI_IRQ, BLUEFRUIT_SPI_RST);

// Create the motor shield object with the default I2C address
Adafruit_MotorShield AFMS = Adafruit_MotorShield();

// And connect 2 DC motors to port M3 & M4 !
Adafruit_DCMotor *L_MOTOR = AFMS.getMotor(3);
Adafruit_DCMotor *R_MOTOR = AFMS.getMotor(4);

int left_speed = 0;
int right_speed = 0;

void setup()
{
  // Initialize serial communication
  if (Serial)
    Serial.begin(9600);
  // while (!Serial)
  //   ;
  if (Serial)
    Serial.println("Starting");

  // Initialize the SPI bus
  SPI.begin();

  // Initialize BLE module
  if (!ble.begin())
  {
    if (Serial)
      Serial.println("Couldn't find Bluefruit, make sure it's in CoMmanD mode & check wiring?");
    while (1)
      ;
  }

  if (!AFMS.begin())
  {
    if (Serial)
      Serial.println("Could not find Motor Shield. Check wiring.");
    while (1)
      ;
  }

  // Set module to Data mode
  ble.setMode(BLUEFRUIT_MODE_DATA);

  // Wait for connection
  if (Serial)
    Serial.println("Waiting for a BLE connection...");

  while (!ble.isConnected())
    ;
  if (Serial)
    Serial.println("Connected to BLE device!");

  L_MOTOR->setSpeed(0);
  R_MOTOR->setSpeed(0);
  L_MOTOR->run(RELEASE);
  R_MOTOR->run(RELEASE);
}

// Clamp a value between a min and max
int clamp(int val, int min, int max)
{
  if (val < min)
    return min;
  if (val > max)
    return max;
  return val;
}

int setDeadband(int val, int deadband)
{
  if (val < deadband && val > -deadband)
    return 0;
  return val;
}

void arcade_drive(double x, double y)
{
  // Map the joystick values to the motor speed
  left_speed = clamp(((y * 2.55) + (x * 2.55) + (left_speed * 2)) / 3, -255, 255);
  right_speed = clamp(((y * 2.55) - (x * 2.55) + (right_speed * 2)) / 3, -255, 255);

  if (left_speed > 0)
  {
    L_MOTOR->run(FORWARD);
  }
  else if (left_speed < 0)
  {
    L_MOTOR->run(BACKWARD);
  }

  if (right_speed > 0)
  {
    R_MOTOR->run(FORWARD);
  }
  else if (right_speed < 0)
  {
    R_MOTOR->run(BACKWARD);
  }

  // Set the motor speed
  L_MOTOR->setSpeed(abs(left_speed));
  R_MOTOR->setSpeed(abs(right_speed));
}

void loop()
{
  if (ble.available())
  {
    int8_t x = ble.read();
    int8_t y = ble.read();
    arcade_drive((double)setDeadband(x, 5), (double)-y);
  }
  else
  {
    L_MOTOR->setSpeed(0);
    R_MOTOR->setSpeed(0);
  }
  delay(100);
}