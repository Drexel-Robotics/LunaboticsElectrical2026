#include "I2C.h" 
#include "Wire.h" 

// DEFINES
#define MPU9250_IMU_ADDRESS       0x68 
#define MPU9250_MAG_ADDRESS       0x0C 
#define GYRO_FULL_SCALE_250_DPS   0x00 
#define GYRO_FULL_SCALE_500_DPS   0x08 
#define GYRO_FULL_SCALE_1000_DPS  0x10 
#define GYRO_FULL_SCALE_2000_DPS  0x18 
#define ACC_FULL_SCALE_2G         0x00 
#define ACC_FULL_SCALE_4G         0x08 
#define ACC_FULL_SCALE_8G         0x10 
#define ACC_FULL_SCALE_16G        0x18 
#define TEMPERATURE_OFFSET        21 // As defined in documentation 
#define INTERVAL_MS_PRINT         1000 
#define G                         9.80665 

// Structs
struct gyroscope_raw { 
	 int16_t x, y, z; 
} gyroscope; 

struct accelerometer_raw { 
	 int16_t x, y, z; 
} accelerometer;

struct magnetometer_raw { 
	 int16_t x, y, z; 
	 struct { 
	   int8_t x, y, z; 
	 } adjustment; 
} magnetometer; 

struct temperature_raw { 
	 int16_t value; 
} temperature; 

struct { 
	 struct { 
	   float x, y, z; 
	 } accelerometer, gyroscope, magnetometer; 
	 float temperature; 
} normalized;

// Variables
unsigned long lastPrintMillis = 0;

// Function Prototypes
void normalize(gyroscope_raw gyroscope);
bool isImuReady();
void readRawImu();

void setup() 
{ 
	 Wire.begin(); 
	 Serial.begin(115200); 
	 I2CwriteByte(MPU9250_IMU_ADDRESS, 27, GYRO_FULL_SCALE_1000_DPS); // Configure gyroscope range 
	 I2CwriteByte(MPU9250_IMU_ADDRESS, 28, ACC_FULL_SCALE_2G);        // Configure accelerometer range 
	 I2CwriteByte(MPU9250_IMU_ADDRESS, 55, 0x02); // Set by pass mode for the magnetometers 
	 I2CwriteByte(MPU9250_IMU_ADDRESS, 56, 0x01); // Enable interrupt pin for raw data 
	//  setMagnetometerAdjustmentValues(); 
	//  //Start magnetometer 
	//  I2CwriteByte(MPU9250_MAG_ADDRESS, 0x0A, 0x12); // Request continuous magnetometer measurements in 16 bits (mode 1) 
} 
void loop() 
{ 
	 unsigned long currentMillis = millis(); 
	 if (isImuReady()) { 
	   readRawImu(); 
	   normalize(gyroscope); 
	   // normalize(accelerometer); 
	   normalize(temperature); 
	 } 

	 if (currentMillis - lastPrintMillis > INTERVAL_MS_PRINT) { 
	   Serial.print("TEMPERATURE: "); 
	   Serial.print(normalized.temperature, 2); 
	   Serial.print("C"); 
	   Serial.println(); 

	   Serial.print("GYROSCOPE DATA: "); 
	   Serial.print(normalized.gyroscope.x, 3); 
	   Serial.print("\t"); 
	   Serial.print(normalized.gyroscope.y, 3); 
	   Serial.print("\t"); 
	   Serial.print(normalized.gyroscope.z, 3); 
	   Serial.println(); 

	  //  Serial.print("ACC (m/s^2):\\t"); 
	  //  Serial.print(normalized.accelerometer.x, 3); 
	  //  Serial.print("\\t\\t"); 
	  //  Serial.print(normalized.accelerometer.y, 3); 
	  //  Serial.print("\\t\\t"); 
	  //  Serial.print(normalized.accelerometer.z, 3); 
	  //  Serial.println(); 
	  //  Serial.print("MAG ("); 
	  //  Serial.print("\\xce\\xbc"); //Print micro symbol 
	  //  Serial.print("T):\\t"); 
	  //  Serial.print(normalized.magnetometer.x, 3); 
	  //  Serial.print("\\t\\t"); 
	  //  Serial.print(normalized.magnetometer.y, 3); 
	  //  Serial.print("\\t\\t"); 
	  //  Serial.print(normalized.magnetometer.z, 3); 
	   Serial.println(); 
	   Serial.println(); 
	   lastPrintMillis = currentMillis; 
	 } 
} 

bool isImuReady()
{
  uint8_t isReady; // Interruption flag

  I2Cread(MPU9250_IMU_ADDRESS, 58, 1, &isReady);

  return isReady & 0x01; // Read register and wait for the RAW_DATA_RDY_INT
}

void readRawImu()
{
  uint8_t buff[14];

  // Read output registers:
  // [59-64] Accelerometer
  // [65-66] Temperature
  // [67-72] Gyroscope
  I2Cread(MPU9250_IMU_ADDRESS, 59, 14, buff);

  // Accelerometer, create 16 bits values from 8 bits data
  accelerometer.x = (buff[0] << 8 | buff[1]);
  accelerometer.y = (buff[2] << 8 | buff[3]);
  accelerometer.z = (buff[4] << 8 | buff[5]);

  // Temperature, create 16 bits values from 8 bits data
  temperature.value = (buff[6] << 8 | buff[7]);

  // Gyroscope, create 16 bits values from 8 bits data
  gyroscope.x = (buff[8] << 8 | buff[9]);
  gyroscope.y = (buff[10] << 8 | buff[11]);
  gyroscope.z = (buff[12] << 8 | buff[13]);
}

void normalize(gyroscope_raw gyroscope)
{
  // Sensitivity Scale Factor (MPU datasheet page 8)
  normalized.gyroscope.x = gyroscope.x / 32.8;
  normalized.gyroscope.y = gyroscope.y / 32.8;
  normalized.gyroscope.z = gyroscope.z / 32.8;
}

void normalize(temperature_raw temperature)
{
  // Sensitivity Scale Factor (MPU datasheet page 11) & formula (MPU registers page 33)
  normalized.temperature = ((temperature.value - TEMPERATURE_OFFSET) / 333.87) + TEMPERATURE_OFFSET;
}
