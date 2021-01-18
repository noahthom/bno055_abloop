#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

//defining sensorData struct which contains linear acceleration and orientation vectors

typedef struct{
  //euclidean acceleration vector components
  int32_t acceleration_x;
  int32_t acceleration_y;
  int32_t acceleration_z;
  //euler orientation vector components
  int32_t orientation_x;
  int32_t orientation_y;
  int32_t orientation_z;
}sensorData;

sensorData data; //declaring struct data of type sensorData



/* Set the delay between fresh samples */
#define BNO055_SAMPLERATE_DELAY_MS (100)

// Check I2C device address and correct line below (by default address is 0x29 or 0x28)
//                                   id, address
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);


void setup() 
{
  Serial.begin(115200); //initialising serial comm at baud rate 115200bps

   /* Initialise the sensor */
  if(!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while(1);
  }

  bno.setExtCrystalUse(true); //initialising ext crystal clk

}

void loop() 
{
   /* Get a new sensor event */
  sensors_event_t event;
  bno.getEvent(&event);
  
  //storing acceleration data in acceleration vector members of struct data
  data.acceleration_x = event.acceleration.x;
  data.acceleration_y = event.acceleration.y;
  data.acceleration_z = event.acceleration.z;

  //storing orientation data in orientation vector members of struct data
  data.orientation_x = event.orientation.x;
  data.orientation_y = event.orientation.y;
  data.orientation_z = event.orientation.z;

    
  Serial.print("Acceleration ");
  Serial.print("X: ");
  Serial.print(event.acceleration.x, 4);
  Serial.print("\tY: ");
  Serial.print(event.acceleration.y, 4);
  Serial.print("\tZ: ");
  Serial.print(event.acceleration.z, 4);

  displayCalStatus();

  Serial.println("");

  
  // delay before data struct is refreshed
  delay(BNO055_SAMPLERATE_DELAY_MS);
  

}


/**************************************************************************/
/*
    Displays some basic information on this sensor from the unified
    sensor API sensor_t type (see Adafruit_Sensor for more information)
*/
/**************************************************************************/
void displaySensorDetails(void)
{
  sensor_t sensor;
  bno.getSensor(&sensor);
  Serial.println("------------------------------------");
  Serial.print  ("Sensor:       "); Serial.println(sensor.name);
  Serial.print  ("Driver Ver:   "); Serial.println(sensor.version);
  Serial.print  ("Unique ID:    "); Serial.println(sensor.sensor_id);
  Serial.print  ("Max Value:    "); Serial.print(sensor.max_value); Serial.println(" xxx");
  Serial.print  ("Min Value:    "); Serial.print(sensor.min_value); Serial.println(" xxx");
  Serial.print  ("Resolution:   "); Serial.print(sensor.resolution); Serial.println(" xxx");
  Serial.println("------------------------------------");
  Serial.println("");
  delay(500);
}

/**************************************************************************/
/*
    Display some basic info about the sensor status
*/
/**************************************************************************/
void displaySensorStatus(void)
{
  /* Get the system status values (mostly for debugging purposes) */
  uint8_t system_status, self_test_results, system_error;
  system_status = self_test_results = system_error = 0;
  bno.getSystemStatus(&system_status, &self_test_results, &system_error);

  /* Display the results in the Serial Monitor */
  Serial.println("");
  Serial.print("System Status: 0x");
  Serial.println(system_status, HEX);
  Serial.print("Self Test:     0x");
  Serial.println(self_test_results, HEX);
  Serial.print("System Error:  0x");
  Serial.println(system_error, HEX);
  Serial.println("");
  delay(500);
}
/**************************************************************************/
/*
    Display sensor calibration status
*/
/**************************************************************************/
void displayCalStatus(void)
{
  /* Get the four calibration values (0..3) */
  /* Any sensor data reporting 0 should be ignored, */
  /* 3 means 'fully calibrated" */
  uint8_t system, gyro, accel, mag;
  system = gyro = accel = mag = 0;
  bno.getCalibration(&system, &gyro, &accel, &mag);

  /* The data should be ignored until the system calibration is > 0 */
  Serial.print("\t");
  if (!system)
  {
    Serial.print("! ");
  }

  /* Display the individual values */
  Serial.print("Sys:");
  Serial.print(system, DEC);
  Serial.print(" G:");
  Serial.print(gyro, DEC);
  Serial.print(" A:");
  Serial.print(accel, DEC);
  Serial.print(" M:");
  Serial.print(mag, DEC);
}
