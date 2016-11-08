#include "Wire.h"

// I2Cdev and MPU9250 must be installed as libraries, or else the .cpp/.h files
// for both classes must be in the include path of your project
#include "I2Cdev.h"
#include "myMPU9250.h"

// class default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 (default for InvenSense evaluation board)
// AD0 high = 0x69
MPU9250 accelgyro;
I2Cdev   I2C_M;

uint8_t buffer_m[6];
uint8_t buffer_;

int16_t ax, ay, az;
int16_t gx, gy, gz;
int16_t   mx, my, mz;



float heading;
float tiltheading;

float Axyz[3];
float Gxyz[3];
float Mxyz[3];


#define sample_num_mdate  5000      

volatile float mx_sample[3];
volatile float my_sample[3];
volatile float mz_sample[3];

static float mx_centre = -23;
static float my_centre = 28.5;
static float mz_centre = 12.2;

volatile int mx_max =0;
volatile int my_max =0;
volatile int mz_max =0;

volatile int mx_min =0;
volatile int my_min =0;
volatile int mz_min =0;

unsigned long startTime;
unsigned long endTime;
unsigned long elapsedTime;

uint8_t gyroDLPFmode;
uint8_t accDLPFmode;

void setup() {
  // join I2C bus (I2Cdev library doesn't do this automatically)
  Wire.begin();

  // initialize serial communication
  // (38400 chosen because it works as well at 8MHz as it does at 16MHz, but
  // it's really up to you depending on your project)
  Serial.begin(115200);

  // initialize device
  Serial.println("Initializing I2C devices...");
  accelgyro.initialize();

  // verify connection
	Serial.println("Testing device connections...");
	Serial.println(accelgyro.testConnection() ? "MPU9250 connection successful" : "MPU9250 connection failed");

  //set gyro output data rate to 1000hz
  accelgyro.setGyroDLPFMode(1);
  //set gyro range to 500dps.
  accelgyro.setFullScaleGyroRange(1);
  //set accel output data rate to 1000hz
  accelgyro.setAccDLPFMode(1);

  //set i2c bypass enable pin to true to access magnetometer
  I2Cdev::writeByte(0x68, MPU9250_RA_INT_PIN_CFG, 0x02);
  delay(10);
  Serial.println("bypass mode enabled");
  
  
  // TODO set mag to continuous measurement mode.
  I2C_M.writeByte(MPU9250_RA_MAG_ADDRESS, 0x0A, 0x06); //0x06 = 00000110 = continuous measurenment mode 2. 100Hz output data rate.
  Serial.println("mag continuous measurement mode set");
  
	delay(1000);
	Serial.println("     ");
 
	//Mxyz_init_calibrated ();
  
}

void loop() 
{   
  //get gyro data and calculate time spent for getting that data.
  startTime = micros();
  getGyro_Data();

	getAccel_Data();
	getCompassDate_calibrated(); // compass data has been calibrated here 


	Serial.print(Axyz[0]); 
	Serial.print(" ");
	Serial.print(Axyz[1]); 
	Serial.print(" ");
	Serial.print(Axyz[2]); 
  Serial.print(" ");
	Serial.print(Gxyz[0]); 
	Serial.print(" ");
	Serial.print(Gxyz[1]); 
	Serial.print(" ");
	Serial.print(Gxyz[2]); 
  Serial.print(" ");
	Serial.print(Mxyz[0]); 
	Serial.print(" ");
	Serial.print(Mxyz[1]); 
	Serial.print(" ");
	Serial.print(Mxyz[2]);
  Serial.print(" ");

  endTime = micros();
  elapsedTime = endTime-startTime;
  Serial.println(elapsedTime);
  
	delay(10);
}


void getHeading(void)
{
  heading=180*atan2(Mxyz[1],Mxyz[0])/PI;
  if(heading <0) heading +=360;
}

void getTiltHeading(void)
{
  float pitch = asin(-Axyz[0]);
  float roll = asin(Axyz[1]/cos(pitch));

  float xh = Mxyz[0] * cos(pitch) + Mxyz[2] * sin(pitch);
  float yh = Mxyz[0] * sin(roll) * sin(pitch) + Mxyz[1] * cos(roll) - Mxyz[2] * sin(roll) * cos(pitch);
  float zh = -Mxyz[0] * cos(roll) * sin(pitch) + Mxyz[1] * sin(roll) + Mxyz[2] * cos(roll) * cos(pitch);
  tiltheading = 180 * atan2(yh, xh)/PI;
  if(yh<0)    tiltheading +=360;
}



void Mxyz_init_calibrated ()
{
	
	Serial.println(F("Before using 9DOF,we need to calibrate the compass frist,It will takes about 2 minutes."));
	Serial.print("  ");
	Serial.println(F("During  calibratting ,you should rotate and turn the 9DOF all the time within 2 minutes."));
	Serial.print("  ");
	Serial.println(F("If you are ready ,please sent a command data 'ready' to start sample and calibrate."));
	while(!Serial.find("ready"));	
	Serial.println("  ");
	Serial.println("ready");
	Serial.println("Sample starting......");
	Serial.println("waiting ......");
	
	get_calibration_Data ();
	
	Serial.println("     ");
	Serial.println("compass calibration parameter ");
	Serial.print(mx_centre);
	Serial.print("     ");
	Serial.print(my_centre);
	Serial.print("     ");
	Serial.println(mz_centre);
	Serial.println("    ");
}


void get_calibration_Data ()
{
		for (int i=0; i<sample_num_mdate;i++)
			{
			get_one_sample_date_mxyz();
			/*
			Serial.print(mx_sample[2]);
			Serial.print(" ");
			Serial.print(my_sample[2]);                            //you can see the sample data here .
			Serial.print(" ");
			Serial.println(mz_sample[2]);
			*/

      Serial.println(i);
      if(i==0){
        mx_sample[1] = mx_sample[2];
        my_sample[1] = my_sample[2];
        mz_sample[1] = mz_sample[2];
        mx_sample[0] = mx_sample[2];
        my_sample[0] = my_sample[2];
        mz_sample[0] = mz_sample[2];
      }
			
			if (mx_sample[2]>=mx_sample[1])mx_sample[1] = mx_sample[2];			
			if (my_sample[2]>=my_sample[1])my_sample[1] = my_sample[2]; //find max value			
			if (mz_sample[2]>=mz_sample[1])mz_sample[1] = mz_sample[2];		
			
			if (mx_sample[2]<=mx_sample[0])mx_sample[0] = mx_sample[2];
			if (my_sample[2]<=my_sample[0])my_sample[0] = my_sample[2];//find min value
			if (mz_sample[2]<=mz_sample[0])mz_sample[0] = mz_sample[2];
						
			}
			
			mx_max = mx_sample[1];
			my_max = my_sample[1];
			mz_max = mz_sample[1];			
					
			mx_min = mx_sample[0];
			my_min = my_sample[0];
			mz_min = mz_sample[0];
	

	
			mx_centre = (mx_max + mx_min)/2;
			my_centre = (my_max + my_min)/2;
			mz_centre = (mz_max + mz_min)/2;	
	
}






void get_one_sample_date_mxyz()
{		
		getCompass_Data();
		mx_sample[2] = Mxyz[0];
		my_sample[2] = Mxyz[1];
		mz_sample[2] = Mxyz[2];
   
}	


void getAccel_Data(void)
{
  accelgyro.getAcceleration(&ax,&ay,&az);
  Axyz[0] = (double) ax / 16384;//16384  LSB/g
  Axyz[1] = (double) ay / 16384;
  Axyz[2] = (double) az / 16384; 
}

void getGyro_Data(void)
{
  accelgyro.getRotation(&gx, &gy, &gz);
  Gxyz[0] = (double) gx * 500 / 32768;//131 LSB(��/s)
  Gxyz[1] = (double) gy * 500 / 32768;
  Gxyz[2] = (double) gz * 500 / 32768;
}

uint8_t getCompass_Data()
{
  uint8_t dataReady = getCompassDataReady();
  if (dataReady == 1){
    I2C_M.readBytes(MPU9250_RA_MAG_ADDRESS, MPU9250_RA_MAG_XOUT_L, 6, buffer_m);
    I2C_M.readByte(MPU9250_RA_MAG_ADDRESS, 0x09, &buffer_);//read ST2 register as required by magnetometer.Otherwise the data is protected and won't be updated.
    mx = ((int16_t)(buffer_m[1]) << 8) | buffer_m[0] ;
    my = ((int16_t)(buffer_m[3]) << 8) | buffer_m[2] ;
    mz = ((int16_t)(buffer_m[5]) << 8) | buffer_m[4] ;  
    //14 bit output.
    Mxyz[0] = (double) mx * 4912 / 8192;
    Mxyz[1] = (double) my * 4912 / 8192;
    Mxyz[2] = (double) mz * 4912 / 8192;
  }
  //else use the previous data.
  return dataReady;
}

void getCompassDate_calibrated ()
{
	uint8_t dataReady = getCompass_Data();
  if (dataReady == 1){
    Mxyz[0] = Mxyz[0] - mx_centre;
    Mxyz[1] = Mxyz[1] - my_centre;
    Mxyz[2] = Mxyz[2] - mz_centre;  
  }
  //else do nothing.
}

uint8_t getCompassDataReady(){
  I2C_M.readByte(MPU9250_RA_MAG_ADDRESS, MPU9250_RA_MAG_ST1, &buffer_);
  buffer_ = (buffer_&0x01);//remove the front 7 bits.
  return buffer_;
}

