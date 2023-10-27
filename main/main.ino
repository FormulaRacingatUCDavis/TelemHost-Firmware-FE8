#include <algorithm>
#include <FlexCAN.h>
#include <SD.h>
#include <vector>

#include "xsens_mti.h"      // Main xsens library
#include "xsens_utility.h"  // Needed for quaternion conversion function


//Function prototypes
void writeLine(unsigned long id, uint8_t len, uint8_t* bytes);
void write_message_to_file(CAN_message_t msg);
void dyingGasp();
char* nextFileName();
char* numToChar(int num);
// Callback function used by the xsens library
void imu_callback(XsensEventFlag_t event, XsensEventData_t *mtdata);
void send_wireless_data(CAN_message_t msg);

//Variables and initialization
File file;
int log_num = 0;
bool isRunning = true;
unsigned long gps_data;
xsens_interface_t imu_interface = XSENS_INTERFACE_RX(&imu_callback);

void setup(){
	// LED's
	pinMode(13, INPUT);
	pinMode(24, INPUT);
	//pinMode(27, OUTPUT); // LED0
	//digitalWrite(27, HIGH);
	//pinMode(28, OUTPUT); // LED1
	//pinMode(29, OUTPUT); // LED2
	
	// initialize data protocalls
  Serial.begin(112500);    // serial port UART
	Serial1.begin(2000000);  //xsens uart
  Serial2.begin(9600, SERIAL_8N1);  // ESP32 UART

	//initialize the can buses
	Can0.begin(500000);
	Can1.begin(500000);
	
  delay(2000);

	// see if the card is present and can be initialized:
	SD.begin(BUILTIN_SDCARD);
	//delay(1000);

  Serial2.println("Connected!\n    ");
	file = SD.open(nextFileName(), FILE_WRITE); // helper function nextFileName creates a new file so previous data isn't overwritten
	if(file)
  {
      Serial2.printf("Log file number: %i\n", log_num);
  }

	//Dying Gasp Setup - NEED TO ADD BIGGER CAP, OTHERWISE SD CARD WON'T OPEN
	attachInterrupt(24, dyingGasp, FALLING); //go to dyingGasp function when pin 24 goes from HIGH to LOW  
  Serial2.printf("Log file number: %i\n", log_num);
}

void loop(){
	// boolean to only read data when we want it to
	if(isRunning){
    CAN_message_t msg; 
		// checks the 0th CAN bus to read data
		//Serial.println(Can0.available());
    if(Can0.available()){
      Can0.read(msg);
			write_message_to_file(msg);
      send_can_wireless(msg);
		}
		// check the 1st CAN bus to read data
		if(Can1.available()){
			Can1.read(msg);
      write_message_to_file(msg);
      send_can_wireless(msg);
		}
   
		// Inbound serial data needs to be parsed
    // When a valid packet is decoded, imu_callback() will fire
    if( Serial1.available() > 0 ){  
      xsens_mti_parse(&imu_interface, Serial1.read());
    }
	}
}

// function for recording can data, takes a bus as a parameter and writes that information
void write_message_to_file(CAN_message_t msg){
  if(!file)  
  {
    return;  // exit if SD card is not mounted
  }
  
	unsigned long t = micros(); // records the time it reads the message
	// print the data in a csv format
  
	file.print(msg.id, HEX);
	file.print(",");
	for(byte i = 0; i < msg.len; i++)
  {
		file.print(msg.buf[i]);
		file.print(",");
	}
	for(byte i = (msg.len); i<8; i++)
  {
		file.print(",");
  }
	file.println(t);
  file.flush();  // synchronize file to avoid corruption on power loss
  Serial.println("CAN");
}

void writeLine(uint32_t id, uint8_t len, float* bytes){
  Serial.println("IMU");
  //digitalWrite(29, HIGH);
  unsigned long t = micros(); // records the time it reads the message
  // print the data in a csv format
  file.print(id, HEX);
  file.print(",");
  for(byte i = 0; i < len; i++){
    file.print(bytes[i]);
    file.print(",");
  }
  for(byte i = len; i<8; i++)
    file.print(",");
  file.println(t);
  file.flush();  // synchronize file to avoid corruption on power loss
}

// Called when the library decoded an inbound packet
//   - If the packet was an MData2 frame (which contains packed motion data)
//   - the callback is called once for each sub-field
void imu_callback(XsensEventFlag_t event, XsensEventData_t *mtdata)
{
    // The library provides a pointer to the a union containing decoded data
    // Use XsensEventFlag_t to determine what kind of packet arrived, 
    // then copy data from the union as needed.

    // union
    // {
    //     uint8_t u1;
    //     uint16_t u2;
    //     uint32_t u4;
    //     float    f4;
    //     float    f4x2[2];
    //     float    f4x3[3];
    //     float    f4x4[4];
    //     float    f4x9[9];
    // } data;

    switch( event )
    {
        case XSENS_EVT_DELTA_V: 
          if( mtdata->type == XSENS_EVT_TYPE_FLOAT3 )
            {
                writeLine(0x999, 3, mtdata->data.f4x3);
            }
            break;
        case XSENS_EVT_EULER:
          if( mtdata->type == XSENS_EVT_TYPE_FLOAT3 )
            {
                writeLine(0xAAA, 3, mtdata->data.f4x3);
            }
            break;
        case XSENS_EVT_FREE_ACCELERATION:
          if(mtdata->type == XSENS_EVT_TYPE_FLOAT3)
            {
                writeLine(0xBBB, 3, mtdata->data.f4x3);
            }
            break;
    }
}

void send_wireless_data(CAN_message_t msg)
{
  // arduino IDE gets mad if you declare variables inside the switch
  static int last_vcu_state = -1;
  static int last_mc_state = -1;
  int vcu_state = 0;
  int16_t temp_phaseA = 0;
  int16_t temp_phaseB = 0;
  int16_t temp_phaseC = 0;
  int16_t temp_gate_driver = 0;
  unsigned int torque = 0;

  switch(msg.id)
  {
    case 0x766:  // VCU status
      vcu_state = msg.buf[4]; 
      if(last_vcu_state != vcu_state)
      {
        //Serial2.printf("VCU state transition: %u -> %u\n", last_vcu_state, vcu_state);
        last_vcu_state = vcu_state;
      }
      break;
    case 0x0C0:  // torque request
      torque = ((msg.buf[1] << 8) + msg.buf[0])/10;
      //Serial2.printf("HV Request: %u\n", msg.buf[5]); 
      //Serial2.printf("Torque request: %uNm\n", torque);
      break;
    case 0x380:  // BMS status
     // Serial2.printf("BMS temp: %uC\n", msg.buf[0]);
      break;
    /*
    case 0x0A0: // MC temps 1
      temp_phaseA = ((msg.buf[1] << 8) + msg.buf[0])/10;
      temp_phaseB = ((msg.buf[3] << 8) + msg.buf[2])/10;
      temp_phaseC = ((msg.buf[5] << 8) + msg.buf[4])/10;
      temp_gate_driver = ((msg.buf[7] << 8) + msg.buf[6])/10;
      Serial2.printf("Phase A: %iC, Phase B: %iC, Phase C: %iC, Gate Driver: %iC\n", temp_phaseA, temp_phaseB, temp_phaseC, temp_gate_driver);
      break;
    */
    /*  
    case 0x0A2: // MC temps 3
      temp_phaseA = ((msg.buf[5] << 8) + msg.buf[4])/10;
      Serial2.printf("Motor Temp: %iC\n", temp_phaseA);
    */
    /*case 0x0AA:  // MC internal states
      if(msg.buf[0] == last_mc_state)
      {
        return;
      }
      last_mc_state = msg.buf[0];
      
      Serial2.print("MC State: ");
      switch(msg.buf[0])
      {
        case 0: 
          Serial2.print("Start\n");
          break;
        case 1: 
          Serial2.print("Precharge Init\n");
          break;
        case 2: 
          Serial2.print("Precharge Active\n");
          break;
        case 3:
          Serial2.print("Precharge Complete\n");
          break;
        case 4: 
          Serial2.print("Wait\n");
          break;
        case 5: 
          Serial2.print("Ready\n");
          break;
        case 6: 
          Serial2.print("Motor Running\n");
          break;
        case 7: 
          Serial2.print("Blink Fault Code\n");
          break;
        case 14: 
          Serial2.print("Shutdown in progress\n");
          break;
        case 15: 
          Serial2.print("Recycle power\n");
          break;
        default: 
          Serial2.print("Invalid state!\n");
          break;
      }
      break;
    */
    /*
    case 0x400:
      Serial2.printf("Temps: MC In/Out: %uC/%uC, MTR In/Out: %uC/%uC\n", 
                    msg.buf[0],msg.buf[1],msg.buf[2],msg.buf[3]);
      Serial2.printf("Pressures: MC In/Out: %upsi/%upsi, MTR In/Out: %upsi/%upsi\n", 
                    msg.buf[4],msg.buf[5],msg.buf[6],msg.buf[7]);
                    
      break;
    */
    case 0x401:
      Serial2.printf("Air Temps: MC In/Out: %uC/%uC, MTR In/Out: %uC/%uC\n", 
                    msg.buf[0],msg.buf[1],msg.buf[2],msg.buf[3]);
                    
      break;
    case 0x500: // front wheel speed
      torque = ((msg.buf[0] << 8) + msg.buf[1]);
      Serial2.printf("FWS: %irpm\n", torque);
      break;
    case 0x0A5: // MC motor position
      torque = ((msg.buf[3] << 8) + msg.buf[2]);
      Serial2.printf("RWS: %irpm\n", torque);
      break;
  }
}

// names files 000, 001, etc.
char* nextFileName(){
  //Initialization Stuff
  int fileNum = 1;
  char tempFileName[] = "000.csv";
  char* fileName = (char*) malloc(30 * sizeof(char));
  memcpy ( fileName, tempFileName, 8 );
  File File1;
  char* nums = numToChar(fileNum);
  fileName[0] = nums[0];
  fileName[1] = nums[1];
  fileName[2] = nums[2];

  File1 = SD.open(fileName, FILE_READ);
  //Loops through until it finds an available file name
  while(File1) {
    //Close file
    File1.close();
    //Try to open next File
    fileNum++;
    nums = numToChar(fileNum);
    fileName[0] = nums[0];
    fileName[1] = nums[1];
    fileName[2] = nums[2];
    File1 = SD.open(fileName, FILE_READ);
  }

  log_num = fileNum;
  //Returns the file name;
  //free(nums);
  return fileName;
}

// helper code for the nextFileName function
char* numToChar(int num){
  char nums[] = "0123456789";
  int hundreds = (num - (num%100))/100;
  int tens = ((num - (num%10))/10)%10;
  int ones = num%10;
  char* output = (char*) malloc(5 * sizeof(char));
  output[0] = nums[hundreds];
  output[1] = nums[tens];
  output[2] = nums[ones];
  return output;
}

//close file on interrupt and stop collecting data
void dyingGasp(){
	isRunning = false;
	file.print("\n");
	file.print(0x800);
	file.close();
  //digitalWrite(13, HIGH);
	while(true){} //the shadow realm
}
