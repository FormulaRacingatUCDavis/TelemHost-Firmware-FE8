#include <algorithm>
#include <FlexCAN.h>
#include <SD.h>
#include <vector>

#include "xsens_mti.h"      // Main xsens library
#include "xsens_utility.h"  // Needed for quaternion conversion function


//Function prototypes
void writeLine(unsigned long id, uint8_t len, uint8_t* bytes);
void canOps(FlexCAN bus);
void dyingGasp();
char* nextFileName();
char* numToChar(int num);
// Callback function used by the xsens library
void imu_callback(XsensEventFlag_t event, XsensEventData_t *mtdata);

//Variables and initialization
const int sdSlot = BUILTIN_SDCARD;
File file;
bool isRunning = true;
CAN_message_t msg;
unsigned long t;
unsigned long gps_data;

xsens_interface_t imu_interface = XSENS_INTERFACE_RX( &imu_callback );

void setup(){
	// LED's
	pinMode(13, OUTPUT); // Teensy's LED
	pinMode(24, INPUT);
	pinMode(27, OUTPUT); // LED0
	digitalWrite(27, HIGH);
	pinMode(28, OUTPUT); // LED1
	pinMode(29, OUTPUT); // LED2
	
	// initialize data protocalls
	Serial1.begin(2000000);  //xsens uart
	//initialize the can buses
	Can0.begin(125000);
	//Can1.begin(1000000);
	
	// see if the card is present and can be initialized:
	SD.begin(sdSlot);
	//delay(1000);
	
	file = SD.open(nextFileName(), FILE_WRITE); // helper function nextFileName creates a new file so previous data isn't overwritten
	if(!file)
		digitalWrite(28, HIGH);
	
	//RF Setup
	// pinMode(RFM95_RST, OUTPUT);
	// digitalWrite(RFM95_RST, HIGH);
	// rf95.init();
	// rf95.setFrequency(RF95_FREQ);
	// rf95.setTxPower(23, false);
	// rf95.setModemConfig(RH_RF95::Bw500Cr45Sf128);

	//Dying Gasp Setup - NEED TO ADD BIGGER CAP, OTHERWISE SD CARD WON'T OPEN
	attachInterrupt(24, dyingGasp, FALLING); //go to dyingGasp function when pin 24 goes from HIGH to LOW  
	/*unsigned long endSetup = micros();
	while((micros() - endSetup) < 10000000){
		if(rf95.recv(datapacket, &len)){
			if(datapacket[0] == 69)
				progMode();
		}
	}*/
}

void loop(){
	// boolean to only read data when we want it to
	if(isRunning){
		// checks the 0th CAN bus to read data
		if(Can0.available()){
			canOps(Can0);
		}
		// check the 1st CAN bus to read data
		if(Can1.available()){
			canOps(Can1);
		}
   
		// Inbound serial data needs to be parsed
    // When a valid packet is decoded, imu_callback() will fire
    if( Serial1.available() > 0 )  
    {  
        xsens_mti_parse(&imu_interface, Serial1.read());
    }
	}
}

void writeLine(uint32_t id, uint8_t len, float* bytes){
  t = micros(); // records the time it reads the message
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
}

// function for recording can data, takes a bus as a parameter and writes that information
void canOps(FlexCAN bus){
	bus.read(msg); // takes what's on the can bus and stores it as a can message
  //writeLine(msg.id, msg.len, msg.buf);
	t = micros(); // records the time it reads the message
	// print the data in a csv format
  
	file.print(msg.id, HEX);
	file.print(",");
	for(byte i = 0; i < msg.len; i++){
		file.print(msg.buf[i]);
		file.print(",");
	}
	for(byte i = (msg.len); i<8; i++)
		file.print(",");
	file.println(t);

	// if(msg.id == filter){
	// 	sendDataToLong(msg.id, msg.len, msg.buf);
	// }
	//digitalWrite(29, HIGH);*/
}

//close file on interrupt and stop collecting data
void dyingGasp(){
	isRunning = false;
	file.print("\n");
	file.print(0x800);
	file.close();
  digitalWrite(13, HIGH);
	while(true){} //the shadow realm
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
                digitalWrite(29, HIGH);
            }
            break;
        case XSENS_EVT_EULER:
          if( mtdata->type == XSENS_EVT_TYPE_FLOAT3 )
            {
                writeLine(0xAAA, 3, mtdata->data.f4x3);
                digitalWrite(29, HIGH);
            }
            break;
    }
}

// packet counter, we increment per xmission
// void sendDataToLong(uint32_t id, uint8_t len, uint8_t* buf) {
// 	digitalWrite(29, HIGH);
// 	//the camparator to extract the rightest 8bits field
// 	unsigned char cmp = 0xFF;
// 	//deal with uint32_t id
// 	radiopacket[0] = id;
// 	radiopacket[1] = id >> 8;
	
// 	for(int i = 1; i >= 0; i--) {
// 		uint8_t c = (id >> i * 8) & cmp;
// 		radiopacket[1-i] = c;
// 	}
	
// 	//deal with uint8_t len
// 	radiopacket[2] = len;
// 	//deal with len * uint8_t len
// 	for (int i = 0; i < len; i++) {
// 		radiopacket[3+i] = buf[i];
// 	}
	
// 	rf95.send(radiopacket, len+3);
// }

// a VERY rough start to programming nodes over CAN
/*
void progMode(){
	while(true){
		digitalWrite(28, HIGH);
		CAN_message_t hostMsg, nodeMsg;
		hostMsg.id = HOSTID;
		hostMsg.len = 4;
		hostMsg.buf [0] = 's';
		hostMsg.buf [1] = 't';
		hostMsg.buf [2] = 'f';
		hostMsg.buf [3] = 'u';
		CANBus1.write(hostMsg);		
		collectBus0IDs();
		hostMsg.buf [0] = 's';
		hostMsg.buf [1] = 'm';
		hostMsg.buf [2] = 'y';
		hostMsg.buf [3] = 's';
		CANBus1.write(hostMsg);
		if(CANBus1.available()){
			CANBus1.read(nodeMsg);
		}
	}
}

void collectBus0IDs(){
	CAN_message_t bus0Msg;
	unsigned long fiveSec = micros();
	while((micros()-fiveSec) < 5000000){
		if(CANBus0.available()){
			CANBus0.read(bus0Msg);
			if(std::find(idList.begin(), idList.end(), bus0Msg.id) == idList.end())
				idList.push_back(bus0Msg.id);
		}
	}
}
*/
