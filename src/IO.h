// IO file
#include "application.h"

//Default MAX17043 I2C address
#define MAX17043_ADDRESS  0x36

// Register definitions
#define VCELL_REGISTER    0x02
#define SOC_REGISTER    0x04
#define MODE_REGISTER   0x06
#define VERSION_REGISTER  0x08
#define CONFIG_REGISTER   0x0C
#define COMMAND_REGISTER  0xFE

class Power {

	public:
	    Power();
		float getVCell();
		float getSoC();
//		int getVersion();
//		byte getCompensateValue();
//		byte getAlertThreshold();
//		void setAlertThreshold(byte threshold);
//		boolean getAlert();
//		void clearAlert();
//		void reset();
//		void quickStart();
	
	private:

//		void readConfigRegister(byte &MSB, byte &LSB);
		void readRegister(byte startAddress, byte &MSB, byte &LSB);
//		void writeRegister(byte address, byte MSB, byte LSB);
        uint8_t buffer[14];
};
