#ifndef IMU_VN100.h
#define IMU_VN100.h

#include <inttypes.h>

#define IMU_BUFFERSIZE 120
#define IMU_BAUD 115200

class IMU_Class
{
	private:			// Internal Variables
	uint8_t IMU_checksum;
	uint8_t	IMU_checksum_calc;
	char buffer[IMU_BUFFERSIZE];
	int bufferidx;
	
	void parse_IMU(void);
	uint8_t parseHex(char c);
	long parsedecimal(char *str,uint8_t num_car);
	long parsenumber(char *str,uint8_t numdec);
	
	public:				// External Variables
	// Methods
	IMU_Class();
	void Init();
	void Read();
	// Properties
	float Yaw;          	// IMU Falman Filtered Yaw Reading
	float YawUpper, YawLower, YawSign;
	float Pitch; 		// IMU Falman Filtered Pitch Reading
	float PitchUpper, PitchLower, PitchSign;
	float Roll;			// IMU Falman Filtered Roll Reading
	float RollUpper, RollLower, RollSign;
	float Q[4] = {0,0,0,0};   // Quarternion measurements
	float Magnetic[3] = {0,0,0};
	float Acceleration[3] = {0,0,0};
	float Angular[3] = {0,0,0};
	float DCM[9] = {0,0,0,0,0,0,0,0,0};
	float Upper, Lower;
	float Sign[13] = {1,1,1,1,1,1,1,1,1,1,1,1,1};
	
	uint8_t Type;       // Type 
	
	uint8_t NewData;    // 1:New IMU Data
	
	uint8_t PrintErrors; // 1: To Print IMU Errors (for debug)
};

extern IMU_Class IMU;

#endif