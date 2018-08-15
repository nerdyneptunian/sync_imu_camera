/*
	IMU.cpp - VectorNav VN-100 library for Arduino
	Code by Luke Nyhof
	This code works with boards based on ATMega168/328 and ATMega1280 (Serial port n)

	This library is free software; you can redistribute it and/or
    modify it under the terms of the GNU Lesser General Public
    License as published by the Free Software Foundation; either
    version 2.1 of the License, or (at your option) any later version.

	IMU configuration : Vectornav Protocol
	Baud rate : 57600
	VN-100 Sentences : 
		$VNYPR : Yaw, Roll, Pitch System Fix Data
		
	Methods:
		Init() : VN-100 Initialization
		Read() : Call this funcion as often as you want to ensure you read the incomming IMU data
	
	Send the request via the serial port to the VN-100(t), the parsing code will capture the 
	response and store it in one of the assiciated variables listed in the header file
	
	Variables:
		Yaw, Pitch, Roll		- Yaw, Pitch and Roll
		Q[0,1,2,3]				- Quarternion measurements
		Magnetic[0,1,2]			- Magnetic [X,Y,Z] measurements
		Acceleration[0,1,2]		- Acceleration [X,Y,Z] measurements
		Angular[0,1,2]			- Acceleration [X,Y,Z] measurements
		DCM[0,1,2,3,4,5,6,7,8]	- Directional Cosine Orientation Matrix
								  =	|0  1  2|
									|3  4  5|
									|6  7  8|
					
	Properties:

*/

#include "IMU_VN100.h"

#include <avr/interrupt.h>
#include "WProgram.h"


// Constructors ////////////////////////////////////////////////////////////////
IMU_Class::IMU_Class()
{
}

// Public Methods //////////////////////////////////////////////////////////////
void IMU_Class::Init(void)
{
	Type = 2;
	IMU_checksum_calc = false;
	bufferidx = 0;
	NewData = 0;
	PrintErrors = 0;	
	
	Serial.begin(57600);					// Initialize serial port
}

// This code doesn´t wait for data, only proccess the data available on serial port
// We can call this function on the main loop (50Hz loop)
// If we get a complete packet this function call parse_IMU() to parse and update the IMU info.
void IMU_Class::Read(void)
{
	char c;
	int numc;
	int i;
	numc = Serial.available();    
  
	if (numc > 0)
	for (i=0; i<numc; i++)
	{
		c = Serial.read();   
		if (c == '$')						// IMU Serial Start
		{                      
			bufferidx = 0;
			buffer[bufferidx++] = c;
			IMU_checksum = 0;
			IMU_checksum_calc = true;
			continue;
		}
		if (c == '\r')						// IMU Serial End
		{                     
			buffer[bufferidx++] = 0;
			parse_IMU();
		}
		else 
		{
			if (bufferidx < (IMU_BUFFERSIZE-1))
			{
				if (c == '*')
					IMU_checksum_calc = false;    // Checksum calculation end
				buffer[bufferidx++] = c;
				if (IMU_checksum_calc)
					IMU_checksum ^= c;            // XOR 
			}
			else
			bufferidx=0;   // Buffer overflow : restart
		}
	}   
}

/****************************************************************
 * 																*
 ****************************************************************/
// Private Methods //////////////////////////////////////////////////////////////
void IMU_Class::parse_IMU(void)
{
	byte IMU_check;
	int YStemp, PStemp, RStemp;
	int temp[13] = {0,0,0,0,0,0,0,0,0,0,0,0,0};
	char *parseptr;									// pointer to parser
	
  /*********************************
  * IMU Roll, Pitch, Yaw Parser    *
  **********************************/
  
	if (strncmp(buffer,"$VNYPR",6)==0)				// Check if sentence begins with $VNYPR    [GPGGA]
	{       		
		if (buffer[bufferidx-4]=='*')					// Check for the "*" character
		{          		
			IMU_check = parseHex(buffer[bufferidx-3])*16 + parseHex(buffer[bufferidx-2]);    // Read the checksums characters
			if (IMU_checksum == IMU_check)				// Checksum validation
			{      							
				YStemp = buffer[7];						// Yaw Sign (+ or -)
				PStemp = buffer[15];					// Pitch Sign (+ or -)
				RStemp = buffer[23];					// Roll Sign (+ or -)
				if (YStemp == 45)						// if equal to - then set modifier to negative
					YawSign = -1;
					else
						YawSign = 1;					// else leave positive
				if (PStemp == 45)						// if equal to - then set modifier to negative
					PitchSign = -1;
					else
						PitchSign = 1;					// else leave positive
				if (RStemp == 45)						// if equal to - then set modifier to negative
					RollSign = -1;
					else
						RollSign = 1;					// else leave positive
			
				NewData = 1;  // New IMU Data				// Flag when new data is available
				parseptr = strchr(buffer, ',')+2;			// look for comma-seperator after the $VNYPR identifier
				
				YawUpper = parsenumber(parseptr,3);        	// get upper (before decimal) numbers
				parseptr = strchr(parseptr, '.')+1;			// look for decimal-seperator
				YawLower = parsenumber(parseptr,2);			// get lower (after decimal) numbers
				Yaw = YawSign*((YawUpper/100) + (YawLower/100000));			// join values together to get whole reading
				parseptr = strchr(parseptr, ',')+2;			// look for comma-seperator

				PitchUpper = parsenumber(parseptr,3);
				parseptr = strchr(parseptr, '.')+1;			// look for decimal-seperator
				PitchLower = parsenumber(parseptr,2);      		// Pitch
				Pitch = PitchSign*((PitchUpper/100) + ( PitchLower/100000));
				parseptr = strchr(parseptr, ',')+2;			// look for comma-seperator

				RollUpper = parsenumber(parseptr,3);
				parseptr = strchr(parseptr, '.')+1;			// look for decimal-seperator
				RollLower = parsenumber(parseptr,2);			// Roll
				Roll = RollSign*((RollUpper/100) + (RollLower/100000));
			}
			else
			{
				if (PrintErrors)
				  Serial.println("IMU ERROR: Checksum error!!");
			}
		}
	}
	// insert code for other VN100 sentences

	// QUARTERNION
	if (strncmp(buffer,"$VNQTN",6)==0)				    // Check if sentence begins with $VNQTN
	{
		if (buffer[bufferidx-4]=='*')					// Check for the "*" character
		{          		
			IMU_check = parseHex(buffer[bufferidx-3])*16 + parseHex(buffer[bufferidx-2]);    // Read the checksums characters
			if (IMU_checksum == IMU_check)				// Checksum validation
			{      				
// CHECK SIGN OF OUTPUT			
				temp[0] = buffer[7];						// Q1 Sign (+ or -)
				temp[1] = buffer[17];					    // Q2 Sign (+ or -)
				temp[2] = buffer[27];					    // Q3 Sign (+ or -)
				temp[3] = buffer[37];  						// Q4 sIGN (+ OR -)
				
				if (temp[0] == 45)						// if equal to - then set modifier to negative
					Sign[0] = -1;
					else
						Sign[0] = 1;					// else leave positive
				if (temp[1] == 45)						// if equal to - then set modifier to negative
					Sign[1] = -1;
					else
						Sign[1] = 1;					// else leave positive
				if (temp[2] == 45)						// if equal to - then set modifier to negative
					Sign[2] = -1;
					else
						Sign[2] = 1;					// else leave positive
				if (temp[3] == 45)						// if equal to - then set modifier to negative
					Sign[3] = -1;
					else
						Sign[3] = 1;					// else leave positive
// END SIGN CHECK
// BEGIN PARSE			
				NewData = 1;  // New IMU Data				// Flag when new data is available
				parseptr = strchr(buffer, ',')+2;			// look for comma-seperator after the $VNYPR identifier
				
				Upper = parsenumber(parseptr,3);        	// get upper (before decimal) numbers
				parseptr = strchr(parseptr, '.')+1;			// look for decimal-seperator
				Lower = parsenumber(parseptr,2);			// get lower (after decimal) numbers
				Q[0] = Sign[0]*((Upper/100) + (Lower/100000));			// join values together to get whole reading
				parseptr = strchr(parseptr, ',')+2;			// look for comma-seperator

				Upper = parsenumber(parseptr,3);
				parseptr = strchr(parseptr, '.')+1;			// look for decimal-seperator
				Lower = parsenumber(parseptr,2);      		 
				Q[1] = Sign[1]*((Upper/100) + (Lower/100000));
				parseptr = strchr(parseptr, ',')+2;			// look for comma-seperator

				Upper = parsenumber(parseptr,3);
				parseptr = strchr(parseptr, '.')+1;			// look for decimal-seperator
				Lower = parsenumber(parseptr,2);			 
				Q[2] = Sign[2]*((Upper/100) + (Lower/100000));
				parseptr = strchr(parseptr, ',')+2;			// look for comma-seperator				
				
				Upper = parsenumber(parseptr,3);
				parseptr = strchr(parseptr, '.')+1;			// look for decimal-seperator
				Lower = parsenumber(parseptr,2);			 
				Q[3] = Sign[3]*((Upper/100) + (Lower/100000));				
// END PARSE
			}
			else
			{
				if (PrintErrors)
				  Serial.println("IMU ERROR: Checksum error!!");
			}
		}
	}
		
	// QUARTERNION AND MAGNETIC
	if (strncmp(buffer,"$VNQTM",6)==0)				    // Check if sentence begins with $VNQTM
	{
		if (buffer[bufferidx-4]=='*')					// Check for the "*" character
		{          		
			IMU_check = parseHex(buffer[bufferidx-3])*16 + parseHex(buffer[bufferidx-2]);    // Read the checksums characters
			if (IMU_checksum == IMU_check)				// Checksum validation
			{      				
// CHECK SIGN OF OUTPUT			
				temp[0] = buffer[7];					// Q1 Sign (+ or -)
				temp[1] = buffer[17];					// Q2 Sign (+ or -)
				temp[2] = buffer[27];					// Q3 Sign (+ or -)
				temp[3] = buffer[37];  					// Q4 Sign (+ or -)
				temp[4] = buffer[47];					// X-Axis Mag. Sign
				temp[5] = buffer[55];					// Y-Axis Mag. Sign
				temp[6] = buffer[63];					// Z-Axis Mag. Sign
				
				if (temp[0] == 45)						// if equal to - then set modifier to negative
					Sign[0] = -1;
					else
						Sign[0] = 1;					// else leave positive
				if (temp[1] == 45)						// if equal to - then set modifier to negative
					Sign[1] = -1;
					else
						Sign[1] = 1;					// else leave positive
				if (temp[2] == 45)						// if equal to - then set modifier to negative
					Sign[2] = -1;
					else
						Sign[2] = 1;					// else leave positive
				if (temp[3] == 45)						// if equal to - then set modifier to negative
					Sign[3] = -1;
					else
						Sign[3] = 1;					// else leave positive
				if (temp[4] == 45)						// if equal to - then set modifier to negative
					Sign[4] = -1;
					else
						Sign[4] = 1;					// else leave positive
				if (temp[5] == 45)						// if equal to - then set modifier to negative
					Sign[5] = -1;
					else
						Sign[5] = 1;					// else leave positive
				if (temp[6] == 45)						// if equal to - then set modifier to negative
					Sign[6] = -1;
					else
						Sign[6] = 1;					// else leave positive
// END SIGN CHECK
// BEGIN PARSE			
				NewData = 1;  // New IMU Data				// Flag when new data is available
				parseptr = strchr(buffer, ',')+2;			// look for comma-seperator after the $VNYPR identifier
				
				Upper = parsenumber(parseptr,3);        	// get upper (before decimal) numbers
				parseptr = strchr(parseptr, '.')+1;			// look for decimal-seperator
				Lower = parsenumber(parseptr,2);			// get lower (after decimal) numbers
				Q[0] = Sign[0]*((Upper/100) + (Lower/100000));			// join values together to get whole reading
				parseptr = strchr(parseptr, ',')+2;			// look for comma-seperator

				Upper = parsenumber(parseptr,3);
				parseptr = strchr(parseptr, '.')+1;			// look for decimal-seperator
				Lower = parsenumber(parseptr,2);      		 
				Q[1] = Sign[1]*((Upper/100) + (Lower/100000));
				parseptr = strchr(parseptr, ',')+2;			// look for comma-seperator

				Upper = parsenumber(parseptr,3);
				parseptr = strchr(parseptr, '.')+1;			// look for decimal-seperator
				Lower = parsenumber(parseptr,2);			 
				Q[2] = Sign[2]*((Upper/100) + (Lower/100000));
				parseptr = strchr(parseptr, ',')+2;			// look for comma-seperator
				
				Upper = parsenumber(parseptr,3);
				parseptr = strchr(parseptr, '.')+1;			// look for decimal-seperator
				Lower = parsenumber(parseptr,2);			 
				Q[3] = Sign[3]*((Upper/100) + (Lower/100000));				
				parseptr = strchr(parseptr, ',')+2;			// look for comma-seperator

				Upper = parsenumber(parseptr,3);
				parseptr = strchr(parseptr, '.')+1;			// look for decimal-seperator
				Lower = parsenumber(parseptr,2);			 
				Magnetic[0] = Sign[4]*((Upper/100) + (Lower/100000));				
				parseptr = strchr(parseptr, ',')+2;			// look for comma-seperator
				
				Upper = parsenumber(parseptr,3);
				parseptr = strchr(parseptr, '.')+1;			// look for decimal-seperator
				Lower = parsenumber(parseptr,2);			 
				Magnetic[1] = Sign[5]*((Upper/100) + (Lower/100000));				
				parseptr = strchr(parseptr, ',')+2;			// look for comma-seperator

				Upper = parsenumber(parseptr,3);
				parseptr = strchr(parseptr, '.')+1;			// look for decimal-seperator
				Lower = parsenumber(parseptr,2);			 
				Magnetic[2] = Sign[6]*((Upper/100) + (Lower/100000));				
// END PARSE
			}
			else
			{
				if (PrintErrors)
				  Serial.println("IMU ERROR: Checksum error!!");
			}
		}
	}
	
	// QUARTERNION AND ACCELERATION
	if (strncmp(buffer,"$VNQTA",6)==0)				    // Check if sentence begins with $VNQTA
	{
		if (buffer[bufferidx-4]=='*')					// Check for the "*" character
		{          		
			IMU_check = parseHex(buffer[bufferidx-3])*16 + parseHex(buffer[bufferidx-2]);    // Read the checksums characters
			if (IMU_checksum == IMU_check)				// Checksum validation
			{      				
// CHECK SIGN OF OUTPUT			
				temp[0] = buffer[7];					// q[0] Sign (+ or -)
				temp[1] = buffer[17];					// q[1] Sign (+ or -)
				temp[2] = buffer[27];					// q[2] Sign (+ or -)
				temp[3] = buffer[37];					// q[3] Sign (scalar term)
				temp[4] = buffer[47];					// X-Axis Acc. Sign
				temp[5] = buffer[55];					// Y-Axis Acc. Sign
				temp[6] = buffer[63];					// Z-Axis Acc. Sign
				
				if (temp[0] == 45)						// if equal to - then set modifier to negative
					Sign[0] = -1;
					else
						Sign[0] = 1;					// else leave positive
				if (temp[1] == 45)						// if equal to - then set modifier to negative
					Sign[1] = -1;
					else
						Sign[1] = 1;					// else leave positive
				if (temp[2] == 45)						// if equal to - then set modifier to negative
					Sign[2] = -1;
					else
						Sign[2] = 1;					// else leave positive
				if (temp[3] == 45)						// if equal to - then set modifier to negative
					Sign[3] = -1;
					else
						Sign[3] = 1;					// else leave positive
				if (temp[4] == 45)						// if equal to - then set modifier to negative
					Sign[4] = -1;
					else
						Sign[4] = 1;					// else leave positive
				if (temp[5] == 45)						// if equal to - then set modifier to negative
					Sign[5] = -1;
					else
						Sign[5] = 1;					// else leave positive
				if (temp[6] == 45)						// if equal to - then set modifier to negative
					Sign[6] = -1;
					else
						Sign[6] = 1;					// else leave positive
// END SIGN CHECK
// BEGIN PARSE			
				NewData = 1;  // New IMU Data				// Flag when new data is available
				parseptr = strchr(buffer, ',')+2;			// look for comma-seperator after the $VNYPR identifier
				
				Upper = parsenumber(parseptr,3);        	// get upper (before decimal) numbers
				parseptr = strchr(parseptr, '.')+1;			// look for decimal-seperator
				Lower = parsenumber(parseptr,2);			// get lower (after decimal) numbers
				Q[0] = Sign[0]*((Upper/100) + (Lower/100000));			// join values together to get whole reading
				parseptr = strchr(parseptr, ',')+2;			// look for comma-seperator

				Upper = parsenumber(parseptr,3);
				parseptr = strchr(parseptr, '.')+1;			// look for decimal-seperator
				Lower = parsenumber(parseptr,2);      		 
				Q[1] = Sign[1]*((Upper/100) + (Lower/100000));
				parseptr = strchr(parseptr, ',')+2;			// look for comma-seperator

				Upper = parsenumber(parseptr,3);
				parseptr = strchr(parseptr, '.')+1;			// look for decimal-seperator
				Lower = parsenumber(parseptr,2);			 
				Q[2] = Sign[2]*((Upper/100) + (Lower/100000));
				parseptr = strchr(parseptr, ',')+2;			// look for comma-seperator
				
				Upper = parsenumber(parseptr,3);
				parseptr = strchr(parseptr, '.')+1;			// look for decimal-seperator
				Lower = parsenumber(parseptr,2);			 
				Q[3] = Sign[3]*((Upper/100) + (Lower/100000));				
				parseptr = strchr(parseptr, ',')+2;			// look for comma-seperator

				Upper = parsenumber(parseptr,3);
				parseptr = strchr(parseptr, '.')+1;			// look for decimal-seperator
				Lower = parsenumber(parseptr,2);			  
				Acceleration[0] = Sign[4]*((Upper/100) + (Lower/100000));				
				parseptr = strchr(parseptr, ',')+2;			// look for comma-seperator
				
				Upper = parsenumber(parseptr,3);
				parseptr = strchr(parseptr, '.')+1;			// look for decimal-seperator
				Lower = parsenumber(parseptr,2);			 
				Acceleration[1] = Sign[5]*((Upper/100) + (Lower/100000));				
				parseptr = strchr(parseptr, ',')+2;			// look for comma-seperator

				Upper = parsenumber(parseptr,3);
				parseptr = strchr(parseptr, '.')+1;			// look for decimal-seperator
				Lower = parsenumber(parseptr,2);			 
				Acceleration[2] = Sign[6]*((Upper/100) + (Lower/100000));				
// END PARSE
			}
			else
			{
				if (PrintErrors)
				  Serial.println("IMU ERROR: Checksum error!!");
			}
		}
	}	
	
	// QUARTERNION AND ANGULAR RATES
	if (strncmp(buffer,"$VNQTR",6)==0)				    // Check if sentence begins with $VNQTR
	{
		if (buffer[bufferidx-4]=='*')					// Check for the "*" character
		{          		
			IMU_check = parseHex(buffer[bufferidx-3])*16 + parseHex(buffer[bufferidx-2]);    // Read the checksums characters
			if (IMU_checksum == IMU_check)				// Checksum validation
			{      				
// CHECK SIGN OF OUTPUT			
				temp[0] = buffer[7];					// q[0] Sign (+ or -)
				temp[1] = buffer[17];					// q[1] Sign (+ or -)
				temp[2] = buffer[27];					// q[2] Sign (+ or -)
				temp[3] = buffer[37];					// q[3] Sign (scalar term)
				temp[4] = buffer[47];					// X-Axis Ang. Sign
				temp[5] = buffer[56];					// Y-Axis Ang. Sign
				temp[6] = buffer[65];					// Z-Axis Ang. Sign
				
				if (temp[0] == 45)						// if equal to - then set modifier to negative
					Sign[0] = -1;
					else
						Sign[0] = 1;					// else leave positive
				if (temp[1] == 45)						// if equal to - then set modifier to negative
					Sign[1] = -1;
					else
						Sign[1] = 1;					// else leave positive
				if (temp[2] == 45)						// if equal to - then set modifier to negative
					Sign[2] = -1;
					else
						Sign[2] = 1;					// else leave positive
				if (temp[3] == 45)						// if equal to - then set modifier to negative
					Sign[3] = -1;
					else
						Sign[3] = 1;					// else leave positive
				if (temp[4] == 45)						// if equal to - then set modifier to negative
					Sign[4] = -1;
					else
						Sign[4] = 1;					// else leave positive
				if (temp[5] == 45)						// if equal to - then set modifier to negative
					Sign[5] = -1;
					else
						Sign[5] = 1;					// else leave positive
				if (temp[6] == 45)						// if equal to - then set modifier to negative
					Sign[6] = -1;
					else
						Sign[6] = 1;					// else leave positive
// END SIGN CHECK
// BEGIN PARSE			
				NewData = 1;  // New IMU Data				// Flag when new data is available
				parseptr = strchr(buffer, ',')+2;			// look for comma-seperator after the $VNYPR identifier
				
				Upper = parsenumber(parseptr,3);        	// get upper (before decimal) numbers
				parseptr = strchr(parseptr, '.')+1;			// look for decimal-seperator
				Lower = parsenumber(parseptr,2);			// get lower (after decimal) numbers
				Q[0] = Sign[0]*((Upper/100) + (Lower/100000));			// join values together to get whole reading
				parseptr = strchr(parseptr, ',')+2;			// look for comma-seperator

				Upper = parsenumber(parseptr,3);
				parseptr = strchr(parseptr, '.')+1;			// look for decimal-seperator
				Lower = parsenumber(parseptr,2);      		 
				Q[1] = Sign[1]*((Upper/100) + (Lower/100000));
				parseptr = strchr(parseptr, ',')+2;			// look for comma-seperator

				Upper = parsenumber(parseptr,3);
				parseptr = strchr(parseptr, '.')+1;			// look for decimal-seperator
				Lower = parsenumber(parseptr,2);			 
				Q[2] = Sign[2]*((Upper/100) + (Lower/100000));
				parseptr = strchr(parseptr, ',')+2;			// look for comma-seperator
				
				Upper = parsenumber(parseptr,3);
				parseptr = strchr(parseptr, '.')+1;			// look for decimal-seperator
				Lower = parsenumber(parseptr,2);			 
				Q[3] = Sign[3]*((Upper/100) + (Lower/100000));				
				parseptr = strchr(parseptr, ',')+2;			// look for comma-seperator

				Upper = parsenumber(parseptr,3);
				parseptr = strchr(parseptr, '.')+1;			// look for decimal-seperator
				Lower = parsenumber(parseptr,2);			 
				Angular[0] = Sign[4]*((Upper/100) + (Lower/100000));				
				parseptr = strchr(parseptr, ',')+2;			// look for comma-seperator
				
				Upper = parsenumber(parseptr,3);
				parseptr = strchr(parseptr, '.')+1;			// look for decimal-seperator
				Lower = parsenumber(parseptr,2);			 
				Angular[1] = Sign[5]*((Upper/100) + (Lower/100000));				
				parseptr = strchr(parseptr, ',')+2;			// look for comma-seperator

				Upper = parsenumber(parseptr,3);
				parseptr = strchr(parseptr, '.')+1;			// look for decimal-seperator
				Lower = parsenumber(parseptr,2);			 
				Angular[2] = Sign[6]*((Upper/100) + (Lower/100000));				
// END PARSE
			}
			else
			{
				if (PrintErrors)
				  Serial.println("IMU ERROR: Checksum error!!");
			}
		}
	}
	
	// QUARTERNION, MAGNETIC AND ACCELERATION
	if (strncmp(buffer,"$VNQMA",6)==0)				    // Check if sentence begins with $VNQMA
	{
		if (buffer[bufferidx-4]=='*')					// Check for the "*" character
		{          		
			IMU_check = parseHex(buffer[bufferidx-3])*16 + parseHex(buffer[bufferidx-2]);    // Read the checksums characters
			if (IMU_checksum == IMU_check)				// Checksum validation
			{      				
// CHECK SIGN OF OUTPUT			
				temp[0] = buffer[7];					// q[0] Sign (+ or -)
				temp[1] = buffer[17];					// q[1] Sign (+ or -)
				temp[2] = buffer[27];					// q[2] Sign (+ or -)
				temp[3] = buffer[37];					// q[3] Sign (scalar term)
				temp[4] = buffer[47];					// X-Axis Mag. Sign
				temp[5] = buffer[55];					// Y-Axis Mag. Sign
				temp[6] = buffer[63];					// Z-Axis Mag. Sign
				temp[7] = buffer[71];					// X-Axis Acc. Sign
				temp[8] = buffer[79];					// Y-Axis Acc. Sign
				temp[9] = buffer[87];					// Z-Axis Acc. Sign
				
				if (temp[0] == 45)						// if equal to - then set modifier to negative
					Sign[0] = -1;
					else
						Sign[0] = 1;					// else leave positive
				if (temp[1] == 45)						// if equal to - then set modifier to negative
					Sign[1] = -1;
					else
						Sign[1] = 1;					// else leave positive
				if (temp[2] == 45)						// if equal to - then set modifier to negative
					Sign[2] = -1;
					else
						Sign[2] = 1;					// else leave positive
				if (temp[3] == 45)						// if equal to - then set modifier to negative
					Sign[3] = -1;
					else
						Sign[3] = 1;					// else leave positive
				if (temp[4] == 45)						// if equal to - then set modifier to negative
					Sign[4] = -1;
					else
						Sign[4] = 1;					// else leave positive
				if (temp[5] == 45)						// if equal to - then set modifier to negative
					Sign[5] = -1;
					else
						Sign[5] = 1;					// else leave positive
				if (temp[6] == 45)						// if equal to - then set modifier to negative
					Sign[6] = -1;
					else
						Sign[6] = 1;					// else leave positive
				if (temp[7] == 45)						// if equal to - then set modifier to negative
					Sign[7] = -1;
					else
						Sign[7] = 1;					// else leave positive
				if (temp[8] == 45)						// if equal to - then set modifier to negative
					Sign[8] = -1;
					else
						Sign[8] = 1;					// else leave positive
				if (temp[9] == 45)						// if equal to - then set modifier to negative
					Sign[9] = -1;
					else
						Sign[9] = 1;					// else leave positive
						
// END SIGN CHECK
// BEGIN PARSE			
				NewData = 1;  // New IMU Data				// Flag when new data is available
				parseptr = strchr(buffer, ',')+2;			// look for comma-seperator after the $VNYPR identifier
				
				Upper = parsenumber(parseptr,3);        	// get upper (before decimal) numbers
				parseptr = strchr(parseptr, '.')+1;			// look for decimal-seperator
				Lower = parsenumber(parseptr,2);			// get lower (after decimal) numbers
				Q[0] = Sign[0]*((Upper/100) + (Lower/100000));			// join values together to get whole reading
				parseptr = strchr(parseptr, ',')+2;			// look for comma-seperator

				Upper = parsenumber(parseptr,3);
				parseptr = strchr(parseptr, '.')+1;			// look for decimal-seperator
				Lower = parsenumber(parseptr,2);      		 
				Q[1] = Sign[1]*((Upper/100) + (Lower/100000));
				parseptr = strchr(parseptr, ',')+2;			// look for comma-seperator

				Upper = parsenumber(parseptr,3);
				parseptr = strchr(parseptr, '.')+1;			// look for decimal-seperator
				Lower = parsenumber(parseptr,2);			 
				Q[2] = Sign[2]*((Upper/100) + (Lower/100000));
				parseptr = strchr(parseptr, ',')+2;			// look for comma-seperator
				
				Upper = parsenumber(parseptr,3);
				parseptr = strchr(parseptr, '.')+1;			// look for decimal-seperator
				Lower = parsenumber(parseptr,2);			 
				Q[3] = Sign[3]*((Upper/100) + (Lower/100000));				
				parseptr = strchr(parseptr, ',')+2;			// look for comma-seperator

				Upper = parsenumber(parseptr,3);
				parseptr = strchr(parseptr, '.')+1;			// look for decimal-seperator
				Lower = parsenumber(parseptr,2);		 
				Magnetic[0] = Sign[4]*((Upper/100) + (Lower/100000));				
				parseptr = strchr(parseptr, ',')+2;			// look for comma-seperator
				
				Upper = parsenumber(parseptr,3);
				parseptr = strchr(parseptr, '.')+1;			// look for decimal-seperator
				Lower = parsenumber(parseptr,2);			 
				Magnetic[1] = Sign[5]*((Upper/100) + (Lower/100000));				
				parseptr = strchr(parseptr, ',')+2;			// look for comma-seperator

				Upper = parsenumber(parseptr,3);
				parseptr = strchr(parseptr, '.')+1;			// look for decimal-seperator
				Lower = parsenumber(parseptr,2);		 
				Magnetic[2] = Sign[6]*((Upper/100) + (Lower/100000));				
				parseptr = strchr(parseptr, ',')+2;			// look for comma-seperator

				Upper = parsenumber(parseptr,3);
				parseptr = strchr(parseptr, '.')+1;			// look for decimal-seperator
				Lower = parsenumber(parseptr,2);			 
				Acceleration[0] = Sign[7]*((Upper/100) + (Lower/100000));				
				parseptr = strchr(parseptr, ',')+2;			// look for comma-seperator

				Upper = parsenumber(parseptr,3);
				parseptr = strchr(parseptr, '.')+1;			// look for decimal-seperator
				Lower = parsenumber(parseptr,2);			 
				Acceleration[1] = Sign[8]*((Upper/100) + (Lower/100000));				
				parseptr = strchr(parseptr, ',')+2;			// look for comma-seperator				
				
				Upper = parsenumber(parseptr,3);
				parseptr = strchr(parseptr, '.')+1;			// look for decimal-seperator
				Lower = parsenumber(parseptr,2);			 
				Acceleration[2] = Sign[9]*((Upper/100) + (Lower/100000));				
// END PARSE
			}
			else
			{
				if (PrintErrors)
				  Serial.println("IMU ERROR: Checksum error!!");
			}
		}
	}
	
	// QUARTERNION, ACCELERATION AND ANGULAR RATES
	if (strncmp(buffer,"$VNQAR",6)==0)				    // Check if sentence begins with $VNQAR
	{
		if (buffer[bufferidx-4]=='*')					// Check for the "*" character
		{          		
			IMU_check = parseHex(buffer[bufferidx-3])*16 + parseHex(buffer[bufferidx-2]);    // Read the checksums characters
			if (IMU_checksum == IMU_check)				// Checksum validation
			{      				
// CHECK SIGN OF OUTPUT			
				temp[0] = buffer[7];					// q[0] Sign (+ or -)
				temp[1] = buffer[17];					// q[1] Sign (+ or -)
				temp[2] = buffer[27];					// q[2] Sign (+ or -)
				temp[3] = buffer[37];					// q[3] Sign (scalar term)
				temp[4] = buffer[47];					// X-Axis Acc. Sign
				temp[5] = buffer[55];					// Y-Axis Acc. Sign
				temp[6] = buffer[63];					// Z-Axis Acc. Sign
				temp[7] = buffer[71];					// X-Axis Ang. Sign
				temp[8] = buffer[80];					// Y-Axis Ang. Sign
				temp[9] = buffer[89];					// Z-Axis Ang. Sign
				
				if (temp[0] == 45)						// if equal to - then set modifier to negative
					Sign[0] = -1;
					else
						Sign[0] = 1;					// else leave positive
				if (temp[1] == 45)						// if equal to - then set modifier to negative
					Sign[1] = -1;
					else
						Sign[1] = 1;					// else leave positive
				if (temp[2] == 45)						// if equal to - then set modifier to negative
					Sign[2] = -1;
					else
						Sign[2] = 1;					// else leave positive
				if (temp[3] == 45)						// if equal to - then set modifier to negative
					Sign[3] = -1;
					else
						Sign[3] = 1;					// else leave positive
				if (temp[4] == 45)						// if equal to - then set modifier to negative
					Sign[4] = -1;
					else
						Sign[4] = 1;					// else leave positive
				if (temp[5] == 45)						// if equal to - then set modifier to negative
					Sign[5] = -1;
					else
						Sign[5] = 1;					// else leave positive
				if (temp[6] == 45)						// if equal to - then set modifier to negative
					Sign[6] = -1;
					else
						Sign[6] = 1;					// else leave positive
				if (temp[7] == 45)						// if equal to - then set modifier to negative
					Sign[7] = -1;
					else
						Sign[7] = 1;					// else leave positive
				if (temp[8] == 45)						// if equal to - then set modifier to negative
					Sign[8] = -1;
					else
						Sign[8] = 1;					// else leave positive					
				if (temp[9] == 45)						// if equal to - then set modifier to negative
					Sign[9] = -1;
					else
						Sign[9] = 1;					// else leave positive								
						
// END SIGN CHECK
// BEGIN PARSE			
				NewData = 1;  // New IMU Data				// Flag when new data is available
				parseptr = strchr(buffer, ',')+2;			// look for comma-seperator after the $VNYPR identifier
				
				Upper = parsenumber(parseptr,3);        	// get upper (before decimal) numbers
				parseptr = strchr(parseptr, '.')+1;			// look for decimal-seperator
				Lower = parsenumber(parseptr,2);			// get lower (after decimal) numbers
				Q[0] = Sign[0]*((Upper/100) + (Lower/100000));			// join values together to get whole reading
				parseptr = strchr(parseptr, ',')+2;			// look for comma-seperator

				Upper = parsenumber(parseptr,3);
				parseptr = strchr(parseptr, '.')+1;			// look for decimal-seperator
				Lower = parsenumber(parseptr,2);      		 
				Q[1] = Sign[1]*((Upper/100) + (Lower/100000));
				parseptr = strchr(parseptr, ',')+2;			// look for comma-seperator

				Upper = parsenumber(parseptr,3);
				parseptr = strchr(parseptr, '.')+1;			// look for decimal-seperator
				Lower = parsenumber(parseptr,2);		 
				Q[2] = Sign[2]*((Upper/100) + (Lower/100000));
				parseptr = strchr(parseptr, ',')+2;			// look for comma-seperator
				
				Upper = parsenumber(parseptr,3);
				parseptr = strchr(parseptr, '.')+1;			// look for decimal-seperator
				Lower = parsenumber(parseptr,2);			 
				Q[3] = Sign[3]*((Upper/100) + (Lower/100000));				
				parseptr = strchr(parseptr, ',')+2;			// look for comma-seperator

				Upper = parsenumber(parseptr,3);
				parseptr = strchr(parseptr, '.')+1;			// look for decimal-seperator
				Lower = parsenumber(parseptr,2); 
				Acceleration[0] = Sign[4]*((Upper/100) + (Lower/100000));				
				parseptr = strchr(parseptr, ',')+2;			// look for comma-seperator
				
				Upper = parsenumber(parseptr,3);
				parseptr = strchr(parseptr, '.')+1;			// look for decimal-seperator
				Lower = parsenumber(parseptr,2);			 
				Acceleration[1] = Sign[5]*((Upper/100) + (Lower/100000));				
				parseptr = strchr(parseptr, ',')+2;			// look for comma-seperator

				Upper = parsenumber(parseptr,3);
				parseptr = strchr(parseptr, '.')+1;			// look for decimal-seperator
				Lower = parsenumber(parseptr,2);			 
				Acceleration[2] = Sign[6]*((Upper/100) + (Lower/100000));				
				parseptr = strchr(parseptr, ',')+2;			// look for comma-seperator
				
				Upper = parsenumber(parseptr,3);
				parseptr = strchr(parseptr, '.')+1;			// look for decimal-seperator
				Lower = parsenumber(parseptr,2);			 
				Angular[0] = Sign[7]*((Upper/100) + (Lower/100000));				
				parseptr = strchr(parseptr, ',')+2;			// look for comma-seperator
				
				Upper = parsenumber(parseptr,3);
				parseptr = strchr(parseptr, '.')+1;			// look for decimal-seperator
				Lower = parsenumber(parseptr,2);			 
				Angular[1] = Sign[8]*((Upper/100) + (Lower/100000));				
				parseptr = strchr(parseptr, ',')+2;			// look for comma-seperator

				Upper = parsenumber(parseptr,3);
				parseptr = strchr(parseptr, '.')+1;			// look for decimal-seperator
				Lower = parsenumber(parseptr,2);			 
				Angular[2] = Sign[9]*((Upper/100) + (Lower/100000));				
// END PARSE
			}
			else
			{
				if (PrintErrors)
				  Serial.println("IMU ERROR: Checksum error!!");
			}
		}
	}	
	
	// QUARTERNION, MAGNETIC, ACCELERATION, AND ANGULAR RATES
	if (strncmp(buffer,"$VNQMR",6)==0)				    // Check if sentence begins with $VNQMR
	{
		if (buffer[bufferidx-4]=='*')					// Check for the "*" character
		{          		
			IMU_check = parseHex(buffer[bufferidx-3])*16 + parseHex(buffer[bufferidx-2]);    // Read the checksums characters
			if (IMU_checksum == IMU_check)				// Checksum validation
			{      				
// CHECK SIGN OF OUTPUT			
				temp[0] = buffer[7];					// q[0] Sign (+ or -)
				temp[1] = buffer[17];					// q[1] Sign (+ or -)
				temp[2] = buffer[27];					// q[2] Sign (+ or -)
				temp[3] = buffer[37];					// q[3] Sign (scalar term)
				temp[4] = buffer[47];					// X-Axis Mag. Sign
				temp[5] = buffer[55];					// Y-Axis Mag. Sign
				temp[6] = buffer[63];					// Z-Axis Mag. Sign
				temp[7] = buffer[71];					// X-Axis Acc. Sign
				temp[8] = buffer[79];					// Y-Axis Acc. Sign
				temp[9] = buffer[87];					// Z-Axis Acc. Sign
				temp[10] = buffer[95];					// X-Axis Ang. Sign
				temp[11] = buffer[104];					// Y-Axis Ang. Sign
				temp[12] = buffer[113];					// Z-Axis Ang. Sign
				
				if (temp[0] == 45)						// if equal to - then set modifier to negative
					Sign[0] = -1;
					else
						Sign[0] = 1;					// else leave positive
				if (temp[1] == 45)						// if equal to - then set modifier to negative
					Sign[1] = -1;
					else
						Sign[1] = 1;					// else leave positive
				if (temp[2] == 45)						// if equal to - then set modifier to negative
					Sign[2] = -1;
					else
						Sign[2] = 1;					// else leave positive
				if (temp[3] == 45)						// if equal to - then set modifier to negative
					Sign[3] = -1;
					else
						Sign[3] = 1;					// else leave positive
				if (temp[4] == 45)						// if equal to - then set modifier to negative
					Sign[4] = -1;
					else
						Sign[4] = 1;					// else leave positive
				if (temp[5] == 45)						// if equal to - then set modifier to negative
					Sign[5] = -1;
					else
						Sign[5] = 1;					// else leave positive
				if (temp[6] == 45)						// if equal to - then set modifier to negative
					Sign[6] = -1;
					else
						Sign[6] = 1;					// else leave positive
				if (temp[7] == 45)						// if equal to - then set modifier to negative
					Sign[7] = -1;
					else
						Sign[7] = 1;					// else leave positive
				if (temp[8] == 45)						// if equal to - then set modifier to negative
					Sign[8] = -1;
					else
						Sign[8] = 1;					// else leave positive
				if (temp[9] == 45)						// if equal to - then set modifier to negative
					Sign[9] = -1;
					else
						Sign[9] = 1;					// else leave positive
				if (temp[10] == 45)						// if equal to - then set modifier to negative
					Sign[10] = -1;
					else
						Sign[10] = 1;					// else leave positive
				if (temp[11] == 45)						// if equal to - then set modifier to negative
					Sign[11] = -1;
					else
						Sign[11] = 1;					// else leave positive
				if (temp[12] == 45)						// if equal to - then set modifier to negative
					Sign[12] = -1;
					else
						Sign[12] = 1;					// else leave positive						
// END SIGN CHECK
// BEGIN PARSE			
				NewData = 1;  // New IMU Data				// Flag when new data is available
				parseptr = strchr(buffer, ',')+2;			// look for comma-seperator after the $VNYPR identifier
				
				Upper = parsenumber(parseptr,3);        	// get upper (before decimal) numbers
				parseptr = strchr(parseptr, '.')+1;			// look for decimal-seperator
				Lower = parsenumber(parseptr,2);			// get lower (after decimal) numbers
				Q[0] = Sign[0]*((Upper/100) + (Lower/100000));			// join values together to get whole reading
				parseptr = strchr(parseptr, ',')+2;			// look for comma-seperator

				Upper = parsenumber(parseptr,3);
				parseptr = strchr(parseptr, '.')+1;			// look for decimal-seperator
				Lower = parsenumber(parseptr,2);      		 
				Q[1] = Sign[1]*((Upper/100) + (Lower/100000));
				parseptr = strchr(parseptr, ',')+2;			// look for comma-seperator

				Upper = parsenumber(parseptr,3);
				parseptr = strchr(parseptr, '.')+1;			// look for decimal-seperator
				Lower = parsenumber(parseptr,2);			 
				Q[2] = Sign[2]*((Upper/100) + (Lower/100000));
				parseptr = strchr(parseptr, ',')+2;			// look for comma-seperator
				
				Upper = parsenumber(parseptr,3);
				parseptr = strchr(parseptr, '.')+1;			// look for decimal-seperator
				Lower = parsenumber(parseptr,2);			 
				Q[3] = Sign[3]*((Upper/100) + (Lower/100000));				
				parseptr = strchr(parseptr, ',')+2;			// look for comma-seperator

				Upper = parsenumber(parseptr,3);
				parseptr = strchr(parseptr, '.')+1;			// look for decimal-seperator
				Lower = parsenumber(parseptr,2);			 
				Magnetic[0] = Sign[4]*((Upper/100) + (Lower/100000));				
				parseptr = strchr(parseptr, ',')+2;			// look for comma-seperator
				
				Upper = parsenumber(parseptr,3);
				parseptr = strchr(parseptr, '.')+1;			// look for decimal-seperator
				Lower = parsenumber(parseptr,2);			 
				Magnetic[1] = Sign[5]*((Upper/100) + (Lower/100000));				
				parseptr = strchr(parseptr, ',')+2;			// look for comma-seperator

				Upper = parsenumber(parseptr,3);
				parseptr = strchr(parseptr, '.')+1;			// look for decimal-seperator
				Lower = parsenumber(parseptr,2);			 
				Magnetic[2] = Sign[6]*((Upper/100) + (Lower/100000));				
				parseptr = strchr(parseptr, ',')+2;			// look for comma-seperator
				
				Upper = parsenumber(parseptr,3);
				parseptr = strchr(parseptr, '.')+1;			// look for decimal-seperator
				Lower = parsenumber(parseptr,2);			 
				Acceleration[0] = Sign[7]*((Upper/100) + (Lower/100000));				
				parseptr = strchr(parseptr, ',')+2;			// look for comma-seperator
				
				Upper = parsenumber(parseptr,3);
				parseptr = strchr(parseptr, '.')+1;			// look for decimal-seperator
				Lower = parsenumber(parseptr,2);			 
				Acceleration[1] = Sign[8]*((Upper/100) + (Lower/100000));				
				parseptr = strchr(parseptr, ',')+2;			// look for comma-seperator
				
				Upper = parsenumber(parseptr,3);
				parseptr = strchr(parseptr, '.')+1;			// look for decimal-seperator
				Lower = parsenumber(parseptr,2);			 
				Acceleration[2] = Sign[9]*((Upper/100) + (Lower/100000));				
				parseptr = strchr(parseptr, ',')+2;			// look for comma-seperator
				
				Upper = parsenumber(parseptr,3);
				parseptr = strchr(parseptr, '.')+1;			// look for decimal-seperator
				Lower = parsenumber(parseptr,2);			 
				Angular[0] = Sign[10]*((Upper/100) + (Lower/100000));				
				parseptr = strchr(parseptr, ',')+2;			// look for comma-seperator
				
				Upper = parsenumber(parseptr,3);
				parseptr = strchr(parseptr, '.')+1;			// look for decimal-seperator
				Lower = parsenumber(parseptr,2);			 
				Angular[1] = Sign[10]*((Upper/100) + (Lower/100000));				
				parseptr = strchr(parseptr, ',')+2;			// look for comma-seperator
				
				Upper = parsenumber(parseptr,3);
				parseptr = strchr(parseptr, '.')+1;			// look for decimal-seperator
				Lower = parsenumber(parseptr,2);			 
				Angular[2] = Sign[10]*((Upper/100) + (Lower/100000));				
// END PARSE
			}
			else
			{
				if (PrintErrors)
				  Serial.println("IMU ERROR: Checksum error!!");
			}
		}
	}
	
	// ATTITUDE [DCM]
	// Doesn't deal with the scientific notation... yet
	if (strncmp(buffer,"$VNDCM",6)==0)				    // Check if sentence begins with $VNDCM
	{
		if (buffer[bufferidx-4]=='*')					// Check for the "*" character
		{          		
			IMU_check = parseHex(buffer[bufferidx-3])*16 + parseHex(buffer[bufferidx-2]);    // Read the checksums characters
			if (IMU_checksum == IMU_check)				// Checksum validation
			{      				
// CHECK SIGN OF OUTPUT			
				temp[0] = buffer[7];					// q[0] Sign (+ or -)
				temp[1] = buffer[21];					// q[1] Sign (+ or -)
				temp[2] = buffer[35];					// q[2] Sign (+ or -)
				temp[3] = buffer[47];					// q[3] Sign (scalar term)
				temp[4] = buffer[63];					// X-Axis Ang. Sign
				temp[5] = buffer[77];					// Y-Axis Ang. Sign
				temp[6] = buffer[91];					// Z-Axis Ang. Sign
				temp[7] = buffer[105];					// Z-Axis Ang. Sign
				temp[8] = buffer[119];					// Z-Axis Ang. Sign
				
				if (temp[0] == 45)						// if equal to - then set modifier to negative
					Sign[0] = -1;
					else
						Sign[0] = 1;					// else leave positive
				if (temp[1] == 45)						// if equal to - then set modifier to negative
					Sign[1] = -1;
					else
						Sign[1] = 1;					// else leave positive
				if (temp[2] == 45)						// if equal to - then set modifier to negative
					Sign[2] = -1;
					else
						Sign[2] = 1;					// else leave positive
				if (temp[3] == 45)						// if equal to - then set modifier to negative
					Sign[3] = -1;
					else
						Sign[3] = 1;					// else leave positive
				if (temp[4] == 45)						// if equal to - then set modifier to negative
					Sign[4] = -1;
					else
						Sign[4] = 1;					// else leave positive
				if (temp[5] == 45)						// if equal to - then set modifier to negative
					Sign[5] = -1;
					else
						Sign[5] = 1;					// else leave positive
				if (temp[6] == 45)						// if equal to - then set modifier to negative
					Sign[6] = -1;
					else
						Sign[6] = 1;					// else leave positive
				if (temp[7] == 45)						// if equal to - then set modifier to negative
					Sign[7] = -1;
					else
						Sign[7] = 1;					// else leave positive
				if (temp[8] == 45)						// if equal to - then set modifier to negative
					Sign[8] = -1;
					else
						Sign[8] = 1;					// else leave positive						
				if (temp[9] == 45)						// if equal to - then set modifier to negative
					Sign[9] = -1;
					else
						Sign[9] = 1;					// else leave positive						
// END SIGN CHECK
// BEGIN PARSE			
				NewData = 1;  // New IMU Data				// Flag when new data is available
				parseptr = strchr(buffer, ',')+2;			// look for comma-seperator after the $VNYPR identifier
				
				Upper = parsenumber(parseptr,3);        	// get upper (before decimal) numbers
				parseptr = strchr(parseptr, '.')+1;			// look for decimal-seperator
				Lower = parsenumber(parseptr,2);			// get lower (after decimal) numbers
				DCM[0] = Sign[0]*((Upper/100) + (Lower/100000));			// join values together to get whole reading
				parseptr = strchr(parseptr, ',')+2;			// look for comma-seperator

				Upper = parsenumber(parseptr,3);
				parseptr = strchr(parseptr, '.')+1;			// look for decimal-seperator
				Lower = parsenumber(parseptr,2);      		 
				DCM[1] = Sign[1]*((Upper/100) + (Lower/100000));
				parseptr = strchr(parseptr, ',')+2;			// look for comma-seperator

				Upper = parsenumber(parseptr,3);
				parseptr = strchr(parseptr, '.')+1;			// look for decimal-seperator
				Lower = parsenumber(parseptr,2);			 
				DCM[2] = Sign[2]*((Upper/100) + (Lower/100000));
				parseptr = strchr(parseptr, ',')+2;			// look for comma-seperator
				
				Upper = parsenumber(parseptr,3);
				parseptr = strchr(parseptr, '.')+1;			// look for decimal-seperator
				Lower = parsenumber(parseptr,2);			 
				DCM[3] = Sign[3]*((Upper/100) + (Lower/100000));				
				parseptr = strchr(parseptr, ',')+2;			// look for comma-seperator

				Upper = parsenumber(parseptr,3);
				parseptr = strchr(parseptr, '.')+1;			// look for decimal-seperator
				Lower = parsenumber(parseptr,2);			 
				DCM[4] = Sign[4]*((Upper/100) + (Lower/100000));				
				parseptr = strchr(parseptr, ',')+2;			// look for comma-seperator
				
				Upper = parsenumber(parseptr,3);
				parseptr = strchr(parseptr, '.')+1;			// look for decimal-seperator
				Lower = parsenumber(parseptr,2);			 
				DCM[5] = Sign[5]*((Upper/100) + (Lower/100000));				
				parseptr = strchr(parseptr, ',')+2;			// look for comma-seperator

				Upper = parsenumber(parseptr,3);
				parseptr = strchr(parseptr, '.')+1;			// look for decimal-seperator
				Lower = parsenumber(parseptr,2);			 
				DCM[6] = Sign[6]*((Upper/100) + (Lower/100000));				
				parseptr = strchr(parseptr, ',')+2;			// look for comma-seperator
				
				Upper = parsenumber(parseptr,3);
				parseptr = strchr(parseptr, '.')+1;			// look for decimal-seperator
				Lower = parsenumber(parseptr,2); 
				DCM[7] = Sign[7]*((Upper/100) + (Lower/100000));				
				parseptr = strchr(parseptr, ',')+2;			// look for comma-seperator
				
				Upper = parsenumber(parseptr,3);
				parseptr = strchr(parseptr, '.')+1;			// look for decimal-seperator
				Lower = parsenumber(parseptr,2);			 
				DCM[8] = Sign[8]*((Upper/100) + (Lower/100000));				
				parseptr = strchr(parseptr, ',')+2;			// look for comma-seperator
				
				Upper = parsenumber(parseptr,3);
				parseptr = strchr(parseptr, '.')+1;			// look for decimal-seperator
				Lower = parsenumber(parseptr,2);			 
				DCM[9] = Sign[9]*((Upper/100) + (Lower/100000));				
// END PARSE
			}
			else
			{
				if (PrintErrors)
				  Serial.println("IMU ERROR: Checksum error!!");
			}
		}
	}
	
	// MAGNETIC MEASUREMENTS
	if (strncmp(buffer,"$VNMAG",6)==0)				    // Check if sentence begins with $VNMAG
	{
		if (buffer[bufferidx-4]=='*')					// Check for the "*" character
		{          		
			IMU_check = parseHex(buffer[bufferidx-3])*16 + parseHex(buffer[bufferidx-2]);    // Read the checksums characters
			if (IMU_checksum == IMU_check)				// Checksum validation
			{      				
// CHECK SIGN OF OUTPUT			
				temp[0] = buffer[7];					// X-Axis Mag. Sign (+ or -)
				temp[1] = buffer[15];					// Y-Axis Mag. Sign (+ or -)
				temp[2] = buffer[23];					// Z-Axis Mag. Sign (+ or -)
				
				if (temp[0] == 45)						// if equal to - then set modifier to negative
					Sign[0] = -1;
					else
						Sign[0] = 1;					// else leave positive
				if (temp[1] == 45)						// if equal to - then set modifier to negative
					Sign[1] = -1;
					else
						Sign[1] = 1;					// else leave positive
				if (temp[2] == 45)						// if equal to - then set modifier to negative
					Sign[2] = -1;
					else
						Sign[2] = 1;					// else leave positive
// END SIGN CHECK
// BEGIN PARSE			
				NewData = 1;  // New IMU Data				// Flag when new data is available
				parseptr = strchr(buffer, ',')+2;			// look for comma-seperator after the $VNYPR identifier
				
				Upper = parsenumber(parseptr,3);        	// get upper (before decimal) numbers
				parseptr = strchr(parseptr, '.')+1;			// look for decimal-seperator
				Lower = parsenumber(parseptr,2);			// get lower (after decimal) numbers
				Magnetic[0] = Sign[0]*((Upper/100) + (Lower/100000));			// join values together to get whole reading
				parseptr = strchr(parseptr, ',')+2;			// look for comma-seperator

				Upper = parsenumber(parseptr,3);
				parseptr = strchr(parseptr, '.')+1;			// look for decimal-seperator
				Lower = parsenumber(parseptr,2);      		 
				Magnetic[1] = Sign[1]*((Upper/100) + (Lower/100000));
				parseptr = strchr(parseptr, ',')+2;			// look for comma-seperator

				Upper = parsenumber(parseptr,3);
				parseptr = strchr(parseptr, '.')+1;			// look for decimal-seperator
				Lower = parsenumber(parseptr,2);			 
				Magnetic[2] = Sign[2]*((Upper/100) + (Lower/100000));		
// END PARSE
			}
			else
			{
				if (PrintErrors)
				  Serial.println("IMU ERROR: Checksum error!!");
			}
		}
	}
	
	// ACCELERATION MEASUREMENTS
	if (strncmp(buffer,"$VNACC",6)==0)				    // Check if sentence begins with $VNACC
	{
		if (buffer[bufferidx-4]=='*')					// Check for the "*" character
		{          		
			IMU_check = parseHex(buffer[bufferidx-3])*16 + parseHex(buffer[bufferidx-2]);    // Read the checksums characters
			if (IMU_checksum == IMU_check)				// Checksum validation
			{      				
// CHECK SIGN OF OUTPUT			
				temp[0] = buffer[7];					// X-Axis Acc. Sign (+ or -)
				temp[1] = buffer[15];					// Y-Axis Acc. Sign (+ or -)
				temp[2] = buffer[23];					// Z-Axis Acc. Sign (+ or -)
				
				if (temp[0] == 45)						// if equal to - then set modifier to negative
					Sign[0] = -1;
					else
						Sign[0] = 1;					// else leave positive
				if (temp[1] == 45)						// if equal to - then set modifier to negative
					Sign[1] = -1;
					else
						Sign[1] = 1;					// else leave positive
				if (temp[2] == 45)						// if equal to - then set modifier to negative
					Sign[2] = -1;
					else
						Sign[2] = 1;					// else leave positive
// END SIGN CHECK
// BEGIN PARSE			
				NewData = 1;  // New IMU Data				// Flag when new data is available
				parseptr = strchr(buffer, ',')+2;			// look for comma-seperator after the $VNYPR identifier
				
				Upper = parsenumber(parseptr,3);        	// get upper (before decimal) numbers
				parseptr = strchr(parseptr, '.')+1;			// look for decimal-seperator
				Lower = parsenumber(parseptr,2);			// get lower (after decimal) numbers
				Acceleration[0] = Sign[0]*((Upper/100) + (Lower/100000));			// join values together to get whole reading
				parseptr = strchr(parseptr, ',')+2;			// look for comma-seperator

				Upper = parsenumber(parseptr,3);
				parseptr = strchr(parseptr, '.')+1;			// look for decimal-seperator
				Lower = parsenumber(parseptr,2);      		
				Acceleration[1] = Sign[1]*((Upper/100) + (Lower/100000));
				parseptr = strchr(parseptr, ',')+2;			// look for comma-seperator

				Upper = parsenumber(parseptr,3);
				parseptr = strchr(parseptr, '.')+1;			// look for decimal-seperator
				Lower = parsenumber(parseptr,2);			 
				Acceleration[2] = Sign[2]*((Upper/100) + (Lower/100000));
// END PARSE
			}
			else
			{
				if (PrintErrors)
				  Serial.println("IMU ERROR: Checksum error!!");
			}
		}
	}
	
	// ANGULAR RATE MEASUREMENTS
	if (strncmp(buffer,"$VNGYR",6)==0)				    // Check if sentence begins with $VNGYR
	{
		if (buffer[bufferidx-4]=='*')					// Check for the "*" character
		{          		
			IMU_check = parseHex(buffer[bufferidx-3])*16 + parseHex(buffer[bufferidx-2]);    // Read the checksums characters
			if (IMU_checksum == IMU_check)				// Checksum validation
			{      				
// CHECK SIGN OF OUTPUT			
				temp[0] = buffer[7];					// X-Axis Ang. Sign (+ or -)
				temp[1] = buffer[16];					// Y-Axis Ang. Sign (+ or -)
				temp[2] = buffer[25];					// Z-Axis Ang. Sign (+ or -)
				
				if (temp[0] == 45)						// if equal to - then set modifier to negative
					Sign[0] = -1;
					else
						Sign[0] = 1;					// else leave positive
				if (temp[1] == 45)						// if equal to - then set modifier to negative
					Sign[1] = -1;
					else
						Sign[1] = 1;					// else leave positive
				if (temp[2] == 45)						// if equal to - then set modifier to negative
					Sign[2] = -1;
					else
						Sign[2] = 1;					// else leave positive
// END SIGN CHECK
// BEGIN PARSE			
				NewData = 1;  // New IMU Data				// Flag when new data is available
				parseptr = strchr(buffer, ',')+2;			// look for comma-seperator after the $VNYPR identifier
				
				Upper = parsenumber(parseptr,3);        	// get upper (before decimal) numbers
				parseptr = strchr(parseptr, '.')+1;			// look for decimal-seperator
				Lower = parsenumber(parseptr,2);			// get lower (after decimal) numbers
				Angular[0] = Sign[0]*((Upper/100) + (Lower/100000));			// join values together to get whole reading
				parseptr = strchr(parseptr, ',')+2;			// look for comma-seperator

				Upper = parsenumber(parseptr,3);
				parseptr = strchr(parseptr, '.')+1;			// look for decimal-seperator
				Lower = parsenumber(parseptr,2);      		 
				Angular[1] = Sign[1]*((Upper/100) + (Lower/100000));
				parseptr = strchr(parseptr, ',')+2;			// look for comma-seperator

				Upper = parsenumber(parseptr,3);
				parseptr = strchr(parseptr, '.')+1;			// look for decimal-seperator
				Lower = parsenumber(parseptr,2);			 
				Angular[2] = Sign[2]*((Upper/100) + (Lower/100000));
// END PARSE
			}
			else
			{
				if (PrintErrors)
				  Serial.println("IMU ERROR: Checksum error!!");
			}
		}
	}	
	// MAGNETIC, ACCELERATION AND ANGULAR RATES	
	if (strncmp(buffer,"$VNMAR",6)==0)				    // Check if sentence begins with $VNMAR
	{
		if (buffer[bufferidx-4]=='*')					// Check for the "*" character
		{          		
			IMU_check = parseHex(buffer[bufferidx-3])*16 + parseHex(buffer[bufferidx-2]);    // Read the checksums characters
			if (IMU_checksum == IMU_check)				// Checksum validation
			{      				
// CHECK SIGN OF OUTPUT			
				temp[0] = buffer[7];					// X-Axis Mag. Sign
				temp[1] = buffer[15];					// Y-Axis Mag. Sign
				temp[2] = buffer[23];					// Z-Axis Mag. Sign
				temp[3] = buffer[31];					// X-Axis Acc. Sign
				temp[4] = buffer[39];					// Y-Axis Acc. Sign
				temp[5] = buffer[47];					// Z-Axis Acc. Sign
				temp[6] = buffer[55];					// X-Axis Ang. Sign
				temp[7] = buffer[64];					// Y-Axis Ang. Sign
				temp[8] = buffer[73];					// Z-Axis Ang. Sign
				
				if (temp[0] == 45)						// if equal to - then set modifier to negative
					Sign[0] = -1;
					else
						Sign[0] = 1;					// else leave positive
				if (temp[1] == 45)						// if equal to - then set modifier to negative
					Sign[1] = -1;
					else
						Sign[1] = 1;					// else leave positive
				if (temp[2] == 45)						// if equal to - then set modifier to negative
					Sign[2] = -1;
					else
						Sign[2] = 1;					// else leave positive
				if (temp[3] == 45)						// if equal to - then set modifier to negative
					Sign[3] = -1;
					else
						Sign[3] = 1;					// else leave positive
				if (temp[4] == 45)						// if equal to - then set modifier to negative
					Sign[4] = -1;
					else
						Sign[4] = 1;					// else leave positive
				if (temp[5] == 45)						// if equal to - then set modifier to negative
					Sign[5] = -1;
					else
						Sign[5] = 1;					// else leave positive
				if (temp[6] == 45)						// if equal to - then set modifier to negative
					Sign[6] = -1;
					else
						Sign[6] = 1;					// else leave positive
				if (temp[7] == 45)						// if equal to - then set modifier to negative
					Sign[7] = -1;
					else
						Sign[7] = 1;					// else leave positive
				if (temp[8] == 45)						// if equal to - then set modifier to negative
					Sign[8] = -1;
					else
						Sign[8] = 1;					// else leave positive
// END SIGN CHECK
// BEGIN PARSE			
				NewData = 1;  // New IMU Data				// Flag when new data is available
				parseptr = strchr(buffer, ',')+2;			// look for comma-seperator after the $VNYPR identifier
				
				Upper = parsenumber(parseptr,3);        	// get upper (before decimal) numbers
				parseptr = strchr(parseptr, '.')+1;			// look for decimal-seperator
				Lower = parsenumber(parseptr,2);			// get lower (after decimal) numbers
				Magnetic[0] = Sign[0]*((Upper/100) + (Lower/100000));			// join values together to get whole reading
				parseptr = strchr(parseptr, ',')+2;			// look for comma-seperator

				Upper = parsenumber(parseptr,3);
				parseptr = strchr(parseptr, '.')+1;			// look for decimal-seperator
				Lower = parsenumber(parseptr,2); 
				Magnetic[1] = Sign[1]*((Upper/100) + (Lower/100000));
				parseptr = strchr(parseptr, ',')+2;			// look for comma-seperator

				Upper = parsenumber(parseptr,3);
				parseptr = strchr(parseptr, '.')+1;			// look for decimal-seperator
				Lower = parsenumber(parseptr,2);			
				Magnetic[2] = Sign[2]*((Upper/100) + (Lower/100000));
				parseptr = strchr(parseptr, ',')+2;			// look for comma-seperator
				
				Upper = parsenumber(parseptr,3);
				parseptr = strchr(parseptr, '.')+1;			// look for decimal-seperator
				Lower = parsenumber(parseptr,2);			 
				Acceleration[0] = Sign[3]*((Upper/100) + (Lower/100000));				
				parseptr = strchr(parseptr, ',')+2;			// look for comma-seperator

				Upper = parsenumber(parseptr,3);
				parseptr = strchr(parseptr, '.')+1;			// look for decimal-seperator
				Lower = parsenumber(parseptr,2);			 
				Acceleration[1] = Sign[4]*((Upper/100) + (Lower/100000));				
				parseptr = strchr(parseptr, ',')+2;			// look for comma-seperator
				
				Upper = parsenumber(parseptr,3);
				parseptr = strchr(parseptr, '.')+1;			// look for decimal-seperator
				Lower = parsenumber(parseptr,2);			 
				Acceleration[2] = Sign[5]*((Upper/100) + (Lower/100000));				
				parseptr = strchr(parseptr, ',')+2;			// look for comma-seperator

				Upper = parsenumber(parseptr,3);
				parseptr = strchr(parseptr, '.')+1;			// look for decimal-seperator
				Lower = parsenumber(parseptr,2);			 
				Angular[0] = Sign[6]*((Upper/100) + (Lower/100000));				
				parseptr = strchr(parseptr, ',')+2;			// look for comma-seperator

				Upper = parsenumber(parseptr,3);
				parseptr = strchr(parseptr, '.')+1;			// look for decimal-seperator
				Lower = parsenumber(parseptr,2);			 
				Angular[1] = Sign[7]*((Upper/100) + (Lower/100000));				
				parseptr = strchr(parseptr, ',')+2;			// look for comma-seperator

				Upper = parsenumber(parseptr,3);
				parseptr = strchr(parseptr, '.')+1;			// look for decimal-seperator
				Lower = parsenumber(parseptr,2);			 
				Angular[2] = Sign[8]*((Upper/100) + (Lower/100000));				
// END PARSE
			}
			else
			{
				if (PrintErrors)
				  Serial.println("IMU ERROR: Checksum error!!");
			}
		}
	}	
	else
	{
		bufferidx = 0;
		if (PrintErrors)
			Serial.println("IMU_ERROR: Bad sentence!!");
	}
	
}


/****************************************************************
 * 
 ****************************************************************/
 // Parse hexadecimal numbers
byte IMU_Class::parseHex(char c)
{
    if (c < '0')
		return (0);
    if (c <= '9')
		return (c - '0');
    if (c < 'A')
		return (0);
    if (c <= 'F')
		return ((c - 'A')+10);
}

// Decimal number parser
long IMU_Class::parsedecimal(char *str,byte num_car) 
{
	long d = 0;
	long sign = 1;
	byte i;

	i = num_car;
	while ((str[0] != 0)&&(i>0)) 
	{
		//-----------------------------
		if (str[0] = '+')
			sign = 1;
		else if (str[0] = '-')
			sign = -1;
		//-----------------------------		
		if ((str[0] > '9') || (str[1] < '0'))
			return d*sign;
		d *= 10;
		d += str[0] - '0';
		str++;
		i--;
	}
	return d*sign;
}

// Function to parse fixed point numbers (numdec=number of decimals)
long IMU_Class::parsenumber(char *str,byte numdec) 
{
	long d = 0;
	byte ndec = 0;

	while (str[0] != 0) 
	{
		if (str[0] == '.')
		{
			ndec = 1;
		}
		else 
		{
			if ((str[0] > '9') || (str[0] < '0'))
				return d;
			d *= 10;
			d += str[0] - '0';
			if (ndec > 0)
				ndec++;
			if (ndec > numdec)   // we reach the number of decimals...
				return d;
		}
		str++;
	}
	return d;
}

IMU_Class IMU; 