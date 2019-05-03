//======================================================================
/**
 *  @file
 *  functions.h
 *
 *  @section testing.h_general General file information
 *
 *  @brief
 *  
 *
 *  @author	Steffen Wolfer
 *  @date	30.04.2012
 *  
 *  
 *  @section testing.h_copyright Copyright
 *  
 *  Copyright 2012 Weiss Robotics, D-71640 Ludwigsburg, Germany
 *  
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the company Weiss Robotics GmbH & Co. KG nor the 
 *       names of its contributors may be used to endorse or promote products
 *	 derived from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 *  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 *  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 *  ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 *  LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 *  CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 *  SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 *  INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 *  CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 *  ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 */
//======================================================================


#ifndef FUNCTIONS_H_
#define FUNCTIONS_H_

 #include <string>

//------------------------------------------------------------------------
// Includes
//------------------------------------------------------------------------


//------------------------------------------------------------------------
// Macros
//------------------------------------------------------------------------



/*#ifdef __cplusplus
extern "C" {
#endif */

//------------------------------------------------------------------------
// Typedefs, enums, structs
//------------------------------------------------------------------------
typedef struct {
	unsigned int state;
	bool ismoving;
	float position, speed;
	float f_motor, f_finger0, f_finger1;
	std::string state_text;
} gripper_response;

//------------------------------------------------------------------------
// Global variables
//------------------------------------------------------------------------



//------------------------------------------------------------------------
// Function declaration
//------------------------------------------------------------------------

float convert(unsigned char *b);
int homing( void );
int move(float width, float speed, bool stop_on_block, bool ignore_response = false);
int stop( bool ignore_response = false );
int grasp( float objWidth, float speed );
int release( float width, float speed );
int ack_fault( void );

int setAcceleration( float acc );
int setGraspingForceLimit( float force );

const char * systemState( void );
int graspingState( void );
float getOpening(int auto_update = 0);
float getForce(int auto_update = 0);
float getSpeed(int auto_update = 0);
int getAcceleration( void );
int getGraspingForceLimit( void );

int script_measure_move (unsigned char cmd_type, float cmd_width, float cmd_speed, gripper_response & info);

//void getStateValues(); //(unsigned char *);

//void test( void );


/*#ifdef __cplusplus
}
#endif */

#endif /* FUNCTIONS_H_ */
