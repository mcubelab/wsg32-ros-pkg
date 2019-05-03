//======================================================================
/**
 *  @file
 *  msg.h
 *
 *  @section msg.h_general General file information
 *
 *  @brief
 *  Raw send and receive functions for command messages (Header file)
 *
 *  @author	Steffen Wolfer
 *  @date	19.07.2011
 *  
 *  
 *  @section msg.h_copyright Copyright
 *  
 *  Copyright 2011 Weiss Robotics, D-71640 Ludwigsburg, Germany
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


#ifndef MSG_H_
#define MSG_H_

//------------------------------------------------------------------------
// Includes
//------------------------------------------------------------------------

#include "common.h"
#include "interface.h"


//------------------------------------------------------------------------
// Macros
//------------------------------------------------------------------------

#define MSG_PREAMBLE_BYTE		0xaa
#define MSG_PREAMBLE_LEN		3

// Combine bytes to different types
#define make_short( lowbyte, highbyte )				( (unsigned short)lowbyte | ( (unsigned short)highbyte << 8 ) )
#define make_signed_short( lowbyte, highbyte )		( (signed short) ( (unsigned short) lowbyte | ( (unsigned short) highbyte << 8 ) ) )
#define make_int( lowbyte, mid1, mid2, highbyte )	( (unsigned int) lowbyte | ( (unsigned int) mid1 << 8 ) | ( (unsigned int) mid2 << 16 ) | ( (unsigned int) highbyte << 24 ) )
#define make_float( result, byteptr )				memcpy( &result, byteptr, sizeof( float ) )

// Byte access
#define hi( x )    	(unsigned char) ( ((x) >> 8) & 0xff )	// Returns the upper byte of the passed short
#define lo( x )    	(unsigned char) ( (x) & 0xff )       	// Returns the lower byte of the passed short


#ifdef __cplusplus
extern "C" {
#endif

//------------------------------------------------------------------------
// Typedefs, enums, structs
//------------------------------------------------------------------------

typedef struct
{
	unsigned char id;
	unsigned int len;
	unsigned char *data;
} msg_t;


//------------------------------------------------------------------------
// Global variables
//------------------------------------------------------------------------


//------------------------------------------------------------------------
// Function declaration
//------------------------------------------------------------------------

int msg_open( const interface_t *iface, const void *params );
void msg_close( void );
int msg_change_interface( const interface_t *iface );
int msg_send( msg_t *msg );
int msg_receive( msg_t *msg );
void msg_free( msg_t *msg );

#ifdef __cplusplus
}
#endif

#endif /* MSG_H_ */
