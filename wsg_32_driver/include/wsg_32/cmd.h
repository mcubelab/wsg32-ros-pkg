//======================================================================
/**
 *  @file
 *  cmd.h
 *
 *  @section cmd.h_general General file information
 *
 *  @brief
 *  Command abstraction layer (Header file)
 *
 *  @author	Steffen Wolfer
 *  @date	20.07.2011
 *  
 *  
 *  @section cmd.h_copyright Copyright
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


#ifndef CMD_H_
#define CMD_H_

//------------------------------------------------------------------------
// Includes
//------------------------------------------------------------------------

#include "common.h"


//------------------------------------------------------------------------
// Macros
//------------------------------------------------------------------------



#ifdef __cplusplus
extern "C" {
#endif

//------------------------------------------------------------------------
// Typedefs, enums, structs
//------------------------------------------------------------------------


//------------------------------------------------------------------------
// Global variables
//------------------------------------------------------------------------


//------------------------------------------------------------------------
// Function declaration
//------------------------------------------------------------------------

int cmd_connect_tcp( const char *addr, unsigned short port );
int cmd_connect_udp( unsigned short local_port, const char *addr, unsigned short remote_port );
int cmd_connect_serial( const char *device, unsigned int bitrate );

void cmd_disconnect( void );
bool cmd_is_connected( void );
status_t cmd_get_response_status( unsigned char *response );

int cmd_submit( unsigned char id, unsigned char *payload, unsigned int len,
			    bool pending, unsigned char **response, unsigned int *response_len );


#ifdef __cplusplus
}
#endif

#endif /* CMD_H_ */
