//======================================================================
/**
 *  @file
 *  udp.h
 *
 *  @section udp.h_general General file information
 *
 *  @brief
 *  
 *
 *  @author	Steffen Wolfer
 *  @date	07.07.2011
 *  
 *  
 *  @section udp.h_copyright Copyright
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


#ifndef UDP_H_
#define UDP_H_

//------------------------------------------------------------------------
// Includes
//------------------------------------------------------------------------

#ifdef WIN32
// Note: Compiler has to link against -lwsock32 or -lws2_32 on MinGW
// @todo Have to adjust some code to make udp work on MinGW
#include "winsock.h"
#include "winsock2.h"
#else
#include <unistd.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <sys/select.h>
#include <arpa/inet.h>
#include <netinet/in.h>
#endif

#include "common.h"


#ifdef __cplusplus
extern "C" {
#endif

//------------------------------------------------------------------------
// Macros
//------------------------------------------------------------------------

#define UDP_RCV_BUFSIZE		1024		// Size of UDP receive buffer. This is the maximum size a command message may have, including preamble etc.
										// Minimum is 8 (3 bytes preamble, 1 byte command id, 2 bytes size, 2 bytes checksum)


//------------------------------------------------------------------------
// Typedefs, enums, structs
//------------------------------------------------------------------------

typedef struct
{
	unsigned short local_port;
	ip_addr_t addr;
	unsigned short remote_port;
} udp_params_t;


typedef struct
{
	int sock;
	unsigned char rcv_buf[UDP_RCV_BUFSIZE];
	unsigned int rcv_bufptr;
	unsigned int rcv_bufsize;
	struct sockaddr_in si_listen;
	struct sockaddr_in si_server;
	struct sockaddr_in si_incoming;
	ip_addr_t server;
} udp_conn_t;


//------------------------------------------------------------------------
// Function declaration
//------------------------------------------------------------------------

int udp_open( const void *params );
void udp_close( void );
int udp_read( unsigned char *buf, unsigned int len );
int udp_write( unsigned char *buf, unsigned int len );


#ifdef __cplusplus
}
#endif

#endif /* UDP_H_ */
