//======================================================================
/**
 *  @file
 *  interface.h
 *
 *  @section interface.h_general General file information
 *
 *  @brief
 *  
 *
 *  @author	Steffen Wolfer
 *  @date	07.07.2011
 *  
 *  
 *  @section interface.h_copyright Copyright
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


#ifndef INTERFACE_H_
#define INTERFACE_H_

//------------------------------------------------------------------------
// Includes
//------------------------------------------------------------------------



#ifdef __cplusplus
extern "C" {
#endif

//------------------------------------------------------------------------
// Macros
//------------------------------------------------------------------------


//------------------------------------------------------------------------
// Typedefs, enums, structs
//------------------------------------------------------------------------

typedef struct
{
	const char *name;
	int ( *open ) ( const void *params );
	void ( *close ) ( void );
	int ( *read ) ( unsigned char *, unsigned int );
	int ( *write ) ( unsigned char *, unsigned int );
} interface_t;


//------------------------------------------------------------------------
// Function declaration
//------------------------------------------------------------------------

const interface_t * interface_get( const char *name );


#ifdef __cplusplus
}
#endif

#endif /* INTERFACE_H_ */
