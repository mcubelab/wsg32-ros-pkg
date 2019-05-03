//======================================================================
/**
 *  @file
 *  char.h
 *
 *  @section char.h_general General file information
 *
 *  @brief
 *  
 *
 *  @author	Steffen Wolfer
 *  @date	16.09.2011
 *  
 *  
 *  @section char.h_copyright Copyright
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


#ifndef CHAR_H_
#define CHAR_H_

//------------------------------------------------------------------------
// Character constants
//------------------------------------------------------------------------

/* Special character encodings */

#define ISO_NEWLINE     ( (unsigned char) 0x0a )	// '\ņ'
#define ISO_TAB     	( (unsigned char) 0x09 )	// '\t'
#define ISO_NULL		( (unsigned char) 0x00 )	// '\0'
#define ISO_CARRIAGE	( (unsigned char) 0x0d )	// '\r' (Carriage return)
#define ISO_ESC			( (unsigned char) 0x1b )	// '\033' (ESCAPE)
#define ISO_SPACE    	( (unsigned char) 0x20 )	// ' '
#define ISO_BANG     	( (unsigned char) 0x21 )	// '!'
#define ISO_PERCENT  	( (unsigned char) 0x25 )	// '%'
#define ISO_PERIOD   	( (unsigned char) 0x2e )	// '.'
#define ISO_SLASH    	( (unsigned char) 0x2f )	// '/'
#define ISO_BACKSLASH	( (unsigned char) 0x5c )	// '\'
#define ISO_COLON    	( (unsigned char) 0x3a )	// ':'
#define ISO_AMP			( (unsigned char) 0x26 )	// '&'
#define ISO_HYPHEN		( (unsigned char) 0x2d )	// '-'
#define ISO_QUOTE		( (unsigned char) 0x22 )	// '"'
#define ISO_EQUALS		( (unsigned char) 0x3d )	// '='
#define ISO_SEMICOLON	( (unsigned char) 0x3b )	// ';'
#define ISO_QUESTION	( (unsigned char) 0x3f )	// '?'


#endif /* CHAR_H_ */
