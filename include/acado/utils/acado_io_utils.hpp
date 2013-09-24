/*
 *    This file is part of ACADO Toolkit.
 *
 *    ACADO Toolkit -- A Toolkit for Automatic Control and Dynamic Optimization.
 *    Copyright (C) 2008-2013 by Boris Houska, Hans Joachim Ferreau,
 *    Milan Vukov, Rien Quirynen, KU Leuven.
 *    Developed within the Optimization in Engineering Center (OPTEC)
 *    under supervision of Moritz Diehl. All rights reserved.
 *
 *    ACADO Toolkit is free software; you can redistribute it and/or
 *    modify it under the terms of the GNU Lesser General Public
 *    License as published by the Free Software Foundation; either
 *    version 3 of the License, or (at your option) any later version.
 *
 *    ACADO Toolkit is distributed in the hope that it will be useful,
 *    but WITHOUT ANY WARRANTY; without even the implied warranty of
 *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 *    Lesser General Public License for more details.
 *
 *    You should have received a copy of the GNU Lesser General Public
 *    License along with ACADO Toolkit; if not, write to the Free Software
 *    Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA
 *
 */


/**
 *    \file include/acado/utils/acado_io_utils.hpp
 *    \author Boris Houska, Hans Joachim Ferreau
 *    \date 29.12.2008
 *
 *    This file declares several utility functions that are
 *    useful for low-level file reading and printing.
 *
 */


#ifndef ACADO_TOOLKIT_ACADO_IO_UTILS_HPP
#define ACADO_TOOLKIT_ACADO_IO_UTILS_HPP


#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdarg.h>

#include <acado/utils/acado_message_handling.hpp>


BEGIN_NAMESPACE_ACADO


/** Global definition of the default line    \n
 *  separation symbol. This constant should  \n
 *  be used for all text-file interaction.   \n
 */
const char LINE_SEPARATOR = '\n';


/** Global definition of the default text     \n
 *  separation symbol. This constant  \n
 *  should be used for all text-files \n
 *  interactions.                     \n
 */
const char TEXT_SEPARATOR = ' ';


/** Global definition of the default text     \n
 *  separation symbol. This constant  \n
 *  should be used for all text-files \n
 *  interactions.                     \n
 */
const char NOT_A_NUMBER[3] = { 'n', 'a', 'n' };



/** Global definition of the default \n
 *  output stream. This constant  \n
 *  should be used for all text-files \n
 *  interactions.                     \n
 */
FILE* const TEXT_OUTPUT_STREAM = stdout; //fopen( "a.txt","wb+" );




const char  DEFAULT_LABEL         [1]   = { '\0'                              };
const char  DEFAULT_START_STRING  [3]   = { '[' , '\t', '\0'                  };
const char  DEFAULT_END_STRING    [4]   = { '\t', ']' , '\n', '\0'            };
const uint  DEFAULT_WIDTH               = 22;
const uint  DEFAULT_PRECISION           = 16;
const char  DEFAULT_COL_SEPARATOR [2]   = { '\t', '\0'                        };
const char  DEFAULT_ROW_SEPARATOR [6]   = { '\t', ']' , '\n', '[', '\t', '\0' };



returnValue getGlobalStringDefinitions(	PrintScheme _printScheme,
										char** _startString,
										char** _endString,
										uint& _width,
										uint& _precision,
										char** _colSeparator,
										char** _rowSeparator
										);


/** Returns the length of a string;
 * \return length of the string (or 0 if not allocated!) */
uint getStringLength( const char* string );




int acadoPrintf( const char* format, ... );

FILE* acadoFOpen( const char * filename, const char * mode );
int acadoFClose ( FILE * stream );
int acadoFPrintf ( FILE * stream, const char * format, ... );
int acadoVFPrintf ( FILE * stream, const char * format, va_list arg );

/** Global utility function to allocate a double* from an open file.   \n
 *  The file is closed at the end of the routine. Note that this       \n
 *  function returns the number "dim" of allocated double values which \n
 *  have been detected in the file. If a non positive value for dim is \n
 *  returned, the routine will also return x = NULL. (Thus, you can    \n
 *  either check for x != 0 or dim > 0 to ensure that x has been       \n
 *  allocated by the routine.)                                         \n
 *                                                                     \n
 *  \return SUCCESSFUL_RETURN                                          \n
 *          RET_FILE_NOT_FOUND                                         \n
 *          RET_FILE_CAN_NOT_BE_OPENED                                 \n
 *          RET_FILE_CAN_NOT_BE_CLOSED                                 \n
 *          RET_FILE_HAS_NO_VALID_ENTRIES                              \n
 */

returnValue allocateDoublePointerFromFile( FILE    *file, /**< the file to read         */
                                           double **x   , /**< the double* to allocate  */
                                           int     &dim   /**< dimension of the double* */ );



/** Global utility function to allocate a double* from an open file.  \n
 *  The file is closed at the end of the routine. Note that the       \n
 *  dimension of the allocated double pointer will be                 \n
 *                                                                    \n
 *                       nCols x nRows  .                             \n
 *                                                                    \n
 *  If a non positive value for nRows and/or nCols is returned, the   \n
 *  routine will also return x = NULL. (Thus, you can either check    \n
 *  for x != 0 or dim > 0 to ensure that x has been                   \n
 *  allocated by the routine.)                                        \n
 *                                                                    \n
 *  \return SUCCESSFUL_RETURN                                         \n
 *          RET_FILE_NOT_FOUND                                        \n
 *          RET_FILE_CAN_NOT_BE_OPENED                                \n
 *          RET_FILE_CAN_NOT_BE_CLOSED                                \n
 *          RET_FILE_HAS_NO_VALID_ENTRIES                             \n
 */

returnValue allocateDoublePointerFromFile( FILE    *file , /**< the file to read         */
                                           double **x    , /**< the double* to allocate  */
                                           int     &nRows, /**< number of rows           */
                                           int     &nCols  /**< number of columns        */ );


/** Global utility function to write a double* into an open file.    \n
 *                                                                   \n
 *  \return SUCCESSFUL_RETURN                                        \n
 *          RET_CAN_NOT_WRITE_INTO_FILE                              \n
 */

returnValue writeDoublePointerToFile( double *x   , /**< the double* to write     */
                                      int     dim , /**< dimension of the double* */
                                      FILE   *file  /**< the file to write to     */ );



/** Global utility function to write a double* into an string.       \n
 *                                                                   \n
 *  \return IF SUCCESSFUL: number entries written to the buffer >= 0 \n
 *          OTHERWISE    : -1                                        \n
 */

int writeDoublePointerToString( double *x     , /**< the double* to write     */
                                int     dim   , /**< dimension of the double* */
                                char   *buffer  /**< the buffer to write to   */ );




/** Global utility function to write a double* into an open file.    \n
 *  (the dimension of the double* should be  nRows x nCols.)         \n
 *                                                                   \n
 *  \return SUCCESSFUL_RETURN                                        \n
 *          RET_CAN_NOT_WRITE_INTO_FILE                              \n
 */

returnValue writeDoublePointerToFile( double *x    , /**< the double* to write     */
                                      int     nRows, /**< number of rows           */
                                      int     nCols, /**< number of columns        */
                                      FILE   *file   /**< the file to write to     */ );



/** Global utility function to write a double* into an string.       \n
 *                                                                   \n
 *  \return IF SUCCESSFUL: number entries written to the buffer >= 0 \n
 *          OTHERWISE    : -1                                        \n
 */

int writeDoublePointerToString( double *x     , /**< the double* to write     */
                                int     nRows , /**< number of rows           */
                                int     nCols , /**< number of columns        */
                                char   *buffer  /**< the buffer to write to   */ );


FILE* readFromFile(	const char* filename
					);




CLOSE_NAMESPACE_ACADO



#endif	// ACADO_TOOLKIT_ACADO_IO_UTILS_HPP


/*
 *	end of file
 */
