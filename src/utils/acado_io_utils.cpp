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
 *    \file src/utils/acado_io_utils.cpp
 *    \author Boris Houska, Hans Joachim Ferreau
 *    \date 29.12.2008
 */

#include <math.h>

#include <acado/utils/acado_types.hpp>
#include <acado/utils/acado_constants.hpp>
#include <acado/utils/acado_io_utils.hpp>


#ifdef __MATLAB__
#include "mex.h"
#endif

#ifdef __MODELICA__
extern "C" {
#include <ModelicaUtilities.h>
}
#endif



BEGIN_NAMESPACE_ACADO



returnValue getGlobalStringDefinitions(	PrintScheme _printScheme,
					char** _startString,
					char** _endString,
					uint& _width,
					uint& _precision,
					char** _colSeparator,
					char** _rowSeparator
					)
{
  switch( _printScheme )
    {
    case PS_DEFAULT:

      *_startString = new char[ getStringLength(DEFAULT_START_STRING)+1 ];
      strcpy( *_startString,DEFAULT_START_STRING );

      *_endString = new char[ getStringLength(DEFAULT_END_STRING)+1 ];
      strcpy( *_endString,DEFAULT_END_STRING );

      _width     = DEFAULT_WIDTH;
      _precision = DEFAULT_PRECISION;

      *_colSeparator = new char[ getStringLength(DEFAULT_COL_SEPARATOR)+1 ];
      strcpy( *_colSeparator,DEFAULT_COL_SEPARATOR );

      *_rowSeparator = new char[ getStringLength(DEFAULT_ROW_SEPARATOR)+1 ];
      strcpy( *_rowSeparator,DEFAULT_ROW_SEPARATOR );

      break;


    case PS_PLAIN:

      *_startString = new char[1];
      (*_startString)[0] = '\0';

      *_endString = new char[2];
      (*_endString)[0] = '\n';
      (*_endString)[1] = '\0';

      _width     = DEFAULT_WIDTH;
      _precision = DEFAULT_PRECISION;

      *_colSeparator = new char[2];
      (*_colSeparator)[0] = ' ';
      (*_colSeparator)[1] = '\0';

      *_rowSeparator = new char[2];
      (*_rowSeparator)[0] = '\n';
      (*_rowSeparator)[1] = '\0';

      break;


    case PS_MATLAB:
    case PS_MATLAB_BINARY:

      *_startString = new char[3];
      (*_startString)[0] = '[';
      (*_startString)[1] = ' ';
      (*_startString)[2] = '\0';

      *_endString = new char[5];
      (*_endString)[0] = ' ';
      (*_endString)[1] = ']';
      (*_endString)[2] = ';';
      (*_endString)[3] = '\n';
      (*_endString)[4] = '\0';

      _width     = DEFAULT_WIDTH;
      _precision = DEFAULT_PRECISION;

      *_colSeparator = new char[3];
      (*_colSeparator)[0] = ',';
      (*_colSeparator)[1] = ' ';
      (*_colSeparator)[2] = '\0';

      *_rowSeparator = new char[3];
      (*_rowSeparator)[0] = ';';
      (*_rowSeparator)[1] = '\n';
      (*_rowSeparator)[2] = '\0';

      break;


    default:
      return ACADOERROR( RET_UNKNOWN_BUG );
    }
	
  return SUCCESSFUL_RETURN;
}


uint getStringLength( const char* string )
{
  if ( string != 0 )
    return strlen( string );
  else
    return 0;
}




int acadoPrintf( const char* format, ... ){

	int returnvalue;
	va_list argPtr;
	va_start( argPtr,format );
#ifdef __MATLAB__
	mexPrintf( format );
	returnvalue = 1;
#else
	returnvalue = acadoVFPrintf ( TEXT_OUTPUT_STREAM, format, argPtr );
	va_end( argPtr );
#endif

	return returnvalue;
}

FILE* acadoFOpen( const char * filename, const char * mode )
{
  return fopen(filename,mode);
}

int acadoFClose ( FILE * stream )
{
  return fclose(stream);
}

int acadoFPrintf ( FILE * stream, const char * format, ... )
{
  va_list argPtr;
  va_start( argPtr,format );

  int returnvalue = acadoVFPrintf ( stream, format, argPtr );

  va_end( argPtr );

  return returnvalue;
}

int acadoVFPrintf ( FILE * stream, const char * format, va_list arg )
{
#ifdef __MATLAB__
#ifdef WIN32
	if(stream==stdout)   // workaround
#else
  if(stream==stdout || stream==stderr)   // workaround
#endif
	    {
	      const int buffersize = 256;
	      char buffer[buffersize];

	      // check length of buffer
	      int count = vsnprintf(NULL, 0, format, arg );

	      if(count < buffersize)
		{
		  // if string fits in buffer
		  vsprintf(buffer,format,arg);
		  mexPrintf( "%s", buffer );
		}
	      else
		{
		  // else dynamically allocate a larger buffer
		  char *newbuffer = new char[count];
		  vsprintf( newbuffer, format, arg );
		  mexPrintf( "%s", newbuffer );
		  delete [] newbuffer;
		}
	      return count;
	    }
	  else // not stdout or stderr -> really print to file
	    {
	      return vfprintf( stream, format, arg );
	    }
#elif defined __MODELICA__
  if(stream==stdout || stream==stderr)   // workaround
    {
      const int buffersize = 256;
      char buffer[buffersize];

      // check length of buffer
      int count = vsnprintf(NULL, 0, format, arg );

      // if string fits in buffer
      if(count < buffersize)
	{
	  vsprintf(buffer,format,arg);
	  ModelicaFormatMessage("%s",buffer);
	}
      else // dynamically allocate a larger buffer
	{
	  char *newbuffer = new char[count];
	  vsprintf( newbuffer, format, arg );
	  ModelicaFormatMessage("%s",newbuffer);
	  delete [] newbuffer;
	}
      return count;
    }
  else // not stdout or stderr -> really print to file
    {
      return vfprintf( stream, format, arg );
    }

#else
  return vfprintf( stream, format, arg );
#endif

}

double AcadoIntDetection( char text ){

  if( text == '0' ) return 0.0;
  if( text == '1' ) return 1.0;
  if( text == '2' ) return 2.0;
  if( text == '3' ) return 3.0;
  if( text == '4' ) return 4.0;
  if( text == '5' ) return 5.0;
  if( text == '6' ) return 6.0;
  if( text == '7' ) return 7.0;
  if( text == '8' ) return 8.0;
  if( text == '9' ) return 9.0;

  return 0.0;
}


double AcadoDoubleDetection( int   textStart      ,
                             int   signPosition   ,
                             int   dotPosition    ,
                             int   expPosition    ,
                             int   expSignPosition,
                             int   textEnd        ,
                             char *text           ){

  double tmp     = 0.0;
  double basis   = 0.1;
  double exp     = 0.0;
  double sign    = 1.0;
  double expSign = 1.0;

  BooleanType      dot = BT_FALSE;

  int start, run, stop;

  if( signPosition == -1 ){
    if( dotPosition == -1 || dotPosition > textStart ){
      start = textStart;
    }
    else{
      start = textStart+1;
      dot   = BT_TRUE    ;
    }
  }
  else{

    if( text[textStart] == '-' ) sign = -1.0;

    if( dotPosition == -1 || dotPosition > textStart+1 ){
      start = textStart+1;
    }
    else{
      start = textStart+2;
      dot   = BT_TRUE    ;
    }
  }

  if( dotPosition > start ){

    stop = dotPosition;
    if( stop > textEnd+1 ) stop = textEnd+1;
  }
  else{

    if( expPosition == -1 ){
      stop = textEnd+1;
    }
    else{
      stop = expPosition;
      if( stop > textEnd+1 ) stop = textEnd+1;
    }
  }

  if( dot == BT_FALSE ){
    for( run = start; run < stop; run++ )
      tmp = 10.0*tmp + AcadoIntDetection( text[run] );

    start = stop+1;
  }

  if( expPosition == -1 ){
    stop = textEnd+1;
  }
  else{
    stop = expPosition;
    if( stop > textEnd+1 ) stop = textEnd+1;
  }

  if( start < stop ){
    for( run = start; run < stop; run++ ){
      tmp = tmp + basis*AcadoIntDetection( text[run] );
      basis = basis*0.1;
    }
    start = stop+1;
  }

  if( start == expSignPosition ){
    start++;
    if( start <= textEnd+1 )
      if( text[start-1] == '-' )
	expSign = -1.0;
  }
  stop = textEnd+1;

  if( stop > start+3 ) stop = start+3;

  for( run = start; run < stop; run++ ){
    exp = 10.0*exp + AcadoIntDetection( text[run] );
  }

  return sign*tmp*pow(10.0,expSign*exp);
}


BooleanType AcadoIsSign( char c ){

  if( c == '+' || c == '-' ) return BT_TRUE ;
  return BT_FALSE;
}


BooleanType AcadoIsNumber( char c ){

  if( c == '0' || c == '1' || c == '2' ||
      c == '3' || c == '4' || c == '5' ||
      c == '6' || c == '7' || c == '8' ||
      c == '9' ){

    return BT_TRUE;
  }
  return BT_FALSE;
}



// --------------------------------------------------------------------
// AcadoIoReadLine is an (hidden and internal) routine which determines
// a double* from a line of "text" with "nEntries" char-entries. Note
// that at any valid number is interpreted as an double independent of
// the special format or keywords.
// The routine returns the number of successfully determined double
// values which have been found in the text.
// --------------------------------------------------------------------

int AcadoIoReadLine( char *text, int nEntries, int offset, double **x ){

  int run1               ;
  int run2               ;
  int counter         = 0;
  int textStart          ;
  int signPosition       ;
  int dotPosition        ;
  int expPosition        ;
  int expSignPosition    ;
  int textEnd            ;
  int check              ;

  for( run1 = 0; run1 < nEntries; run1++ ){

    textStart = -1;

    if( AcadoIsNumber(text[run1]) == BT_TRUE ){
      textStart = run1;
    }

    if( AcadoIsSign(text[run1]) == BT_TRUE && run1+1 < nEntries ){
      if( AcadoIsNumber(text[run1+1]) == BT_TRUE ){
	textStart = run1;
      }
      else{
	if( text[run1] == '.' && run1+2 < nEntries ){
	  if( AcadoIsNumber(text[run1+2]) == BT_TRUE ){
	    textStart = run1;
	  }
	}
      }
    }

    if( text[run1] == '.' && run1+1 < nEntries ){
      if( AcadoIsNumber(text[run1+1]) == BT_TRUE ){
	textStart = run1;
      }
    }


    // CHECK EXPLICITLY FOR nan, NaN, NAN:
    // -----------------------------------

    if( text[run1] == 'n' && run1+2 < nEntries ){
      if( text[run1+1] == 'a' && text[run1+2] == 'n' ){
	counter++;
	if( x[0] != 0 ) x[0] = (double*)realloc(x[0],(offset+counter)*sizeof(double));
	else            x[0] = (double*)calloc(1,sizeof(double));
	x[0][offset+counter-1] = ACADO_NAN;
      }
    }
    if( text[run1] == 'N' && run1+2 < nEntries ){
      if( (text[run1+1] == 'a' && text[run1+2] == 'N') ||
	  (text[run1+1] == 'A' && text[run1+2] == 'N') ){
	counter++;
	if( x[0] != 0 ) x[0] = (double*)realloc(x[0],(offset+counter)*sizeof(double));
	else            x[0] = (double*)calloc(1,sizeof(double));
	x[0][offset+counter-1] = ACADO_NAN;
      }
    }


    if( textStart >= 0 ){

      check            = -1       ;
      run2             = textStart;
      signPosition     = -1       ;
      dotPosition      = -1       ;
      expPosition      = -1       ;
      expSignPosition  = -1       ;
      textEnd          = -1       ;

      while( run2 < nEntries ){

	if( AcadoIsSign(text[run2]) == BT_TRUE ){

	  switch(check){

	  case -1:
	    signPosition = run2;
	    check = 0;
	    break;

	  case 4:
	    check = 5;
	    expSignPosition = run2;
	    break;

	  default:
	    check = 7;
	    textEnd = run2-1;
	    break;
	  }
	}

	if( AcadoIsNumber(text[run2]) == BT_TRUE ){

	  switch(check){

	  case -1:
	    check = 1;
	    break;

	  case 0:
	    check = 1;
	    break;

	  case 1:
	    break;

	  case 2:
	    check = 3;
	    break;

	  case 3:
	    break;

	  case 4:
	    check = 6;
	    break;

	  case 5:
	    check = 6;
	    break;

	  case 6:
	    textEnd = run2;
	    break;

	  default:
	    check = 7;
	    break;
	  }
	}

	if( text[run2] == '.' ){

	  switch(check){

	  case -1:
	    dotPosition = run2;
	    check = 2;
	    break;

	  case 0:
	    dotPosition = run2;
	    check = 2;
	    break;

	  case 1:
	    dotPosition = run2;
	    check = 2;
	    break;

	  default:
	    check = 7;
	    textEnd = run2-1;
	    break;
	  }
	}

	if( text[run2] == 'e' || text[run2] == 'E' ){

	  switch(check){

	  case 1:
	    expPosition = run2;
	    check = 4;
	    break;

	  case 2:
	    expPosition = run2;
	    check = 4;
	    break;

	  case 3:
	    expPosition = run2;
	    check = 4;
	    break;

	  default:
	    check = 7;
	    textEnd = run2-1;
	    break;
	  }
	}

	if(               text[run2]  != 'e'      &&
			  text[run2]  != 'E'      &&
			  text[run2]  != '.'      &&
			  AcadoIsNumber(text[run2]) == BT_FALSE &&
			  AcadoIsSign  (text[run2]) == BT_FALSE    ){

	  textEnd = run2-1;
	  check   = 7;
	}

	run2++;
	if( check == 7 ){
	  break;
	}
      }

      if( textEnd == -1 ){

	if( AcadoIsNumber(text[run2-1]) == BT_TRUE || text[run2-1] == '.' ){
	  textEnd = run2-1;
	}
	else{
	  if( text[run2-1] == 'e' || text[run2-1] == 'E' ){
	    textEnd = run2-2;
	  }
	  else{
	    textEnd = run2-3;
	  }
	}
      }

      counter++;

      if( x[0] != 0 ) x[0] = (double*)realloc(x[0],(offset+counter)*sizeof(double));
      else            x[0] = (double*)calloc(1,sizeof(double));

      x[0][offset+counter-1] = AcadoDoubleDetection( textStart, signPosition, dotPosition,
						     expPosition, expSignPosition, textEnd,
						     text );

      run1 = textEnd;
    }
  }
  return counter;
}


returnValue allocateDoublePointerFromFile( FILE *file, double **x, int &dim ){

  if( !file ) return ACADOERROR(RET_FILE_CAN_NOT_BE_OPENED);

  int nEntries   = 0;
  int maxEntries = 1;

  if( x == 0 ) return ACADOERROR(RET_INVALID_ARGUMENTS);
  *x = 0;

  char *myChar = (char*)calloc(maxEntries,sizeof(char));

  while( fscanf(file,"%c",&myChar[nEntries]) != EOF ){

    nEntries++;

    if( nEntries >= maxEntries ){
      maxEntries += maxEntries;
      myChar = (char*)realloc(myChar,maxEntries*sizeof(char));
    }
  }

  returnValue             returnvalue = SUCCESSFUL_RETURN         ;
  if( fclose(file) != 0 ) returnvalue = RET_FILE_CAN_NOT_BE_CLOSED;

  dim = AcadoIoReadLine( myChar, nEntries, 0, x );

  free(myChar);

  if( dim <= 0 ) returnvalue = RET_FILE_HAS_NO_VALID_ENTRIES;

  if( returnvalue != SUCCESSFUL_RETURN ) ACADOWARNING(returnvalue);
  return             SUCCESSFUL_RETURN;
}


returnValue allocateDoublePointerFromFile( FILE *file , double **x, int &nRows, int &nCols ){

  if( !file ) return ACADOERROR(RET_FILE_CAN_NOT_BE_OPENED);

  int          nEntries   ;
  int          maxEntries ;
  int          checkEntry ;
  char        *myChar     ;
  returnValue  returnvalue;
  int          offset     ;

  if( x == 0 ) return ACADOERROR(RET_INVALID_ARGUMENTS);
  *x = 0;

  nRows       =  0                ;
  nCols       = -1                ;
  returnvalue =  SUCCESSFUL_RETURN;

  while( !feof(file) ){

    nRows++;
    nEntries   = 0;
    maxEntries = 1;
    myChar     = (char*)calloc(maxEntries,sizeof(char));


    while( fscanf(file,"%c",&myChar[nEntries]) != EOF  ){

      if( myChar[nEntries] == LINE_SEPARATOR ) break;
      nEntries++;

      if( nEntries >= maxEntries ){
	maxEntries += maxEntries;
	myChar = (char*)realloc(myChar,maxEntries*sizeof(char));
      }
    }

    if( nCols == -1 )  offset = 0;
    else               offset = (nRows-1)*nCols;

    checkEntry = AcadoIoReadLine( myChar, nEntries, offset, x );

    free(myChar);

    if( checkEntry > 0 ){
      if( nCols == -1         ) nCols = checkEntry;
      if( nCols != checkEntry ) return ACADOERROR(RET_FILE_HAS_NO_VALID_ENTRIES);
    }
    else{
      nRows--;
    }
  }

  if( nRows <= 0 || nCols <= 0 ) return ACADOWARNING(RET_FILE_HAS_NO_VALID_ENTRIES);

  if( fclose(file) != 0 ) return ACADOWARNING(RET_FILE_CAN_NOT_BE_CLOSED);

  if( returnvalue != SUCCESSFUL_RETURN ) ACADOWARNING(returnvalue);
  return             SUCCESSFUL_RETURN;
}


returnValue writeDoublePointerToFile( double *x, int dim, FILE *file ){

  int run1;

  if( !file ) return ACADOERROR(RET_CAN_NOT_WRITE_INTO_FILE);

  for( run1 = 0; run1 < dim; run1++ ){
    if( x[run1] <= ACADO_NAN - 1.0 ) acadoFPrintf(file,"%.16e%c",x[run1], LINE_SEPARATOR );
    else                             acadoFPrintf(file,"%s%c", NOT_A_NUMBER, LINE_SEPARATOR );
  }
  acadoFPrintf(file,"%c",LINE_SEPARATOR);

  return SUCCESSFUL_RETURN;
}


int writeDoublePointerToString( double *x, int dim, char *buffer ){

  int run1, run2, run3, counter;

  if( x == 0 ) return -1;

  if( buffer == 0 ){
    run2 = 0;
    for( run1 = 0; run1 < dim; run1++ ){
      if( x[run1] <= ACADO_NAN - 1.0 ) run2 += snprintf( buffer, 0, "%.16e%c",x[run1], LINE_SEPARATOR );
      else                             run2 += snprintf( buffer, 0, "%s%c", NOT_A_NUMBER, LINE_SEPARATOR );
    }
    return run2;
  }

  char *temp = new char[MAX_LENGTH_STRING];
  counter = 0;

  for( run1 = 0; run1 < dim; run1++ ){
    if( x[run1] <= ACADO_NAN - 1.0 ) run2 = snprintf( temp, MAX_LENGTH_STRING, "%.16e%c",x[run1], LINE_SEPARATOR );
    else                             run2 = snprintf( temp, MAX_LENGTH_STRING, "%s%c",NOT_A_NUMBER, LINE_SEPARATOR );
    for( run3 = 0; run3 < run2; run3++ ){
      buffer[counter] = temp[run3];
      counter++;
    }
  }

  delete[] temp;
  return counter;
}


returnValue writeDoublePointerToFile( double *x, int nRows, int nCols, FILE *file ){

  int run1, run2;

  if( !file ) return ACADOERROR(RET_CAN_NOT_WRITE_INTO_FILE);

  for( run1 = 0; run1 < nRows; run1++ ){
    for( run2 = 0; run2 < nCols; run2++ ){
      if( x[nCols*run1+run2] <= ACADO_NAN - 1.0 ) acadoFPrintf(file,"%.16e%c",x[nCols*run1+run2], TEXT_SEPARATOR );
      else                                        acadoFPrintf(file,"%s%c",NOT_A_NUMBER, TEXT_SEPARATOR );
    }
    acadoFPrintf(file,"%c",LINE_SEPARATOR);
  }
  acadoFPrintf(file,"%c",LINE_SEPARATOR);

  return SUCCESSFUL_RETURN;
}


int writeDoublePointerToString( double *x, int nRows, int nCols, char *buffer ){

  int run1, run2, run3, run4, counter;

  if( x == 0 ) return -1;

  if( buffer == 0 ){
    run2 = 0;
    for( run1 = 0; run1 < nRows; run1++ ){
      for( run4 = 0; run4 < nCols; run4++ ){
	if( x[nCols*run1+run4] <= ACADO_NAN - 1.0 ) run2 += snprintf( buffer, 0, "%.16e%c",x[nCols*run1+run4], TEXT_SEPARATOR );
	else                                        run2 += snprintf( buffer, 0, "%s%c",NOT_A_NUMBER, TEXT_SEPARATOR );
      }
      run2 += snprintf( buffer, 0, "%c",LINE_SEPARATOR );
    }
    return run2;
  }

  char *temp = new char[MAX_LENGTH_STRING];
  counter = 0;

  for( run1 = 0; run1 < nRows; run1++ ){
    for( run4 = 0; run4 < nCols; run4++ ){
      if( x[nCols*run1+run4] <= ACADO_NAN - 1.0 ) run2 = snprintf( temp, MAX_LENGTH_STRING, "%.16e%c",x[nCols*run1+run4], TEXT_SEPARATOR );
      else                                        run2 = snprintf( temp, MAX_LENGTH_STRING, "%s%c",NOT_A_NUMBER, TEXT_SEPARATOR );
      for( run3 = 0; run3 < run2; run3++ ){
	buffer[counter] = temp[run3];
	counter++;
      }
    }
    run2 = snprintf( temp, MAX_LENGTH_STRING, "%c",LINE_SEPARATOR );
    for( run3 = 0; run3 < run2; run3++ ){
      buffer[counter] = temp[run3];
      counter++;
    }
  }

  delete[] temp;
  return counter;
}


FILE* readFromFile( const char* filename )
{
	return fopen( filename,"r" );
}



CLOSE_NAMESPACE_ACADO


/*
 *    end of file
 */
