/*
 *    This file is part of ACADO Toolkit.
 *
 *    ACADO Toolkit -- A Toolkit for Automatic Control and Dynamic Optimization.
 *    Copyright (C) 2008-2009 by Boris Houska and Hans Joachim Ferreau, K.U.Leuven.
 *    Developed within the Optimization in Engineering Center (OPTEC) under
 *    supervision of Moritz Diehl. All rights reserved.
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
*    \file   src/utils/acado_string.cpp
*    \author Boris Houska, Hans Joachim Ferreau
*    \date   2010
*/


#include <acado/utils/acado_string.hpp>
#include <acado/utils/acado_stream.hpp>


BEGIN_NAMESPACE_ACADO


String::String()
{
    length = 0;
    name   = 0;
}


String::String( const String& arg )
{
	length = 0;
	name   = 0;

    copy( arg );
}


String::String( const char* name_ ){

    length = 0;
    while( length < MAX_LENGTH_STRING ){
        if( name_[length] == '\0' ) break;
        length++;
    }
    if( length > 0 ) length++;

    if( length > 0 ){
        name = new char[length];
        uint run1;
        for( run1 = 0; run1 < length-1; run1++ )
            name[run1] = name_[run1];
        name[length-1] = '\0';
    }
    else name = 0;
}


String::String( const double& val_ ){

    int          run1      ;
    char        *char_value;
    char        *tmp       ;
	char        *dummy     ;
    BooleanType  dot       ;

    const int precision = 16;

    dot = BT_FALSE;

    char_value = (char*)malloc(MAX_LENGTH_STRING*sizeof(char));
    tmp        = (char*)malloc(MAX_LENGTH_STRING*sizeof(char));

    dummy = gcvt( val_ , precision , char_value );

    length = 0;
    run1 = 0;

    while( char_value[run1] == '1' || char_value[run1] == '0' ||
           char_value[run1] == '3' || char_value[run1] == '2' ||
           char_value[run1] == '5' || char_value[run1] == '4' ||
           char_value[run1] == '7' || char_value[run1] == '6' ||
           char_value[run1] == '9' || char_value[run1] == '8' ||
           char_value[run1] == '.' || char_value[run1] == 'e' ||
           char_value[run1] == 'n' || char_value[run1] == 'a' ||
           char_value[run1] == 'i' || char_value[run1] == 'n' ||
           char_value[run1] == 'f' || char_value[run1] == ' ' ||
           char_value[run1] == '+' || char_value[run1] == '-' ){

       if( dot == BT_FALSE && char_value[run1] == 'e' ){
            tmp[length] = '.';
            length++;
            dot = BT_TRUE;
       }
       tmp[length] = char_value[run1];
       length++;
       if( char_value[run1] == '.' ) dot = BT_TRUE;
       run1++;
    }
    free(char_value);

    if( dot == BT_FALSE ){
        tmp[length] = '.';
        length++;
    }

    tmp[length] = '\0';
    length++;

    name = new char[length];
    for( run1 = 0; run1 < (int) length; run1++ )
        name[run1] = tmp[run1];

    free(tmp);
}


String::String( const int& val_ ){

    int n = val_;
    const int b = 10;
    char *s = new char[MAX_LENGTH_STRING];
    const char digits[] = "0123456789";
    int sign;

    length = 0;

    if( (sign = n) < 0 ) n = -n;

    do{
        s[length] = digits[n % b];
        length++;
    }
    while ((n /= b) > 0);

    if (sign < 0){
        s[length] = '-';
        length++;
    }

    length++;

    name = new char[length];
    for( sign = 0; sign < (int) length-1; sign++ )
        name[sign] = s[length-2-sign];
    name[length-1] = '\0';

    delete[] s;
}


String::String( const uint& val_ )
{
    int n = val_;
    const int b = 10;
    char *s = new char[MAX_LENGTH_STRING];
    const char digits[] = "0123456789";
    int sign;

    length = 0;

    if( (sign = n) < 0 ) n = -n;

    do{
        s[length] = digits[n % b];
        length++;
    }
    while ((n /= b) > 0);

    if (sign < 0){
        s[length] = '-';
        length++;
    }

    length++;

    name = new char[length];
    for( sign = 0; sign < (int) length-1; sign++ )
        name[sign] = s[length-2-sign];
    name[length-1] = '\0';

    delete[] s;
}
	
	

String::~String(){

    if( name != 0 )
    {
        delete[] name;

        name = 0;
    }
}


String& String::operator=( const String& arg ){

    if( this != &arg ){

        if( name != 0 )
        {
            delete[] name;

            name = 0;
        }

        copy( arg );
    }
    return *this;
}


String& String::operator=( const char* name_ ){

    return operator=( String(name_) );
}


char String::operator()( uint idx ) const{

    ASSERT( idx < getLength() );
    return name[idx];
}


String& String::operator+=( const char* arg ){

    return operator+=( String(arg) );
}


String& String::operator+=( const String& arg ){

    if( arg.getLength() <= 1 )  return *this;
    if(     getLength() <= 1 )  return operator=(arg);

    String tmp;

    tmp.length = getLength() + arg.getLength()-1;
    tmp.name = new char[tmp.length];

    uint run1;

    for( run1 = 0; run1 < getLength(); run1++ )
        tmp.name[run1] = name[run1];

    for( run1 = 0; run1 < arg.getLength(); run1++ )
        tmp.name[getLength()-1+run1] = arg.name[run1];

    return operator=(tmp);
}


String& String::operator<<( const char* arg )
{
	return operator+=( arg );
}


String& String::operator<<( const String& arg )
{
	return operator+=( arg );
}


BooleanType String::operator==( const String& arg ) const
{
	if ( getLength() != arg.getLength() )
		return BT_FALSE;
	
	for( uint i=0; i<getLength(); ++i )
		if ( name[i] != arg.name[i] )
			return BT_FALSE;
		
	return BT_TRUE;
}


BooleanType String::operator==( const char* const arg ) const
{
	if ( getLength() != strlen( arg ) )
		return BT_FALSE;
	
	for( uint i=0; i<getLength(); ++i )
		if ( name[i] != arg[i] )
			return BT_FALSE;
		
	return BT_TRUE;
}


uint String::getLength() const{

    return length;
}


returnValue String::print() const{

    acadoPrintf( name );
    acadoPrintf( "\n" );
    return SUCCESSFUL_RETURN;
}


returnValue String::print( FILE *file ) const{

    acadoFPrintf( file, name );
    return SUCCESSFUL_RETURN;
}



returnValue String::copy( const String& arg )
{

    if ( name )
    {
    	delete [] name;

    	name  = 0;
    }

    length = arg.length;

    if( length > 0 )
    {
    	name = new char[ length ];

    	for( uint run1 = 0; run1 < length; run1++ )
    		name[run1] = arg.name[run1];
    }
    else{
    	name = 0;
    }

    return SUCCESSFUL_RETURN;
}


BooleanType String::isEmpty() const{

    if( length <= 1 ) return BT_TRUE;
    return BT_FALSE;
}

const char* String::getName() const{

    return name;
}



String operator<<(	const char* _lhs,
					const String& rhs
					)
{
	String newString = _lhs;
	newString << rhs;
	return newString;
}



CLOSE_NAMESPACE_ACADO

// end of file.
