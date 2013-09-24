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
*    \file   src/utils/acado_string.cpp
*    \author Boris Houska, Hans Joachim Ferreau
*    \date   2010
*/


#include <acado/utils/acado_string.hpp>
#include <acado/utils/acado_stream.hpp>


BEGIN_NAMESPACE_ACADO


String::String(){}


String::String( const String& arg ){ copy( arg ); }


String::String( const char* name_ ){

//     printf("string constructor \n");
 
    int length = 0;
    while( length < (int) MAX_LENGTH_STRING ){
        if( name_[length] == '\0' ) break;
        length++;
    }
    if( length > 0 ) length++;

    if( length > 0 ){
        for( int run1 = 0; run1 < length-1; run1++ )
            name.push_back( name_[run1] );
        name.push_back('\0');
    }
}


String::String( const double& val_ ){

    int          run1      ;
    char        *char_value;
    char        *tmp       ;
    BooleanType  dot       ;

    const int precision = 16;

    dot = BT_FALSE;

    char_value = (char*)malloc(MAX_LENGTH_STRING*sizeof(char));
    tmp        = (char*)malloc(MAX_LENGTH_STRING*sizeof(char));

    /* return value of gcvt is its third argument; however, in glibc 2.17
       it has attribute warn_unused_result, so to placate
       -Wunused-result, we have... */
    char_value = gcvt( val_ , precision , char_value );

    int length = 0;
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

    for( run1 = 0; run1 < length; run1++ )
        name.push_back(tmp[run1]);

    free(tmp);
}


String::String( const int& val_ ){

    int n = val_;
    const int b = 10;
    char *s = new char[MAX_LENGTH_STRING];
    const char digits[] = "0123456789";
    int sign;

    int length = 0;

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

    for( sign = 0; sign < (int) length-1; sign++ )
        name.push_back(s[length-2-sign]);
    name.push_back('\0');

    delete[] s;
}


String::String( const uint& val_ )
{
    int n = val_;
    const int b = 10;
    char *s = new char[MAX_LENGTH_STRING];
    const char digits[] = "0123456789";
    int sign;

    int length = 0;

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

    for( sign = 0; sign < (int) length-1; sign++ )
        name.push_back(s[length-2-sign]);
    name.push_back('\0');

    delete[] s;
}
	
	

String::~String(){}


String& String::operator=( const String& arg ){

    if( this != &arg ){
        copy( arg );
    }
    return *this;
}


String& String::operator=( const char* name_ ){

    return operator=( String(name_) );
}


char String::operator()( uint idx ) const{

    ASSERT( idx < getLength() );
    return (name.data())[idx];
}


String& String::operator+=( const char* arg ){

    return operator+=( String(arg) );
}


String& String::operator+=( const String& arg ){
 
    if( getLength() > 0 ) name.pop_back();
    for( uint run1 = 0; run1 < arg.getLength(); run1++ )
		name.push_back(((arg.name).data())[run1]);
    return *this;
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

    return name.size();
}


returnValue String::print() const{

    acadoPrintf( (char*) name.data() );
    acadoPrintf( "\n" );
    return SUCCESSFUL_RETURN;
}


returnValue String::print( FILE *file ) const{

    acadoFPrintf( file, (char*) name.data() );
    return SUCCESSFUL_RETURN;
}



returnValue String::copy( const String& arg )
{
    name = arg.name;
    return SUCCESSFUL_RETURN;
}


BooleanType String::isEmpty() const{

    if( getLength() <= 1 ) return BT_TRUE;
    return BT_FALSE;
}

const char* String::getName() const{

    return name.data();
}



String operator<<(	const char* _lhs,
					const String& rhs
					){
	 
	String newString = _lhs;
	newString << rhs;
	return rhs;
}

String operator+(	const String& arg1,
					const String& arg2
					)
{
	String tmp( arg1 );
	return tmp << arg2;
}

std::ostream& operator<<(std::ostream& os, const String& s)
{
  os << s.getName();
  return os;
}

CLOSE_NAMESPACE_ACADO

// end of file.
