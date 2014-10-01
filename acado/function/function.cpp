/*
 *    This file is part of ACADO Toolkit.
 *
 *    ACADO Toolkit -- A Toolkit for Automatic Control and Dynamic Optimization.
 *    Copyright (C) 2008-2014 by Boris Houska, Hans Joachim Ferreau,
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
 *    \file src/function/function.cpp
 *    \authors Boris Houska, Hans Joachim Ferreau, Milan Vukov
 *    \date 2008 - 2013
 */


#include <acado/utils/acado_utils.hpp>
#include <acado/symbolic_expression/symbolic_expression.hpp>
#include <acado/function/evaluation_point.hpp>
#include <acado/function/function_.hpp>



BEGIN_NAMESPACE_ACADO


//
// PUBLIC MEMBER FUNCTIONS:
//

Function::Function(){

    memoryOffset = 0;
	result       = 0;
}


Function::Function( const Function& arg ){

    evaluationTree = arg.evaluationTree;
    memoryOffset   = arg.memoryOffset  ;
	
	if ( arg.getDim() != 0 )
	{
		result = (double*) calloc( arg.getDim(),sizeof(double) );
		for( int i=0; i<getDim(); ++i )
			result[i] = arg.result[i];
	}
	else
		result = 0;
}


Function::~Function( ){ 
	if ( result != 0 )
		free( result );
}


Function& Function::operator=( const Function& arg ){

    if ( this != &arg ){

		if ( result != 0 )
			free( result );

        evaluationTree = arg.evaluationTree;
        memoryOffset   = arg.memoryOffset  ;
		
		if ( arg.getDim() != 0 )
		{
			result = (double*) calloc( arg.getDim(),sizeof(double) );
			for( int i=0; i<getDim(); ++i )
				result[i] = arg.result[i];
		}
		else
			result = 0;
    }

    return *this;
}


Function& Function::operator<<( const Expression& arg ){

    evaluationTree.operator<<( arg );

	result = (double*) realloc( result,getDim()*sizeof(double) );

    return *this;
}


Function& Function::operator<<( const double &arg ){

    Expression tmp;
    tmp = arg;

    return operator<<(tmp);
}


Function& Function::operator<<( const DVector& arg ){

    Expression tmp;
    tmp = arg;

    return operator<<(tmp);
}


Function& Function::operator<<( const DMatrix& arg ){

    Expression tmp;
    tmp = arg;

    return operator<<(tmp);
}


returnValue Function::getExpression( Expression& expression ) const{

    return evaluationTree.getExpression( expression );
}


Function Function::operator()(	uint idx
								) const
{
	Function tmp;

	if ( (int)idx >= getDim() )
	{
		ACADOERROR( RET_INDEX_OUT_OF_BOUNDS );
		return tmp;
	}

	tmp.evaluationTree = evaluationTree;
	tmp.memoryOffset   = memoryOffset  ;

	return tmp;
}


returnValue Function::reset( ){

    FunctionEvaluationTree tmp;
    evaluationTree = tmp;
    memoryOffset = 0;

	if ( result != 0 )
		free( result );

    return SUCCESSFUL_RETURN;
}


int Function::index( VariableType variableType_, int index_ ) const{

    return evaluationTree.index( variableType_, index_ );
}


double Function::scale( VariableType variableType_, int index_ ) const{

    return evaluationTree.scale( variableType_, index_ );
}

int Function::getN   (VariableType &variableType_) const{
    switch(variableType_) {
	case VT_DIFFERENTIAL_STATE 	: return getNX(); break;	
	case VT_ALGEBRAIC_STATE 	: return getNXA(); break;		
	case VT_CONTROL 			: return getNU(); break;
	case VT_INTEGER_CONTROL 	: return getNUI(); break;	
	case VT_PARAMETER 			: return getNP(); break;
	case VT_ONLINE_DATA 		: return getNOD(); break;
	case VT_INTEGER_PARAMETER 	: return getNPI(); break;
	case VT_DISTURBANCE 		: return getNW(); break;
	case VT_TIME 				: return 1; break;
	case VT_INTERMEDIATE_STATE 	: return getN(); break;
	case VT_DDIFFERENTIAL_STATE	: return getNDX();	 break;
	case VT_VARIABLE			: return getNX()+getNXA()+getNU()+getNUI()+getNP()+getNPI()+getNW()+getNDX();	 break;
	case VT_OUTPUT 	 			: return 0; break;
	case VT_UNKNOWN 			: return 0;
    }
    return evaluationTree.getNX();
}

int Function::getNX   () const{

    return evaluationTree.getNX();
}

int Function::getNXA  () const{

    return evaluationTree.getNXA();
}

int Function::getNDX   () const{

    return evaluationTree.getNDX();
}

int Function::getNU   () const{

    return evaluationTree.getNU();
}

int Function::getNUI   () const{


    return evaluationTree.getNUI();
}

int Function::getNP   () const{


    return evaluationTree.getNP();
}

int Function::getNPI   () const{


    return evaluationTree.getNPI();
}

int Function::getNW   () const{


    return evaluationTree.getNW();
}

int Function::getNT   () const{


    return evaluationTree.getNT();
}

int Function::getNOD   () const{


    return evaluationTree.getNOD();
}


int Function::getNumberOfVariables() const{


    return evaluationTree.getNumberOfVariables();
}


Operator* Function::getExpression( uint componentIdx ) const{

    return evaluationTree.getExpression( componentIdx );
}


returnValue Function::evaluate( int number, double *x, double *_result ){

//     return evaluationTree.evaluate( number+memoryOffset, x, _result );

    evaluationTree.evaluate( number+memoryOffset, x, _result );



    return SUCCESSFUL_RETURN;
}



returnValue Function::substitute( VariableType variableType_, int index_,
                                  double sub_ ){



    if( isSymbolic() == BT_TRUE ){
        FunctionEvaluationTree tmp;
        tmp = evaluationTree.substitute(variableType_,index_,sub_);
        evaluationTree = tmp;
        return SUCCESSFUL_RETURN;
    }

    return ACADOERROR(RET_ONLY_SUPPORTED_FOR_SYMBOLIC_FUNCTIONS);
}



NeutralElement Function::isOneOrZero(){

    return evaluationTree.isOneOrZero();
}



BooleanType Function::isDependingOn( const Expression     &variable ){

    return evaluationTree.isDependingOn( variable );
}

BooleanType Function::isLinearIn( const Expression     &variable ){

    return evaluationTree.isLinearIn( variable );
}


BooleanType Function::isPolynomialIn( const Expression     &variable ){

    return evaluationTree.isPolynomialIn( variable );
}


BooleanType Function::isRationalIn( const Expression &variable ){

    return evaluationTree.isRationalIn( variable );
}


BooleanType Function::isNondecreasing(){



    if( evaluationTree.getMonotonicity() == MT_NONDECREASING ||
        evaluationTree.getMonotonicity() == MT_CONSTANT       )
        return BT_TRUE;

    return BT_FALSE;
}


BooleanType Function::isNonincreasing(){



    if( evaluationTree.getMonotonicity() == MT_NONINCREASING ||
        evaluationTree.getMonotonicity() == MT_CONSTANT       )
        return BT_TRUE;

    return BT_FALSE;
}

BooleanType Function::isConstant()
{
	if (evaluationTree.getMonotonicity() == MT_CONSTANT)
		return BT_TRUE;
	return BT_FALSE;
}


BooleanType Function::isAffine(){

     // (should be implemented more efficiently)

    if( isConvex() == BT_TRUE && isConcave() == BT_TRUE )
        return BT_TRUE;
    return BT_FALSE;
}


BooleanType Function::isConvex(){



    if( evaluationTree.getCurvature() == CT_CONVEX   ||
        evaluationTree.getCurvature() == CT_AFFINE   ||
        evaluationTree.getCurvature() == CT_CONSTANT  )
        return BT_TRUE;

    return BT_FALSE;
}


BooleanType Function::isConcave(){



    if( evaluationTree.getCurvature() == CT_CONCAVE  ||
        evaluationTree.getCurvature() == CT_AFFINE   ||
        evaluationTree.getCurvature() == CT_CONSTANT  )
        return BT_TRUE;

    return BT_FALSE;
}

returnValue Function::jacobian(DMatrix &x) {
    int n=getDim();
    int N=getNumberOfVariables();
    returnValue ret;

    x=DMatrix(getNX(),n);x.setAll(0);
    //u=DMatrix(getNU(),n);u.setAll(0);
    //p=DMatrix(getNP(),n);p.setAll(0);
    //w=DMatrix(getNW(),n);w.setAll(0);
    double *Jr=new double[N];
    double *seed=new double[n];
    for (int i=0;i<n;i++) seed[i]=0;
    for (int i=0;i<n;i++) {
	if (i>0) seed[i-1]=0;
	seed[i]=1;
	for (int j=0;j<N;j++) Jr[j]=0;
    	ret=AD_backward(0,seed,Jr);
	if (ret != SUCCESSFUL_RETURN) return ret;
	for (int j=0;j<getNX();j++) x(j,i)=Jr[index(VT_DIFFERENTIAL_STATE,j)];
    }
    delete[] Jr;
    delete[] seed;
    return SUCCESSFUL_RETURN;
}


returnValue Function::AD_forward( int number, double *seed, double *df  ){

    return evaluationTree.AD_forward( number+memoryOffset, seed, df );
}


returnValue Function::AD_backward( int number, double *seed, double  *df ){

    return evaluationTree.AD_backward( number+memoryOffset, seed, df );
}


returnValue Function::AD_forward2( int number, double *seed, double *dseed,
                                   double *df, double *ddf ){

    return evaluationTree.AD_forward2( number+memoryOffset, seed, dseed, df, ddf );
}


returnValue Function::AD_backward2( int number, double *seed1, double *seed2,
                                    double *df, double *ddf ){

    return evaluationTree.AD_backward2( number+memoryOffset, seed1, seed2, df, ddf );
}



std::ostream& operator<<(std::ostream& stream, const Function& arg)
{
    arg.print( stream );

    return stream;
}

returnValue Function::print(	std::ostream& stream,
								const char *fcnName ,
								const char *realString
								) const
{
	if (getDim() > 0)
		return evaluationTree.C_print(stream, fcnName, realString);

	return SUCCESSFUL_RETURN;
}

returnValue Function::exportForwardDeclarations(	std::ostream& stream,
													const char *fcnName  ,
													const char *realString
													) const
{
	if (getDim() > 0)
		return evaluationTree.exportForwardDeclarations(stream, fcnName, realString);

	return SUCCESSFUL_RETURN;
}


returnValue Function::exportCode(	std::ostream& stream,
									const char *fcnName,
									const char *realString,
									uint        _numX,
									uint		_numXA,
									uint		_numU,
									uint		_numP,
									uint		_numDX,
									uint		_numOD,
									bool       allocateMemory,
									bool       staticMemory
									) const
{
	if (getDim() > 0)
		return evaluationTree.exportCode(stream, fcnName, realString,
				_numX, _numXA, _numU, _numP, _numDX, _numOD, allocateMemory, staticMemory);

	return SUCCESSFUL_RETURN;
}


returnValue Function::clearBuffer(){

    return evaluationTree.clearBuffer();
}



returnValue Function::setScale( double *scale_ ){

    return evaluationTree.setScale(scale_);
}


DVector Function::evaluate( const EvaluationPoint &x,
                           const int        &number  ){

    //double *result = new double[getDim()];

    evaluate( number, x.getEvaluationPointer(), result );
    DVector res( getDim(), result );

    //delete[] result;
    return   res;
}


DVector Function::AD_forward( const EvaluationPoint &x,
                             const int        &number  ){

    //double *result = new double[getDim()];

    AD_forward( number, x.getEvaluationPointer(), result );
    DVector res( getDim(), result );

    //delete[] result;
    return   res;
}


returnValue Function::AD_backward( const    DVector &seed  ,
                                   EvaluationPoint &df    ,
                                   const int       &number  ){


    if( ADisSupported() == BT_TRUE ){

        int run1;
        double *seed_ = new double[getDim()];

        for( run1 = 0; run1 < getDim(); run1++ )
            seed_[run1] = seed(run1);

        AD_backward( number, seed_, df.getEvaluationPointer() );

        delete[] seed_;
        return SUCCESSFUL_RETURN;
    }

    uint run1, run2;
    double *fseed = new double[getNumberOfVariables()+1];
    for( run2 = 0; (int) run2 < getNumberOfVariables()+1; run2++ )
        fseed[run2] = 0.0;

    double *J;
    J = new double[seed.getDim()];

    for( run2 = 0; (int) run2 < getNumberOfVariables()+1; run2++ ){

        fseed[run2] = 1.0;
        AD_forward( 0, fseed, J );
        fseed[run2] = 0.0;

        df.z[run2] = 0.0;
        for( run1 = 0; run1 < seed.getDim(); run1++ )
            df.z[run2] += J[run1]*seed(run1);
    }

    delete[] J    ;
    delete[] fseed;

    return SUCCESSFUL_RETURN;
}

std::string Function::getGlobalExportVariableName( ) const
{
	return evaluationTree.getGlobalExportVariableName( );
}

returnValue Function::setGlobalExportVariableName(const std::string& var)
{
	return evaluationTree.setGlobalExportVariableName( var );
}

unsigned Function::getGlobalExportVariableSize( ) const
{
	return evaluationTree.getGlobalExportVariableSize( );
}


CLOSE_NAMESPACE_ACADO

// end of file.
