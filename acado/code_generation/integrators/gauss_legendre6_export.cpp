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
 *    \file src/code_generation/gauss_legendre6_export.cpp
 *    \author Rien Quirynen
 *    \date 2012
 */

#include <acado/code_generation/integrators/gauss_legendre6_export.hpp>

#include <acado/code_generation/export_algorithm_factory.hpp>

BEGIN_NAMESPACE_ACADO


//
// PUBLIC MEMBER FUNCTIONS:
//

GaussLegendre6Export::GaussLegendre6Export(	UserInteraction* _userInteraction,
									const std::string& _commonHeaderName
									) : ImplicitRungeKuttaExport( _userInteraction,_commonHeaderName )
{
	numStages = 3;
}


GaussLegendre6Export::GaussLegendre6Export(	const GaussLegendre6Export& arg
									) : ImplicitRungeKuttaExport( arg )
{
	numStages = 3;
	copy( arg );
}


GaussLegendre6Export::~GaussLegendre6Export( )
{
	clear( );
}


// PROTECTED:

//
// Register the integrator
//

IntegratorExport* createGaussLegendre6Export(	UserInteraction* _userInteraction,
												const std::string &_commonHeaderName)
{
	DMatrix AA(3,3);
	DVector bb(3);
	DVector cc(3);

	AA(0,0) = 5.0/36.0;
	AA(0,1) = 2.0/9.0-1.0/15.0*sqrt(15.0);
	AA(0,2) = 5.0/36.0-1.0/30.0*sqrt(15.0);
	AA(1,0) = 5.0/36.0+1.0/24.0*sqrt(15.0);
	AA(1,1) = 2.0/9.0;
	AA(1,2) = 5.0/36.0-1.0/24.0*sqrt(15.0);
	AA(2,0) = 5.0/36.0+1.0/30.0*sqrt(15.0);
	AA(2,1) = 2.0/9.0+1.0/15.0*sqrt(15.0);
	AA(2,2) = 5.0/36.0;

	bb(0) = 5.0/18.0;
	bb(1) = 4.0/9.0;
	bb(2) = 5.0/18.0;

	cc(0) = 1.0/2.0-sqrt(15.0)/10.0;
	cc(1) = 1.0/2.0;
	cc(2) = 1.0/2.0+sqrt(15.0)/10.0;

	// SIMPLIFIED NEWTON:
	DMatrix _eig(2,2);
	_eig(0,0) = 3.677814645373912;	_eig(0,1) = 3.508761919567443;	// the second column is the imaginary part
	_eig(1,0) = 4.644370709252173;	_eig(1,1) = 0.0;

	DMatrix _simplified_transf1(3,3);
	_simplified_transf1(0,0) = -18.636486890695167; 	_simplified_transf1(0,1) = -15.250990917842101; 	_simplified_transf1(0,2) = 3.156971624093098;
	_simplified_transf1(1,0) = -27.017033488529613; 	_simplified_transf1(1,1) = 7.134863930477463; 		_simplified_transf1(1,2) = 1.101474723408009;
	_simplified_transf1(2,0) = 28.095293818520428; 	_simplified_transf1(2,1) = 5.341817943917859; 		_simplified_transf1(2,2) = 2.027127483041268;

	DMatrix _simplified_transf2(3,3);
	_simplified_transf2(0,0) = -0.077948357550038; 	_simplified_transf2(0,1) = -0.056982523211087; 	_simplified_transf2(0,2) = 0.071464556714801;
	_simplified_transf2(1,0) = 0.050295169925554; 		_simplified_transf2(1,1) = 0.299699605816585; 		_simplified_transf2(1,2) = 0.117700617809852;
	_simplified_transf2(2,0) = 0.947801449544836; 		_simplified_transf2(2,1) = 0.0; 					_simplified_transf2(2,2) = 0.990474321575646;

	DMatrix _simplified_transf1_T(3,3);
	_simplified_transf1_T(0,0) = -4.866177185043021e-01; 	_simplified_transf1_T(0,1) = 1.236550876742378e+00; 		_simplified_transf1_T(0,2) = 3.485838052042621e+00;
	_simplified_transf1_T(1,0) = 6.393107026830797e-02; 	_simplified_transf1_T(1,1) = 9.257658225120670e-01; 		_simplified_transf1_T(1,2) = -3.325609633473744e+00;
	_simplified_transf1_T(2,0) = 3.319078939559111e-01; 	_simplified_transf1_T(2,1) = 5.466453018169597e-01; 		_simplified_transf1_T(2,2) = 4.600129927392348e+00;

	DMatrix _simplified_transf2_T(3,3);
	_simplified_transf2_T(0,0) = -6.321680651380311e+00; 	_simplified_transf2_T(0,1) = -1.314846346969297e+00; 		_simplified_transf2_T(0,2) = 6.049321980812397e+00;
	_simplified_transf2_T(1,0) = -1.201954582051780e+00; 	_simplified_transf2_T(1,1) = 3.086679860636697e+00; 		_simplified_transf2_T(1,2) = 1.150170448985970e+00;
	_simplified_transf2_T(2,0) = 5.989523296137209e-01; 	_simplified_transf2_T(2,1) = -2.719295284863776e-01; 		_simplified_transf2_T(2,2) = 4.364697845938469e-01;


	// SINGLE NEWTON:
	double _single_tau = 0.202740066519113;
	DVector _lower_triang(3);
	_lower_triang(0) = 1.291180015430378;	_lower_triang(1) = -1.430998974529736;	_lower_triang(2) = 2.108070244672127;

	DMatrix _single_transf1(3,3);
	_single_transf1(0,0) = 1.000000000000000; 	_single_transf1(0,1) = 0.358037439038676; 	_single_transf1(0,2) = -0.031545235223924;
	_single_transf1(1,0) = -1.291180015430378; 	_single_transf1(1,1) = 0.537709213937389; 	_single_transf1(1,2) = 0.059994748344832;
	_single_transf1(2,0) = 1.430998974529736; 	_single_transf1(2,1) = -1.595719036564529; 	_single_transf1(2,2) = 0.914248574982086;

	DMatrix _single_transf2(3,3);
	_single_transf2(0,0) = 1.0; 		_single_transf2(0,1) = -0.358037439038676; 	_single_transf2(0,2) = 0.038442529688880;
	_single_transf2(1,0) = 0.0; 		_single_transf2(1,1) = 1.0; 				_single_transf2(1,2) = -0.019264171041652;
	_single_transf2(2,0) = 0.0; 		_single_transf2(2,1) = 0.0; 				_single_transf2(2,2) = 1.0;

	DMatrix _single_transf1_T(3,3);
	_single_transf1_T(0,0) = 1.517302006625726e+00; 	_single_transf1_T(0,1) = -1.318747024436147e+00; 	_single_transf1_T(0,2) = 1.430998974529736e+00;
	_single_transf1_T(1,0) = -4.390769920057278e-01; 	_single_transf1_T(1,1) = 1.040610225761180e+00; 	_single_transf1_T(1,2) = -2.108070244672127e+00;
	_single_transf1_T(2,0) = 3.844252968887955e-02; 	_single_transf1_T(2,1) = -1.926417104165177e-02; 	_single_transf1_T(2,2) = 1.000000000000000e+00;

	DMatrix _single_transf2_T(3,3);
	_single_transf2_T(0,0) = 1.0; 							_single_transf2_T(0,1) = 0.0; 					_single_transf2_T(0,2) = 0.0;
	_single_transf2_T(1,0) = 3.580374390386760e-01; 		_single_transf2_T(1,1) = 1.0; 					_single_transf2_T(1,2) = 0.0;
	_single_transf2_T(2,0) = -3.154523522392352e-02; 		_single_transf2_T(2,1) = 1.926417104165177e-02; _single_transf2_T(2,2) = 1.0;


	ImplicitRungeKuttaExport* integrator = createImplicitRungeKuttaExport(_userInteraction, _commonHeaderName);
	integrator->initializeButcherTableau(AA, bb, cc);

	integrator->setEigenvalues(_eig);
	integrator->setSimplifiedTransformations(_simplified_transf1, _simplified_transf2, _simplified_transf1_T, _simplified_transf2_T);

	integrator->setSingleTransformations(_single_tau, _lower_triang, _single_transf1, _single_transf2, _single_transf1_T, _single_transf2_T);

	return integrator;
}

CLOSE_NAMESPACE_ACADO

// end of file.
