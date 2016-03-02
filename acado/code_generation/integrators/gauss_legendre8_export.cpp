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
 *    \file src/code_generation/gauss_legendre8_export.cpp
 *    \author Rien Quirynen
 *    \date 2012
 */

#include <acado/code_generation/integrators/gauss_legendre8_export.hpp>

#include <acado/code_generation/export_algorithm_factory.hpp>

BEGIN_NAMESPACE_ACADO


//
// PUBLIC MEMBER FUNCTIONS:
//

GaussLegendre8Export::GaussLegendre8Export(	UserInteraction* _userInteraction,
									const std::string& _commonHeaderName
									) : ImplicitRungeKuttaExport( _userInteraction,_commonHeaderName )
{
	numStages = 4;
}


GaussLegendre8Export::GaussLegendre8Export(	const GaussLegendre8Export& arg
									) : ImplicitRungeKuttaExport( arg )
{
	numStages = 4;
	copy( arg );
}


GaussLegendre8Export::~GaussLegendre8Export( )
{
	clear( );
}


// PROTECTED:

//
// Register the integrator
//

IntegratorExport* createGaussLegendre8Export(	UserInteraction* _userInteraction,
												const std::string &_commonHeaderName)
{
	DMatrix AA(4,4);
	DVector bb(4);
	DVector cc(4);

	AA(0,0) = (1/(double)144)*(double)sqrt((double)(double)30)+(double)1/(double)8;
	AA(0,1) = -(double)(1/(double)840)*(double)sqrt((double)(double)525-(double)70*(double)sqrt((double)(double)30))*(double)sqrt((double)(double)30)+(double)(1/(double)144)*(double)sqrt((double)(double)30)-(double)(1/(double)105)*(double)sqrt((double)(double)525-(double)70*(double)sqrt((double)(double)30))+(double)1/(double)8;		
	AA(0,2) = (1/(double)2352)*(double)sqrt((double)(double)525+(double)70*(double)sqrt((double)(double)30))*(double)sqrt((double)(double)30)-(double)(1/(double)144)*(double)sqrt((double)(double)30)+(double)(1/(double)1680)*(double)sqrt((double)(double)525-(double)70*(double)sqrt((double)(double)30))*(double)sqrt((double)(double)30)+(double)1/(double)8+(double)(1/(double)1470)*(double)sqrt((double)(double)525+(double)70*(double)sqrt((double)(double)30))-(double)(1/(double)420)*(double)sqrt((double)(double)525-(double)70*(double)sqrt((double)(double)30));	
	AA(0,3) = -(double)(1/(double)2352)*(double)sqrt((double)(double)525+(double)70*(double)sqrt((double)(double)30))*(double)sqrt((double)(double)30)-(double)(1/(double)144)*(double)sqrt((double)(double)30)+(double)(1/(double)1680)*(double)sqrt((double)(double)525-(double)70*(double)sqrt((double)(double)30))*(double)sqrt((double)(double)30)+(double)1/(double)8-(double)(1/(double)1470)*(double)sqrt((double)(double)525+(double)70*(double)sqrt((double)(double)30))-(double)(1/(double)420)*(double)sqrt((double)(double)525-(double)70*(double)sqrt((double)(double)30));
	AA(1,0) = (1/(double)840)*(double)sqrt((double)(double)525-(double)70*(double)sqrt((double)(double)30))*(double)sqrt((double)(double)30)+(double)(1/(double)144)*(double)sqrt((double)(double)30)+(double)(1/(double)105)*(double)sqrt((double)(double)525-(double)70*(double)sqrt((double)(double)30))+(double)1/(double)8;			
	AA(1,1) = (1/(double)144)*(double)sqrt((double)(double)30)+(double)1/(double)8;		
	AA(1,2) = (1/(double)2352)*(double)sqrt((double)(double)525+(double)70*(double)sqrt((double)(double)30))*(double)sqrt((double)(double)30)-(double)(1/(double)144)*(double)sqrt((double)(double)30)-(double)(1/(double)1680)*(double)sqrt((double)(double)525-(double)70*(double)sqrt((double)(double)30))*(double)sqrt((double)(double)30)+(double)1/(double)8+(double)(1/(double)1470)*(double)sqrt((double)(double)525+(double)70*(double)sqrt((double)(double)30))+(double)(1/(double)420)*(double)sqrt((double)(double)525-(double)70*(double)sqrt((double)(double)30));
	AA(1,3) = -(double)(1/(double)2352)*(double)sqrt((double)(double)525+(double)70*(double)sqrt((double)(double)30))*(double)sqrt((double)(double)30)-(double)(1/(double)144)*(double)sqrt((double)(double)30)-(double)(1/(double)1680)*(double)sqrt((double)(double)525-(double)70*(double)sqrt((double)(double)30))*(double)sqrt((double)(double)30)+(double)1/(double)8-(double)(1/(double)1470)*(double)sqrt((double)(double)525+(double)70*(double)sqrt((double)(double)30))+(double)(1/(double)420)*(double)sqrt((double)(double)525-(double)70*(double)sqrt((double)(double)30));
	AA(2,0) = -(double)(1/(double)2352)*(double)sqrt((double)(double)525-(double)70*(double)sqrt((double)(double)30))*(double)sqrt((double)(double)30)+(double)(1/(double)144)*(double)sqrt((double)(double)30)-(double)(1/(double)1680)*(double)sqrt((double)(double)525+(double)70*(double)sqrt((double)(double)30))*(double)sqrt((double)(double)30)+(double)(1/(double)1470)*(double)sqrt((double)(double)525-(double)70*(double)sqrt((double)(double)30))+(double)1/(double)8-(double)(1/(double)420)*(double)sqrt((double)(double)525+(double)70*(double)sqrt((double)(double)30));	
	AA(2,1) = (1/(double)2352)*(double)sqrt((double)(double)525-(double)70*(double)sqrt((double)(double)30))*(double)sqrt((double)(double)30)+(double)(1/(double)144)*(double)sqrt((double)(double)30)-(double)(1/(double)1680)*(double)sqrt((double)(double)525+(double)70*(double)sqrt((double)(double)30))*(double)sqrt((double)(double)30)-(double)(1/(double)1470)*(double)sqrt((double)(double)525-(double)70*(double)sqrt((double)(double)30))+(double)1/(double)8-(double)(1/(double)420)*(double)sqrt((double)(double)525+(double)70*(double)sqrt((double)(double)30));				
	AA(2,2) = -(double)(1/(double)144)*(double)sqrt((double)(double)30)+(double)1/(double)8;	
	AA(2,3) = (1/(double)840)*(double)sqrt((double)(double)525+(double)70*(double)sqrt((double)30))*(double)sqrt((double)30)-(double)(1/(double)144)*(double)sqrt((double)30)-(double)(1/(double)105)*(double)sqrt((double)525+(double)70*(double)sqrt((double)30))+(double)1/(double)8;
	AA(3,0) = -(double)(1/(double)2352)*(double)sqrt((double)525-(double)70*(double)sqrt((double)30))*(double)sqrt((double)30)+(double)(1/(double)144)*(double)sqrt((double)30)+(double)(1/(double)1680)*(double)sqrt((double)525+(double)70*(double)sqrt((double)30))*(double)sqrt((double)30)+(double)(1/(double)1470)*(double)sqrt((double)525-(double)70*(double)sqrt((double)30))+(double)1/(double)8+(double)(1/(double)420)*(double)sqrt((double)525+(double)70*(double)sqrt((double)30));
	AA(3,1) = (1/(double)2352)*(double)sqrt((double)525-(double)70*(double)sqrt((double)30))*(double)sqrt((double)30)+(double)(1/(double)144)*(double)sqrt((double)30)+(double)(1/(double)1680)*(double)sqrt((double)525+(double)70*(double)sqrt((double)30))*(double)sqrt((double)30)-(double)(1/(double)1470)*(double)sqrt((double)525-(double)70*(double)sqrt((double)30))+(double)1/(double)8+(double)(1/(double)420)*(double)sqrt((double)525+(double)70*(double)sqrt((double)30));
	AA(3,2) = -(double)(1/(double)840)*(double)sqrt((double)525+(double)70*(double)sqrt((double)30))*(double)sqrt((double)30)-(double)(1/(double)144)*(double)sqrt((double)30)+(double)(1/(double)105)*(double)sqrt((double)525+(double)70*(double)sqrt((double)30))+(double)1/(double)8;
	AA(3,3) = -(double)(1/(double)144)*(double)sqrt((double)30)+(double)1/(double)8;		

	bb(0) = (1/(double)72)*(double)sqrt((double)30)+(double)1/(double)4;		
	bb(1) = (1/(double)72)*(double)sqrt((double)30)+(double)1/(double)4;					
	bb(2) = -(double)(1/(double)72)*(double)sqrt((double)30)+(double)1/(double)4;
	bb(3) = -(double)(1/(double)72)*(double)sqrt((double)30)+(double)1/(double)4;

	cc(0) = 1/(double)2-(double)(1/(double)70)*(double)sqrt((double)525-(double)70*(double)sqrt((double)30));		
	cc(1) = 1/(double)2+(double)(1/(double)70)*(double)sqrt((double)525-(double)70*(double)sqrt((double)30));	
	cc(2) = 1/(double)2-(double)(1/(double)70)*(double)sqrt((double)525+(double)70*(double)sqrt((double)30));
	cc(3) = 1/(double)2+(double)(1/(double)70)*(double)sqrt((double)525+(double)70*(double)sqrt((double)30));


	// SIMPLIFIED NEWTON:
	DMatrix _eig(2,2);
	_eig(0,0) = 4.207578794359248;	_eig(0,1) = 5.314836083713501;	// the second column is the imaginary part
	_eig(1,0) = 5.792421205640746;	_eig(1,1) = 1.734468257869010;

	DMatrix _simplified_transf1(4,4);
	_simplified_transf1(0,0) = -4.511944766014135e+00; 	_simplified_transf1(0,1) = 1.410200318172509e+01; 		_simplified_transf1(0,2) = -6.490069796781762e+01; 		_simplified_transf1(0,3) = -3.260131534153535e+00;
	_simplified_transf1(1,0) = 4.646043280390278e+01; 	_simplified_transf1(1,1) = -7.224788843443845e+00; 		_simplified_transf1(1,2) = 1.062619320722611e+01; 		_simplified_transf1(1,3) = 1.316684254020140e-01;
	_simplified_transf1(2,0) = 7.890091964536154e+00; 	_simplified_transf1(2,1) = -4.304210837983620e-01; 		_simplified_transf1(2,2) = -7.755516515388237e+01; 		_simplified_transf1(2,3) = 4.826224952821270e+00;
	_simplified_transf1(3,0) = 7.293296595763626e+01; 	_simplified_transf1(3,1) = 1.055652513964904e+01; 		_simplified_transf1(3,2) = 1.646623485807168e+02; 		_simplified_transf1(3,3) = -2.099157718135189e-01;

	DMatrix _simplified_transf2(4,4);
	_simplified_transf2(0,0) = 1.134792325317037e-01; 		_simplified_transf2(0,1) = 3.649668660358009e-02; 		_simplified_transf2(0,2) = 2.786586696875059e-02; 		_simplified_transf2(0,3) = 1.176262506147764e-02;
	_simplified_transf2(1,0) = -8.997398076613361e-02; 		_simplified_transf2(1,1) = -3.394498084881714e-01; 		_simplified_transf2(1,2) = 1.792676149982848e-01; 		_simplified_transf2(1,3) = 1.094022826731096e-01;
	_simplified_transf2(2,0) = -4.567693894474303e-02; 		_simplified_transf2(2,1) = 5.596900829318879e-03; 		_simplified_transf2(2,2) = -1.205649800968665e-02; 		_simplified_transf2(2,3) = 2.295382130562243e-02;
	_simplified_transf2(3,0) = -9.275512157844222e-01; 		_simplified_transf2(3,1) = 0.0; 						_simplified_transf2(3,2) = 9.768864620884530e-01; 		_simplified_transf2(3,3) = 0.0;

	DMatrix _simplified_transf1_T(4,4);
	_simplified_transf1_T(0,0) = 6.714467192972491e-01; 		_simplified_transf1_T(0,1) = -2.182692704278241e+00; 		_simplified_transf1_T(0,2) = -1.624427092105129e-01; 			_simplified_transf1_T(0,3) = -3.902744826216674e+00;
	_simplified_transf1_T(1,0) = -4.495608351940147e-01; 		_simplified_transf1_T(1,1) = -9.500648563729466e-01; 		_simplified_transf1_T(1,2) = 2.663148445406726e-01; 			_simplified_transf1_T(1,3) = 4.929782671143375e+00;
	_simplified_transf1_T(2,0) = 1.818127385417024e-01; 		_simplified_transf1_T(2,1) = 1.228148321235627e+00; 		_simplified_transf1_T(2,2) = -3.002364028567489e-02; 			_simplified_transf1_T(2,3) = 5.658537858504520e+00;
	_simplified_transf1_T(3,0) = 1.980161710480598e-02; 	_simplified_transf1_T(3,1) = 3.227701142228159e-01; 			_simplified_transf1_T(3,2) = 1.538698143800381e-01; 			_simplified_transf1_T(3,3) = -1.694378560034379e+00;

	DMatrix _simplified_transf2_T(4,4);
	_simplified_transf2_T(0,0) = 4.960593241309737e+00; 		_simplified_transf2_T(0,1) = 4.776070474379495e+00; 		_simplified_transf2_T(0,2) = 4.710070689435160e+00; 		_simplified_transf2_T(0,3) = 1.118073005316397e+01;
	_simplified_transf2_T(1,0) = 4.556294554649469e-01; 		_simplified_transf2_T(1,1) = -2.292621287825622e+00; 		_simplified_transf2_T(1,2) = 4.326190112822336e-01; 		_simplified_transf2_T(1,3) = 1.692929579651415e+00;
	_simplified_transf2_T(2,0) = -4.713659630803845e+00; 		_simplified_transf2_T(2,1) = 8.479584873656380e+00; 		_simplified_transf2_T(2,2) = -4.475607853136751e+00; 		_simplified_transf2_T(2,3) = 2.976737053733966e+01;
	_simplified_transf2_T(3,0) = -2.832888732208244e-01; 		_simplified_transf2_T(3,1) = 3.891317146824644e-01; 		_simplified_transf2_T(3,2) = 7.546783478294038e-01; 		_simplified_transf2_T(3,3) = -2.622187436137307e-01;


	// SINGLE NEWTON:
	double _single_tau = 1.561969968460128e-01;
	DVector _lower_triang(6);
	_lower_triang(0) = 9.627423789846739e-01;	_lower_triang(1) = -1.194428300588649e+00;	_lower_triang(2) = 1.918753137082504e+00;
	_lower_triang(3) = 1.649572580382698e+00;	_lower_triang(4) = -2.628995768624925e+00;	_lower_triang(5) = 2.357166809194904e+00;

	DMatrix _single_transf1(4,4);
	_single_transf1(0,0) = 1.000000000000000e+00; 	_single_transf1(0,1) = 6.677448107835342e-01; 	_single_transf1(0,2) = 1.416759981647328e-02; 	_single_transf1(0,3) = -6.505535658246753e-02;
	_single_transf1(1,0) = -9.627423789846739e-01; 	_single_transf1(1,1) = 3.571337723115894e-01; 	_single_transf1(1,2) = 2.017094296173482e-01; 	_single_transf1(1,3) = -2.664322715789577e-02;
	_single_transf1(2,0) = 1.194428300588649e+00; 	_single_transf1(2,1) = -1.121179837511438e+00; 	_single_transf1(2,2) = 6.037202706082401e-01; 	_single_transf1(2,3) = 1.783722705962468e-02;
	_single_transf1(3,0) = -1.649572580382698e+00; 	_single_transf1(3,1) = 1.527502238063574e+00; 	_single_transf1(3,2) = -1.814385214672611e+00; 	_single_transf1(3,3) = 1.051177861607520e+00;

	DMatrix _single_transf2(4,4);
	_single_transf2(0,0) = 1.0; 		_single_transf2(0,1) = -6.677448107835342e-01; 	_single_transf2(0,2) = 1.296306965460327e-01; 	_single_transf2(0,3) = 1.526277075698497e-02;
	_single_transf2(1,0) = 0.0; 		_single_transf2(1,1) = 1.0; 					_single_transf2(1,2) = -2.153491783691625e-01; 	_single_transf2(1,3) = 7.296098377515141e-02;
	_single_transf2(2,0) = 0.0; 		_single_transf2(2,1) = 0.0; 					_single_transf2(2,2) = 1.0; 					_single_transf2(2,3) = 7.575507029183778e-02;
	_single_transf2(3,0) = 0.0; 		_single_transf2(3,1) = 0.0; 					_single_transf2(3,2) = 0.0; 					_single_transf2(3,3) = 1.0;

	DMatrix _single_transf1_T(4,4);
	_single_transf1_T(0,0) = 1.772523752126622e+00; 	_single_transf1_T(0,1) = -1.340315970410551e+00; 	_single_transf1_T(0,2) = 1.069464813810269e+00; 	_single_transf1_T(0,3) = -1.649572580382698e+00;
	_single_transf1_T(1,0) = -8.763483567058189e-01; 	_single_transf1_T(1,1) = 1.605016029183555e+00; 	_single_transf1_T(1,2) = -1.719593377833379e+00; 	_single_transf1_T(1,3) = 2.628995768624925e+00;
	_single_transf1_T(2,0) = 9.365379990131717e-02; 	_single_transf1_T(2,1) = -3.873303876901573e-01; 	_single_transf1_T(2,2) = 8.214326626798530e-01; 	_single_transf1_T(2,3) = -2.357166809194904e+00;
	_single_transf1_T(3,0) = 1.526277075698497e-02; 	_single_transf1_T(3,1) = 7.296098377515141e-02; 	_single_transf1_T(3,2) = 7.575507029183778e-02; 	_single_transf1_T(3,3) = 1.000000000000000e+00;

	DMatrix _single_transf2_T(4,4);
	_single_transf2_T(0,0) = 1.0; 							_single_transf2_T(0,1) = 0.0; 						_single_transf2_T(0,2) = 0.0; 						_single_transf2_T(0,3) = 0.0;
	_single_transf2_T(1,0) = 6.677448107835342e-01; 		_single_transf2_T(1,1) = 1.0; 						_single_transf2_T(1,2) = 0.0; 						_single_transf2_T(1,3) = 0.0;
	_single_transf2_T(2,0) = 1.416759981647328e-02; 		_single_transf2_T(2,1) = 2.153491783691625e-01; 	_single_transf2_T(2,2) = 1.0; 						_single_transf2_T(2,3) = 0.0;
	_single_transf2_T(3,0) = -6.505535658246753e-02; 		_single_transf2_T(3,1) = -8.927477591979682e-02; 	_single_transf2_T(3,2) = -7.575507029183778e-02; 	_single_transf2_T(3,3) = 1.0;


	ImplicitRungeKuttaExport* integrator = createImplicitRungeKuttaExport(_userInteraction, _commonHeaderName);
	integrator->initializeButcherTableau(AA, bb, cc);

	integrator->setEigenvalues(_eig);
	integrator->setSimplifiedTransformations(_simplified_transf1, _simplified_transf2,_simplified_transf1_T, _simplified_transf2_T);

	integrator->setSingleTransformations(_single_tau, _lower_triang, _single_transf1, _single_transf2, _single_transf1_T, _single_transf2_T);

	return integrator;
}

CLOSE_NAMESPACE_ACADO

// end of file.
