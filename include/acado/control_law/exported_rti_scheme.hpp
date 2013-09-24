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
 *	\file include/acado/control_law/exported_rti_scheme.hpp
 *	\author Hans Joachim Ferreau, Boris Houska
 */


#ifndef ACADO_TOOLKIT_EXPORTED_RTI_SCHEME_HPP
#define ACADO_TOOLKIT_EXPORTED_RTI_SCHEME_HPP


#include <acado/control_law/control_law.hpp>


typedef double real_t;
// typedef float real_t;


BEGIN_NAMESPACE_ACADO


typedef void (*fcnVoidVoid)( void );
typedef void (*fcnVoidDoublePtr)( double* );
typedef double* (*fcnDoublePtrVoid)( void );
typedef int (*fcnIntDoublePtrVoid)( double* );


/** 
 *	\brief Base class for interfacing online feedback laws to be used within a Controller.
 *
 *	\ingroup UserInterfaces
 *
 *  The class ExportedRTIscheme serves as a base class for interfacing online 
 *	control laws to be used within a Controller. Most prominently, the
 *	control law can be a ExportedRTIscheme solving dynamic optimization
 *	problems. But also classical feedback laws like LQR or PID controller
 *	or feedforward laws can be interfaced.
 *
 *	After initialization, the ExportedRTIscheme is evaluated with a given fixed 
 *	sampling time by calling the step-routines. Additionally, the steps
 *	can be divided into a preparation step and a feedback step that actually
 *	computes the feedback. This feature has mainly been added to deal with 
 *	ExportedRTIscheme can make use of this division in order to reduce the
 *	feedback delay.
 *
 *	\author Hans Joachim Ferreau, Boris Houska
 */
class ExportedRTIscheme : public ControlLaw
{
	//
	// PUBLIC MEMBER FUNCTIONS:
	//
	public:

		/** Constructor which takes the optimal control problem to be solved online
		 *	together with the sampling time.
		 *
		 *	@param[in] ocp_				Optimal control problem to be solved online.
		 *	@param[in] _samplingTime	Sampling time.
		 */
		ExportedRTIscheme(	uint _nX,
							uint _nU,
							uint _nPH,
							double _samplingTime,

							fcnVoidVoid _preparationStep,
							fcnIntDoublePtrVoid _feedbackStep,
							fcnVoidDoublePtr _shiftControls,
							fcnVoidDoublePtr _shiftStates,

							fcnDoublePtrVoid _getAcadoVariablesX,
							fcnDoublePtrVoid _getAcadoVariablesU,
							fcnDoublePtrVoid _getAcadoVariablesXRef,
							fcnDoublePtrVoid _getAcadoVariablesURef
							);

		/** Copy constructor (deep copy).
		 *
		 *	@param[in] rhs	Right-hand side object.
		 */
		ExportedRTIscheme(	const ExportedRTIscheme& rhs
							);

		/** Destructor. 
		 */
		virtual ~ExportedRTIscheme( );

		/** Assignment operator (deep copy).
		 *
		 *	@param[in] rhs	Right-hand side object.
		 */
		ExportedRTIscheme& operator=(	const ExportedRTIscheme& rhs
										);

		/** Clone constructor (deep copy).
		 *
		 *	\return Pointer to deep copy of base class type
		 */
		virtual ControlLaw* clone( ) const;


		/** Initializes controls of the control law.
		 *
		 *	@param[in]  _u_init	Initial value for controls.
		 *
		 *  \return SUCCESSFUL_RETURN
		 */
		virtual returnValue initializeControls(	const VariablesGrid& _u_init
												);

		/** Initializes controls of the control law from data file.
		 *
		 *	@param[in]  fileName	Name of file containing initial value for controls.
		 *
		 *  \return SUCCESSFUL_RETURN, \n
		 *	        RET_FILE_CAN_NOT_BE_OPENED
		 */
		virtual returnValue initializeControls(	const char* fileName
												);


		/** Initializes the control law with given start values and 
		 *	performs a number of consistency checks.
		 *
		 *	@param[in]  _startTime	Start time.
		 *	@param[in]  _x			Initial value for differential states.
		 *	@param[in]  _p			Initial value for parameters.
		 *	@param[in]  _yRef		Initial value for reference trajectory.
		 *
		 *  \return SUCCESSFUL_RETURN
		 */
		virtual returnValue init(	double startTime,
									const Vector &_x = emptyConstVector,
									const Vector &_p = emptyConstVector,
									const VariablesGrid& _yRef = emptyConstVariablesGrid
									);


		/** Performs next step of the control law based on given inputs.
		 *
		 *	@param[in]  currentTime	Current time.
		 *	@param[in]  _x			Most recent value for differential states.
		 *	@param[in]  _p			Most recent value for parameters.
		 *	@param[in]  _yRef		Piece of reference trajectory for next step (required for hotstarting).
		 *
		 *  \return SUCCESSFUL_RETURN
		 */
		virtual returnValue step(	double currentTime,
									const Vector& _x,
									const Vector& _p = emptyConstVector,
									const VariablesGrid& _yRef = emptyConstVariablesGrid
									);

		/** Performs next feedback step of the control law based on given inputs.
		 *
		 *	@param[in]  currentTime	Current time.
		 *	@param[in]  _x			Most recent value for differential states.
		 *	@param[in]  _p			Most recent value for parameters.
		 *	@param[in]  _yRef		Current piece of reference trajectory (if not specified during previous preparationStep).
		 *
		 *	\note If a non-empty reference trajectory is provided, this one is used
		 *	      instead of the possibly set-up build-in one.
		 * 
		 *  \return SUCCESSFUL_RETURN
		 */
		virtual returnValue feedbackStep(	double currentTime,
											const Vector &_x,
											const Vector &_p = emptyConstVector,
											const VariablesGrid& _yRef = emptyConstVariablesGrid
											);

		/** Performs next preparation step of the control law based on given inputs.
		 *
		 *	@param[in]  nextTime	Time at next step.
		 *	@param[in]  _yRef		Piece of reference trajectory for next step (required for hotstarting).
		 *
		 *  \return SUCCESSFUL_RETURN
		 */
		virtual returnValue preparationStep(	double nextTime = 0.0,
												const VariablesGrid& _yRef = emptyConstVariablesGrid
												);


		/** Shifts the data for preparating the next real-time step.
		 *
		 *	\return RET_NOT_YET_IMPLEMENTED
		 */
		virtual returnValue shift( );


		/** Assigns new reference trajectory for the next real-time step.
		 *
		 *	@param[in]  ref		Current piece of new reference trajectory.
		 *
		 *	\return SUCCESSFUL_RETURN, \n
		 *	        RET_REFERENCE_SHIFTING_WORKS_FOR_LSQ_TERMS_ONLY, \n
		 *	        RET_MEMBER_NOT_INITIALISED
		 */
		virtual returnValue setReference(	const VariablesGrid &ref
											);

		
		/** Returns number of (estimated) differential states.
		 *
		 *  \return Number of (estimated) differential states
		 */
		virtual uint getNX( ) const;

		/** Returns number of (estimated) algebraic states.
		 *
		 *  \return Number of (estimated) algebraic states
		 */
		virtual uint getNXA( ) const;

		/** Returns number of controls.
		 *
		 *  \return Number of controls
		 */
		virtual uint getNU( ) const;

		/** Returns number of parameters.
		 *
		 *  \return Number of parameters 
		 */
		virtual uint getNP( ) const;

		/** Returns number of (estimated) disturbances.
		 *
		 *  \return Number of (estimated) disturbances 
		 */
		virtual uint getNW( ) const;

		/** Returns number of process outputs.
		 *
		 *  \return Number of process outputs
		 */
		virtual uint getNY( ) const;


		/** Returns length of the prediction horizon (for the case a predictive control law is used).
		 *
		 *  \return Length of the prediction horizon
		 */
		virtual double getLengthPredictionHorizon( ) const;

		/** Returns length of the control horizon (for the case a predictive control law is used).
		 *
		 *  \return Length of the control horizon
		 */
		virtual double getLengthControlHorizon( ) const;


		/** Returns whether the control law is based on dynamic optimization or 
		 *	a static one.
		 *
		 *  \return BT_TRUE  iff control law is based on dynamic optimization, \n
		 *	        BT_FALSE otherwise
		 */
		virtual BooleanType isDynamic( ) const;

		/** Returns whether the control law is a static one or based on dynamic optimization.
		 *
		 *  \return BT_TRUE  iff control law is a static one, \n
		 *	        BT_FALSE otherwise
		 */
		virtual BooleanType isStatic( ) const;

		/** Returns whether the control law is working in real-time mode.
		 *
		 *  \return BT_TRUE  iff control law is working in real-time mode, \n
		 *	        BT_FALSE otherwise
		 */
		virtual BooleanType isInRealTimeMode( ) const;


	//
	// PROTECTED MEMBER FUNCTIONS:
	//
	protected:


	//
	// DATA MEMBERS:
	//
	protected:

		uint nX;
		uint nU;
		uint nPH;

		fcnVoidVoid preparationStepPtr;
		fcnIntDoublePtrVoid feedbackStepPtr;
		fcnVoidDoublePtr shiftControlsPtr;
		fcnVoidDoublePtr shiftStatesPtr;

		fcnDoublePtrVoid getAcadoVariablesXPtr;
		fcnDoublePtrVoid getAcadoVariablesUPtr;
		fcnDoublePtrVoid getAcadoVariablesXRefPtr;
		fcnDoublePtrVoid getAcadoVariablesURefPtr;

	private:

		/** Default constructor.
		 */
		ExportedRTIscheme( );
};


CLOSE_NAMESPACE_ACADO


//#include <acado/control_law/exported_rti_scheme.ipp>


#endif  // ACADO_TOOLKIT_EXPORTED_RTI_SCHEME_HPP


/*
 *	end of file
 */
