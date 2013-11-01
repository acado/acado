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
 *    \file include/acado/ocp/model_container.hpp
 *    \author Rien Quirynen
 */


#ifndef ACADO_TOOLKIT_MODELCONTAINER_HPP
#define ACADO_TOOLKIT_MODELCONTAINER_HPP

#include <acado/ocp/model_data.hpp>


BEGIN_NAMESPACE_ACADO


/**
 *	\brief Container class to store and pass data to the ModelData class.
 *
 *	\ingroup BasicDataStructures
 *
 *	TODO: Rien
 *
 *  \author Rien Quirynen
 */

class ModelContainer {


//
// PUBLIC MEMBER FUNCTIONS:
//
public:


    /**
     * Default constructor.
     */
    ModelContainer( );


    /** Assigns the model dimensions to be used by the integrator.
     *
     *	@param[in] _NX1		Number of differential states in linear input subsystem.
     *	@param[in] _NX2		Number of differential states in nonlinear subsystem.
     *	@param[in] _NX3		Number of differential states in linear output subsystem.
     *	@param[in] _NDX		Number of differential states derivatives.
     *	@param[in] _NDX3	Number of differential states derivatives in the linear output subsystem.
     *	@param[in] _NXA		Number of algebraic states.
     *	@param[in] _NXA3	Number of algebraic states in the linear output subsystem.
     *	@param[in] _NU		Number of control inputs
     *	@param[in] _NP		Number of parameters
     *
     *	\return SUCCESSFUL_RETURN
     */
    returnValue setDimensions( uint _NX1, uint _NX2, uint _NX3, uint _NDX, uint _NDX3, uint _NXA, uint _NXA3, uint _NU, uint _NP );
    returnValue setDimensions( uint _NX1, uint _NX2, uint _NX3, uint _NDX, uint _NDX3, uint _NXA, uint _NXA3, uint _NU );


    /** Assigns the model dimensions to be used by the integrator.
     *
     *	@param[in] _NX1		Number of differential states in linear input subsystem.
     *	@param[in] _NX2		Number of differential states in nonlinear subsystem.
     *	@param[in] _NX3		Number of differential states in linear output subsystem.
     *	@param[in] _NDX		Number of differential states derivatives.
     *	@param[in] _NXA		Number of algebraic states.
     *	@param[in] _NU		Number of control inputs
     *	@param[in] _NP		Number of parameters
     *
     *	\return SUCCESSFUL_RETURN
     */
    returnValue setDimensions( uint _NX1, uint _NX2, uint _NX3, uint _NDX, uint _NXA, uint _NU, uint _NP );
    returnValue setDimensions( uint _NX1, uint _NX2, uint _NX3, uint _NDX, uint _NXA, uint _NU );


	/** Assigns the model dimensions to be used by the integrator.
	 *
	 *	@param[in] _NX		Number of differential states.
	 *	@param[in] _NDX		Number of differential states derivatives.
	 *	@param[in] _NXA		Number of algebraic states.
	 *	@param[in] _NU		Number of control inputs
     *	@param[in] _NP		Number of parameters
	 *
	 *	\return SUCCESSFUL_RETURN
	 */
	returnValue setDimensions( uint _NX, uint _NDX, uint _NXA, uint _NU, uint _NP );
	returnValue setDimensions( uint _NX, uint _NDX, uint _NXA, uint _NU );


	/** Assigns the model dimensions to be used by the integrator.
	 *
	 *	@param[in] _NX		Number of differential states.
	 *	@param[in] _NU		Number of control inputs
	 *	@param[in] _NP		Number of parameters
	 *
	 *	\return SUCCESSFUL_RETURN
	 */
	returnValue setDimensions( uint _NX, uint _NU, uint _NP );
	returnValue setDimensions( uint _NX, uint _NU );


    /** Assigns Differential Equation to be used by the integrator.
     *
     *	@param[in] f		Differential equation.
     *
     *	\return SUCCESSFUL_RETURN
     */

    returnValue setModel( const DifferentialEquation& _f );


    /** Assigns a polynomial NARX model to be used by the integrator.
	 *
	 *	@param[in] delay		The delay for the states in the NARX model.
	 *	@param[in] parms		The parameters defining the polynomial NARX model.
	 *
	 *	\return SUCCESSFUL_RETURN
	 */
	returnValue setNARXmodel( const uint _delay, const Matrix& _parms );


    /** .
     *
     *	@param[in] 		.
     *
     *	\return SUCCESSFUL_RETURN
     */
    returnValue setLinearInput( const Matrix& A1_, const Matrix& B1_ );


    /** .
     *
     *	@param[in] 		.
     *
     *	\return SUCCESSFUL_RETURN
     */
    returnValue setLinearInput( const Matrix& M1_, const Matrix& A1_, const Matrix& B1_ );


    /** .
     *
     *	@param[in] 		.
     *
     *	\return SUCCESSFUL_RETURN
     */
    returnValue setLinearOutput( const Matrix& A3_, const OutputFcn& rhs_ );


    /** .
     *
     *	@param[in] 		.
     *
     *	\return SUCCESSFUL_RETURN
     */
    returnValue setLinearOutput( const Matrix& M3_, const Matrix& A3_, const OutputFcn& rhs_ );


    /** .
     *
     *	@param[in] 		.
     *
     *	\return SUCCESSFUL_RETURN
     */
    returnValue setLinearOutput( const Matrix& A3_, const String& _rhs3, const String& _diffs_rhs3 );


    /** .
     *
     *	@param[in] 		.
     *
     *	\return SUCCESSFUL_RETURN
     */
    returnValue setLinearOutput( const Matrix& M3_, const Matrix& A3_, const String& _rhs3, const String& _diffs_rhs3 );


    /** Assigns the model to be used by the integrator.
     *
     *	@param[in] _rhs_ODE				Name of the function, evaluating the ODE right-hand side.
     *	@param[in] _diffs_rhs_ODE		Name of the function, evaluating the derivatives of the ODE right-hand side.
     *
     *	\return SUCCESSFUL_RETURN
     */

    returnValue setModel( 	const String& fileName,
   		 	 	 	 		const String& _rhs_ODE,
   		 	 	 	 		const String& _diffs_rhs_ODE );


	/** Adds an output function.
	 *
	 *  \param outputEquation_ 	  	an output function to be added
     *  \param measurements	  		the measurement points per interval
	 *
	 *  \return SUCCESSFUL_RETURN
	 */
	uint addOutput( const OutputFcn& outputEquation_, const Vector& measurements );


	/** Adds an output function.
	 *
	 *  \param outputEquation_ 	  	an output function to be added
     *  \param numberMeasurements	the number of measurements per interval
	 *
	 *  \return SUCCESSFUL_RETURN
	 */
	uint addOutput( const OutputFcn& outputEquation_, const uint numberMeasurements );


	/** Adds an output function.
	 *
	 *  \param output 	  			The output function to be added.
	 *  \param diffs_output 	  	The derivatives of the output function to be added.
	 *  \param dim					The dimension of the output function.
     *  \param measurements	  		The measurement points per interval
	 *
	 *  \return SUCCESSFUL_RETURN
	 */
	uint addOutput( const String& output, const String& diffs_output, const uint dim, const Vector& measurements );


	/** Adds an output function.
	 *
	 *  \param output 	  			The output function to be added.
	 *  \param diffs_output 	  	The derivatives of the output function to be added.
	 *  \param dim					The dimension of the output function.
     *  \param numberMeasurements	The number of measurements per interval
	 *
	 *  \return SUCCESSFUL_RETURN
	 */
	uint addOutput( const String& output, const String& diffs_output, const uint dim, const uint numberMeasurements );


	/** Adds an output function.
	 *
	 *  \param output 	  			The output function to be added.
	 *  \param diffs_output 	  	The derivatives of the output function to be added.
	 *  \param dim					The dimension of the output function.
     *  \param measurements	  		The measurement points per interval
	 *  \param colInd				Vector stores the column indices of the elements for Compressed Row Storage (CRS).
	 *  \param rowPtr				Vector stores the locations that start a row for Compressed Row Storage (CRS).
	 *
	 *  \return SUCCESSFUL_RETURN
	 */
	uint addOutput( 	const String& output, const String& diffs_output, const uint dim,
						const Vector& measurements, const String& colInd, const String& rowPtr	);


	/** Adds an output function.
	 *
	 *  \param output 	  			The output function to be added.
	 *  \param diffs_output 	  	The derivatives of the output function to be added.
	 *  \param dim					The dimension of the output function.
     *  \param numberMeasurements	The number of measurements per interval
	 *  \param colInd				Vector stores the column indices of the elements for Compressed Row Storage (CRS).
	 *  \param rowPtr				Vector stores the locations that start a row for Compressed Row Storage (CRS).
	 *
	 *  \return SUCCESSFUL_RETURN
	 */
	uint addOutput( 	const String& output, const String& diffs_output, const uint dim,
						const uint numberMeasurements, const String& colInd, const String& rowPtr	);


    /** Sets integration grid.
     *
     *	@param[in] _ocpGrid		Evaluation grid for optimal control.
     *	@param[in] numSteps		The number of integration steps along the horizon.
     *
     *	\return SUCCESSFUL_RETURN
     */
    returnValue setIntegrationGrid(	const Grid& _ocpGrid,
   		 	 	 	 				const uint _numSteps	);


    /** Sets up the output functions. 																			\n
     *                                                                      									\n
     *  \param numberMeasurements	  the number of measurements per horizon for each output function  			\n
     *                                                                      									\n
     *  \return SUCCESSFUL_RETURN
     */
    returnValue setupOutput( const Vector& numberMeasurements );


    /** Returns the differential equations in the model.
     *
     *  \return SUCCESSFUL_RETURN
     */
    returnValue getModel( DifferentialEquation& _f ) const;


    BooleanType hasOutputs() const;
    BooleanType hasDifferentialEquation() const;
    BooleanType modelDimensionsSet() const;
    BooleanType hasEquidistantControlGrid		() const;
    BooleanType exportRhs() const;


    /** Returns number of differential states.
     *
     *  \return Number of differential states
     */
    uint getNX( ) const;


    /** Returns number of differential state derivatives.
     *
     *  \return Number of differential state derivatives
     */
    uint getNDX( ) const;


    /** Returns number of algebraic states.
     *
     *  \return Number of algebraic states
     */
    uint getNXA( ) const;

    /** Returns number of control inputs.
     *
     *  \return Number of control inputs
     */
    uint getNU( ) const;

    /** Returns number of parameters.
     *
     *  \return Number of parameters
     */
    uint getNP( ) const;

    /** Returns number of control intervals.
     *
     *  \return Number of control intervals
     */
    uint getN( ) const;

    /** Sets the number of shooting intervals.
     *
     *  @param[in] N_		The number of shooting intervals.
     *
     *	\return SUCCESSFUL_RETURN
     */
    returnValue setN( const uint N_ );


    /** Returns the dimensions of the different output functions.
     *
     *  \return dimensions of the different output functions.
     */
    Vector getDimOutputs( ) const;


    /** Returns the number of measurements for the different output functions.
     *
     *  \return number of measurements for the different output functions.
     */
    Vector getNumMeas( ) const;


    /** Returns the model data object.
     *
     *  \return the model data object.
     */
    ModelData& getModelData( );


    /** Sets the model data object.
     *
     *  @param[in] data		 the model data object.
     *
     *  \return SUCCESSFUL_RETURN
     */
    returnValue setModelData( const ModelData& data );


    const String getFileNameModel() const;


    //
    // PROTECTED FUNCTIONS:
    //
    protected:


    //
    // DATA MEMBERS:
    //
    protected:

     	 ModelData modelData;			/**< The model data. */
};


CLOSE_NAMESPACE_ACADO



#endif  // ACADO_TOOLKIT_MODELCONTAINER_HPP

/*
 *   end of file
 */
