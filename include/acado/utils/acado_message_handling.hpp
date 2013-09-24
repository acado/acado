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
 *	\file include/acado/utils/acado_message_handling.hpp
 *	\author Hans Joachim Ferreau, Boris Houska, Milan Vukov
 */


#ifndef ACADO_TOOLKIT_ACADO_MESSAGE_HANDLING_HPP
#define ACADO_TOOLKIT_ACADO_MESSAGE_HANDLING_HPP


#include <acado/utils/acado_namespace_macros.hpp>
#include <stdio.h>
#include <stdlib.h>

#include <iostream>
#include <vector>


BEGIN_NAMESPACE_ACADO


/**
 *	\brief Defines all symbols for global return values.
 *	
 *	\ingroup BasicDataStructures
 *
 *  The enumeration returnValueType defines all symbols for global return values.
 *	Important: All return values are assumed to be nonnegative!
 *
 *	\author Hans Joachim Ferreau, Boris Houska
 */
enum returnValueType
{
TERMINAL_LIST_ELEMENT = -1,						/**< Terminal list element, internal usage only! */
/* miscellaneous */
SUCCESSFUL_RETURN = 0,							/**< Successful return. */
RET_DIV_BY_ZERO,									/**< Division by zero. */
RET_INDEX_OUT_OF_BOUNDS,						/**< Index out of bounds. */
RET_INVALID_ARGUMENTS,							/**< At least one of the arguments is invalid. */
RET_ERROR_UNDEFINED,							/**< Error number undefined. */
RET_WARNING_UNDEFINED,							/**< Warning number undefined. */
RET_INFO_UNDEFINED,								/**< Info number undefined. */
RET_EWI_UNDEFINED,								/**< Error/warning/info number undefined. */
RET_AVAILABLE_WITH_LINUX_ONLY,					/**< This function is available under Linux only. */
RET_UNKNOWN_BUG,								/**< The error occured is not yet known. */
RET_PRINTLEVEL_CHANGED,							/**< Print level changed. */
RET_NOT_YET_IMPLEMENTED,						/**< Requested function is not yet implemented. */
RET_NOT_IMPLEMENTED_YET,						/**< Requested function is not yet implemented. */
RET_NOT_IMPLEMENTED_IN_BASE_CLASS,				/**< Requested function is not implemented within this class. */
RET_ASSERTION,									/**< An assertion has been violated. */
RET_MEMBER_NOT_INITIALISED, 					/**< Requested member has not been initialised. */
RET_ABSTRACT_BASE_CLASS,						/**< Invalid call to member function of abstract base class. */
RET_NO_DATA_FOUND,								/**< There has no data been found. */
RET_INPUT_DIMENSION_MISMATCH,					/**< The dimensions of the input are wrong. */
RET_STRING_EXCEEDS_LENGTH,						/**< String exceeds maximum length. */

/* IO utils: */
RET_FILE_NOT_FOUND,								/**< The file has not been found.*/
RET_FILE_CAN_NOT_BE_OPENED,						/**< The file can not be opened. */
RET_CAN_NOT_WRITE_INTO_FILE,					/**< The routine has no write access or writing into the file failed. */
RET_FILE_CAN_NOT_BE_CLOSED,						/**< The file could not be closed. */
RET_FILE_HAS_NO_VALID_ENTRIES,					/**< The file has no valid entries. */
RET_DOES_DIRECTORY_EXISTS,						/**< Could not open file, check if given directory exists. */

/* Matrix/Vector */
RET_VECTOR_DIMENSION_MISMATCH,					/**< Incompatible vector dimensions. */
RET_DIFFERENTIAL_STATE_DIMENSION_MISMATCH,		/**< Incompatible differential state vector dimensions. */
RET_ALGEBRAIC_STATE_DIMENSION_MISMATCH,			/**< Incompatible algebraic state vector dimensions. */
RET_CONTROL_DIMENSION_MISMATCH,					/**< Incompatible control vector dimensions. */
RET_PARAMETER_DIMENSION_MISMATCH,				/**< Incompatible parameter vector dimensions. */
RET_DISTURBANCE_DIMENSION_MISMATCH,				/**< Incompatible disturbance vector dimensions. */
RET_OUTPUT_DIMENSION_MISMATCH,					/**< Incompatible output vector dimensions. */
RET_MATRIX_NOT_SQUARE,							/**< Operation requires square matrix. */

/* Sparse Solver */
RET_LINEAR_SYSTEM_NUMERICALLY_SINGULAR,			/**< Linear system could not be solved with required accuracy. Check whether the system is singular or ill-conditioned. */

/* Grid */
RET_GRIDPOINT_SETUP_FAILED,						/**< Failed to setup grid point. */
RET_GRIDPOINT_HAS_INVALID_TIME,					/**< Unable to setup a grid point with this time. */
RET_CONFLICTING_GRIDS,							/**< Conflicting grids detected. */
RET_TIME_INTERVAL_NOT_VALID,					/**< The time interval is not valid / not in range. */
RET_INVALID_TIME_POINT,							/**< A time point is not in its permissible range. */

/* Options */
RET_OPTION_ALREADY_EXISTS,						/**< An option with this name already exists. */
RET_OPTION_DOESNT_EXIST,						/**< An option with this name does not exist. */
RET_OPTIONS_LIST_CORRUPTED,						/**< Internal options list is corrupted. */
RET_INVALID_OPTION,								/**< A user-defined option has an invalid value. */

/* Plotting */
RET_PLOTTING_FAILED,							/**< Unable to plot current window. */
RET_EMPTY_PLOT_DATA,							/**< Unable to plot subplot as plot data is empty. */
RET_PLOT_WINDOW_CORRUPTED,						/**< PlotWindow has corrupted list of subplots. */
RET_PLOT_COLLECTION_CORRUPTED,					/**< PlotCollection has corrupted list of plot windows. */

/* Logging */
RET_LOG_RECORD_CORRUPTED,						/**< LogRecord has corrupted list of items. */
RET_LOG_ENTRY_DOESNT_EXIST,						/**< An log entry with this name does not exist. */
RET_LOG_COLLECTION_CORRUPTED,					/**< LogCollection has corrupted list of records. */

/* SimulationEnvironment */
RET_BLOCK_DIMENSION_MISMATCH,					/**< Blocks with incompatible dimensions. */
RET_NO_PROCESS_SPECIFIED,						/**< No process has been specified. */
RET_NO_CONTROLLER_SPECIFIED,					/**< No controller has been specified. */
RET_ENVIRONMENT_INIT_FAILED,					/**< Unable to initialize simulation environment. */
RET_ENVIRONMENT_STEP_FAILED,					/**< Unable to perform simulation environment step. */
RET_COMPUTATIONAL_DELAY_TOO_BIG,				/**< Simulation stops as computational delay is too big. */
RET_COMPUTATIONAL_DELAY_NOT_SUPPORTED,			/**< Simulation of computational delay is not yet supported. */

/* Block */
RET_BLOCK_NOT_READY,							/**< Block is not ready. */

/* Time */
RET_NO_SYSTEM_TIME,								/**< No system time available. */
RET_CLOCK_NOT_READY,							/**< Unable to start the clock as it is not ready. */

/* Process */
RET_PROCESS_INIT_FAILED,						/**< Unable to initialize process. */
RET_PROCESS_STEP_FAILED,						/**< Unable to perform process step. */
RET_PROCESS_STEP_FAILED_DISTURBANCE,			/**< Unable to perform process step due to error in disturbance evaluation. */
RET_PROCESS_RUN_FAILED,							/**< Unable to run process simulation. */
RET_NO_DYNAMICSYSTEM_SPECIFIED,					/**< No dynamic system has been specified. */
RET_NO_INTEGRATIONALGORITHM_SPECIFIED,			/**< No integration algorithm has been specified. */
RET_NO_DISCRETE_TIME_SYSTEMS_SUPPORTED,			/**< Discrete-time systems are not yet supported. */
RET_WRONG_DISTURBANCE_HORIZON,					/**< Process disturbance is defined over an incompatible time horizon. */

/* Actuator / Sensor */
RET_ACTUATOR_INIT_FAILED,						/**< Unable to initialize actuator. */
RET_ACTUATOR_STEP_FAILED,						/**< Unable to perform actuator step. */
RET_SENSOR_INIT_FAILED,							/**< Unable to initialize sensor. */
RET_SENSOR_STEP_FAILED,							/**< Unable to perform sensor step. */
RET_GENERATING_NOISE_FAILED,					/**< Unable to generate noise. */
RET_DELAYING_INPUTS_FAILED,						/**< Unable to delay inputs. */
RET_DELAYING_OUTPUTS_FAILED,					/**< Unable to delay outputs. */
RET_INCOMPATIBLE_ACTUATOR_SAMPLING_TIME,		/**< Actuator sampling time has to be an integer multiple of dynamic system sample time. */
RET_INCOMPATIBLE_SENSOR_SAMPLING_TIME,			/**< Sensor sampling time has to be an integer multiple of dynamic system sample time. */
RET_NO_DIFFERENT_NOISE_SAMPLING_FOR_DISCRETE,	/**< When using discrete-time systems, noise has to be sampled equally for all components. */

/* Noise */
RET_NO_NOISE_GENERATED,							/**< No noise has been generated. */
RET_NO_NOISE_SETTINGS,							/**< No noise settings has been defined. */
RET_INVALID_NOISE_SETTINGS,						/**< Specified noise settings are invalid. */

/* Controller */
RET_CONTROLLER_INIT_FAILED,						/**< Unable to initialize controller. */
RET_CONTROLLER_STEP_FAILED,						/**< Unable to perform controller step. */
RET_NO_ESTIMATOR_SPECIFIED,						/**< No estimator has been specified. */
RET_NO_CONTROLLAW_SPECIFIED,					/**< No control law has been specified. */
RET_NO_REALTIME_MODE_AVAILABLE,					/**< Control law does not support real-time mode. */

/* DynamicControlUnit / Estimator / Optimizer */
RET_DCU_INIT_FAILED,							/**< Unable to initialize dynamic control unit. */
RET_DCU_STEP_FAILED,							/**< Unable to perform step of dynamic control unit. */
RET_ESTIMATOR_INIT_FAILED,						/**< Unable to initialize estimator. */
RET_ESTIMATOR_STEP_FAILED,						/**< Unable to perform estimator step. */
RET_OPTIMIZER_INIT_FAILED,						/**< Unable to initialize optimizer. */
RET_OPTIMIZER_STEP_FAILED,						/**< Unable to perform optimizer step. */
RET_NO_OCP_SPECIFIED,							/**< No optimal control problem has been specified. */
RET_NO_SOLUTIONALGORITHM_SPECIFIED,				/**< No solution algorithm has been specified. */

/* ControlLaw */
RET_CONTROLLAW_INIT_FAILED,						/**< Unable to initialize feedback law. */
RET_CONTROLLAW_STEP_FAILED,						/**< Unable to perform feedback law step. */
RET_NO_OPTIMIZER_SPECIFIED,						/**< No optimizer has been specified. */
RET_INVALID_PID_OUTPUT_DIMENSION,				/**< Invalid output dimension of PID controller, reset to 1. */

/* RealTimeAlgorithm */
RET_IMMEDIATE_FEEDBACK_ONE_ITERATION,			/**< Resetting maximum number of iterations to 1 as required for using immediate feedback. */

/* OutputTransformator */
RET_OUTPUTTRANSFORMATOR_INIT_FAILED,			/**< Unable to initialize output transformator. */
RET_OUTPUTTRANSFORMATOR_STEP_FAILED,			/**< Unable to perform output transformator step. */

/* Function */
RET_INVALID_USE_OF_FUNCTION,					/**< Invalid use of the class function. */
RET_INFEASIBLE_CONSTRAINT,						/**< Infeasible Constraints detected. */
RET_ONLY_SUPPORTED_FOR_SYMBOLIC_FUNCTIONS,		/**< This routine is for symbolic functions only. */
RET_INFEASIBLE_ALGEBRAIC_CONSTRAINT,			/**< Infeasible algebraic constraints are not allowed and will be ignored. */
RET_ILLFORMED_ODE,								/**< ODE needs to depend on all differential states. */

/* Expression */
RET_PRECISION_OUT_OF_RANGE,						/**< the requested precision is out of range. */
RET_ERROR_WHILE_PRINTING_A_FILE,				/**< An error has occured while printing a file. */
RET_INDEX_OUT_OF_RANGE,							/**< An index was not in the range. */
RET_INTERMEDIATE_STATE_HAS_NO_ARGUMENT,			/**< The intermediate state has no argument. */
RET_DIMENSION_NOT_SPECIFIED,					/**< The dimension of a array was not specified. */

/* Modeling Tools */
RET_DDQ_DIMENSION_MISMATCH,						/**< ddq argument must be of size 3x1 */
RET_CAN_ONLY_SOLVE_2ND_ORDER_KINVECS,			/**< can only solve 2nd order KinVecs */


/* OBJECTIVE */
RET_GAUSS_NEWTON_APPROXIMATION_NOT_SUPPORTED,	/**< The objective does not support Gauss-Newton Hessian approximations. */
RET_REFERENCE_SHIFTING_WORKS_FOR_LSQ_TERMS_ONLY,	/**< The reference shifting works only for LSQ objectives. */

/* Integrator */
RET_TRIVIAL_RHS,								/**< the dimension of the rhs is zero. */
RET_MISSING_INPUTS,								/**< the integration routine misses some inputs. */
RET_TO_SMALL_OR_NEGATIVE_TIME_INTERVAL,			/**< the time interval was too small or negative.*/
RET_FINAL_STEP_NOT_PERFORMED_YET,				/**< the integration routine is not ready. */
RET_ALREADY_FROZEN,								/**< the integrator is already freezing or frozen. */
RET_MAX_NUMBER_OF_STEPS_EXCEEDED,				/**< the maximum number of steps has been exceeded. */
RET_WRONG_DEFINITION_OF_SEEDS,					/**< the seeds are not set correctly or in the wrong order. */
RET_NOT_FROZEN,									/**< the mesh is not frozen and/or forward results not stored. */
RET_TO_MANY_DIFFERENTIAL_STATES,				/**< there are to many differential states. */
RET_TO_MANY_DIFFERENTIAL_STATE_DERIVATIVES,		/**< there are to many diff. state derivatives. */
RET_RK45_CAN_NOT_TREAT_DAE,						/**< An explicit Runge-Kutta solver cannot treat DAEs. */
RET_CANNOT_TREAT_DAE,							/**< The algorithm cannot treat DAEs. */
RET_INPUT_HAS_WRONG_DIMENSION,					/**< At least one of the inputs has a wrong dimension. */
RET_INPUT_OUT_OF_RANGE,							/**< One of the inputs is out of range. */
RET_THE_DAE_INDEX_IS_TOO_LARGE,					/**< The index of the DAE is larger than 1. */
RET_UNSUCCESSFUL_RETURN_FROM_INTEGRATOR_RK45,	/**< the integration routine stopped due to a problem during the function evaluation. */
RET_UNSUCCESSFUL_RETURN_FROM_INTEGRATOR_BDF,	/**< the integration routine stopped as the required accuracy can not be obtained. */
RET_CANNOT_TREAT_DISCRETE_DE,					/**< This integrator cannot treat discrete-time differential equations. */
RET_CANNOT_TREAT_CONTINUOUS_DE,					/**< This integrator cannot treat time-continuous differential equations. */
RET_CANNOT_TREAT_IMPLICIT_DE,					/**< This integrator cannot treat differential equations in implicit form. */
RET_CANNOT_TREAT_EXPLICIT_DE,					/**< This integrator cannot treat differential equations in explicit form. */

/* DynamicDiscretization */
RET_TO_MANY_DIFFERENTIAL_EQUATIONS,				/**< The number of differential equations is too large. */
RET_BREAK_POINT_SETUP_FAILED,					/**< The break point setup failed. */
RET_WRONG_DEFINITION_OF_STAGE_TRANSITIONS,		/**< The definition of stage transitions is wrong. */
RET_TRANSITION_DEPENDS_ON_ALGEBRAIC_STATES,		/**< A transition should never depend on algebraic states.*/

/* OPTIMIZATION_ALGORITHM: */
RET_NO_VALID_OBJECTIVE,							/**< No valid objective found. */
RET_INCONSISTENT_BOUNDS,						/**< The bounds are inconsistent. */
RET_INCOMPATIBLE_DIMENSIONS,					/**< Incopatible dimensions detected. */
RET_GRID_SETUP_FAILED,							/**< Discretization of the OCP failed. */
RET_OPTALG_INIT_FAILED, 						/**< Initialization of optimization algorithm failed. */
RET_OPTALG_STEP_FAILED, 						/**< Step of optimization algorithm failed. */
RET_OPTALG_FEEDBACK_FAILED, 					/**< Feedback step of optimization algorithm failed. */
RET_OPTALG_PREPARE_FAILED, 						/**< Preparation step of optimization algorithm failed. */
RET_OPTALG_SOLVE_FAILED, 						/**< Problem could not be solved with given optimization algorithm. */
RET_REALTIME_NO_INITIAL_VALUE, 					/**< No initial value has been specified. */

/* INTEGRATION_ALGORITHM: */
RET_INTALG_INIT_FAILED, 						/**< Initialization of integration algorithm failed. */
RET_INTALG_INTEGRATION_FAILED, 					/**< Integration algorithm failed to integrate dynamic system. */
RET_INTALG_NOT_READY, 							/**< Integration algorithm has not been initialized. */

/* PLOT WINDOW */
RET_PLOT_WINDOW_CAN_NOT_BE_OPEN,				/**< ACADO was not able to open the plot window. */

/* NLP SOLVER */
CONVERGENCE_ACHIEVED,							/**< convergence achieved. */
CONVERGENCE_NOT_YET_ACHIEVED,					/**< convergence not yet achieved. */
RET_NLP_INIT_FAILED, 							/**< Initialization of NLP solver failed. */
RET_NLP_STEP_FAILED, 							/**< Step of NLP solver failed. */
RET_NLP_SOLUTION_FAILED,						/**< NLP solution failed. */
RET_INITIALIZE_FIRST, 							/**< Object needs to be initialized first. */
RET_SOLVER_NOT_SUTIABLE_FOR_REAL_TIME_MODE,		/**< The specified NLP solver is not designed for a real-time mode. */
RET_ILLFORMED_HESSIAN_MATRIX,					/**< Hessian matrix is too ill-conditioned to continue. */
RET_NONSYMMETRIC_HESSIAN_MATRIX,				/**< Hessian matrix is not symmetric, proceeding with symmetrized Hessian. */
RET_UNABLE_TO_EVALUATE_OBJECTIVE,				/**< Evaluation of objective function failed. */
RET_UNABLE_TO_EVALUATE_CONSTRAINTS,				/**< Evaluation of constraints failed. */
RET_UNABLE_TO_INTEGRATE_SYSTEM,					/**< Integration of dynamic system failed. Try to adjust integrator tolerances using set( ABSOLUTE_TOLERANCE,<double> ) and set( INTEGRATOR_TOLERANCE,<double> ). */
RET_NEED_TO_ACTIVATE_RTI,						/**< Feedback step requires real-time iterations to be activated. Use set( USE_REALTIME_ITERATIONS,YES ) to do so. */

/* CONIC SOLVER */
RET_CONIC_PROGRAM_INFEASIBLE,					/**< The optimization problem is infeasible. */
RET_CONIC_PROGRAM_SOLUTION_FAILED,				/**< Conic Program solution failed. The optimization problem might be infeasible. */
RET_CONIC_PROGRAM_NOT_SOLVED,					/**< The Conic Program has not been solved successfully. */

/* CP SOLVER */
RET_UNABLE_TO_CONDENSE,							/**< Unable to condense banded CP. */
RET_UNABLE_TO_EXPAND,							/**< Unable to expand condensed CP. */
RET_NEED_TO_CONDENSE_FIRST,						/**< Condensing cannot be frozen as banded CP needs to be condensed first. */
RET_BANDED_CP_INIT_FAILED,						/**< Initialization of banded CP solver failed. */
RET_BANDED_CP_SOLUTION_FAILED,					/**< Solution of banded CP failed. */

/* OCP */
RET_TRANSITION_NOT_DEFINED,						/**> No transition function found. */

/* QP SOLVER */
RET_QP_INIT_FAILED,								/**< QP initialization failed. */
RET_QP_SOLUTION_FAILED,							/**< QP solution failed. */
RET_QP_SOLUTION_REACHED_LIMIT,					/**< QP solution stopped as iteration limit is reached. */
RET_QP_INFEASIBLE, 								/**< QP solution failed due to infeasibility. */
RET_QP_UNBOUNDED, 								/**< QP solution failed due to unboundedness. */
RET_QP_NOT_SOLVED,								/**< QP has not been solved. */
RET_RELAXING_QP,								/**< QP needs to be relaxed due to infeasibility. */
RET_COULD_NOT_RELAX_QP, 						/**< QP could not be relaxed. */
RET_QP_SOLVER_CAN_ONLY_SOLVE_QP,				/**< The QP solver can not deal with general conic problems. */
RET_QP_HAS_INCONSISTENT_BOUNDS,					/**< QP cannot be solved due to inconsistent bounds. */
RET_UNABLE_TO_HOTSTART_QP,						/**< Unable to hotstart QP with given solver. */

/* MPC SOLVER */
RET_NONPOSITIVE_WEIGHT, 						/**< Weighting matrices must be positive semi-definite. */
RET_INITIAL_CHOLESKY_FAILED, 					/**< Setting up initial Cholesky decompostion failed. */
RET_HOMOTOPY_STEP_FAILED,						/**< Unable to perform homotopy step. */
RET_STEPDIRECTION_DETERMINATION_FAILED,			/**< Determination of step direction failed. */
RET_STEPDIRECTION_FAILED_CHOLESKY,				/**< Abnormal termination due to Cholesky factorisation. */
RET_STEPLENGTH_DETERMINATION_FAILED,			/**< Determination of step direction failed. */
RET_OPTIMAL_SOLUTION_FOUND,						/**< Optimal solution of neighbouring QP found. */
RET_MAX_NWSR_REACHED,							/**< Maximum number of working set recalculations performed. */
RET_MATRIX_NOT_SPD,								/**< Matrix is not positive definite. */

/* CODE EXPORT */
RET_CODE_EXPORT_SUCCESSFUL,						/**< Code generation successful. */
RET_UNABLE_TO_EXPORT_CODE,						/**< Unable to generate code. */
RET_INVALID_OBJECTIVE_FOR_CODE_EXPORT,			/**< Only standard LSQ objective supported for code generation. */
RET_NO_DISCRETE_ODE_FOR_CODE_EXPORT,			/**< No discrete-time ODEs supported for code generation. */
RET_ONLY_ODE_FOR_CODE_EXPORT,					/**< Only ODEs supported for code generation. */
RET_ONLY_STATES_AND_CONTROLS_FOR_CODE_EXPORT,	/**< No parameters, disturbances or algebraic states supported for code generation. */
RET_ONLY_EQUIDISTANT_GRID_FOR_CODE_EXPORT,		/**< Only equidistant evaluation grids supported for  code generation. */
RET_ONLY_BOUNDS_FOR_CODE_EXPORT,				/**< Only state and control bounds supported for code generation. */
RET_QPOASES_EMBEDDED_NOT_FOUND,					/**< Embedded qpOASES code not found. */
RET_UNABLE_TO_EXPORT_STATEMENT,					/**< Unable to export statement due to incomplete definition. */
RET_INVALID_CALL_TO_EXPORTED_FUNCTION			/**< Invalid call to export functions (check number of calling arguments). */
};


/** Defines visibility status of a message. */
enum VisibilityStatus
{
	VS_VISIBLE,									/**< Message visible. */
	VS_HIDDEN									/**< Message not visible. */
};


/** Defines the importance level of the message */
enum returnValueLevel
{
	LVL_DEBUG = 0,		///< Lowest level, the debug level.
	LVL_FATAL,			///< Returned value is a fatal error, assert like use, aborts execution is unhandled
	LVL_ERROR,			///< Returned value is a error
	LVL_WARNING,		///< Returned value is a warning
	LVL_INFO			///< Returned value is a information
};

/** Colored/formatted terminal output */
#ifndef __MATLAB__

#define COL_DEBUG		"\033[1;34m"
#define COL_FATAL		"\033[0;31m"
#define COL_ERROR		"\033[1;31m"
#define COL_WARNING		"\033[1;33m"
#define COL_INFO		"\033[0m"

#else

#define COL_DEBUG		""
#define COL_FATAL		""
#define COL_ERROR		""
#define COL_WARNING		""
#define COL_INFO		""

#endif /* __MATLAB__ */

/** Defines whether user has handled the returned value */
enum returnValueStatus {
	STATUS_UNHANDLED, ///< returnValue was not yet handled by user
	STATUS_HANDLED    ///< returnValue was handled by user
};

/** Converts returnValueLevel enum type to a const char* */
const char* returnValueLevelToString(returnValueLevel level);

/** Converts returnValueType enum type to a const char* */
const char* returnValueTypeToString(returnValueType type);


/** 
 *  \brief Allows to pass back messages to the calling function.
 *
 *	\ingroup BasicDataStructures
 *
 *	An instance of the class returnValue is returned by all ACADO functions for 
 *	passing back messages to the calling function.
 *
 *  \author Martin Lauko, Hans Joachim Ferreau, Boris Houska
 */
class returnValue
{
private:

	/** Internal data holding class for returnValue class
	 *  Owner is always set to the last returnValue instance which was copy constructed/assigned to it.
	 */
	class returnValueData {
	public:
		returnValue* owner;
		std::vector<const char*> messages;
	};
	returnValueType type;
	returnValueLevel level;
	returnValueStatus status;

	returnValueData* data;
public:

	/** Construct default returnValue.
	 *
	 */
	inline returnValue();

	/** Construct returnValue only from typedef.
	 *
	 */
	inline returnValue(returnValueType _type);

	/** Construct returnValue from int, for compatibility
	 *
	 */
	inline returnValue(int _type);

	/** Copy constructor with minimum performance cost.
	 *  Newly constructed instance takes ownership of data.
	 */
	inline returnValue(const returnValue& old);
	
	/** Constructor used by the ACADOERROR and similar macros.
	 *
	 */
	returnValue(const char* msg, returnValueLevel level = LVL_ERROR, returnValueType type = RET_UNKNOWN_BUG);

	/** Constructor used by the ACADOERROR and similar macros. Special case.
	 *  Constructs returnValue from old one, changing level and adding message
	 */
	returnValue(const char* msg, returnValueLevel level,const returnValue& old);

	/** Adds another message to the end of messages list.
	 *
	 */	
	returnValue& addMessage(const char* msg);

	/** Change the importance level of the returned value
	 *
	 */
	returnValue& changeLevel(returnValueLevel level);

	/** Change the type of the returned message
	 *
	 */
	returnValue& changeType(returnValueType type);

	inline returnValueLevel getLevel() const;

	/** Prints all messages to the standard output.
	 *
	 */	
	void print();

	/** Prints only the most basic information, no messages, to the standard output.
	 *
	 */
	void printBasic();

	/** Destroys data instance only if it owns it
	 *
	 */
	~returnValue();

	/** Compares the returnValue type to its enum
	 *
	 */
	inline bool operator!=(returnValueType cmp_type) const;

	/** Compares the returnValue type to its enum
	 *
	 */
	inline bool operator==(returnValueType cmp_type) const;
	
	/** Returns true if return value type is not SUCCESSFUL_RETURN
	 *
	 */
	inline bool operator!() const;

	/** Assignment operator.
	 *  Left hand side instance takes ownership of data.
	 */
	inline returnValue& operator=(const returnValue& old);

	/** Compatibility function, allows returnValue to be used as a number, similar to a enum.
     *
     */
	inline operator int();

};

#include <acado/utils/acado_message_handling.ipp>

// define DEBUG macros if necessary
#ifndef __FUNCTION__
	#define __FUNCTION__ 0
#endif

#ifndef __FILE__
	#define __FILE__ 0
#endif

#ifndef __LINE__
	#define __LINE__ 0
#endif

// define SNPRINTF if necessary
#ifdef _snprintf
	#define snprintf _snprintf
#endif

// Macro to quote macro values as strings, e.g. __LINE__ number to string, used in other macros
#define QUOTE_(x) #x
#define QUOTE(x) QUOTE_(x)


/** Macro to return a error */
#define ACADOERROR(retval) \
		returnValue("Code: ("#retval") \n  File: "__FILE__"\n  Line: "QUOTE(__LINE__), LVL_ERROR, retval)

/** Macro to return a error, with user message */
#define ACADOERRORTEXT(retval, text) \
		returnValue("Message: "#text"\n  Code:    ("#retval") \n  File:    "__FILE__"\n  Line:    "QUOTE(__LINE__), LVL_ERROR, retval)

/** Macro to return a fatal error */
#define ACADOFATAL(retval) \
		returnValue("Code: ("#retval") \n  File: "__FILE__"\n  Line: "QUOTE(__LINE__), LVL_FATAL, retval)

/** Macro to return a fatal error, with user message */
#define ACADOFATALTEXT(retval, text) \
		returnValue("Message: "#text"\n  Code:    ("#retval") \n  File:    "__FILE__"\n  Line:    "QUOTE(__LINE__), LVL_FATAL, retval)

/** Macro to return a warning */
#define ACADOWARNING(retval) \
		returnValue("Code: ("#retval") \n  File: "__FILE__"\n  Line: "QUOTE(__LINE__), LVL_WARNING, retval)

/** Macro to return a warning, with user message */
#define ACADOWARNINGTEXT(retval,text) \
		returnValue("Message: "#text"\n  Code:    ("#retval") \n  File:    "__FILE__"\n  Line:    "QUOTE(__LINE__), LVL_WARNING, retval)

/** Macro to return a information */
#define ACADOINFO(retval) \
		returnValue("", LVL_INFO, retval)

/** Macro to return a information, with user message */
#define ACADOINFOTEXT(retval,text) \
		returnValue("Message: "#text"\n  Code:    ("#retval") \n  File:    "__FILE__"\n  Line:    "QUOTE(__LINE__), LVL_INFO, retval)


/** Executes the statement X and handles returned message.
 *  This is the default message handler. Statement X must return type returnValue.
 *  If message is not equal to SUCCESSFUL_RETURN a message is added informing where and what this statement is and imediately returned.
 *  Example: ACADO_TRY( func() );
 *  Example 2, extended use: ADACO_TRY( func() ).addMessage( "func() failed" );
 */
#define ACADO_TRY(X) for(returnValue ACADO_R = X; !ACADO_R;) return ACADO_R

/**
 *  \brief A very simple logging class.
 *  \ingroup BasicDataStructures
 *  \author Milan Vukov
 *  \date 2013.
 */
class Logger
{
public:
	/** Get an instance of the logger. */
	static Logger& instance()
	{
		static Logger instance;
		return instance;
	}

	/** Set the log level. */
	Logger& setLogLevel(returnValueLevel level)
	{
		logLevel = level;

		return *this;
	}

	/** Get log level. */
	returnValueLevel getLogLevel()
	{
		return logLevel;
	}

	/** Get a reference to the output stream. */
	std::ostream& get(returnValueLevel level);

private:
	returnValueLevel logLevel;

	Logger()
		: logLevel( LVL_FATAL )
	{}

	Logger(const Logger&);
	Logger& operator=(const Logger&);
	~Logger()
	{}
};

/// Just define a handy macro for getting the logger
#define LOG( level ) \
		if (level < Logger::instance().getLogLevel()); \
		else Logger::instance().get( level )

CLOSE_NAMESPACE_ACADO



#endif	// ACADO_TOOLKIT_ACADO_MESSAGE_HANDLING_HPP

/*
 *	end of file
 */
