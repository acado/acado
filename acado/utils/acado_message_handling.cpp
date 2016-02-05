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
 *	\file src/utils/acado_message_handling.cpp
 *	\author Hans Joachim Ferreau, Boris Houska
 */


#include <string.h>


#include <acado/utils/acado_message_handling.hpp>
#include <acado/utils/acado_utils.hpp>

using namespace std;

BEGIN_NAMESPACE_ACADO

/** 
 *  \brief Data structure for entries in returnValueList.
 *
 *	\ingroup BasicDataStructures
 *
 *	Data structure for entries in returnValueList.
 *
 *  \author Hans Joachim Ferreau, Boris Houska
 */
typedef struct {
	returnValueType key;						/**< Global return value. */
	const char* data;							/**< Corresponding message. */
	VisibilityStatus globalVisibilityStatus; 	/**< Determines if message can be printed.
												* 	 If this value is set to VS_HIDDEN, no message is printed! */
} ReturnValueList;

/**
 *	\brief Defines all pairs of global return values and messages within the ACADO Toolkit.
 *	
 *  The class MessageHandling::ReturnValueList defines all pairs of 
 *	global return values and messages within the ACADO Toolkit.
 *
 *	\author Hans Joachim Ferreau, Boris Houska
 */
ReturnValueList returnValueList[] =
{
/* miscellaneous */
{ SUCCESSFUL_RETURN,							"Successful return", VS_VISIBLE },
{ RET_DIV_BY_ZERO,								"Division by zero", VS_VISIBLE },
{ RET_INDEX_OUT_OF_BOUNDS,						"Index out of bounds", VS_VISIBLE },
{ RET_INVALID_ARGUMENTS,						"At least one of the arguments is invalid", VS_VISIBLE },
{ RET_ERROR_UNDEFINED,							"Error number undefined", VS_VISIBLE },
{ RET_WARNING_UNDEFINED,						"Warning number undefined", VS_VISIBLE },
{ RET_INFO_UNDEFINED,							"Info number undefined", VS_VISIBLE },
{ RET_EWI_UNDEFINED,							"Error/warning/info number undefined", VS_VISIBLE },
{ RET_AVAILABLE_WITH_LINUX_ONLY,				"This function is available under Linux only", VS_HIDDEN },
{ RET_UNKNOWN_BUG,								"The error occured is not yet known", VS_VISIBLE },
{ RET_PRINTLEVEL_CHANGED,						"Print level changed", VS_VISIBLE },
{ RET_NOT_YET_IMPLEMENTED,						"Requested function is not yet implemented", VS_VISIBLE },
{ RET_NOT_IMPLEMENTED_YET,						"Requested function is not yet implemented", VS_VISIBLE },
{ RET_NOT_IMPLEMENTED_IN_BASE_CLASS,			"Requested function is not implemented within this class", VS_VISIBLE },
{ RET_ASSERTION,								"An assertion has been violated", VS_VISIBLE },
{ RET_MEMBER_NOT_INITIALISED,					"Requested member has not been initialised", VS_VISIBLE },
{ RET_ABSTRACT_BASE_CLASS,						"Invalid call to member function of abstract base class", VS_VISIBLE },
{ RET_NO_DATA_FOUND,							"There has no data been found", VS_VISIBLE },
{ RET_INPUT_DIMENSION_MISMATCH,					"The dimensions of the input are wrong", VS_VISIBLE },
{ RET_STRING_EXCEEDS_LENGTH,					"String exceeds maximum length", VS_VISIBLE },

/* IO utils: */
{ RET_FILE_NOT_FOUND,							"The file has not been found", VS_VISIBLE },
{ RET_FILE_CAN_NOT_BE_OPENED,					"The file can not be opened", VS_VISIBLE },
{ RET_CAN_NOT_WRITE_INTO_FILE,					"The routine has no write access or writing into the file failed", VS_VISIBLE },
{ RET_FILE_CAN_NOT_BE_CLOSED,					"The file could not be closed", VS_VISIBLE },
{ RET_FILE_HAS_NO_VALID_ENTRIES,				"The file has no valid entries", VS_VISIBLE },
{ RET_DOES_DIRECTORY_EXISTS,					"Could not open file, check if given directory exists", VS_VISIBLE },

/* DMatrix/DVector */
{ RET_VECTOR_DIMENSION_MISMATCH,				"Incompatible vector dimensions", VS_VISIBLE },
{ RET_DIFFERENTIAL_STATE_DIMENSION_MISMATCH,	"Incompatible differential state vector dimensions", VS_VISIBLE },
{ RET_ALGEBRAIC_STATE_DIMENSION_MISMATCH,		"Incompatible algebraic state vector dimensions", VS_VISIBLE },
{ RET_CONTROL_DIMENSION_MISMATCH,				"Incompatible control vector dimensions", VS_VISIBLE },
{ RET_PARAMETER_DIMENSION_MISMATCH,				"Incompatible parameter vector dimensions", VS_VISIBLE },
{ RET_DISTURBANCE_DIMENSION_MISMATCH,			"Incompatible disturbance vector dimensions", VS_VISIBLE },
{ RET_OUTPUT_DIMENSION_MISMATCH,				"Incompatible output vector dimensions", VS_VISIBLE },
{ RET_MATRIX_NOT_SQUARE,						"Operation requires square matrix", VS_VISIBLE },

/* Sparse Solver */
{ RET_LINEAR_SYSTEM_NUMERICALLY_SINGULAR,		"Linear system could not be solved with required accuracy. Check whether the system is singular or ill-conditioned", VS_VISIBLE },

/* Grid */
{ RET_GRIDPOINT_SETUP_FAILED,					"Failed to setup grid point", VS_VISIBLE },
{ RET_GRIDPOINT_HAS_INVALID_TIME,				"Unable to setup a grid point with this time", VS_VISIBLE },
{ RET_CONFLICTING_GRIDS,						"Conflicting grids detected", VS_VISIBLE },
{ RET_TIME_INTERVAL_NOT_VALID,					"The time interval is not valid / not in range", VS_VISIBLE },
{ RET_INVALID_TIME_POINT,						"A time point is not in its permissible range", VS_VISIBLE },

/* Options */
{ RET_OPTION_ALREADY_EXISTS,					"An option with this name already exists", VS_VISIBLE },
{ RET_OPTION_DOESNT_EXIST,						"An option with this name does not exist", VS_VISIBLE },
{ RET_OPTIONS_LIST_CORRUPTED,					"Internal options list is corrupted", VS_VISIBLE },
{ RET_INVALID_OPTION,							"A user-defined option has an invalid value", VS_VISIBLE },

/* Plotting */
{ RET_PLOTTING_FAILED,							"Unable to plot current window", VS_VISIBLE },
{ RET_EMPTY_PLOT_DATA,							"Unable to plot subplot as plot data is empty", VS_VISIBLE },
{ RET_PLOT_WINDOW_CORRUPTED,					"PlotWindow has corrupted list of subplots", VS_VISIBLE },
{ RET_PLOT_COLLECTION_CORRUPTED,				"PlotCollection has corrupted list of plot windows", VS_VISIBLE },

/* Logging */
{ RET_LOG_RECORD_CORRUPTED,						"LogRecord has corrupted list of items", VS_VISIBLE },
{ RET_LOG_ENTRY_DOESNT_EXIST,					"An log entry with this name does not exist", VS_VISIBLE },
{ RET_LOG_COLLECTION_CORRUPTED,					"LogCollection has corrupted list of records", VS_VISIBLE },

/* SimulationEnvironment */
{ RET_BLOCK_DIMENSION_MISMATCH,					"Blocks with incompatible dimensions", VS_VISIBLE },
{ RET_NO_PROCESS_SPECIFIED,						"No process has been specified", VS_VISIBLE },
{ RET_NO_CONTROLLER_SPECIFIED,					"No controller has been specified", VS_VISIBLE },
{ RET_ENVIRONMENT_INIT_FAILED,					"Unable to initialize simulation environment", VS_VISIBLE },
{ RET_ENVIRONMENT_STEP_FAILED,					"Unable to perform simulation environment step", VS_VISIBLE },
{ RET_COMPUTATIONAL_DELAY_TOO_BIG,				"Simulation stops as computational delay is too big", VS_VISIBLE },
{ RET_COMPUTATIONAL_DELAY_NOT_SUPPORTED,		"Simulation of computational delay is not yet supported", VS_VISIBLE },

/* Block */
{ RET_BLOCK_NOT_READY,							"Block is not ready", VS_VISIBLE },

/* Time */
{ RET_NO_SYSTEM_TIME,							"No system time available", VS_VISIBLE },
{ RET_CLOCK_NOT_READY,							"Unable to start the clock as it is not ready", VS_VISIBLE },

/* Process */
{ RET_PROCESS_INIT_FAILED,						"Unable to initialize process", VS_VISIBLE },
{ RET_PROCESS_STEP_FAILED,						"Unable to perform process step", VS_VISIBLE },
{ RET_PROCESS_STEP_FAILED_DISTURBANCE,			"Unable to perform process step due to error in disturbance evaluation", VS_VISIBLE },
{ RET_PROCESS_RUN_FAILED,						"Unable to run process simulation", VS_VISIBLE },
{ RET_NO_DYNAMICSYSTEM_SPECIFIED,				"No dynamic system has been specified", VS_VISIBLE },
{ RET_NO_INTEGRATIONALGORITHM_SPECIFIED,		"No integration algorithm has been specified", VS_VISIBLE },
{ RET_NO_DISCRETE_TIME_SYSTEMS_SUPPORTED,		"Discrete-time systems are not yet supported", VS_VISIBLE },
{ RET_WRONG_DISTURBANCE_HORIZON,				"Process disturbance is defined over an incompatible time horizon", VS_VISIBLE },

/* Actuator / Sensor */
{ RET_ACTUATOR_INIT_FAILED,						"Unable to initialize actuator", VS_VISIBLE },
{ RET_ACTUATOR_STEP_FAILED,						"Unable to perform actuator step", VS_VISIBLE },
{ RET_SENSOR_INIT_FAILED,						"Unable to initialize sensor", VS_VISIBLE },
{ RET_SENSOR_STEP_FAILED,						"Unable to perform sensor step", VS_VISIBLE },
{ RET_GENERATING_NOISE_FAILED,					"Unable to generate noise", VS_VISIBLE },
{ RET_DELAYING_INPUTS_FAILED,					"Unable to delay inputs", VS_VISIBLE },
{ RET_DELAYING_OUTPUTS_FAILED,					"Unable to delay outputs", VS_VISIBLE },
{ RET_INCOMPATIBLE_ACTUATOR_SAMPLING_TIME,		"Actuator sampling time has to be an integer multiple of dynamic system sample time", VS_VISIBLE },
{ RET_INCOMPATIBLE_SENSOR_SAMPLING_TIME,		"Sensor sampling time has to be an integer multiple of dynamic system sample time", VS_VISIBLE },
{ RET_NO_DIFFERENT_NOISE_SAMPLING_FOR_DISCRETE,	"When using discrete-time systems, noise has to be sampled equally for all components", VS_VISIBLE },

/* Noise */
{ RET_NO_NOISE_GENERATED,						"No noise has been generated", VS_VISIBLE },
{ RET_NO_NOISE_SETTINGS,						"No noise settings has been defined", VS_VISIBLE },
{ RET_INVALID_NOISE_SETTINGS,					"Specified noise settings are invalid", VS_VISIBLE },

/* Controller */
{ RET_CONTROLLER_INIT_FAILED,					"Unable to initialize controller", VS_VISIBLE },
{ RET_CONTROLLER_STEP_FAILED,					"Unable to perform controller step", VS_VISIBLE },
{ RET_NO_ESTIMATOR_SPECIFIED,					"No estimator has been specified", VS_VISIBLE },
{ RET_NO_CONTROLLAW_SPECIFIED,					"No control law has been specified", VS_VISIBLE },
{ RET_NO_REALTIME_MODE_AVAILABLE,				"Control law does not support real-time mode", VS_VISIBLE },

/* DynamicControlUnit / Estimator / Optimizer */
{ RET_DCU_INIT_FAILED,							"Unable to initialize dynamic control unit", VS_VISIBLE },
{ RET_DCU_STEP_FAILED,							"Unable to perform step of dynamic control unit", VS_VISIBLE },
{ RET_ESTIMATOR_INIT_FAILED,					"Unable to initialize estimator", VS_VISIBLE },
{ RET_ESTIMATOR_STEP_FAILED,					"Unable to perform estimator step", VS_VISIBLE },
{ RET_OPTIMIZER_INIT_FAILED,					"Unable to initialize optimizer", VS_VISIBLE },
{ RET_OPTIMIZER_STEP_FAILED,					"Unable to perform optimizer step", VS_VISIBLE },
{ RET_NO_OCP_SPECIFIED,							"No optimal control problem has been specified", VS_VISIBLE },
{ RET_NO_SOLUTIONALGORITHM_SPECIFIED,			"No solution algorithm has been specified", VS_VISIBLE },

/* ControlLaw */
{ RET_CONTROLLAW_INIT_FAILED,					"Unable to initialize feedback law", VS_VISIBLE },
{ RET_CONTROLLAW_STEP_FAILED,					"Unable to perform feedback law step", VS_VISIBLE },
{ RET_NO_OPTIMIZER_SPECIFIED,					"No optimizer has been specified", VS_VISIBLE },
{ RET_INVALID_PID_OUTPUT_DIMENSION,				"Invalid output dimension of PID controller, reset to 1", VS_VISIBLE },

/* RealTimeAlgorithm */
{ RET_IMMEDIATE_FEEDBACK_ONE_ITERATION,			"Resetting maximum number of iterations to 1 as required for using immediate feedback", VS_VISIBLE },

/* OutputTransformator */
{ RET_OUTPUTTRANSFORMATOR_INIT_FAILED,			"Unable to initialize output transformator", VS_VISIBLE },
{ RET_OUTPUTTRANSFORMATOR_STEP_FAILED,			"Unable to perform output transformator step", VS_VISIBLE },

/* Function */
{ RET_INVALID_USE_OF_FUNCTION,					"Invalid use of the class function", VS_VISIBLE },
{ RET_INFEASIBLE_CONSTRAINT,					"Infeasible constraints detected", VS_VISIBLE },
{ RET_ONLY_SUPPORTED_FOR_SYMBOLIC_FUNCTIONS,	"This routine is for symbolic functions only", VS_VISIBLE },
{ RET_INFEASIBLE_ALGEBRAIC_CONSTRAINT,			"Infeasible algebraic constraints are not allowed and will be ignored", VS_VISIBLE },
{ RET_ILLFORMED_ODE,							"ODE needs to depend on all differential states", VS_VISIBLE },

/* Expression */
{ RET_PRECISION_OUT_OF_RANGE,					"The requested precision is out of range", VS_VISIBLE },
{ RET_ERROR_WHILE_PRINTING_A_FILE,				"An error has occured while printing a file", VS_VISIBLE },
{ RET_INDEX_OUT_OF_RANGE,						"An index was not in the range", VS_VISIBLE },
{ RET_INTERMEDIATE_STATE_HAS_NO_ARGUMENT,		"The intermediate state has no argument", VS_VISIBLE },
{ RET_DIMENSION_NOT_SPECIFIED,					"The dimension of a array was not specified", VS_VISIBLE },

/* Modeling Tools */
{ RET_DDQ_DIMENSION_MISMATCH,					"ddq argument must be of size 3x1", VS_VISIBLE },
{ RET_CAN_ONLY_SOLVE_2ND_ORDER_KINVECS,			"can only solve 2nd order KinVecs", VS_VISIBLE },


/* OBJECTIVE */
{ RET_GAUSS_NEWTON_APPROXIMATION_NOT_SUPPORTED,	"The objective does not support Gauss-Newton Hessian approximations", VS_VISIBLE },
{ RET_REFERENCE_SHIFTING_WORKS_FOR_LSQ_TERMS_ONLY,	"The reference shifting works only for LSQ objectives", VS_VISIBLE },

/* Integrator */
{ RET_TRIVIAL_RHS,								"The dimension of the rhs is zero", VS_VISIBLE },
{ RET_MISSING_INPUTS,							"There are some inputs missing", VS_VISIBLE },
{ RET_TO_SMALL_OR_NEGATIVE_TIME_INTERVAL,		"The time interval was too small or negative", VS_VISIBLE },
{ RET_FINAL_STEP_NOT_PERFORMED_YET,				"The integration routine is not ready", VS_VISIBLE },
{ RET_ALREADY_FROZEN,							"The integrator is already freezing or frozen", VS_VISIBLE },
{ RET_MAX_NUMBER_OF_STEPS_EXCEEDED,				"The maximum number of steps has been exceeded", VS_VISIBLE },
{ RET_WRONG_DEFINITION_OF_SEEDS,				"The seeds are not set correctly", VS_VISIBLE },
{ RET_NOT_FROZEN,								"The mesh is not frozen and/or forward results not stored", VS_VISIBLE },
{ RET_TO_MANY_DIFFERENTIAL_STATES,				"There are to many differential states", VS_VISIBLE },
{ RET_TO_MANY_DIFFERENTIAL_STATE_DERIVATIVES,	"There are to many diff. state derivatives", VS_VISIBLE },
{ RET_RK45_CAN_NOT_TREAT_DAE,					"An explicit Runge-Kutta solver cannot treat DAEs", VS_VISIBLE },
{ RET_CANNOT_TREAT_DAE,							"The algorithm cannot treat DAEs.", VS_VISIBLE },
{ RET_INPUT_HAS_WRONG_DIMENSION,				"At least one of the inputs has a wrong dimension", VS_VISIBLE },
{ RET_INPUT_OUT_OF_RANGE,						"One of the inputs is out of range", VS_VISIBLE },
{ RET_THE_DAE_INDEX_IS_TOO_LARGE,				"The index of the DAE is larger than 1", VS_VISIBLE },
{ RET_UNSUCCESSFUL_RETURN_FROM_INTEGRATOR_RK45,	"The integration routine stopped as the required accuracy can not be obtained", VS_VISIBLE },
{ RET_UNSUCCESSFUL_RETURN_FROM_INTEGRATOR_BDF,	"The integration routine stopped as the required accuracy can not be obtained", VS_VISIBLE },
{ RET_CANNOT_TREAT_DISCRETE_DE,					"This integrator cannot treat discrete-time differential equations", VS_VISIBLE },
{ RET_CANNOT_TREAT_CONTINUOUS_DE,				"This integrator cannot treat time-continuous differential equations", VS_VISIBLE },
{ RET_CANNOT_TREAT_IMPLICIT_DE,					"This integrator cannot treat differential equations in implicit form", VS_VISIBLE },
{ RET_CANNOT_TREAT_EXPLICIT_DE,					"This integrator cannot treat differential equations in explicit form", VS_VISIBLE },

/* DynamicDiscretization */
{ RET_TO_MANY_DIFFERENTIAL_EQUATIONS,			"The number of differential equations is too large", VS_VISIBLE },
{ RET_BREAK_POINT_SETUP_FAILED,					"The break point setup failed", VS_VISIBLE },
{ RET_WRONG_DEFINITION_OF_STAGE_TRANSITIONS,	"The definition of stage transitions is wrong", VS_VISIBLE },
{ RET_TRANSITION_DEPENDS_ON_ALGEBRAIC_STATES,	"A transition should never depend on algebraic states", VS_VISIBLE },

/* OPTIMIZATION_ALGORITHM: */
{ RET_NO_VALID_OBJECTIVE,						"No valid objective found", VS_VISIBLE },
{ RET_INCONSISTENT_BOUNDS,						"The bounds are inconsistent", VS_VISIBLE },
{ RET_INCOMPATIBLE_DIMENSIONS,					"Incompatible dimensions detected", VS_VISIBLE },
{ RET_GRID_SETUP_FAILED,						"Discretization of the OCP failed", VS_VISIBLE },
{ RET_OPTALG_INIT_FAILED,						"Initialization of optimization algorithm failed", VS_VISIBLE },
{ RET_OPTALG_STEP_FAILED,						"Step of optimization algorithm failed", VS_VISIBLE },
{ RET_OPTALG_FEEDBACK_FAILED,					"Feedback step of optimization algorithm failed", VS_VISIBLE },
{ RET_OPTALG_PREPARE_FAILED,					"Preparation step of optimization algorithm failed", VS_VISIBLE },
{ RET_OPTALG_SOLVE_FAILED,						"Problem could not be solved with given optimization algorithm", VS_VISIBLE },
{ RET_REALTIME_NO_INITIAL_VALUE,				"No initial value has been specified", VS_VISIBLE },

/* INTEGRATION_ALGORITHM: */
{ RET_INTALG_INIT_FAILED, 						"Initialization of integration algorithm failed", VS_VISIBLE },
{ RET_INTALG_INTEGRATION_FAILED, 				"Integration algorithm failed to integrate dynamic system", VS_VISIBLE },
{ RET_INTALG_NOT_READY, 						"Integration algorithm has not been initialized", VS_VISIBLE },

/* PLOT WINDOW */
{ RET_PLOT_WINDOW_CAN_NOT_BE_OPEN,				"ACADO was not able to open the plot window", VS_VISIBLE },

/* NLP SOLVER */
{ CONVERGENCE_ACHIEVED,							"Convergence achieved", VS_VISIBLE },
{ CONVERGENCE_NOT_YET_ACHIEVED,					"Convergence not yet achieved", VS_VISIBLE },
{ RET_NLP_INIT_FAILED,							"Initialization of NLP solver failed", VS_VISIBLE },
{ RET_NLP_STEP_FAILED,							"Step of NLP solver failed", VS_VISIBLE },
{ RET_NLP_SOLUTION_FAILED,						"NLP solution failed", VS_VISIBLE },
{ RET_INITIALIZE_FIRST,							"Object needs to be initialized first", VS_VISIBLE },
{ RET_SOLVER_NOT_SUTIABLE_FOR_REAL_TIME_MODE,	"The specified NLP solver is not designed for a real-time mode", VS_VISIBLE },
{ RET_ILLFORMED_HESSIAN_MATRIX,					"Hessian matrix is too ill-conditioned to continue", VS_VISIBLE },
{ RET_NONSYMMETRIC_HESSIAN_MATRIX,				"Hessian matrix is not symmetric, proceeding with symmetrized Hessian", VS_VISIBLE },
{ RET_UNABLE_TO_EVALUATE_OBJECTIVE,				"Evaluation of objective function failed", VS_VISIBLE },
{ RET_UNABLE_TO_EVALUATE_CONSTRAINTS,			"Evaluation of constraints failed", VS_VISIBLE },
{ RET_UNABLE_TO_INTEGRATE_SYSTEM,				"Integration of dynamic system failed. Try to adjust integrator tolerances using set( ABSOLUTE_TOLERANCE,<double> ) and set( INTEGRATOR_TOLERANCE,<double> )", VS_VISIBLE },
{ RET_NEED_TO_ACTIVATE_RTI,						"Feedback step requires real-time iterations to be activated. Use set( USE_REALTIME_ITERATIONS,YES ) to do so", VS_VISIBLE },

/* CONIC SOLVER */
{ RET_CONIC_PROGRAM_INFEASIBLE,					"The optimization problem is infeasible", VS_VISIBLE },
{ RET_CONIC_PROGRAM_SOLUTION_FAILED,			"Conic Program solution failed. The optimization problem might be infeasible", VS_VISIBLE },
{ RET_CONIC_PROGRAM_NOT_SOLVED,					"The Conic Program has not been solved successfully", VS_VISIBLE },

/* CP SOLVER */
{ RET_UNABLE_TO_CONDENSE,						"Unable to condense banded CP", VS_VISIBLE },
{ RET_UNABLE_TO_EXPAND,							"Unable to expand condensed CP", VS_VISIBLE },
{ RET_NEED_TO_CONDENSE_FIRST,					"Condensing cannot be frozen as banded CP needs to be condensed first", VS_VISIBLE },
{ RET_BANDED_CP_INIT_FAILED,					"Initialization of banded CP solver failed", VS_VISIBLE },
{ RET_BANDED_CP_SOLUTION_FAILED,				"Solution of banded CP failed", VS_VISIBLE },

/* OCP */
{ RET_TRANSITION_NOT_DEFINED,					"No transition function found", VS_VISIBLE },

/* QP SOLVER */
{ RET_QP_INIT_FAILED,							"QP initialization failed", VS_VISIBLE },
{ RET_QP_SOLUTION_FAILED,						"QP solution failed", VS_VISIBLE },
{ RET_QP_SOLUTION_REACHED_LIMIT,				"QP solution stopped as iteration limit is reached", VS_VISIBLE },
{ RET_QP_INFEASIBLE,							"QP solution failed due to infeasibility", VS_VISIBLE },
{ RET_QP_UNBOUNDED,								"QP solution failed due to unboundedness", VS_VISIBLE },
{ RET_QP_NOT_SOLVED,							"QP has not been solved", VS_VISIBLE },
{ RET_RELAXING_QP,								"QP needs to be relaxed due to infeasibility", VS_VISIBLE },
{ RET_COULD_NOT_RELAX_QP,						"QP could not be relaxed", VS_VISIBLE },
{ RET_QP_SOLVER_CAN_ONLY_SOLVE_QP,				"The QP solver can not deal with general conic problems.", VS_VISIBLE },
{ RET_QP_HAS_INCONSISTENT_BOUNDS,				"QP cannot be solved due to inconsistent bounds", VS_VISIBLE },
{ RET_UNABLE_TO_HOTSTART_QP,					"Unable to hotstart QP with given solver", VS_VISIBLE },

/* MPC SOLVER */
{ RET_NONPOSITIVE_WEIGHT,						"Weighting matrices must be positive definite", VS_VISIBLE },
{ RET_INITIAL_CHOLESKY_FAILED,					"Setting up initial Cholesky decompostion failed", VS_VISIBLE },
{ RET_HOMOTOPY_STEP_FAILED,						"Unable to perform homotopy step", VS_VISIBLE },
{ RET_STEPDIRECTION_DETERMINATION_FAILED,		"Determination of step direction failed", VS_VISIBLE },
{ RET_STEPDIRECTION_FAILED_CHOLESKY,			"Abnormal termination due to Cholesky factorisation", VS_VISIBLE },
{ RET_STEPLENGTH_DETERMINATION_FAILED,			"Determination of step direction failed", VS_VISIBLE },
{ RET_OPTIMAL_SOLUTION_FOUND,					"Optimal solution of neighbouring QP found", VS_VISIBLE },
{ RET_MAX_NWSR_REACHED,							"Maximum number of working set recalculations performed", VS_VISIBLE },
{ RET_MATRIX_NOT_SPD,							"DMatrix not positive definite", VS_VISIBLE },

/* CODE EXPORT */
{ RET_CODE_EXPORT_SUCCESSFUL,					"Code generation successful", VS_VISIBLE },
{ RET_UNABLE_TO_EXPORT_CODE,					"Unable to generate code", VS_VISIBLE },
{ RET_INVALID_OBJECTIVE_FOR_CODE_EXPORT,		"Only standard LSQ objective supported for code generation", VS_VISIBLE },
{ RET_NO_DISCRETE_ODE_FOR_CODE_EXPORT,			"No discrete-time ODEs supported for code generation", VS_VISIBLE },
{ RET_ONLY_ODE_FOR_CODE_EXPORT,					"Only ODEs supported for code generation", VS_VISIBLE },
{ RET_ONLY_STATES_AND_CONTROLS_FOR_CODE_EXPORT,	"No parameters, disturbances or algebraic states supported for code generation", VS_VISIBLE },
{ RET_ONLY_EQUIDISTANT_GRID_FOR_CODE_EXPORT,	"Only equidistant evaluation grids supported for code generation", VS_VISIBLE },
{ RET_ONLY_BOUNDS_FOR_CODE_EXPORT,				"Only state and control bounds supported for code generation", VS_VISIBLE },
{ RET_QPOASES_EMBEDDED_NOT_FOUND,				"Embedded qpOASES code not found", VS_VISIBLE },
{ RET_UNABLE_TO_EXPORT_STATEMENT,				"Unable to export statement due to incomplete definition", VS_VISIBLE },
{ RET_INVALID_CALL_TO_EXPORTED_FUNCTION,		"Invalid call to export functions (check number of calling arguments)", VS_VISIBLE },

/* IMPORTANT: Terminal list element! */
{ TERMINAL_LIST_ELEMENT,						" ", VS_HIDDEN }
};


//
// HELPER FUNCTIONS:
//


// Converts returnValueLevel enum type to a const char*
const char* returnValueLevelToString(returnValueLevel level)
{
	switch ( level )
	{
	case LVL_DEBUG:
		return COL_DEBUG"Debug";

	case LVL_FATAL:
		return COL_FATAL"Fatal error";

	case LVL_ERROR:
		return COL_ERROR"Error";

	case LVL_WARNING:
		return COL_WARNING"Warning";

	case LVL_INFO:
		return COL_INFO"Information";
	}

	return 0;
}

// Converts returnValueType enum type to a const char*
const char* returnValueTypeToString(returnValueType type) {
	ReturnValueList* p = returnValueList;
	while( (p->key != type) && (p->key != TERMINAL_LIST_ELEMENT) ) p++;
	return p->data;
}

/** Internal data holding class for returnValue class
 *  Owner is always set to the last returnValue instance which was copy constructed/assigned to it.
 */
class returnValue::returnValueData
{
public:
	returnValue* owner;
	std::vector<const char*> messages;
};

//
// PUBLIC MEMBER FUNCTIONS:
//


/* Constructor used by the ACADOERROR and similar macros.
 *
 */
returnValue::returnValue(const char* msg, returnValueLevel _level, returnValueType _type) {
	data = new returnValueData();
	data->owner = this;
	data->messages.push_back(msg);
	status = STATUS_UNHANDLED;
	type = _type;
	level = _level;
}

/* Construct default returnValue.
 *
 */
returnValue::returnValue() : data(0) {}

/* Construct returnValue only from typedef.
 *
 */
returnValue::returnValue(returnValueType _type) : type(_type), level(LVL_ERROR), status(STATUS_UNHANDLED), data(0) {}

/* Construct returnValue from int, for compatibility
 *
 */
returnValue::returnValue(int _type) : level(LVL_ERROR), status(STATUS_UNHANDLED), data(0) {
	type = returnValueType(_type);
}

/* Copy constructor with minimum performance cost.
 *  Newly constructed instance takes ownership of data.
 */
returnValue::returnValue(const returnValue& old) {
	// Copy data
	if (old.data) {
		data = old.data;
		data->owner = this;
	} else {
		data = 0;
	}
	// Copy details
	type = old.type;
	level = old.level;
	status = old.status;
}

returnValueLevel returnValue::getLevel() const {
	return level;
}

/* Compares the returnValue type to its enum
 *
 */
bool returnValue::operator!=(returnValueType cmp_type) const {
	return type != cmp_type;
}

/* Compares the returnValue type to its enum
 *
 */
bool returnValue::operator==(returnValueType cmp_type) const {
	return type == cmp_type;
}

/* Returns true if return value type is not SUCCESSFUL_RETURN
 *
 */
bool returnValue::operator!() const {
	return type != SUCCESSFUL_RETURN;
}

/*  Assignment operator.
 *  Left hand side instance takes ownership of data.
 */
returnValue& returnValue::operator=(const returnValue& old) {
	// Clean up data if already existing
	if (data && (data->owner == this)) delete data;

	// Copy data
	if (old.data) {
		data = old.data;
		data->owner = this;
	} else {
		data = 0;
	}
	// Copy details
	type = old.type;
	level = old.level;
	status = old.status;

	return *this;
}

/* Compatibility function, allows returnValue to be used as a number, similar to a enum.
 *
 */
returnValue::operator int() {
	return type;
}

/* Destroys data instance only if it owns it
 *
 */
returnValue::~returnValue() {
	if ( data )
	{
		if (data->owner == this)
		{
			// If is not succesful and was not handled yet, by default print message to standard output
			if (status != STATUS_HANDLED)
			{
				print();
			}
		
			if ( data )
			{
				delete data;
			}

#ifdef ACADO_WITH_TESTING
			if (level == LVL_FATAL || level == LVL_ERROR || level == LVL_WARNING)
#else
			if (level == LVL_FATAL)
#endif
			{
				abort();
			}
		}
	}
}

/* Constructor used by the ACADOERROR and similar macros. Special case.
 * Constructs returnValue from old one, changing level and adding message
 */
returnValue::returnValue(const char* msg, returnValueLevel _level, const returnValue& old) {
	if (old.data) {
		data = old.data;
		data->owner = this;
		data->messages.push_back(msg);
	}
	status = STATUS_UNHANDLED;
	level = _level;
	type = old.type;
}

/* Adds another message to the end of messages list.
 *
 */	
returnValue& returnValue::addMessage(const char* msg) {
	if (!data) {
		data = new returnValueData();
		data->owner = this;
	}
	data->messages.push_back(msg);
	return *this;
}

/* Change the importance level of the returned value
 *
 */
returnValue& returnValue::changeLevel(returnValueLevel _level) {
	level = _level;
	return *this;
}

/* Change the type of the returned message
 *
 */
returnValue& returnValue::changeType(returnValueType _type) {

	type = _type;
	return *this;
}

/* Prints all messages to the standard output.
 *
 */	
void returnValue::print() {

	cout 	<< COL_INFO"[ACADO] " << returnValueLevelToString( level )
			<< ": " << returnValueTypeToString( type ) << COL_INFO << endl;

	if ( data )
		for (vector<const char*>::iterator it = data->messages.begin(); it != data->messages.end(); it++)
			cout << "  " << (*it) << endl;
	cout << endl;

	status = STATUS_HANDLED;
}

/* Prints only the most basic information, no messages, to the standard output.
 *
 */
void returnValue::printBasic() {

	std::cout << returnValueLevelToString(level) << ": " << returnValueTypeToString(type) << std::endl; 
	status = STATUS_HANDLED;
}

// Logging class

ostream& Logger::get(returnValueLevel level)
{
	cout << COL_INFO"[ACADO] " << returnValueLevelToString( level ) << ": " << COL_INFO;

	return cout;
}

CLOSE_NAMESPACE_ACADO


/*
 *	end of file
 */
