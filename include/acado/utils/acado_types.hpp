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
 *    \file include/acado/utils/acado_types.hpp
 *    \author Hans Joachim Ferreau, Boris Houska
 *
 *    This file collects all declarations of all non-built-in types
 *    (except for classes).
 */


#ifndef ACADO_TOOLKIT_ACADO_TYPES_HPP
#define ACADO_TOOLKIT_ACADO_TYPES_HPP


#include <acado/utils/acado_namespace_macros.hpp>


BEGIN_NAMESPACE_ACADO


/** Short-cut for unsigned integer. */
typedef unsigned int uint;


/** Alias for DifferentialEquation. */
//typedef DifferentialEquation DynamicModel;


/** Function pointer type for functions given as C source code. */
typedef void (*cFcnPtr)( double* x, double* f, void *userData );

/** Function pointer type for derivatives given as C source code. */
typedef void (*cFcnDPtr)( int number, double* x, double* seed, double* f, double* df, void *userData );

/** Summarises all possible logical values. */
enum BooleanType{

    BT_FALSE,    /**< Logical value for "false". */
    BT_TRUE      /**< Logical value for "true".  */
};

#define NO  BT_FALSE
#define YES BT_TRUE


/** Defines the Neutral Elements ZERO and ONE as well as the default \n
 *  NEITHER_ONE_NOR_ZERO                                             \n
 */
enum NeutralElement{

    NE_ZERO,
    NE_ONE,
    NE_NEITHER_ONE_NOR_ZERO
};


/** Defines the names of all implemented symbolic operators.
*/
enum OperatorName{

    ON_SIN,
    ON_COS,
    ON_TAN,
    ON_ASIN,
    ON_ACOS,
    ON_ATAN,
    ON_LOGARITHM,
    ON_EXP,
    ON_ADDITION,
    ON_SUBTRACTION,
    ON_POWER,
    ON_POWER_INT,
    ON_PRODUCT,
    ON_QUOTIENT,
    ON_VARIABLE,
    ON_DOUBLE_CONSTANT,
    ON_DIFFERENTIAL_STATE,
    ON_CEXPRESSION
};



/** Defines the names of all implemented variable types. */
enum VariableType{

    VT_DIFFERENTIAL_STATE,
    VT_ALGEBRAIC_STATE,
    VT_CONTROL,
    VT_INTEGER_CONTROL,
    VT_PARAMETER,
    VT_INTEGER_PARAMETER,
    VT_DISTURBANCE,
    VT_TIME,
    VT_INTERMEDIATE_STATE,
    VT_DDIFFERENTIAL_STATE,
	VT_OUTPUT,
    VT_UNKNOWN
};


/** Defines all possible methods of merging variables grids in case a grid point 
 *	exists in both grids. */
enum MergeMethod
{
	MM_KEEP,			/**< Keeps original values. */
	MM_REPLACE,			/**< Replace by new values. */
	MM_DUPLICATE		/**< Duplicate grid point (i.e. keeping old and adding new). */
};



/** Defines all possible sub-block matrix types. */
enum SubBlockMatrixType{

    SBMT_ZERO,
    SBMT_ONE,
    SBMT_DENSE,
    SBMT_UNKNOWN
};


/** Defines all possible relaxation type used in DAE integration routines */
enum AlgebraicRelaxationType{

    ART_EXPONENTIAL,
    ART_ADAPTIVE_POLYNOMIAL,
    ART_UNKNOWN
};


/** Defines all possible linear algebra solvers. */
enum LinearAlgebraSolver{

	HOUSEHOLDER_QR,
	GAUSS_LU,
    HOUSEHOLDER_METHOD,
	SPARSE_LU,
    LAS_UNKNOWN
};


/** Defines the mode of the exported implicit integrator. */
enum ImplicitIntegratorMode{

	IFTR,			/**< With the reuse of the matrix evaluation and factorization from the previous step (1 evaluation and factorization per integration step). */
	IFT				/**< Without the reuse of the matrix from the previous step (2 evaluations and factorizations per integration step). */
};


/** Defines all possible monotonicity types. */
enum MonotonicityType{

    MT_CONSTANT,
    MT_NONDECREASING,
    MT_NONINCREASING,
    MT_NONMONOTONIC,
    MT_UNKNOWN
};


/** Defines all possible curvature types. */
enum CurvatureType{

    CT_CONSTANT,                     /**< constant expression        */
    CT_AFFINE,                       /**< affine expression          */
    CT_CONVEX,                       /**< convex expression          */
    CT_CONCAVE,                      /**< concave expression         */
    CT_NEITHER_CONVEX_NOR_CONCAVE,   /**< neither convex nor concave */
    CT_UNKNOWN                       /**< unknown                    */
};


enum DifferentialEquationType{

    DET_ODE,                         /**< ordinary differential equation  */
    DET_DAE,                         /**< differential algebraic equation */
    DET_UNKNOWN                      /**< unknown                         */
};



/** Summarises all possible ways of discretising the system's states. */
enum StateDiscretizationType{

    SINGLE_SHOOTING,        /**< Single shooting discretisation.   */
    MULTIPLE_SHOOTING,      /**< Multiple shooting discretisation. */
    COLLOCATION,            /**< Collocation discretisation.       */
    UNKNOWN_DISCRETIZATION  /**< Discretisation type unknown.      */
};


/** Summarises all possible ways of discretising the system's states. */
enum ControlParameterizationType{

    CPT_CONSTANT,           /**< piece wise constant parametrization */
    CPT_LINEAR,             /**< piece wise linear   parametrization */
    CPT_LINEAR_CONTINUOUS,  /**< contious linear     parametrization */
    CPT_CUSTOMIZED,         /**< costumized                          */
    CPT_UNKNOWN             /**< unknown                             */
};


/** Summarises all possible ways of globablizing NLP steps. */
enum GlobalizationStrategy
{
	GS_FULLSTEP,					/**< Full step. */
	GS_LINESEARCH,					/**< Linesearch. */
	GS_UNKNOWN						/**< Unknown. */
};


/** Summarises all possible interpolation modes for VariablesGrids, Curves and the like. */
enum InterpolationMode
{
	IM_CONSTANT,					/**< Piecewise constant interpolation (not continous). */
	IM_LINEAR,						/**< Linear interpolation. */
	IM_QUADRATIC,					/**< Quadratic interpolation. */
	IM_CUBIC,						/**< Cubic interpolation. */
	IM_UNKNOWN						/**< Unknown interpolation mode. */
};


/** Summarizes all possible states of aggregation. (e.g. a mesh for
 *  an integration routine can be freezed, unfreezed, etc.)
 */
enum StateOfAggregation{

    SOA_FREEZING_MESH,             /**< freeze the mesh during next evaluation   */
    SOA_FREEZING_ALL,              /**< freeze everything during next evaluation */
    SOA_MESH_FROZEN,               /**< the mesh is frozen                       */
    SOA_MESH_FROZEN_FREEZING_ALL,  /**< the mesh is frozen, freeze also trajectory during next evaluation */
    SOA_EVERYTHING_FROZEN,         /**< everything is frozen                     */
    SOA_UNFROZEN,                  /**< everything is unfrozed                   */
    SOA_UNKNOWN                    /**< unknown                                  */
};


/** Summarizes all available integrators in standard ACADO. */
enum IntegratorType{

     INT_RK12,             	/**< Explicit Runge-Kutta integrator of order 1/2          */
     INT_RK23,             	/**< Explicit Runge-Kutta integrator of order 2/3          */
     INT_RK45,             	/**< Explicit Runge-Kutta integrator of order 4/5          */
     INT_RK78,            	/**< Explicit Runge-Kutta integrator of order 7/8          */
     INT_BDF,             	/**< Implicit backward differentiation formula integrator. */
     INT_DISCRETE,        	/**< Discrete time integrator                              */
     INT_LYAPUNOV45,        /**< Explicit Runge-Kutta integrator of order 4/5  with Lyapunov structure exploiting        */
     INT_UNKNOWN           	/**< unkown.                                               */
};


/** The available options for providing the grid of measurements.
 */
enum MeasurementGrid{

	OFFLINE_GRID,       	/**< An equidistant grid specified independent of the integration grid.         */
	ONLINE_GRID         	/**< A random grid, provided online by the user.           						*/
};


/** Unrolling option.
 */
enum UnrollOption{

     UNROLL,
     NO_UNROLL,
     HEURISTIC_UNROLL
};



/** Summarises all possible print levels. Print levels are used to describe
 *  the desired amount of output during runtime of ACADO Toolkit.
 */
enum PrintLevel
{
    NONE,        /**< No output.                                                         */
    LOW,         /**< Print error messages only.                                         */
    MEDIUM,      /**< Print error and warning messages as well as concise info messages. */
    HIGH,        /**< Print all messages with full details.                              */
    DEBUG        /**< Print all messages with full details as well                       *
                     *   all ugly messages that might be helpful for                        *
                     *   debugging the code.                                                */
};


/** Summarises all possible types of OptionItems.
 */
enum OptionsItemType
{
	OIT_INT,			/**< Option item comprising a value of integer type. */
	OIT_DOUBLE,			/**< Option item comprising a value of double type.  */
	OIT_UNKNOWN			/**< Option item comprising a value of unknown type. */
};


enum LogRecordItemType{

    LRT_ENUM,
    LRT_VARIABLE,
    LRT_UNKNOWN
};


/** Defines logging frequency.
 */
enum LogFrequency{

    LOG_AT_START,
    LOG_AT_END,
    LOG_AT_EACH_ITERATION
};


/** Defines all possibilities to print LogRecords.
 */
enum LogPrintMode
{
	PRINT_ITEM_BY_ITEM,		/**< Print all numerical values of one item and continue with next item. */
	PRINT_ITER_BY_ITER,		/**< Print all numerical values of all items at one time instant (or iteration) and continue with next time instant. */
	PRINT_LAST_ITER			/**< Print all numerical values of all items at last time instant (or iteration) only. */
};


enum OptionsName
{
	CG_USE_ARRIVAL_COST,						/**< Enable interface for arival cost calculation. */
	CG_USE_OPENMP,								/**< Use OpenMP for parallelization in multiple shooting. */
	CG_USE_VARIABLE_WEIGHTING_MATRIX,			/**< Use variable weighting matrix S on first N shooting nodes. */
	CG_USE_C99,									/**< Code generation is allowed (or not) to export C-code that conforms C99 standard. */
	CG_COMPUTE_COVARIANCE_MATRIX,				/**< Enable computation of the variance-covariance matrix for the last estimate. */
	CG_HARDCODE_CONSTRAINT_VALUES,				/**< Enable/disable hard-coding of the constraint values. */
	IMPLICIT_INTEGRATOR_MODE,					/**< This determines the mode of the implicit integrator (see enum ImplicitIntegratorMode). */
	IMPLICIT_INTEGRATOR_NUM_ITS,				/**< This is the performed number of Newton iterations in the implicit integrator. */
	IMPLICIT_INTEGRATOR_NUM_ITS_INIT,			/**< This is the performed number of Newton iterations in the implicit integrator for the initialization of the first step. */
	UNROLL_LINEAR_SOLVER,						/**< This option of the boolean type determines the unrolling of the linear solver (no unrolling recommended for larger systems). */
	INTEGRATOR_DEBUG_MODE,
	OPT_UNKNOWN,
	MAX_NUM_INTEGRATOR_STEPS,
	NUM_INTEGRATOR_STEPS,
	INTEGRATOR_TOLERANCE,
	MEX_ITERATION_STEPS,						/**< The number of real-time iterations performed in the auto generated mex function. */
	MEX_VERBOSE,
	ABSOLUTE_TOLERANCE,
	INITIAL_INTEGRATOR_STEPSIZE,
	MIN_INTEGRATOR_STEPSIZE,
	MAX_INTEGRATOR_STEPSIZE,
	STEPSIZE_TUNING,
	CORRECTOR_TOLERANCE,
	INTEGRATOR_PRINTLEVEL,
	LINEAR_ALGEBRA_SOLVER,
	ALGEBRAIC_RELAXATION,
	RELAXATION_PARAMETER,
	PRINT_INTEGRATOR_PROFILE,
	FEASIBILITY_CHECK,
	MAX_NUM_ITERATIONS,
	KKT_TOLERANCE,
	KKT_TOLERANCE_SAFEGUARD,
	LEVENBERG_MARQUARDT,
	PRINTLEVEL,
	PRINT_COPYRIGHT,
	HESSIAN_APPROXIMATION,
	DYNAMIC_HESSIAN_APPROXIMATION,
	HESSIAN_PROJECTION_FACTOR,
	DYNAMIC_SENSITIVITY,
	OBJECTIVE_SENSITIVITY,
	CONSTRAINT_SENSITIVITY,
	DISCRETIZATION_TYPE,
	LINESEARCH_TOLERANCE,
	MIN_LINESEARCH_PARAMETER,
	QP_SOLVER,
	MAX_NUM_QP_ITERATIONS,
	HOTSTART_QP,
	INFEASIBLE_QP_RELAXATION,
	INFEASIBLE_QP_HANDLING,
	USE_REALTIME_ITERATIONS,
	USE_REALTIME_SHIFTS,
	USE_IMMEDIATE_FEEDBACK,
	TERMINATE_AT_CONVERGENCE,
	USE_REFERENCE_PREDICTION,
	FREEZE_INTEGRATOR,
	INTEGRATOR_TYPE,
	MEASUREMENT_GRID,
	SAMPLING_TIME,
	SIMULATE_COMPUTATIONAL_DELAY,
	COMPUTATIONAL_DELAY_FACTOR,
	COMPUTATIONAL_DELAY_OFFSET,
	PARETO_FRONT_DISCRETIZATION,
	PARETO_FRONT_GENERATION,
	PARETO_FRONT_HOTSTART,
	SIMULATION_ALGORITHM,
	CONTROL_PLOTTING,
	PARAMETER_PLOTTING,
	OUTPUT_PLOTTING,
	SPARSE_QP_SOLUTION,
	GLOBALIZATION_STRATEGY,
	CONIC_SOLVER_MAXIMUM_NUMBER_OF_STEPS,
	CONIC_SOLVER_TOLERANCE,
	CONIC_SOLVER_LINE_SEARCH_TUNING,
	CONIC_SOLVER_BARRIER_TUNING,
	CONIC_SOLVER_MEHROTRA_CORRECTION,
	CONIC_SOLVER_PRINT_LEVEL,
	PRINT_SCP_METHOD_PROFILE,
	PLOT_RESOLUTION,
	FIX_INITIAL_STATE,
	GENERATE_TEST_FILE,
	GENERATE_MAKE_FILE,
	GENERATE_SIMULINK_INTERFACE,
	GENERATE_MATLAB_INTERFACE,
	OPERATING_SYSTEM,
	USE_SINGLE_PRECISION
};


/** Defines possible logging output
 */
enum LogName
{
    LOG_NOTHING,
	// 1
	LOG_NUM_NLP_ITERATIONS, /**< Log number of NLP interations */
	LOG_NUM_SQP_ITERATIONS, /**< Log number of SQP interations */
	LOG_NUM_QP_ITERATIONS, /**< Log number of QP iterations */
	LOG_KKT_TOLERANCE,   /**< Log KKT tolerances */
	LOG_OBJECTIVE_VALUE, /**< Log values objective function */
	LOG_MERIT_FUNCTION_VALUE, /**< Log Merit function value*/
	LOG_LINESEARCH_STEPLENGTH, /**< Steplength of the line search routine (if used) */
	LOG_NORM_LAGRANGE_GRADIENT, /**< Log norm of Lagrange gradient*/
	LOG_IS_QP_RELAXED, /**< Log whether the QP is relaxed or not */
	// 10
	LOG_DUAL_RESIDUUM,
	LOG_PRIMAL_RESIDUUM,
	LOG_SURROGATE_DUALITY_GAP,
	LOG_NUM_INTEGRATOR_STEPS,
	LOG_TIME_SQP_ITERATION,
	LOG_TIME_CONDENSING,
	LOG_TIME_QP,
	LOG_TIME_RELAXED_QP,
	LOG_TIME_EXPAND,
	LOG_TIME_EVALUATION,
	// 20
	LOG_TIME_HESSIAN_COMPUTATION,
	LOG_TIME_GLOBALIZATION,
	LOG_TIME_SENSITIVITIES,
	LOG_TIME_LAGRANGE_GRADIENT,
	LOG_TIME_PROCESS,
	LOG_TIME_CONTROLLER,
	LOG_TIME_ESTIMATOR,
	LOG_TIME_CONTROL_LAW,
	LOG_DIFFERENTIAL_STATES, /**< Log all differential states in the order of occurrence*/
	LOG_ALGEBRAIC_STATES, /**< Log all algebraic states in the order of occurrence*/
	LOG_PARAMETERS, /**< Log all parameters in the order of occurrence*/
	LOG_CONTROLS, /**< Log all controls in the order of occurrence*/
	LOG_DISTURBANCES, /**< Log all disturbances in the order of occurrence*/
	LOG_INTERMEDIATE_STATES, /**< Log all intermediate states in the order of occurrence*/
	// 30
	LOG_DISCRETIZATION_INTERVALS, /**< Log discretization intervals*/
	LOG_STAGE_BREAK_POINTS,
	LOG_FEEDBACK_CONTROL,
	LOG_NOMINAL_CONTROLS,
	LOG_NOMINAL_PARAMETERS,
	LOG_SIMULATED_DIFFERENTIAL_STATES,
	LOG_SIMULATED_ALGEBRAIC_STATES,
	LOG_SIMULATED_CONTROLS,
	LOG_SIMULATED_PARAMETERS,
	LOG_SIMULATED_DISTURBANCES,
	// 40
	LOG_SIMULATED_INTERMEDIATE_STATES,
	LOG_SIMULATED_OUTPUT,
	LOG_PROCESS_OUTPUT,
    LOG_NUMBER_OF_INTEGRATOR_STEPS,
    LOG_NUMBER_OF_INTEGRATOR_REJECTED_STEPS,
    LOG_NUMBER_OF_INTEGRATOR_FUNCTION_EVALUATIONS,
    LOG_NUMBER_OF_BDF_INTEGRATOR_JACOBIAN_EVALUATIONS,
    LOG_TIME_INTEGRATOR,
    LOG_TIME_INTEGRATOR_FUNCTION_EVALUATIONS,
    LOG_TIME_BDF_INTEGRATOR_JACOBIAN_EVALUATION,
	// 50
    LOG_TIME_BDF_INTEGRATOR_JACOBIAN_DECOMPOSITION
};


enum PlotFrequency
{
	PLOT_AT_START,
	PLOT_AT_END,
	PLOT_AT_EACH_ITERATION,
	PLOT_IN_ANY_CASE,
	PLOT_NEVER
};


enum PlotName
{
	PLOT_NOTHING,
	// 1
// 	PLOT_DIFFERENTIAL_STATES,
// 	PLOT_ALGEBRAIC_STATES,
// 	PLOT_CONTROLS,
// 	PLOT_PARAMETERS,
// 	PLOT_DISTURBANCES,
// 	PLOT_INTERMEDIATE_STATES,
	PLOT_KKT_TOLERANCE,
	PLOT_OBJECTIVE_VALUE,
	PLOT_MERIT_FUNCTION_VALUE,
	PLOT_LINESEARCH_STEPLENGTH,
	PLOT_NORM_LAGRANGE_GRADIENT
};


enum ProcessPlotName
{
	PLOT_NOMINAL,
	PLOT_REAL
};


/** Defines all possible plot formats.
 */
enum PlotFormat
{
	PF_PLAIN,						/**< Plot with linear x- and y-axes. */
	PF_LOG,							/**< Plot with linear x-axis and logarithmic y-axis. */
	PF_LOG_LOG,						/**< Plot with logarithmic x- and y-axes. */
	PF_UNKNOWN						/**< Plot format unknown. */
};



/** Defines all possible plot modes.
 */
enum PlotMode
{
	PM_LINES,						/**< Plot data points linearly interpolated with lines. */
	PM_POINTS,						/**< Plot data points as single points. */
	PM_UNKNOWN						/**< Plot mode unknown. */
};



/** Defines all possible sub-plot types.
 */
enum SubPlotType
{
	SPT_VARIABLE,
	SPT_VARIABLE_VARIABLE,
	SPT_VARIABLE_EXPRESSION,
	SPT_VARIABLES_GRID,
	SPT_EXPRESSION,
	SPT_EXPRESSION_EXPRESSION,
	SPT_EXPRESSION_VARIABLE,
	SPT_ENUM,
	SPT_UNKNOWN
};


/** Defines possible printing types used in logging.
 */
enum PrintScheme
{
	PS_DEFAULT,			/**< Default printing, each row starts with [ and ends with ] and a newline. Colums are separated with a space. */
	PS_PLAIN,			/**< Plain printing, rows are separated with a newline and columns with a space */
	PS_MATLAB,			/**< Matlab style output. List starts with [ and ends with ]. Rows are separated by ; and columns by ,. */
	PS_MATLAB_BINARY	/**< Outputs a binary data file that can be read by Matlab. */
};


/**  Summarizes all possible sensitivity types */
enum SensitivityType{

    FORWARD_SENSITIVITY       ,    /**< Sensitivities are computed in forward mode                              */
    FORWARD_SENSITIVITY_LIFTED,    /**< Sensitivities are computed in forward mode using "lifting" if possible. */
    BACKWARD_SENSITIVITY      ,    /**< Sensitivities are computed in backward mode                             */
    UNKNOWN_SENSITIVITY            /**< unknown                                                                 */
};


/**  Condensing type */
enum CondensingType{

    CT_LIFTING,                 /**< Sensitivities are lifted                    */
    CT_SPARSE                   /**< Sensitivities are sparse                    */
};


/** Summarizes all possible algorithms for simulating the process. */
enum ProcessSimulationAlgorithm
{
	SIMULATION_BY_INTEGRATION,		/**< Simulation by using an integrator. */
	SIMULATION_BY_COLLOCATION		/**< Simulation by using a collocation scheme. */
};



/** Definition of several Hessian approximation modes. */
enum HessianApproximationMode{

    CONSTANT_HESSIAN,
	// 1
    GAUSS_NEWTON,
    FULL_BFGS_UPDATE,
    BLOCK_BFGS_UPDATE,
    GAUSS_NEWTON_WITH_BLOCK_BFGS,
    EXACT_HESSIAN,
    DEFAULT_HESSIAN_APPROXIMATION
};


enum QPSolverName
{
	QP_QPOASES,
	QP_QPOASES3,
	QP_FORCES,
	QP_QPDUNES,
	QP_NONE
};


/** Summarises all possible states of the Conic Solver. */
enum ConicSolverStatus{

    CSS_NOTINITIALISED,			/**< The ConicSolver object is freshly instantiated or reset.          */
    CSS_INITIALIZED,			/**< The ConicSolver object is initialised                             */
    CSS_SOLVED,					/**< The solution of the actual Convex Optimization Problem was found. */
    CSS_UNKNOWN					/**< Status unknown.                                                   */
};



/** Summarizes all available strategies for handling infeasible QPs within
 *	an SQP-type NLP solver. */
enum InfeasibleQPhandling
{
	IQH_STOP,					/**< Stop solution. */
	IQH_IGNORE,					/**< Ignore infeasibility and continue solution. */
	IQH_RELAX_L1,				/**< Re-solve relaxed QP using a L1 penalty. */
	IQH_RELAX_L2,				/**< Re-solve relaxed QP using a L2 penalty. */
	IQH_UNDEFINED				/**< No infeasibility handling strategy defined. */
};


/**  Summarizes all possible states of a QP problem. */
enum QPStatus
{
    QPS_NOT_INITIALIZED,		/**< QP problem has not been initialised yet. */
    QPS_INITIALIZED,			/**< QP problem has been initialised.         */
    QPS_SOLVING,				/**< QP problem is being solved.              */
    QPS_SOLVED,					/**< QP problem successfully solved.          */
    QPS_RELAXED,				/**< QP problem has been relaxed.             */
    QPS_SOLVING_RELAXATION,		/**< A relaxed QP problem is being solved.    */
    QPS_SOLVED_RELAXATION,		/**< A relaxed QP problem has been solved.    */
    QPS_INFEASIBLE,				/**< QP problem is infeasible.                */
    QPS_UNBOUNDED,				/**< QP problem is unbounded.                 */
    QPS_NOTSOLVED				/**< QP problem could not been solved.        */
};


/**  Summarizes all possible states of the condensing used within condensing based CP solvers. */
enum CondensingStatus
{
    COS_NOT_INITIALIZED,		/**< Condensing has not been initialised yet. */
    COS_INITIALIZED,			/**< Condensing has been initialised, banded CP ready for condensing. */
    COS_CONDENSED,				/**< Banded CP has been condensed.            */
    COS_FROZEN					/**< Banded CP has been condensed and is frozen in this status. */
};


/**  Summarizes all possible block names. */
enum BlockName
{
    BN_DEFAULT,
    BN_SIMULATION_ENVIRONMENT,
    BN_PROCESS,
    BN_ACTUATOR,
    BN_SENSOR,
    BN_CONTROLLER,
    BN_ESTIMATOR,
    BN_REFERENCE_TRAJECTORY,
    BN_CONTROL_LAW
};


/**  Summarizes all possible states of a block or algorithmic module. */
enum BlockStatus
{
	BS_UNDEFINED,				/**< Status is undefined. */
	BS_NOT_INITIALIZED,			/**< Block/algorithm has been instantiated but not initialized. */
	BS_READY,					/**< Block/algorithm has been initialized and is ready to run. */
	BS_RUNNING					/**< Block/algorithm is running. */
};


/**  Summarizes all possible states of a clock. */
enum ClockStatus
{
	CS_NOT_INITIALIZED,			/**< Clock has not been initialized. */
	CS_RUNNING,					/**< Clock is running. */
	CS_STOPPED					/**< Clock has been initialized and stopped. */
};


/** Defines flags for different vector norms.
 */
enum VectorNorm
{
    VN_L1,
    VN_L2,
    VN_LINF
};


/** Defines flags for different vector norms.
 */
enum MatrixNorm
{
    MN_COLUMN_SUM,
    MN_ROW_SUM,
    MN_FROBENIUS
};



/** Defines the time horizon start and end. \n
 */
enum TimeHorizonElement{

    AT_START      ,
    AT_END        ,
    AT_TRANSITION
};


/** Defines the pareto front generation options. \n
 */
enum ParetoFrontGeneration{

    PFG_FIRST_OBJECTIVE,
    PFG_SECOND_OBJECTIVE,
    PFG_WEIGHTED_SUM,
    PFG_NORMALIZED_NORMAL_CONSTRAINT,
    PFG_NORMAL_BOUNDARY_INTERSECTION,
    PFG_ENHANCED_NORMALIZED_NORMAL_CONSTRAINT,
    PFG_EPSILON_CONSTRAINT,
    PFG_UNKNOWN
};



/** Defines . \n
 */
enum SparseQPsolutionMethods
{
	SPARSE_SOLVER,
	CONDENSING,
	FULL_CONDENSING,
	FULL_CONDENSING_N2
};



/** Defines . \n
 */
enum ExportStatementOperator
{
	ESO_ADD,
	ESO_SUBTRACT,
	ESO_ADD_ASSIGN,
	ESO_SUBTRACT_ASSIGN,
	ESO_MULTIPLY,
	ESO_MULTIPLY_TRANSPOSE,
	ESO_DIVIDE,
	ESO_MODULO,
	ESO_ASSIGN,
	ESO_UNDEFINED
};


enum OperatingSystem
{
	OS_DEFAULT,
	OS_UNIX,
	OS_WINDOWS
};


enum ExportType
{
	INT,
	REAL,
	STATIC_CONST_INT,
	STATIC_CONST_REAL
};


enum ExportStruct
{
	ACADO_VARIABLES,
	ACADO_WORKSPACE,
	ACADO_PARAMS,
	ACADO_VARS,
	ACADO_LOCAL,
	ACADO_ANY,
	FORCES_PARAMS,
	FORCES_OUTPUT,
	FORCES_INFO
};



CLOSE_NAMESPACE_ACADO



#endif	// ACADO_TOOLKIT_ACADO_TYPES_HPP


/*
 *    end of file
 */
