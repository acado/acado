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
 *    \file include/acado/objective/objective.hpp
 *    \author Boris Houska, Hans Joachim Ferreau
 *
 */


#ifndef ACADO_TOOLKIT_OBJECTIVE_HPP
#define ACADO_TOOLKIT_OBJECTIVE_HPP

#include <acado/objective/lagrange_term.hpp>
#include <acado/objective/lsq_term.hpp>
#include <acado/objective/lsq_end_term.hpp>
#include <acado/objective/mayer_term.hpp>
#include <acado/constraint/constraint.hpp>


BEGIN_NAMESPACE_ACADO

class ExportVariable;

/** 
 *	\brief Stores and evaluates the objective function of optimal control problems.
 *
 *	\ingroup BasicDataStructures
 *
 *	The class Objective is class is designed to formulate
 *  objecive functionals that can be part of an optimal
 *  control problem (OCP).
 *  Mainly, an objective can have additive terms with
 *  different structures that can be added by using
 *  various routines that are implemented in this class
 *
 *  Note that a this class is derived from the class
 *  LagrangeTerm while it has Mayer and LSQ-Terms as
 *  a member. The reason for this assymmetry is that a
 *  Lagrange - term will be reformulated as a Mayer term as
 *  soon as the function init( ... ) is called. (I.e. the
 *  class LagrangeTerm is only used a kind of temporary
 *  memory to store Expressions that are added by the
 *  user.)
 *
 *	\author Boris Houska, Hans Joachim Ferreau
 */

class Objective : public LagrangeTerm{


    //
    // PUBLIC MEMBER FUNCTIONS:
    //
    public:

        /** Default constructor. */
        Objective( );

        /** Default constructor. */
        Objective( const Grid &grid_ );

        /** Copy constructor (deep copy). */
        Objective( const Objective& rhs );

        /** Destructor. */
        virtual ~Objective( );

        /** Assignment operator (deep copy). */
        Objective& operator=( const Objective& rhs );



        /**  Sets the discretization grid.   \n
         *                                   \n
         *   \return SUCCESSFUL_RETURN       \n
         */
        returnValue init( const Grid &grid_ );


// =======================================================================================
//
//                                    LOADING ROUTINES
//
// =======================================================================================


        /** Adds an expression for the Mayer term.
         *  \return SUCCESSFUL_RETURN
         */
        inline returnValue addMayerTerm( const Expression& arg );
        inline returnValue addMayerTerm( const Function& arg );



        /** Adds an Least Square term of the general form                          \n
         *                                                                         \n
         *   0.5* sum_i || h(t_i,x(t_i),u(t_i),p(t_i),...) - r_i ||^2_S_i          \n
         *                                                                         \n
         *  Here, the matrices S_i and the reference vectors r_i should be given   \n
         *  in MatrixVariablesGrid and VariablesGrid format respectively. If S_ is \n
         *  a NULL-pointer the matrices S_i will be unit matrices. If r_ is a      \n
         *  a NULL-pointer the reference will be equal to zero by default.         \n
         *                                                                         \n
         *  \return SUCCESSFUL_RETURN                                              \n
         */
        returnValue addLSQ( const MatrixVariablesGrid *S_,   /**< the weighting matrix  */
                            const Function&            h ,   /**< the LSQ function      */
                            const VariablesGrid       *r_    /**< the reference vectors */ );



        /** Adds an Least Square term that is only evaluated at the end:          \n
         *                                                                        \n
         *        0.5* || m(T,x(T),p,...) - r ||^2_S                              \n
         *                                                                        \n
         *  where  S  is a weighting matrix, r a reference vector and T the time  \n
         *  at the last objective grid point.                                     \n
         *                                                                        \n
         *  \return SUCCESSFUL_RETURN                                             \n
         */
        returnValue addLSQEndTerm( const Matrix   & S,  /**< a weighting matrix */
                                   const Function & m,  /**< the LSQ-Function   */
                                   const Vector   & r   /**< the reference      */ );

        //
        // Code generation related functions
        //

        returnValue addLSQ(const ExportVariable& S, const Function& h);

        returnValue addLSQEndTerm(const ExportVariable& S, const Function& h);

        returnValue addLSQ(const ExportVariable& S, const String& h);

        returnValue addLSQEndTerm(const ExportVariable& S, const String& h);

        returnValue addLSQLinearTerms(const Vector& Slx, const Vector& Slu);

        returnValue addLSQLinearTerms(const ExportVariable& Slx, const ExportVariable& Slu);


// =======================================================================================
//
//                                 INITIALIZATION ROUTINES
//
// =======================================================================================

        /** Initializes the objective and reformulates Lagrange-Terms if         \n
         *  there are any. The RHS function that is passed in the argument       \n
         *  will be augmented by one component if there is a Lagrange term. If   \n
         *  the RHS function is   NULL   but there is Lagrange term then the     \n
         *  routine will allocate memory for fcn and add one component !!        \n
         *                                                                       \n
         *  \param nStages          the number of stages                         \n
         *  \param nTransitions     the number of transitions                    \n
         *  \param fcn              the right-hand side functions                \n
         *  \param transitions      the transition functions                     \n
         *  \param constraint_      the constraint (to be reformulated)          \n
         *                                                                       \n
         *  \return SUCCESSFUL_RETURN                                            \n
         */
        returnValue init( const int              nStages     ,
                          const int              nTransitions,
                          DifferentialEquation **fcn         ,
                          Transition            *transitions ,
                          Constraint            *constraint_   );


// =======================================================================================
//
//                                  DEFINITION OF SEEDS:
//
// =======================================================================================


    /** Define a forward seed in form of a block matrix.   \n
     *                                                     \n
     *  \return SUCCESFUL RETURN                           \n
     *          RET_INPUT_OUT_OF_RANGE                     \n
     */
    virtual returnValue setForwardSeed( BlockMatrix *xSeed_ ,   /**< the seed in x -direction */
                                        BlockMatrix *xaSeed_,   /**< the seed in xa-direction */
                                        BlockMatrix *pSeed_ ,   /**< the seed in p -direction */
                                        BlockMatrix *uSeed_ ,   /**< the seed in u -direction */
                                        BlockMatrix *wSeed_ ,   /**< the seed in w -direction */
                                        int          order      /**< the order of the  seed. */ );



    /**  Define a backward seed in form of a block matrix.  \n
     *                                                      \n
     *   \return SUCCESFUL_RETURN                           \n
     *           RET_INPUT_OUT_OF_RANGE                     \n
     */
    virtual returnValue setBackwardSeed( BlockMatrix *seed,    /**< the seed matrix       */
                                         int          order    /**< the order of the seed.*/  );



    /**  Defines the first order backward seed to be        \n
     *   a unit matrix.                                     \n
     *                                                      \n
     *   \return SUCCESFUL_RETURN                           \n
     *           RET_INPUT_OUT_OF_RANGE                     \n
     */
    virtual returnValue setUnitBackwardSeed( );



// =======================================================================================
//
//                                   EVALUATION ROUTINES
//
// =======================================================================================



        /** Evaluates the objective with all its terms.                 \n
         *                                                              \n
         *  \return SUCCESSFUL_RETURN if the evaluation was successful. \n
         *          of an error message if unsuccessful.                \n
         */
        returnValue evaluate( const OCPiterate &x );



        /** Evaluates the objective gradient. (please use evaluate to specify \n
         *  the evaluation point)                                             \n
         *                                                                    \n
         *  \return SUCCESSFUL_RETURN                                         \n
         */
        returnValue evaluateSensitivities();


        /** Evaluates the objective gradient and the associated Hessian.      \n
         *  (please use evaluate to specify the evaluation point)             \n
         *                                                                    \n
         *  \return SUCCESSFUL_RETURN                                         \n
         */
        returnValue evaluateSensitivities( BlockMatrix &hessian );


        /** Evaluates the objective gradient. (please use evaluate to specify \n
         *  the evaluation point)                                             \n
         *  in addition a Gauss-Newton hessian approximation is provided      \n
         *                                                                    \n
         *  \return SUCCESSFUL_RETURN                                         \n
         *          RET_GAUSS_NEWTON_APPROXIMATION_NOT_SUPPORTED              \n
         */
        returnValue evaluateSensitivitiesGN( BlockMatrix &hessian );




// =======================================================================================
//
//                               RESULTS OF THE EVALUATION
//
// =======================================================================================


    /** Returns the result for the residuum of the bounds.      \n
     *                                                          \n
     *  \return SUCCESSFUL_RETURN                               \n
     */
    virtual returnValue getObjectiveValue( double &objectiveValue );



    /** Returns the result for the forward sensitivities in BlockMatrix form.        \n
     *                                                                               \n
     *  \return SUCCESSFUL_RETURN                                                    \n
     *          RET_INPUT_OUT_OF_RANGE                                               \n
     */
    virtual returnValue getForwardSensitivities( BlockMatrix &D  /**< the result for the
                                                                  *   forward sensitivi-
                                                                  *   ties               */,
                                                 int order       /**< the order          */  );



    /** Returns the result for the backward sensitivities in BlockMatrix form.       \n
     *                                                                               \n
     *  \return SUCCESSFUL_RETURN                                                    \n
     *          RET_INPUT_OUT_OF_RANGE                                               \n
     */
    virtual returnValue getBackwardSensitivities( BlockMatrix &D  /**< the result for the
                                                                   *   forward sensitivi-
                                                                   *   ties               */,
                                                  int order       /**< the order          */  );





// =======================================================================================
//
//                                     DIMENSIONS
//
// =======================================================================================


        /** Returns the number of differential states                 \n
         *  \return The requested number of differential states.      \n
         */
        inline int getNX    () const;

        /** Returns the number of algebraic states                    \n
         *  \return The requested number of algebraic states.         \n
         */
        inline int getNXA   () const;

        /** Returns the number of parameters                          \n
         *  \return The requested number of parameters.               \n
         */
        inline int getNP   () const;

        /** Returns the number of controls                            \n
         *  \return The requested number of controls.                 \n
         */
        inline int getNU   () const;

        /** Returns the number of disturbances                        \n
         *  \return The requested number of disturbances.             \n
         */
        inline int getNW  () const;


// =======================================================================================


        /** Asks the objective whether all terms have Least-Square form. If the   \n
         *  returned answer is "BT_TRUE", the computation of Gauss-Newton hessian \n
         *  approximation is supported.                                           \n
         *                                                                        \n
         *  \return BT_TRUE if all objective terms have LSQ form.                 \n
         */
        inline BooleanType hasLSQform();


        /** returns whether the constraint element is affine. */
        inline BooleanType isAffine();


        /** returns whether the objective is quadratic. */
        inline BooleanType isQuadratic();


        /** returns whether the objective is convex. */
        inline BooleanType isConvex();


        /** overwrites the reference (only for LSQ tracking objectives) \n
         *                                                              \n
         *  \return SUCCESSFUL_RETURN                                   \n
         */
        inline returnValue setReference( const VariablesGrid &ref );


        /** Returns whether or not the objective is empty.    \n
         *                                                    \n
         *  \return BT_TRUE if no objective is specified yet. \n
         *          BT_FALSE otherwise.                       \n
         */
        BooleanType isEmpty() const;

        //
        // Code generation related stuff
        //
        returnValue getLSQTerms( std::vector<ExportVariable>& _matrices, std::vector<Function>& _functions ) const;
        returnValue getLSQEndTerms( std::vector<ExportVariable>& _matrices, std::vector<Function>& _functions ) const;

        returnValue getLSQTerms( std::vector<ExportVariable>& _matrices, std::vector<String>& _functions ) const;
        returnValue getLSQEndTerms( std::vector<ExportVariable>& _matrices, std::vector<String>& _functions ) const;

        returnValue getLSQLinearTerms(std::vector<ExportVariable>& _vSlx, std::vector<ExportVariable>& _vSlu) const;

    //
    // DATA MEMBERS:
    //
    protected:

    LSQTerm    **lsqTerm   ;   /**< The Least Square Terms.     */
    LSQEndTerm **lsqEndTerm;   /**< The Least Square End Terms. */
    MayerTerm  **mayerTerm ;   /**< The Mayer Terms.            */

    uint        nLSQ      ;   /**< number of LSQ terms         */
    uint        nEndLSQ   ;   /**< number of end LSQ terms     */
    uint        nMayer    ;   /**< number of Mayer terms       */

    std::vector<ExportVariable> cgLSQWeightingMatrices;
    std::vector<Function> cgLSQFunctions;

    std::vector<ExportVariable> cgLSQEndTermWeightingMatrices;
    std::vector<Function> cgLSQEndTermFunctions;

    std::vector<ExportVariable> cgExternLSQWeightingMatrices;
    std::vector<String> cgExternLSQFunctions;

    std::vector<ExportVariable> cgExternLSQEndTermWeightingMatrices;
    std::vector<String> cgExternLSQEndTermFunctions;

    std::vector<ExportVariable> cgLSQWeightingVectorsSlx;
    std::vector<ExportVariable> cgLSQWeightingVectorsSlu;
};


CLOSE_NAMESPACE_ACADO



#include <acado/objective/objective.ipp>

#endif  // ACADO_TOOLKIT_OBJECTIVE_HPP

/*
 *    end of file
 */
