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
 *    \file src/constraint/constraint.cpp
 *    \author Boris Houska, Hans Joachim Ferreau
 *
 */


#include <acado/function/function.hpp>
#include <acado/constraint/constraint.hpp>



BEGIN_NAMESPACE_ACADO



//
// PUBLIC MEMBER FUNCTIONS:
//


Constraint::Constraint( )
           :BoxConstraint(){

     boundary_constraint              = 0;
     coupled_path_constraint          = 0;
     path_constraint                  = 0;
     algebraic_consistency_constraint = 0;
     point_constraints                = 0;
}



Constraint::Constraint( const Constraint& rhs )
           :BoxConstraint(rhs){

    uint run1;

    if( rhs.boundary_constraint != 0 )
            boundary_constraint = new BoundaryConstraint(*rhs.boundary_constraint);
    else    boundary_constraint = 0                                               ;

    if( rhs.coupled_path_constraint != 0 )
            coupled_path_constraint = new CoupledPathConstraint(*rhs.coupled_path_constraint);
    else    coupled_path_constraint = 0                                                      ;

    if( rhs.path_constraint != 0 )
            path_constraint = new PathConstraint(*rhs.path_constraint);
    else    path_constraint = 0                                       ;

    if( rhs.algebraic_consistency_constraint != 0 )
            algebraic_consistency_constraint = new AlgebraicConsistencyConstraint(*rhs.algebraic_consistency_constraint);
    else    algebraic_consistency_constraint = 0;

    if( rhs.point_constraints != 0 ){
        point_constraints = new PointConstraint*[grid.getNumPoints()];
        for( run1 = 0; run1 < grid.getNumPoints(); run1++ ){
            if( rhs.point_constraints[run1] != 0 ){
                    point_constraints[run1] = new PointConstraint(*rhs.point_constraints[run1]);
            }
            else    point_constraints[run1] = 0;
        }
    }
    else    point_constraints = 0;
}


Constraint::~Constraint( ){

    deleteAll();
}


void Constraint::deleteAll(){

   uint run1;

    if( boundary_constraint != 0 )
        delete boundary_constraint;

    if( coupled_path_constraint != 0 )
        delete coupled_path_constraint;

    if( path_constraint != 0 )
        delete path_constraint;

    if( algebraic_consistency_constraint != 0 )
        delete algebraic_consistency_constraint;

    if( point_constraints != 0 ){

        for( run1 = 0; run1 < grid.getNumPoints(); run1++ )
            if( point_constraints[run1] != 0 )
                delete point_constraints[run1];

        delete[] point_constraints;
    }
}



returnValue Constraint::init( const Grid& grid_, const int& numberOfStages_ ){

     deleteAll();

     BoxConstraint::init( grid_ );

     uint run1;

     boundary_constraint               = new BoundaryConstraint            (grid                );
     coupled_path_constraint           = new CoupledPathConstraint         (grid                );
     path_constraint                   = new PathConstraint                (grid                );
     algebraic_consistency_constraint  = new AlgebraicConsistencyConstraint(grid,numberOfStages_);
     point_constraints                 = new PointConstraint*[grid.getNumPoints()];

     for( run1 = 0; run1 < grid.getNumPoints(); run1++ )
         point_constraints[run1] = 0;

    return SUCCESSFUL_RETURN;
}



Constraint& Constraint::operator=( const Constraint& rhs ){

    uint run1;

    if( this != &rhs ){

        deleteAll();

        BoxConstraint::operator=(rhs);

        if( rhs.boundary_constraint != 0 )
                boundary_constraint = new BoundaryConstraint(*rhs.boundary_constraint);
        else    boundary_constraint = 0                                               ;

        if( rhs.coupled_path_constraint != 0 )
                coupled_path_constraint = new CoupledPathConstraint(*rhs.coupled_path_constraint);
        else    coupled_path_constraint = 0                                                      ;

        if( rhs.path_constraint != 0 )
                path_constraint = new PathConstraint(*rhs.path_constraint);
        else    path_constraint = 0                                       ;

        if( rhs.algebraic_consistency_constraint != 0 )
                algebraic_consistency_constraint = new AlgebraicConsistencyConstraint(*rhs.algebraic_consistency_constraint);
        else    algebraic_consistency_constraint = 0;

        if( rhs.point_constraints != 0 ){
            point_constraints = new PointConstraint*[grid.getNumPoints()];
            for( run1 = 0; run1 < grid.getNumPoints(); run1++ ){
                if( rhs.point_constraints[run1] != 0 ){
                        point_constraints[run1] = new PointConstraint(*rhs.point_constraints[run1]);
                }
                else    point_constraints[run1] = 0;
            }
        }
        else    point_constraints = 0;

    }
    return *this;
}



returnValue Constraint::add( const double lb_, const Expression& arg, const double ub_  ){

    DVector tmp_lb(grid.getNumPoints());
    DVector tmp_ub(grid.getNumPoints());

    tmp_lb.setAll(lb_);
    tmp_ub.setAll(ub_);

    return add( tmp_lb, arg, tmp_ub );
}



returnValue Constraint::add( const DVector lb_, const Expression& arg, const double ub_  ){

    DVector tmp_ub(grid.getNumPoints());
    tmp_ub.setAll(ub_);

    return add( lb_, arg, tmp_ub );
}


returnValue Constraint::add( const double lb_, const Expression& arg, const DVector ub_  ){

    DVector tmp_lb(grid.getNumPoints());
    tmp_lb.setAll(lb_);

    return add( tmp_lb, arg, ub_ );
}


returnValue Constraint::add( const DVector lb_, const Expression &arg, const DVector ub_  ){

    // CHECK FEASIBILITY:
    // ------------------
    if( lb_.getDim() != ub_.getDim()        )  return ACADOERROR(RET_INFEASIBLE_CONSTRAINT);
    if( lb_.getDim() != grid.getNumPoints() )  return ACADOERROR(RET_INFEASIBLE_CONSTRAINT);
    if( (lb_ <= (const DVector&)ub_) == BT_FALSE            )  return ACADOERROR(RET_INFEASIBLE_CONSTRAINT);


    // CHECK FOR A BOUND:
    // -----------------------
    VariableType varType   = arg.getVariableType();
    int          component = arg.getComponent(0)  ;

    if( arg.isVariable( ) == BT_TRUE ){
        if( varType != VT_INTERMEDIATE_STATE ){

             nb++;
             var   = (VariableType*)realloc(var  , nb*sizeof(VariableType));
             index = (int*         )realloc(index, nb*sizeof(int         ));
             blb   = (DVector**     )realloc(blb  , nb*sizeof(DVector*     ));
             bub   = (DVector**     )realloc(bub  , nb*sizeof(DVector*     ));

             var  [nb-1] = varType  ;
             index[nb-1] = component;
             blb  [nb-1] = new DVector( lb_ );
             bub  [nb-1] = new DVector( ub_ );

             return SUCCESSFUL_RETURN;
       }
    }


    // SAVE THE ARGUMENT AS A Path Constraint:
    // ---------------------------------------
    return path_constraint->add( lb_, arg, ub_ );
}


returnValue Constraint::add( const double lb_, const Expression *arguments, const double ub_ ){

    // CHECK FEASIBILITY:
    // ------------------
    if( lb_ > ub_ + EPS )  return ACADOERROR(RET_INFEASIBLE_CONSTRAINT);

    return coupled_path_constraint->add( lb_, arguments, ub_ );
}


returnValue Constraint::add( const uint&                 endOfStage_ ,
                                    const DifferentialEquation& dae           ){

    return algebraic_consistency_constraint->add( endOfStage_, dae );
}


returnValue Constraint::add( const ConstraintComponent& component ){


    DVector tmp_ub(grid.getNumPoints());
    DVector tmp_lb(grid.getNumPoints());

    uint run1;

    if( component.hasLBgrid() == 0 ){

        for( run1 = 0; run1 < grid.getNumPoints(); run1++ ){
            if( (component.getLB()).getDim() == 1 )
                tmp_lb(run1) = (component.getLB()).operator()(0);
            else{
                if( (component.getLB()).getDim() <= run1 )
                    return ACADOWARNING(RET_INFEASIBLE_CONSTRAINT);
                tmp_lb(run1) = (component.getLB()).operator()(run1);
            }
        }
    }
    else{

        VariablesGrid LBgrid = component.getLBgrid();

        for( run1 = 0; run1 < grid.getNumPoints(); run1++ ){
            DVector tmp = LBgrid.linearInterpolation( grid.getTime(run1) );
            tmp_lb(run1) = tmp(0);
        }
    }


    if( component.hasUBgrid() == 0 ){
        for( run1 = 0; run1 < grid.getNumPoints(); run1++ ){
            if( (component.getUB()).getDim() == 1 )
                tmp_ub(run1) = (component.getUB()).operator()(0);
            else{
                if( (component.getUB()).getDim() <= run1 )
                    return ACADOWARNING(RET_INFEASIBLE_CONSTRAINT);
                tmp_ub(run1) = (component.getUB()).operator()(run1);
            }
        }
    }
    else{

        VariablesGrid UBgrid = component.getUBgrid();

        for( run1 = 0; run1 < grid.getNumPoints(); run1++ ){
            DVector tmp = UBgrid.linearInterpolation( grid.getTime(run1) );
            tmp_ub(run1) = tmp(0);
        }
    }

    return add( tmp_lb, component.getExpression(), tmp_ub );
}


returnValue Constraint::add( const int index_, const ConstraintComponent& component ){

    DVector tmp_ub(grid.getNumPoints());
    DVector tmp_lb(grid.getNumPoints());

    if ( !(index_ < (int) grid.getNumPoints()) )
    	return ACADOERRORTEXT(RET_ASSERTION,
    			"The constraint component can not be set as the associated "
    			"discretization point is not in the time horizon.");

    uint run1;

    if( component.hasLBgrid() == 0 ){

        for( run1 = 0; run1 < grid.getNumPoints(); run1++ ){
            if( (component.getLB()).getDim() == 1 )
                tmp_lb(run1) = (component.getLB()).operator()(0);
            else{
                if( (component.getLB()).getDim() <= run1 )
                    return ACADOWARNING(RET_INFEASIBLE_CONSTRAINT);
                tmp_lb(run1) = (component.getLB()).operator()(run1);
            }
        }
    }
    else{

        VariablesGrid LBgrid = component.getLBgrid();

        for( run1 = 0; run1 < grid.getNumPoints(); run1++ ){
            DVector tmp = LBgrid.linearInterpolation( grid.getTime(run1) );
            tmp_lb(run1) = tmp(0);
        }
    }


    if( component.hasUBgrid() == 0 ){
        for( run1 = 0; run1 < grid.getNumPoints(); run1++ ){
            if( (component.getUB()).getDim() == 1 )
                tmp_ub(run1) = (component.getUB()).operator()(0);
            else{
                if( (component.getUB()).getDim() <= run1 )
                    return ACADOWARNING(RET_INFEASIBLE_CONSTRAINT);
                tmp_ub(run1) = (component.getUB()).operator()(run1);
            }
        }
    }
    else{

        VariablesGrid UBgrid = component.getUBgrid();

        for( run1 = 0; run1 < grid.getNumPoints(); run1++ ){
            DVector tmp = UBgrid.linearInterpolation( grid.getTime(run1) );
            tmp_ub(run1) = tmp(0);
        }
    }

    ACADO_TRY( add( index_, tmp_lb(index_), component.getExpression(), tmp_ub(index_) ) );

    return SUCCESSFUL_RETURN;
}


returnValue Constraint::evaluate( const OCPiterate& iter ){


    if( grid.getNumPoints() == 0 ) return ACADOERROR(RET_MEMBER_NOT_INITIALISED);

    uint run1;
    returnValue returnvalue;


    // EVALUATE BOUNDARY CONSTRAINS:
    // -----------------------------

    if( boundary_constraint->getNC() != 0 ){
        returnvalue = boundary_constraint->init( iter );
        if( returnvalue != SUCCESSFUL_RETURN ) return ACADOERROR(returnvalue);
        returnvalue = boundary_constraint->evaluate( iter );
        if( returnvalue != SUCCESSFUL_RETURN ) return ACADOERROR(returnvalue);
    }


    // EVALUATE COUPLED PATH CONSTRAINS:
    // ---------------------------------

    if( coupled_path_constraint->getNC() != 0 ){
        returnvalue = coupled_path_constraint->init( iter );
        if( returnvalue != SUCCESSFUL_RETURN ) return ACADOERROR(returnvalue);
        returnvalue = coupled_path_constraint->evaluate( iter );
        if( returnvalue != SUCCESSFUL_RETURN ) return ACADOERROR(returnvalue);
    }


    // EVALUATE PATH CONSTRAINS:
    // -------------------------

    if( path_constraint->getNC() != 0 ){
        returnvalue = path_constraint->init( iter );
        if( returnvalue != SUCCESSFUL_RETURN ) return ACADOERROR(returnvalue);
        returnvalue = path_constraint->evaluate( iter );
        if( returnvalue != SUCCESSFUL_RETURN ) return ACADOERROR(returnvalue);
    }


    // EVALUATE ALGEBRAIC CONSISTENCY CONSTRAINS:
    // ------------------------------------------

    if( algebraic_consistency_constraint->getNC() != 0 ){
        returnvalue = algebraic_consistency_constraint->init( iter );
        if( returnvalue != SUCCESSFUL_RETURN ) return ACADOERROR(returnvalue);
        returnvalue = algebraic_consistency_constraint->evaluate( iter );
        if( returnvalue != SUCCESSFUL_RETURN ) return ACADOERROR(returnvalue);
    }


    // EVALUATE POINT CONSTRAINS:
    // --------------------------

    if( point_constraints != 0 ){
        for( run1 = 0; run1 < grid.getNumPoints(); run1++ ){
            if( point_constraints[run1] != 0 ){
                returnvalue = point_constraints[run1]->init( iter );
                if( returnvalue != SUCCESSFUL_RETURN ) return ACADOERROR(returnvalue);
                returnvalue = point_constraints[run1]->evaluate( iter );
                if( returnvalue != SUCCESSFUL_RETURN ) return ACADOERROR(returnvalue);
            }
        }
    }


    // EVALUATE BOUNDS:
    // ----------------

    return evaluateBounds( iter );
}



returnValue Constraint::evaluateSensitivities( ){

    uint run1;
    returnValue returnvalue;


    // EVALUATE BOUNDARY CONSTRAINS:
    // -----------------------------

    if( boundary_constraint->getNC() != 0 ){
        returnvalue = boundary_constraint->evaluateSensitivities( );
        if( returnvalue != SUCCESSFUL_RETURN ) return ACADOERROR(returnvalue);
    }


    // EVALUATE COUPLED PATH CONSTRAINS:
    // ---------------------------------

    if( coupled_path_constraint->getNC() != 0 ){
        returnvalue = coupled_path_constraint->evaluateSensitivities( );
        if( returnvalue != SUCCESSFUL_RETURN ) return ACADOERROR(returnvalue);
    }


    // EVALUATE PATH CONSTRAINS:
    // -------------------------

    if( path_constraint->getNC() != 0 ){
        returnvalue = path_constraint->evaluateSensitivities( );
        if( returnvalue != SUCCESSFUL_RETURN ) return ACADOERROR(returnvalue);
    }


    // EVALUATE ALGEBRAIC CONSISTENCY CONSTRAINS:
    // ------------------------------------------

    if( algebraic_consistency_constraint->getNC() != 0 ){
        returnvalue = algebraic_consistency_constraint->evaluateSensitivities( );
        if( returnvalue != SUCCESSFUL_RETURN ) return ACADOERROR(returnvalue);
    }

    // EVALUATE POINT CONSTRAINS:
    // --------------------------

    if( point_constraints != 0 ){
        for( run1 = 0; run1 < grid.getNumPoints(); run1++ ){
            if( point_constraints[run1] != 0 ){
                returnvalue = point_constraints[run1]->evaluateSensitivities( );
                if( returnvalue != SUCCESSFUL_RETURN ) return ACADOERROR(returnvalue);
            }
        }
    }

    return SUCCESSFUL_RETURN;
}


returnValue Constraint::evaluateSensitivities( const BlockMatrix &seed, BlockMatrix &hessian ){


    uint run1 ;
    int  count;
    returnValue returnvalue;

    count = 0;
    DMatrix tmp;

    // EVALUATE BOUNDARY CONSTRAINS:
    // -----------------------------

    if( boundary_constraint->getNC() != 0 ){
        seed.getSubBlock( count, 0, tmp, boundary_constraint->getNC(), 1 );
        returnvalue = boundary_constraint->evaluateSensitivities( tmp, hessian );
        if( returnvalue != SUCCESSFUL_RETURN ) return ACADOERROR(returnvalue);
        count++;
    }


    // EVALUATE COUPLED PATH CONSTRAINS:
    // ---------------------------------

    if( coupled_path_constraint->getNC() != 0 ){
        seed.getSubBlock( count, 0, tmp, coupled_path_constraint->getNC(), 1 );
        returnvalue = coupled_path_constraint->evaluateSensitivities( tmp, hessian );
        if( returnvalue != SUCCESSFUL_RETURN ) return ACADOERROR(returnvalue);
        count++;
    }


    // EVALUATE PATH CONSTRAINS:
    // -------------------------

    if( path_constraint->getNC() != 0 ){
        returnvalue = path_constraint->evaluateSensitivities( count, seed, hessian );
        if( returnvalue != SUCCESSFUL_RETURN ) return ACADOERROR(returnvalue);
    }


    // EVALUATE ALGEBRAIC CONSISTENCY CONSTRAINS:
    // ------------------------------------------

    if( algebraic_consistency_constraint->getNC() != 0 ){
        returnvalue = algebraic_consistency_constraint->evaluateSensitivities( count, seed, hessian );
        if( returnvalue != SUCCESSFUL_RETURN ) return ACADOERROR(returnvalue);
    }

    // EVALUATE POINT CONSTRAINS:
    // --------------------------

    if( point_constraints != 0 ){
        for( run1 = 0; run1 < grid.getNumPoints(); run1++ ){
            if( point_constraints[run1] != 0 ){
                seed.getSubBlock( count, 0, tmp, point_constraints[run1]->getNC(), 1 );
                returnvalue = point_constraints[run1]->evaluateSensitivities( tmp, hessian );
                if( returnvalue != SUCCESSFUL_RETURN ) return ACADOERROR(returnvalue);
                count++;
            }
        }
    }
    return SUCCESSFUL_RETURN;
}



returnValue Constraint::setForwardSeed( BlockMatrix *xSeed_ ,
                                        BlockMatrix *xaSeed_,
                                        BlockMatrix *pSeed_ ,
                                        BlockMatrix *uSeed_ ,
                                        BlockMatrix *wSeed_ ,
                                        int          order    ){

    uint run1;
    returnValue returnvalue;


    // BOUNDARY CONSTRAINTS:
    // ---------------------

    if( boundary_constraint->getNC() != 0 ){
        returnvalue = boundary_constraint->setForwardSeed( xSeed_, xaSeed_, pSeed_, uSeed_, wSeed_, order );
        if( returnvalue != SUCCESSFUL_RETURN ) return ACADOERROR(returnvalue);
    }


    // COUPLED PATH CONSTRAINTS:
    // -------------------------

    if( coupled_path_constraint->getNC() != 0 ){
        returnvalue = coupled_path_constraint->setForwardSeed( xSeed_, xaSeed_, pSeed_, uSeed_, wSeed_, order );
        if( returnvalue != SUCCESSFUL_RETURN ) return ACADOERROR(returnvalue);
    }


    // PATH CONSTRAINTS:
    // -----------------

    if( path_constraint->getNC() != 0 ){
        returnvalue = path_constraint->setForwardSeed( xSeed_, xaSeed_, pSeed_, uSeed_, wSeed_, order );
        if( returnvalue != SUCCESSFUL_RETURN ) return ACADOERROR(returnvalue);
    }


    // ALGEBRAIC CONSISTENCY CONSTRAINTS:
    // ----------------------------------

    if( algebraic_consistency_constraint->getNC() != 0 ){
        returnvalue = algebraic_consistency_constraint->setForwardSeed( xSeed_, xaSeed_, pSeed_, uSeed_, wSeed_, order );
        if( returnvalue != SUCCESSFUL_RETURN ) return ACADOERROR(returnvalue);
    }


    // POINT CONSTRAINTS:
    // ------------------

    if( point_constraints != 0 ){
        for( run1 = 0; run1 < grid.getNumPoints(); run1++ ){
            if( point_constraints[run1] != 0 ){
                returnvalue = point_constraints[run1]->setForwardSeed( xSeed_, xaSeed_, pSeed_, uSeed_, wSeed_, order );
                if( returnvalue != SUCCESSFUL_RETURN ) return ACADOERROR(returnvalue);
            }
        }
    }

    return SUCCESSFUL_RETURN;
}


returnValue Constraint::setUnitForwardSeed( ){

    uint run1;
    returnValue returnvalue;


    // BOUNDARY CONSTRAINTS:
    // ---------------------

    if( boundary_constraint->getNC() != 0 ){
        returnvalue = boundary_constraint->setUnitForwardSeed();
        if( returnvalue != SUCCESSFUL_RETURN ) return ACADOERROR(returnvalue);
    }


    // COUPLED PATH CONSTRAINTS:
    // -------------------------

    if( coupled_path_constraint->getNC() != 0 ){
        returnvalue = coupled_path_constraint->setUnitForwardSeed();
        if( returnvalue != SUCCESSFUL_RETURN ) return ACADOERROR(returnvalue);
    }


    // PATH CONSTRAINTS:
    // -----------------

    if( path_constraint->getNC() != 0 ){
        returnvalue = path_constraint->setUnitForwardSeed();
        if( returnvalue != SUCCESSFUL_RETURN ) return ACADOERROR(returnvalue);
    }

    // ALGEBRAIC CONSISTENCY CONSTRAINTS:
    // ----------------------------------

    if( algebraic_consistency_constraint->getNC() != 0 ){
        returnvalue = algebraic_consistency_constraint->setUnitForwardSeed();
        if( returnvalue != SUCCESSFUL_RETURN ) return ACADOERROR(returnvalue);
    }


    // POINT CONSTRAINTS:
    // ------------------

    if( point_constraints != 0 ){
        for( run1 = 0; run1 < grid.getNumPoints(); run1++ ){
            if( point_constraints[run1] != 0 ){
                returnvalue = point_constraints[run1]->setUnitForwardSeed();
                if( returnvalue != SUCCESSFUL_RETURN ) return ACADOERROR(returnvalue);
            }
        }
    }

    return SUCCESSFUL_RETURN;
}


returnValue Constraint::setBackwardSeed( BlockMatrix *seed, int order ){

    return SUCCESSFUL_RETURN;
}


returnValue Constraint::setUnitBackwardSeed( ){


    uint run1;
    returnValue returnvalue;

    const uint N = grid.getNumPoints();

    // BOUNDARY CONSTRAINTS:
    // ---------------------

    if( boundary_constraint->getNC() != 0 ){
        BlockMatrix seed( 1, 1 );
        seed.setIdentity( 0, 0, boundary_constraint->getNC() );
        returnvalue = boundary_constraint->setBackwardSeed( &seed, 1 );
        if( returnvalue != SUCCESSFUL_RETURN ) return ACADOERROR(returnvalue);
    }


    // COUPLED PATH CONSTRAINTS:
    // -------------------------

    if( coupled_path_constraint->getNC() != 0 ){
        BlockMatrix seed( 1, 1 );
        seed.setIdentity( 0, 0, coupled_path_constraint->getNC() );
        returnvalue = coupled_path_constraint->setBackwardSeed( &seed, 1 );
        if( returnvalue != SUCCESSFUL_RETURN ) return ACADOERROR(returnvalue);
    }


    // PATH CONSTRAINTS:
    // -----------------

    if( path_constraint->getNC() != 0 ){
        BlockMatrix seed( 1, N );
        for( run1 = 0; run1 < N; run1++ )
            seed.setIdentity( 0, run1, path_constraint->getDim(run1) );
        returnvalue = path_constraint->setBackwardSeed( &seed, 1 );
        if( returnvalue != SUCCESSFUL_RETURN ) return ACADOERROR(returnvalue);
    }

    // ALGEBRAIC CONSISTENCY CONSTRAINTS:
    // ----------------------------------

    if( algebraic_consistency_constraint->getNC() != 0 ){
        BlockMatrix seed( 1, N );
        for( run1 = 0; run1 < N; run1++ )
            seed.setIdentity( 0, run1, algebraic_consistency_constraint->getDim(run1) );
        returnvalue = algebraic_consistency_constraint->setBackwardSeed( &seed, 1 );
        if( returnvalue != SUCCESSFUL_RETURN ) return ACADOERROR(returnvalue);
    }


    // POINT CONSTRAINTS:
    // ------------------

    if( point_constraints != 0 ){
        for( run1 = 0; run1 < grid.getNumPoints(); run1++ ){
            if( point_constraints[run1] != 0 ){
                BlockMatrix seed( 1, 1 );
                seed.setIdentity( 0, 0, point_constraints[run1]->getNC() );
                returnvalue = point_constraints[run1]->setBackwardSeed( &seed, 1 );
                if( returnvalue != SUCCESSFUL_RETURN ) return ACADOERROR(returnvalue);
            }
        }
    }

    return SUCCESSFUL_RETURN;
}



returnValue Constraint::getConstraintResiduum( BlockMatrix &lowerRes, BlockMatrix &upperRes ){

    const int N = grid.getNumPoints();

    returnValue returnvalue;

    BlockMatrix residuumL;
    BlockMatrix residuumU;

    residuumL.init( getNumberOfBlocks(), 1 );
    residuumU.init( getNumberOfBlocks(), 1 );

    int nc, run1;

    nc = 0;

    // BOUNDARY CONSTRAINTS:
    // ---------------------

    if( boundary_constraint->getNC() != 0 ){
        BlockMatrix resL, resU;
        returnvalue = boundary_constraint->getResiduum( resL, resU );
        if( returnvalue != SUCCESSFUL_RETURN ) return ACADOERROR(returnvalue);
        DMatrix resL_;
        DMatrix resU_;
        resL.getSubBlock( 0, 0, resL_ );
        resU.getSubBlock( 0, 0, resU_ );
        residuumL.setDense( nc, 0, resL_ );
        residuumU.setDense( nc, 0, resU_ );
        nc++;
    }


    // COUPLED PATH CONSTRAINTS:
    // -------------------------

    if( coupled_path_constraint->getNC() != 0 ){
        BlockMatrix resL, resU;
        returnvalue = coupled_path_constraint->getResiduum( resL, resU );
        if( returnvalue != SUCCESSFUL_RETURN ) return ACADOERROR(returnvalue);
        DMatrix resL_;
        DMatrix resU_;
        resL.getSubBlock( 0, 0, resL_ );
        resU.getSubBlock( 0, 0, resU_ );
        residuumL.setDense( nc, 0, resL_ );
        residuumU.setDense( nc, 0, resU_ );
        nc++;
    }


    // PATH CONSTRAINTS:
    // -----------------

    if( path_constraint->getNC() != 0 ){
        BlockMatrix resL, resU;
        returnvalue = path_constraint->getResiduum( resL, resU );
        if( returnvalue != SUCCESSFUL_RETURN ) return ACADOERROR(returnvalue);
        DMatrix resL_;
        DMatrix resU_;
        for( run1 = 0; run1 < N; run1++ ){
            resL.getSubBlock( run1, 0, resL_ );
            resU.getSubBlock( run1, 0, resU_ );
            residuumL.setDense( nc, 0, resL_ );
            residuumU.setDense( nc, 0, resU_ );
            nc++;
        }
    }

    // ALGEBRAIC CONSISTENCY CONSTRAINTS:
    // ----------------------------------

    if( algebraic_consistency_constraint->getNC() != 0 ){
        BlockMatrix resL, resU;
        returnvalue = algebraic_consistency_constraint->getResiduum( resL, resU );
        if( returnvalue != SUCCESSFUL_RETURN ) return ACADOERROR(returnvalue);
        DMatrix resL_;
        DMatrix resU_;
        for( run1 = 0; run1 < N; run1++ ){
            resL.getSubBlock( run1, 0, resL_ );
            resU.getSubBlock( run1, 0, resU_ );
            residuumL.setDense( nc, 0, resL_ );
            residuumU.setDense( nc, 0, resU_ );
            nc++;
        }
    }

    // POINT CONSTRAINTS:
    // ------------------

    if( point_constraints != 0 ){
        for( run1 = 0; run1 < (int) grid.getNumPoints(); run1++ ){
            if( point_constraints[run1] != 0 ){
                BlockMatrix resL, resU;
                returnvalue = point_constraints[run1]->getResiduum( resL, resU );
                if( returnvalue != SUCCESSFUL_RETURN ) return ACADOERROR(returnvalue);
                DMatrix resL_;
                DMatrix resU_;
                resL.getSubBlock( 0, 0, resL_ );
                resU.getSubBlock( 0, 0, resU_ );
                residuumL.setDense( nc, 0, resL_ );
                residuumU.setDense( nc, 0, resU_ );
                nc++;
            }
        }
    }

    lowerRes = residuumL;
    upperRes = residuumU;

    return SUCCESSFUL_RETURN;
}


returnValue Constraint::getBoundResiduum( BlockMatrix &lowerRes,
                                          BlockMatrix &upperRes ){


    int run1;
    const int N = grid.getNumPoints();

    lowerRes.init( 4*N+1, 1 );
    upperRes.init( 4*N+1, 1 );

    for( run1 = 0; run1 < N; run1++ ){

        lowerRes.setDense( run1, 0, residuumXL[run1] );
        upperRes.setDense( run1, 0, residuumXU[run1] );

        lowerRes.setDense( N+run1, 0, residuumXAL[run1] );
        upperRes.setDense( N+run1, 0, residuumXAU[run1] );

        lowerRes.setDense( 2*N+1+run1, 0, residuumUL[run1] );
        upperRes.setDense( 2*N+1+run1, 0, residuumUU[run1] );

        lowerRes.setDense( 3*N+1+run1, 0, residuumWL[run1] );
        upperRes.setDense( 3*N+1+run1, 0, residuumWU[run1] );
    }
    lowerRes.setDense( 2*N, 0, residuumPL[0] );
    upperRes.setDense( 2*N, 0, residuumPU[0] );

    return SUCCESSFUL_RETURN;
}


returnValue Constraint::getForwardSensitivities( BlockMatrix &D, int order ){

    const int N = grid.getNumPoints();

    returnValue returnvalue;
    BlockMatrix result;

    result.init( getNumberOfBlocks(), 5*N );

    int nc, run1, run2;
    nc = 0;

    // BOUNDARY CONSTRAINTS:
    // ---------------------

    if( boundary_constraint->getNC() != 0 ){
        BlockMatrix res;
        returnvalue = boundary_constraint->getForwardSensitivities( &res, order );
        if( returnvalue != SUCCESSFUL_RETURN ) return ACADOERROR(returnvalue);
        DMatrix res_;
        for( run2 = 0; run2 < 5*N; run2++ ){
            res.getSubBlock( 0 , run2, res_ );
            if( res_.getDim() > 0 )
                result.setDense( nc, run2, res_ );
        }
        nc++;
    }

    // COUPLED PATH CONSTRAINTS:
    // -------------------------

    if( coupled_path_constraint->getNC() != 0 ){
        BlockMatrix res;
        returnvalue = coupled_path_constraint->getForwardSensitivities( &res, order );
        if( returnvalue != SUCCESSFUL_RETURN ) return ACADOERROR(returnvalue);
        DMatrix res_;
        for( run2 = 0; run2 < 5*N; run2++ ){
            res.getSubBlock( 0 , run2, res_ );
            if( res_.getDim() > 0 )
                result.setDense( nc, run2, res_ );
        }
        nc++;
    }


    // PATH CONSTRAINTS:
    // -----------------

    if( path_constraint->getNC() != 0 ){
        BlockMatrix res;
        returnvalue = path_constraint->getForwardSensitivities( &res, order );
        if( returnvalue != SUCCESSFUL_RETURN ) return ACADOERROR(returnvalue);
        DMatrix res_;
        for( run1 = 0; run1 < N; run1++ ){
            for( run2 = 0; run2 < 5*N; run2++ ){
                res.getSubBlock( run1, run2, res_ );
                if( res_.getDim() > 0 )
                    result.setDense( nc  , run2, res_ );
            }
            nc++;
        }
    }


    // ALGEBRAIC CONSISTENCY CONSTRAINTS:
    // ----------------------------------

    if( algebraic_consistency_constraint->getNC() != 0 ){
        BlockMatrix res;
        returnvalue = algebraic_consistency_constraint->getForwardSensitivities( &res, order );
        if( returnvalue != SUCCESSFUL_RETURN ) return ACADOERROR(returnvalue);
        DMatrix res_;
        for( run1 = 0; run1 < N; run1++ ){
            for( run2 = 0; run2 < 5*N; run2++ ){
                res.getSubBlock( run1, run2, res_ );
                if( res_.getDim() > 0 )
                    result.setDense( nc  , run2, res_ );
            }
            nc++;
        }
    }



    // POINT CONSTRAINTS:
    // ------------------

    if( point_constraints != 0 ){
        for( run1 = 0; run1 < (int) grid.getNumPoints(); run1++ ){
            if( point_constraints[run1] != 0 ){
                BlockMatrix res;
                returnvalue = point_constraints[run1]->getForwardSensitivities( &res, order );
                if( returnvalue != SUCCESSFUL_RETURN ) return ACADOERROR(returnvalue);
                DMatrix res_;
                for( run2 = 0; run2 < 5*N; run2++ ){
                    res.getSubBlock( 0 , run2, res_ );
                    if( res_.getDim() > 0 )
                        result.setDense( nc, run2, res_ );
                }
                nc++;
            }
        }
    }

    D = result;

    return SUCCESSFUL_RETURN;
}


returnValue Constraint::getBackwardSensitivities( BlockMatrix &D, int order ){

    const int N = grid.getNumPoints();

    returnValue returnvalue;
    BlockMatrix result;

    result.init( getNumberOfBlocks(), 5*N );

    int nc, run1, run2;
    nc = 0;


    // BOUNDARY CONSTRAINTS:
    // ---------------------

    if( boundary_constraint->getNC() != 0 ){
        BlockMatrix res;
        returnvalue = boundary_constraint->getBackwardSensitivities( &res, order );
        if( returnvalue != SUCCESSFUL_RETURN ) return ACADOERROR(returnvalue);
        DMatrix res_;
        for( run2 = 0; run2 < 5*N; run2++ ){
            res.getSubBlock( 0 , run2, res_ );
            if( res_.getDim() > 0 )
                result.setDense( nc, run2, res_ );
        }
        nc++;
    }


    // COUPLED PATH CONSTRAINTS:
    // -------------------------

    if( coupled_path_constraint->getNC() != 0 ){
        BlockMatrix res;
        returnvalue = coupled_path_constraint->getBackwardSensitivities( &res, order );
        if( returnvalue != SUCCESSFUL_RETURN ) return ACADOERROR(returnvalue);
        DMatrix res_;
        for( run2 = 0; run2 < 5*N; run2++ ){
            res.getSubBlock( 0 , run2, res_ );
            if( res_.getDim() > 0 )
                result.setDense( nc, run2, res_ );
        }
        nc++;
    }


    // PATH CONSTRAINTS:
    // -----------------

    if( path_constraint->getNC() != 0 ){
        BlockMatrix res;
        returnvalue = path_constraint->getBackwardSensitivities( &res, order );
        if( returnvalue != SUCCESSFUL_RETURN ) return ACADOERROR(returnvalue);
        DMatrix res_;
        for( run1 = 0; run1 < N; run1++ ){
            for( run2 = 0; run2 < 5*N; run2++ ){
                res.getSubBlock( run1, run2, res_ );
                if( res_.getDim() > 0 )
                    result.setDense( nc  , run2, res_ );
            }
            nc++;
        }
    }


    // ALGEBRAIC CONSISTENCY CONSTRAINTS:
    // ----------------------------------

    if( algebraic_consistency_constraint->getNC() != 0 ){
        BlockMatrix res;
        returnvalue = algebraic_consistency_constraint->getBackwardSensitivities( &res, order );
        if( returnvalue != SUCCESSFUL_RETURN ) return ACADOERROR(returnvalue);
        DMatrix res_;
        for( run1 = 0; run1 < N; run1++ ){
            for( run2 = 0; run2 < 5*N; run2++ ){
                res.getSubBlock( run1, run2, res_ );
                if( res_.getDim() > 0 )
                    result.setDense( nc  , run2, res_ );
            }
            nc++;
        }
    }



    // POINT CONSTRAINTS:
    // ------------------

    if( point_constraints != 0 ){
        for( run1 = 0; run1 < (int) grid.getNumPoints(); run1++ ){
            if( point_constraints[run1] != 0 ){
                BlockMatrix res;
                returnvalue = point_constraints[run1]->getBackwardSensitivities( &res, order );
                if( returnvalue != SUCCESSFUL_RETURN ) return ACADOERROR(returnvalue);
                DMatrix res_;
                for( run2 = 0; run2 < 5*N; run2++ ){
                    res.getSubBlock( 0 , run2, res_ );
                    if( res_.getDim() > 0 )
                        result.setDense( nc, run2, res_ );
                }
                nc++;
            }
        }
    }

    D = result;

    return SUCCESSFUL_RETURN;
}



BooleanType Constraint::isEmpty() const{

    if( nb                               == 0 &&
        boundary_constraint              == 0 &&
        coupled_path_constraint          == 0 &&
        path_constraint                  == 0 &&
        algebraic_consistency_constraint == 0 &&
        point_constraints                == 0    ) return BT_TRUE;

    return BT_FALSE;
}



//
// PROTECTED MEMBER FUNCTIONS:
//

returnValue Constraint::add( const int index_, const double lb_,
                                    const Expression &arg, const double ub_  ){

    if( index_ >= (int) grid.getNumPoints() )  return ACADOERROR(RET_INDEX_OUT_OF_BOUNDS);
    if( index_ <  0                         )  return ACADOERROR(RET_INDEX_OUT_OF_BOUNDS);


    // CHECK FEASIBILITY:
    // ------------------
    if( lb_ > ub_ + EPS )  return ACADOERROR(RET_INFEASIBLE_CONSTRAINT);

    if( point_constraints[index_] == 0 )
        point_constraints[index_] = new PointConstraint(grid,index_);

    return point_constraints[index_]->add( lb_, arg, ub_ );
}


returnValue Constraint::add( const double lb_, const Expression& arg1,
                                    const Expression& arg2, const double ub_ ){

    // CHECK FEASIBILITY:
    // ------------------
    if( lb_ > ub_ + EPS )  return ACADOERROR(RET_INFEASIBLE_CONSTRAINT);

    return boundary_constraint->add( lb_, arg1, arg2, ub_ );
}


returnValue Constraint::getBounds( const OCPiterate& iter ){

    returnValue returnvalue;
    returnvalue = BoxConstraint::getBounds( iter );

    if( returnvalue != SUCCESSFUL_RETURN ) return ACADOWARNING(returnvalue);

    if( point_constraints != 0 )
	{
		for( uint i=0; i<grid.getNumPoints(); ++i )
			if( point_constraints[i] != 0 )
				returnvalue = point_constraints[i]->getBounds( iter );
	}

    return returnvalue;
}

returnValue Constraint::getPathConstraints(Function& function_, DMatrix& lb_, DMatrix& ub_) const
{
	return path_constraint->get(function_, lb_, ub_);
}

returnValue Constraint::getPointConstraint(const unsigned index_, Function& function_, DMatrix& lb_, DMatrix& ub_) const
{
	if (point_constraints[ index_ ] == 0)
	{
		Function tmp;

		function_ = tmp;

		lb_.init(0, 0);
		ub_.init(0, 0);

		return SUCCESSFUL_RETURN;
	}

	return point_constraints[ index_ ]->get(function_, lb_, ub_);
}

CLOSE_NAMESPACE_ACADO

// end of file.
