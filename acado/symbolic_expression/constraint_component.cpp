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
*    \file src/symbolic_expression/constraint_component.cpp
*    \author Boris Houska, Hans Joachim Ferreau
*    \date 2008
*/




#include <acado/symbolic_expression/constraint_component.hpp>


BEGIN_NAMESPACE_ACADO



// --------------------------------------------------------------------------------------------
// --------------------------------------------------------------------------------------------


ConstraintComponent::ConstraintComponent(){

    ub.init(1);
    lb.init(1);
    ub.setAll(INFTY);
    lb.setAll(-INFTY);
}

ConstraintComponent::ConstraintComponent( const ConstraintComponent &arg ){

    expression = arg.expression;
    ub         = arg.ub        ;
    lb         = arg.lb        ;
    lbGrid     = arg.lbGrid    ;
    ubGrid     = arg.ubGrid    ;
}


ConstraintComponent::~ConstraintComponent(){

}


ConstraintComponent& ConstraintComponent::operator=( const ConstraintComponent &arg ){

    if( this != &arg ){

       expression = arg.expression;
       ub         = arg.ub        ;
       lb         = arg.lb        ;
       lbGrid     = arg.lbGrid    ;
       ubGrid     = arg.ubGrid    ;
    }
    return *this;
}


returnValue ConstraintComponent::initialize( const DVector& lb_, Expression arg, const DVector& ub_ ){

    expression = arg;
    ub         = ub_;
    lb         = lb_;

    return SUCCESSFUL_RETURN;
}


returnValue ConstraintComponent::initialize( const VariablesGrid& lb_, Expression arg, const VariablesGrid& ub_ ){

    expression = arg;
    ubGrid     = ub_;
    lbGrid     = lb_;

    return SUCCESSFUL_RETURN;
}




ConstraintComponent ConstraintComponent::operator()( const uint &index ) const{

    ASSERT( index < getDim() );

    ConstraintComponent tmp;

    tmp.expression = expression(index);

    if( lbGrid.isEmpty() == BT_TRUE ) tmp.lb(0)  = lb    (index);
    else                              tmp.lbGrid = lbGrid(index);

    if( ubGrid.isEmpty() == BT_TRUE ) tmp.ub(0)  = ub    (index);
    else                              tmp.ubGrid = ubGrid(index);

    return tmp;
}



ConstraintComponent operator<=( double lb_, const ConstraintComponent &arg ){

    ConstraintComponent tmp(arg);
    tmp.setLB( lb_ );
    return tmp;
}

ConstraintComponent operator>=( double ub_, const ConstraintComponent &arg ){

    ConstraintComponent tmp(arg);
    tmp.setUB( ub_ );
    return tmp;
}

ConstraintComponent operator<=( DVector lb_, const ConstraintComponent &arg ){

    ConstraintComponent tmp(arg);
    tmp.setLB( lb_ );
    return tmp;
}

ConstraintComponent operator>=( DVector ub_, const ConstraintComponent &arg ){

    ConstraintComponent tmp(arg);
    tmp.setUB( ub_ );
    return tmp;
}

ConstraintComponent operator<=( VariablesGrid lb_, const ConstraintComponent &arg ){

    ConstraintComponent tmp(arg);
    tmp.setLB( lb_ );
    return tmp;
}

ConstraintComponent operator>=( VariablesGrid ub_, const ConstraintComponent &arg ){

    ConstraintComponent tmp(arg);
    tmp.setUB( ub_ );
    return tmp;
}

ConstraintComponent operator<=( const Expression& arg, const double& ub ) {

    DVector ub_(arg.getDim());
    ub_.setAll(ub);

    return operator<=(arg, ub_);
}

ConstraintComponent operator>=( const Expression& arg, const double& lb ) {

    DVector lb_(arg.getDim());
    lb_.setAll(lb);

    return operator>=(arg, lb_);
}

ConstraintComponent operator==( const Expression& arg, const double& b ) {

    DVector b_(arg.getDim());
    b_.setAll(b);

    return operator==(arg, b_);
}



ConstraintComponent operator<=( const Expression& arg, const DVector& ub ) {

    ConstraintComponent tmp;

    DVector lb = ub;
    lb.setAll(-INFTY);

    tmp.initialize( lb, arg, ub );

    return tmp;
}


ConstraintComponent operator>=( const Expression& arg, const DVector& lb ) {

    ConstraintComponent tmp;

    DVector ub = lb;
    ub.setAll(INFTY);

    tmp.initialize( lb, arg, ub );

    return tmp;
}


ConstraintComponent operator==( const Expression& arg, const DVector& b ) {

    ConstraintComponent tmp;

    tmp.initialize( b, arg, b );
    return tmp;
}



ConstraintComponent operator<=( const Expression& arg, const VariablesGrid& ub ) {

    ConstraintComponent tmp;

    VariablesGrid lb = ub;
    lb.setAll(-INFTY);

    tmp.initialize( lb, arg, ub );

    return tmp;
}


ConstraintComponent operator>=( const Expression& arg, const VariablesGrid& lb ) {

    ConstraintComponent tmp;

    VariablesGrid ub = lb;
    ub.setAll(INFTY);

    tmp.initialize( lb, arg, ub );

    return tmp;
}


ConstraintComponent operator==( const Expression& arg, const VariablesGrid& b ) {

    ConstraintComponent tmp;

    tmp.initialize( b, arg, b );
    return tmp;
}


ConstraintComponent operator<=( double lb, const Expression &arg ){ return ( arg >= lb ); }
ConstraintComponent operator>=( double ub, const Expression &arg ){ return ( arg <= ub ); }
ConstraintComponent operator==( double  b, const Expression &arg ){ return ( arg == b  ); }

ConstraintComponent operator<=( DVector lb, const Expression &arg ){ return ( arg >= lb ); }
ConstraintComponent operator>=( DVector ub, const Expression &arg ){ return ( arg <= ub ); }
ConstraintComponent operator==( DVector  b, const Expression &arg ){ return ( arg == b  ); }

ConstraintComponent operator<=( VariablesGrid lb, const Expression &arg ){ return ( arg >= lb ); }
ConstraintComponent operator>=( VariablesGrid ub, const Expression &arg ){ return ( arg <= ub ); }
ConstraintComponent operator==( VariablesGrid b , const Expression &arg ){ return ( arg == b  ); }


CLOSE_NAMESPACE_ACADO

// end of file.
