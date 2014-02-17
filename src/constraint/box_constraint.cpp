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
 *    \file src/constraint/box_constraint.cpp
 *    \author Boris Houska, Hans Joachim Ferreau
 *
 */


#include <acado/constraint/box_constraint.hpp>


BEGIN_NAMESPACE_ACADO



//
// PUBLIC MEMBER FUNCTIONS:
//


BoxConstraint::BoxConstraint( ){

    nb    = 0;
    var   = 0;
    index = 0;
    blb   = 0;
    bub   = 0;

    residuumXL  = 0;
    residuumXU  = 0;
    residuumXAL = 0;
    residuumXAU = 0;
    residuumPL  = 0;
    residuumPU  = 0;
    residuumUL  = 0;
    residuumUU  = 0;
    residuumWL  = 0;
    residuumWU  = 0;
}


returnValue BoxConstraint::init( const Grid& grid_ ){

    deleteAll();

    grid  = grid_;

    nb    = 0;
    var   = 0;
    index = 0;
    blb   = 0;
    bub   = 0;

    residuumXL  = new DMatrix[grid.getNumPoints()];
    residuumXU  = new DMatrix[grid.getNumPoints()];
    residuumXAL = new DMatrix[grid.getNumPoints()];
    residuumXAU = new DMatrix[grid.getNumPoints()];
    residuumPL  = new DMatrix[1                  ];
    residuumPU  = new DMatrix[1                  ];
    residuumUL  = new DMatrix[grid.getNumPoints()];
    residuumUU  = new DMatrix[grid.getNumPoints()];
    residuumWL  = new DMatrix[grid.getNumPoints()];
    residuumWU  = new DMatrix[grid.getNumPoints()];

    return SUCCESSFUL_RETURN;
}



BoxConstraint::BoxConstraint( const BoxConstraint& rhs ){

    int   run1    ;
    grid = rhs.grid;

    nb = rhs.nb;
    if( nb > 0 ){
        var   = (VariableType*)calloc(nb,sizeof(VariableType));
        index = (int*)calloc(nb,sizeof(int));
        blb   = (DVector**)calloc(nb,sizeof(DVector*));
        bub   = (DVector**)calloc(nb,sizeof(DVector*));

        for( run1 = 0; run1 < nb; run1++ ){

            var  [run1] = rhs.var  [run1];
            index[run1] = rhs.index[run1];
            blb  [run1] = new DVector(*rhs.blb[run1]);
            bub  [run1] = new DVector(*rhs.bub[run1]);
        }
    }
    else{
        var   = 0;
        index = 0;
        blb   = 0;
        bub   = 0;
    }

    residuumXL  = new DMatrix[grid.getNumPoints()];
    residuumXU  = new DMatrix[grid.getNumPoints()];
    residuumXAL = new DMatrix[grid.getNumPoints()];
    residuumXAU = new DMatrix[grid.getNumPoints()];
    residuumPL  = new DMatrix[1                  ];
    residuumPU  = new DMatrix[1                  ];
    residuumUL  = new DMatrix[grid.getNumPoints()];
    residuumUU  = new DMatrix[grid.getNumPoints()];
    residuumWL  = new DMatrix[grid.getNumPoints()];
    residuumWU  = new DMatrix[grid.getNumPoints()];
}

BoxConstraint::~BoxConstraint( ){

    deleteAll();
}


void BoxConstraint::deleteAll(){

    int run1;

    if(   var != 0 ) free(    var);
    if( index != 0 ) free(  index);
    if(   blb != 0 ){
        for( run1 = 0; run1 < nb; run1++  )  delete blb[run1];
        free(blb);
    }
    if(   bub != 0 ){
        for( run1 = 0; run1 < nb; run1++  )  delete bub[run1];
        free(bub);
    }

    if( residuumXL  != 0 ) delete[] residuumXL ;
    if( residuumXU  != 0 ) delete[] residuumXU ;
    if( residuumXAL != 0 ) delete[] residuumXAL;
    if( residuumXAU != 0 ) delete[] residuumXAU;
    if( residuumPL  != 0 ) delete[] residuumPL ;
    if( residuumPU  != 0 ) delete[] residuumPU ;
    if( residuumUL  != 0 ) delete[] residuumUL ;
    if( residuumUU  != 0 ) delete[] residuumUU ;
    if( residuumWL  != 0 ) delete[] residuumWL ;
    if( residuumWU  != 0 ) delete[] residuumWU ;
}


BoxConstraint& BoxConstraint::operator=( const BoxConstraint& rhs ){

    int run1;

    if( this != &rhs ){

        deleteAll();

        grid = rhs.grid;

        nb = rhs.nb;
        if( nb > 0 ){
            var   = (VariableType*)calloc(nb,sizeof(VariableType));
            index = (int*)calloc(nb,sizeof(int));
            blb   = (DVector**)calloc(nb,sizeof(DVector*));
            bub   = (DVector**)calloc(nb,sizeof(DVector*));

            for( run1 = 0; run1 < nb; run1++ ){

                var  [run1] = rhs.var  [run1];
                index[run1] = rhs.index[run1];
                blb  [run1] = new DVector(*rhs.blb[run1]);
                bub  [run1] = new DVector(*rhs.bub[run1]);
            }
        }
        else{
            var   = 0;
            index = 0;
            blb   = 0;
            bub   = 0;
        }

        residuumXL  = new DMatrix[grid.getNumPoints()];
        residuumXU  = new DMatrix[grid.getNumPoints()];
        residuumXAL = new DMatrix[grid.getNumPoints()];
        residuumXAU = new DMatrix[grid.getNumPoints()];
        residuumPL  = new DMatrix[1                  ];
        residuumPU  = new DMatrix[1                  ];
        residuumUL  = new DMatrix[grid.getNumPoints()];
        residuumUU  = new DMatrix[grid.getNumPoints()];
        residuumWL  = new DMatrix[grid.getNumPoints()];
        residuumWU  = new DMatrix[grid.getNumPoints()];
    }
    return *this;
}





returnValue BoxConstraint::evaluateBounds( const OCPiterate& iter ){


    uint run1, run2;

    const uint N = grid.getNumPoints();

    // EVALUATE BOUNDS:
    // ----------------

    for( run1 = 0; run1 < N; run1++ ){

        if( iter.x != NULL ){
            residuumXL[run1].init ( iter.x->getNumValues(), 1 );
            residuumXU[run1].init ( iter.x->getNumValues(), 1 );

            for( run2 = 0; run2 < iter.x->getNumValues(); run2++ ){
                residuumXL[run1](run2,0) = -INFTY;
                residuumXU[run1](run2,0) =  INFTY;
            }
        }
        else{
            residuumXL[run1].init ( 0, 0 );
            residuumXU[run1].init ( 0, 0 );
        }

        if( iter.xa != NULL ){
            residuumXAL[run1].init ( iter.xa->getNumValues(), 1 );
            residuumXAU[run1].init ( iter.xa->getNumValues(), 1 );

            for( run2 = 0; run2 < iter.xa->getNumValues(); run2++ ){
                residuumXAL[run1](run2,0) = -INFTY;
                residuumXAU[run1](run2,0) =  INFTY;
            }
        }
        else{
            residuumXAL[run1].init ( 0, 0 );
            residuumXAU[run1].init ( 0, 0 );
        }

        if( iter.u != NULL ){
            residuumUL[run1].init ( iter.u->getNumValues(), 1 );
            residuumUU[run1].init ( iter.u->getNumValues(), 1 );

            for( run2 = 0; run2 < iter.u->getNumValues(); run2++ ){
                residuumUL[run1](run2,0) = -INFTY;
                residuumUU[run1](run2,0) =  INFTY;
            }
        }
        else{
            residuumUL[run1].init ( 0, 0 );
            residuumUU[run1].init ( 0, 0 );
        }

        if( iter.w != NULL ){
            residuumWL[run1].init ( iter.w->getNumValues(), 1 );
            residuumWU[run1].init ( iter.w->getNumValues(), 1 );

            for( run2 = 0; run2 < iter.w->getNumValues(); run2++ ){
                residuumWL[run1](run2,0) = -INFTY;
                residuumWU[run1](run2,0) =  INFTY;
            }
        }
        else{
            residuumWL[run1].init ( 0, 0 );
            residuumWU[run1].init ( 0, 0 );
        }
    }


    if( iter.p != NULL ){
        residuumPL[0].init ( iter.p->getNumValues(), 1 );
        residuumPU[0].init ( iter.p->getNumValues(), 1 );

        for( run2 = 0; run2 < iter.p->getNumValues(); run2++ ){
            residuumPL[0](run2,0) = -INFTY;
            residuumPU[0](run2,0) =  INFTY;
        }
    }
    else{
        residuumPL[0].init ( 0, 0 );
        residuumPU[0].init ( 0, 0 );
    }


    for( run1 = 0; (int) run1 < nb; run1++ ){

        switch( var[run1] ){

            case VT_DIFFERENTIAL_STATE:
                if( iter.x != NULL ){
                    for( run2 = 0; run2 < N; run2++ ){
                        residuumXL[run2](index[run1],0) = blb[run1][0](run2) - iter.x->operator()(run2,index[run1]);
                        residuumXU[run2](index[run1],0) = bub[run1][0](run2) - iter.x->operator()(run2,index[run1]);
                    }
                }
                else ACADOERROR( RET_INVALID_ARGUMENTS );
                break;


            case VT_ALGEBRAIC_STATE:
                if( iter.xa != NULL ){
                    for( run2 = 0; run2 < N; run2++ ){
                        residuumXAL[run2](index[run1],0) = blb[run1][0](run2) - iter.xa->operator()(run2,index[run1]);
                        residuumXAU[run2](index[run1],0) = bub[run1][0](run2) - iter.xa->operator()(run2,index[run1]);
                    }
                }
                else ACADOERROR( RET_INVALID_ARGUMENTS );
                break;


            case VT_PARAMETER:
                if( iter.p != NULL ){
                    residuumPL[0](index[run1],0) = blb[run1][0](0) - iter.p->operator()(0,index[run1]);
                    residuumPU[0](index[run1],0) = bub[run1][0](0) - iter.p->operator()(0,index[run1]);
                }
                else ACADOERROR( RET_INVALID_ARGUMENTS );
                break;

            case VT_CONTROL:
                if( iter.u != NULL ){
                    for( run2 = 0; run2 < N; run2++ ){
                        residuumUL[run2](index[run1],0) = blb[run1][0](run2) - iter.u->operator()(run2,index[run1]);
                        residuumUU[run2](index[run1],0) = bub[run1][0](run2) - iter.u->operator()(run2,index[run1]);
                    }
                }
                else ACADOERROR( RET_INVALID_ARGUMENTS );
                break;

            case VT_DISTURBANCE:
                if( iter.w != NULL ){
                    for( run2 = 0; run2 < N; run2++ ){
                        residuumWL[run2](index[run1],0) = blb[run1][0](run2) - iter.w->operator()(run2,index[run1]);
                        residuumWU[run2](index[run1],0) = bub[run1][0](run2) - iter.w->operator()(run2,index[run1]);
                    }
                }
                else {ASSERT(1==0);ACADOERROR( RET_INVALID_ARGUMENTS );}
                break;


            default:
                ACADOERRORTEXT(RET_NOT_IMPLEMENTED_YET,"Variables in constraints should be of type DIFFERENTIAL_STATE, ALGEBRAIC_STATE, PARAMETER, CONTROL or DISTURBANCE\n");
                break;

        }
    }

    return SUCCESSFUL_RETURN;
}


inline returnValue BoxConstraint::getBounds( const OCPiterate& iter ){


    uint run1, run2;

    const uint N = grid.getNumPoints();

    for( run1 = 0; (int) run1 < nb; run1++ ){

        switch( var[run1] ){

            case VT_DIFFERENTIAL_STATE:
                if( iter.x != NULL ){
                    for( run2 = 0; run2 < N; run2++ ){

                        if( bub[run1][0](run2) - blb[run1][0](run2) < -0.5*BOUNDTOL )
                            return ACADOERROR( RET_INCONSISTENT_BOUNDS );

                        iter.x->setLowerBound(run2,index[run1],blb[run1][0](run2));
                        iter.x->setUpperBound(run2,index[run1],bub[run1][0](run2));
                    }
                }
                else ACADOERROR( RET_INVALID_ARGUMENTS );
                break;


            case VT_ALGEBRAIC_STATE:
                if( iter.xa != NULL ){
                    for( run2 = 0; run2 < N; run2++ ){

                        if( bub[run1][0](run2) - blb[run1][0](run2) < -0.5*BOUNDTOL )
                            return ACADOERROR( RET_INCONSISTENT_BOUNDS );

                        iter.xa->setLowerBound(run2,index[run1],blb[run1][0](run2));
                        iter.xa->setUpperBound(run2,index[run1],bub[run1][0](run2));
                    }
                }
                else ACADOERROR( RET_INVALID_ARGUMENTS );
                break;


            case VT_PARAMETER:
                if( iter.p != NULL ){
					for( run2 = 0; run2 < N; run2++ ){
                        if( bub[run1][0](0) - blb[run1][0](0) < -0.5*BOUNDTOL )
                            return ACADOERROR( RET_INCONSISTENT_BOUNDS );

                        iter.p->setLowerBound(run2,index[run1],blb[run1][0](0));
                        iter.p->setUpperBound(run2,index[run1],bub[run1][0](0));
					}
                }
                else ACADOERROR( RET_INVALID_ARGUMENTS );
                break;

            case VT_CONTROL:
                if( iter.u != NULL ){
                    for( run2 = 0; run2 < N; run2++ ){

                        if( bub[run1][0](run2) - blb[run1][0](run2) < -0.5*BOUNDTOL )
                            return ACADOERROR( RET_INCONSISTENT_BOUNDS );

                        iter.u->setLowerBound(run2,index[run1],blb[run1][0](run2));
                        iter.u->setUpperBound(run2,index[run1],bub[run1][0](run2));
                    }
                }
                else ACADOERROR( RET_INVALID_ARGUMENTS );
                break;

            case VT_DISTURBANCE:
                if( iter.w != NULL ){
                    for( run2 = 0; run2 < N; run2++ ){

                        if( bub[run1][0](run2) - blb[run1][0](run2) < -0.5*BOUNDTOL )
                            return ACADOERROR( RET_INCONSISTENT_BOUNDS );

                        iter.w->setLowerBound(run2,index[run1],blb[run1][0](run2));
                        iter.w->setUpperBound(run2,index[run1],bub[run1][0](run2));
                    }
                }
                else ACADOERROR( RET_INVALID_ARGUMENTS );
                break;


            default:
                ACADOERROR(RET_NOT_IMPLEMENTED_YET);
                break;

        }
    }

    return SUCCESSFUL_RETURN;
}



CLOSE_NAMESPACE_ACADO

// end of file.
