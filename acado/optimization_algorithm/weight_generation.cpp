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
 *    \file src/optimization_algorithm/weight_generation.cpp
 *    \author Boris Houska, Hans Joachim Ferreau
 *    \date 2009
 */


#include <acado/optimization_algorithm/weight_generation.hpp>



BEGIN_NAMESPACE_ACADO



//
// PUBLIC MEMBER FUNCTIONS:
//


WeightGeneration::WeightGeneration(){

}

WeightGeneration::WeightGeneration( const WeightGeneration& arg ){

}

WeightGeneration::~WeightGeneration( ){

}

WeightGeneration& WeightGeneration::operator=( const WeightGeneration& arg ){

    if( this != &arg ){

    }
    return *this;
}


returnValue WeightGeneration::getWeights( const int    &m        ,
                                          const int    &pnts     ,
                                          const DVector &weightsLB,
                                          const DVector &weightsUB,
                                          DMatrix       &Weights  ,
                                          DVector       &formers    ) const{

    DVector weight(m);
    weight.setZero();

    const int layer = m-1;

    int lastone     = -1            ;
    int currentone  = -1            ;
    double    step        = 1.0/(pnts-1.0);

    const int n = 0;

    WeightGeneration child;
    return child.generateWeights( n, weight, Weights, weightsLB, weightsUB, formers, layer, lastone, currentone, step );
}




//
// PROTECTED MEMBER FUNCTIONS:
//


returnValue WeightGeneration::generateWeights( const  int    &n         ,
                                               DVector        &weight    ,
                                               DMatrix        &Weights   ,
                                               const  DVector &weightsLB ,
                                               const  DVector &weightsUB ,
                                               DVector        &formers   ,
                                               const int     &layer     ,
                                               int           &lastOne   ,
                                               int           &currentOne,
                                               double        &step
                                             ) const{


   int run1, run2;

    if( n == layer ){

        //printf("case1:  n = %d  layer = %d  \n", n, layer );

        double weight_test = 1.0;
        for( run1 = 0; run1 < (int) weight.getDim()-1; run1++ )
            weight_test -= weight(run1);

        if( ( weight_test >= weightsLB(layer) - 100.0*EPS ) &&
            ( weight_test <= weightsUB(layer) + 100.0*EPS )    ){

            if( currentOne >= 0 ){

                DVector tmp( formers.getDim()+1 );

                for( run1 = 0; run1 < (int) formers.getDim(); run1++ )
                    tmp(run1) = formers(run1);

                tmp(formers.getDim()) = lastOne;
                formers = tmp;
                lastOne = currentOne;
                currentOne = -1;
            }
            else{

                DVector tmp( formers.getDim()+1 );

                for( run1 = 0; run1 < (int) formers.getDim(); run1++ )
                    tmp(run1) = formers(run1);

                tmp(formers.getDim()) = Weights.getNumCols();
                formers = tmp;
            }

             weight(n) = weight_test;

             DMatrix tmp( weight.getDim(), Weights.getNumCols()+1 );

             for( run1 = 0; run1 < (int) Weights.getNumRows(); run1++ ){
                 for( run2 = 0; run2 < (int) Weights.getNumCols(); run2++ )
                     tmp(run1,run2) = Weights(run1,run2);
             }

             for( run2 = 0; run2 < (int) weight.getDim(); run2++ )
                     tmp(run2,Weights.getNumCols()) = weight(run2);

             Weights = tmp;
        }
    }
    else{

        //printf("case2:  n = %d  layer = %d  \n", n, layer );

        double weight_test = weightsLB(n);

        while( weight_test <= weightsUB(n) + 100.0*EPS ){

            if( n == layer-2 ) currentOne = Weights.getNumCols()+1;

            weight(n) = weight_test;

            WeightGeneration child;
            child.generateWeights( n+1, weight, Weights, weightsLB, weightsUB, formers, layer, lastOne, currentOne, step );

            weight_test += (weightsUB(n)-weightsLB(n))*step;
        }
    }

    return SUCCESSFUL_RETURN;
}



CLOSE_NAMESPACE_ACADO

// end of file.
