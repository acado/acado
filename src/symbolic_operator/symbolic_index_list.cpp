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
 *    \file src/symbolic_operator/symbolic_index_list.cpp
 *    \author Boris Houska, Hans Joachim Ferreau
 *    \date 2008
 */


#include <acado/utils/acado_utils.hpp>
#include <acado/symbolic_operator/symbolic_operator.hpp>



BEGIN_NAMESPACE_ACADO




SymbolicIndexList::SymbolicIndexList(){

    numberOfOperators = 0;
    comp              = 0;
    expression        = 0;

    int run1                             ;
    const int numberOfVariableTypes = 11 ;

    entryExists = new BooleanType*[numberOfVariableTypes];

    nC = 1;
    cExist = (BooleanType*)calloc( nC, sizeof(BooleanType) );
    cIdx   = (int**)calloc( nC, sizeof(int*) );
    cDim   = (uint*)calloc( nC, sizeof(uint) );

    cExist[0] = BT_FALSE;
    cIdx  [0] = 0;
    cDim  [0] = 0;

    maxNumberOfEntries = new int[numberOfVariableTypes];

    variableCounter = 0;
    variableIndex   = new int*[numberOfVariableTypes];
    variableScale   = new double*[numberOfVariableTypes];

    for( run1 = 0; run1 < numberOfVariableTypes; run1++ ){

        maxNumberOfEntries[run1] = 0;
        entryExists[run1]        = 0;
        variableIndex[run1]      = 0;
        variableScale[run1]      = 0;
    }
}


SymbolicIndexList::~SymbolicIndexList(){

    int run1;

    if( expression != NULL ){
        for( run1 = 0; run1 < numberOfOperators; run1++ ){
             delete expression[run1];
        }
        free(expression);
    }
    if( comp != NULL ){
        free(comp);
    }

    const int numberOfVariableTypes = 11 ;

    for( run1 = 0; run1 < numberOfVariableTypes; run1++ ){

        if( maxNumberOfEntries[run1] > 0 ){

            free(entryExists  [run1]);
            free(variableIndex[run1]);
            free(variableScale[run1]);
        }
    }

    free(cExist);

    for( run1 = 0; run1 < nC; run1++ ){
         if( cIdx[run1] != 0 )
             delete[] cIdx[run1];
    }
    free(cIdx);
    free(cDim);

    delete[] maxNumberOfEntries;
    delete[] variableIndex;
    delete[] variableScale;
    delete[] entryExists;
}



SymbolicIndexList::SymbolicIndexList( const SymbolicIndexList &arg ){

    int run1;

    numberOfOperators = arg.numberOfOperators;

    if( arg.expression != NULL ){

        if( numberOfOperators > 0 ){
            expression = (Operator**)calloc(numberOfOperators,sizeof(Operator*));
        }
        else{
            expression = NULL;
        }
        for( run1 = 0; run1 < numberOfOperators; run1++ ){
             expression[run1] = arg.expression[run1]->clone();
        }
    }
    else{
        expression = NULL;
    }

    if( arg.comp != NULL ){
        if( numberOfOperators > 0 ){
            comp = (int*)calloc(numberOfOperators,sizeof(int));
        }
        else{
            comp = NULL;
        }
        for( run1 = 0; run1 < numberOfOperators; run1++ ){
            comp[run1] = arg.comp[run1];
        }
    }
    else{
        comp = NULL;
    }

    int run2                       ;
    const int numberOfVariableTypes = 11 ;

    entryExists   = new BooleanType*[numberOfVariableTypes];
    variableIndex = new int*[numberOfVariableTypes];
    variableScale = new double*[numberOfVariableTypes];


    maxNumberOfEntries = new int[numberOfVariableTypes];

    for( run1 = 0; run1 < numberOfVariableTypes; run1++ ){

        maxNumberOfEntries[run1] = arg.maxNumberOfEntries[run1];

        if( maxNumberOfEntries[run1] > 0 ){
            entryExists       [run1] = (BooleanType*)calloc(maxNumberOfEntries[run1],
                                       sizeof(BooleanType));
            variableIndex     [run1] = (int*)calloc(maxNumberOfEntries[run1],
                                       sizeof(int));
            variableScale     [run1] = (double*)calloc(maxNumberOfEntries[run1],
                                       sizeof(double));
        }
        else{
            entryExists  [run1] = 0;
            variableIndex[run1] = 0;
            variableScale[run1] = 0;
        }
        for( run2 = 0; run2 < maxNumberOfEntries[run1]; run2++ ){
            entryExists  [run1][run2] = arg.entryExists  [run1][run2];
            variableIndex[run1][run2] = arg.variableIndex[run1][run2];
            variableScale[run1][run2] = arg.variableScale[run1][run2];
        }
    }

    nC = arg.nC;
    cExist = (BooleanType*)calloc( nC, sizeof(BooleanType) );
    cIdx   = (int**)calloc( nC, sizeof(int*) );
    cDim   = (uint*)calloc( nC, sizeof(uint) );

    for( run1 = 0; run1 < nC; run1++ ){
        cExist[run1] = arg.cExist[run1];
        cDim[run1] = arg.cDim[run1];
        if( arg.cIdx[run1] != 0 ){
            cIdx[run1] = new int[cDim[run1]];
            for( run2 = 0; run2 < (int) cDim[run1]; run2++ )
                cIdx[run1][run2] = arg.cIdx[run1][run2];
        }
        else{
            cIdx[run1] = 0; 
        }
    }

    variableCounter = arg.variableCounter;
}



SymbolicIndexList& SymbolicIndexList::operator=( const SymbolicIndexList &arg ){

    if( this != &arg ){

        int run1;

        if( expression != NULL ){
            for( run1 = 0; run1 < numberOfOperators; run1++ ){
                 delete expression[run1];
            }
            free(expression);
        }

        if( comp != NULL ){
            free(comp);
        }

        int run2                       ;
        const int numberOfVariableTypes = 11 ;

        for( run1 = 0; run1 < numberOfVariableTypes; run1++ ){

            if( maxNumberOfEntries[run1] > 0 ){

                free(entryExists  [run1]);
                free(variableIndex[run1]);
                free(variableScale[run1]);
            }
        }

        delete[] maxNumberOfEntries;
        delete[] variableIndex;
        delete[] variableScale;
        delete[] entryExists;

        free(cExist);

        for( run1 = 0; run1 < nC; run1++ ){
             if( cIdx[run1] != 0 )
                 delete[] cIdx[run1];
        }
        free(cIdx);
        free(cDim);

        entryExists   = new BooleanType*[numberOfVariableTypes];
        variableIndex = new int*[numberOfVariableTypes];
        variableScale = new double*[numberOfVariableTypes];

        maxNumberOfEntries = new int[numberOfVariableTypes];

        for( run1 = 0; run1 < numberOfVariableTypes; run1++ ){

            maxNumberOfEntries[run1] = arg.maxNumberOfEntries[run1];

            if( maxNumberOfEntries[run1] > 0 ){
                entryExists       [run1] = (BooleanType*)calloc(maxNumberOfEntries[run1],
                                           sizeof(BooleanType));
                variableIndex     [run1] = (int*)calloc(maxNumberOfEntries[run1],
                                           sizeof(int));
                variableScale     [run1] = (double*)calloc(maxNumberOfEntries[run1],
                                           sizeof(double));
            }
            else{
                entryExists  [run1] = 0;
                variableIndex[run1] = 0;
                variableScale[run1] = 0;
            }

            for( run2 = 0; run2 < maxNumberOfEntries[run1]; run2++ ){
                entryExists  [run1][run2] = arg.entryExists  [run1][run2];
                variableIndex[run1][run2] = arg.variableIndex[run1][run2];
                variableScale[run1][run2] = arg.variableScale[run1][run2];
            }
        }

        nC = arg.nC;
        cExist = (BooleanType*)calloc( nC, sizeof(BooleanType) );
        cIdx   = (int**)calloc( nC, sizeof(int*) );
        cDim   = (uint*)calloc( nC, sizeof(uint) );

        for( run1 = 0; run1 < nC; run1++ ){
            cExist[run1] = arg.cExist[run1];
            cDim[run1] = arg.cDim[run1];
            if( arg.cIdx[run1] != 0 ){
                cIdx[run1] = new int[cDim[run1]];
                for( run2 = 0; run2 < (int) cDim[run1]; run2++ )
                    cIdx[run1][run2] = arg.cIdx[run1][run2];
            }
            else{
                cIdx[run1] = 0; 
            }
        }

        variableCounter = arg.variableCounter;

        numberOfOperators = arg.numberOfOperators;

        if( arg.expression != NULL ){
            if( numberOfOperators > 0 ){
                expression = (Operator**)calloc(numberOfOperators,sizeof(Operator*));
            }
            else{
                expression = NULL;
            }
            for( run1 = 0; run1 < numberOfOperators; run1++ ){
                 expression[run1] = arg.expression[run1]->clone();
            }
        }
        else{
            expression = NULL;
        }

        if( arg.comp != NULL ){
            if( numberOfOperators > 0 ){
                comp = (int*)calloc(numberOfOperators,sizeof(int));
            }
            else{
                comp = NULL;
            }
            for( run1 = 0; run1 < numberOfOperators; run1++ ){
                comp[run1] = arg.comp[run1];
            }
        }
        else{
            comp = NULL;
        }

    }

    return *this;
}


//
//  PUBLIC MEMBER FUNCTIONS:
//  ------------------------


int SymbolicIndexList::addOperatorPointer( Operator* intermediateOperator,
                                                     int comp_ ){

    expression = (Operator**)realloc(expression,
                                      (numberOfOperators+1)*sizeof(Operator*));
    expression[numberOfOperators] = intermediateOperator->clone();

    comp = (int*)realloc(comp,(numberOfOperators+1)*sizeof(int));
    comp[numberOfOperators] = comp_;

    numberOfOperators++;

    return numberOfOperators-1;
}


returnValue SymbolicIndexList::getOperators( Operator **sub, int *comp_, int *n ){

    while( n[0] < numberOfOperators ){

        sub   [*n]      = expression[*n]->clone();
        comp_ [*n]      = comp[*n];
        *n = *n+1;
    }

    return SUCCESSFUL_RETURN;
}


BooleanType SymbolicIndexList::addNewElement( VariableType variableType_, int index_ ){

    switch(variableType_){

        case VT_DIFFERENTIAL_STATE:
             if( index_ >= maxNumberOfEntries[0] ){

                 entryExists[0]   = (BooleanType*)realloc(entryExists[0],
                                     (index_+1)*sizeof(BooleanType));
                 variableIndex[0] = (int*)realloc(variableIndex[0],
                                     (index_+1)*sizeof(int));
                 variableScale[0] = (double*)realloc(variableScale[0],
                                     (index_+1)*sizeof(double));

                 int run1;
                 for( run1 = maxNumberOfEntries[0]; run1 < index_+1; run1++ ){
                      entryExists[0]  [run1] = BT_FALSE;
                      variableIndex[0][run1] = -1      ;
                      variableScale[0][run1] = 1.0     ;
                 }
                 maxNumberOfEntries[0] = index_+1;
             }
             if( entryExists[0][index_] == BT_TRUE ){
                 return BT_FALSE;
             }
             entryExists[0][index_] = BT_TRUE;
             return BT_TRUE;


        case VT_ALGEBRAIC_STATE:
             if( index_ >= maxNumberOfEntries[1] ){

                 entryExists[1]   = (BooleanType*)realloc(entryExists[1],
                                     (index_+1)*sizeof(BooleanType));
                 variableIndex[1] = (int*)realloc(variableIndex[1],
                                     (index_+1)*sizeof(int));
                 variableScale[1] = (double*)realloc(variableScale[1],
                                     (index_+1)*sizeof(double));

                 int run1;
                 for( run1 = maxNumberOfEntries[1]; run1 < index_+1; run1++ ){
                      entryExists[1]  [run1] = BT_FALSE;
                      variableIndex[1][run1] = -1      ;
                      variableScale[1][run1] = 1.0     ;
                 }
                 maxNumberOfEntries[1] = index_+1;
             }
             if( entryExists[1][index_] == BT_TRUE ){
                 return BT_FALSE;
             }
             entryExists[1][index_] = BT_TRUE;
             return BT_TRUE;


        case VT_CONTROL:
             if( index_ >= maxNumberOfEntries[2] ){

                 entryExists[2]   = (BooleanType*)realloc(entryExists[2],
                                     (index_+1)*sizeof(BooleanType));
                 variableIndex[2] = (int*)realloc(variableIndex[2],
                                     (index_+1)*sizeof(int));
                 variableScale[2] = (double*)realloc(variableScale[2],
                                     (index_+1)*sizeof(double));

                 int run1;
                 for( run1 = maxNumberOfEntries[2]; run1 < index_+1; run1++ ){
                      entryExists[2]  [run1] = BT_FALSE;
                      variableIndex[2][run1] = -1      ;
                      variableScale[2][run1] = 1.0     ;
                 }
                 maxNumberOfEntries[2] = index_+1;
             }
             if( entryExists[2][index_] == BT_TRUE ){
                 return BT_FALSE;
             }
             entryExists[2][index_] = BT_TRUE;
             return BT_TRUE;


        case VT_INTEGER_CONTROL:
             if( index_ >= maxNumberOfEntries[3] ){

                 entryExists[3]   = (BooleanType*)realloc(entryExists[3],
                                     (index_+1)*sizeof(BooleanType));
                 variableIndex[3] = (int*)realloc(variableIndex[3],
                                     (index_+1)*sizeof(int));
                 variableScale[3] = (double*)realloc(variableScale[3],
                                     (index_+1)*sizeof(double));

                 int run1;
                 for( run1 = maxNumberOfEntries[3]; run1 < index_+1; run1++ ){
                      entryExists[3]  [run1] = BT_FALSE;
                      variableIndex[3][run1] = -1      ;
                      variableScale[3][run1] = 1.0     ;
                 }
                 maxNumberOfEntries[3] = index_+1;
             }
             if( entryExists[3][index_] == BT_TRUE ){
                 return BT_FALSE;
             }
             entryExists[3][index_] = BT_TRUE;
             return BT_TRUE;


        case VT_PARAMETER:
             if( index_ >= maxNumberOfEntries[4] ){

                 entryExists[4]   = (BooleanType*)realloc(entryExists[4],
                                     (index_+1)*sizeof(BooleanType));
                 variableIndex[4] = (int*)realloc(variableIndex[4],
                                     (index_+1)*sizeof(int));
                 variableScale[4] = (double*)realloc(variableScale[4],
                                     (index_+1)*sizeof(double));

                 int run1;
                 for( run1 = maxNumberOfEntries[4]; run1 < index_+1; run1++ ){
                      entryExists[4]  [run1] = BT_FALSE;
                      variableIndex[4][run1] = -1      ;
                      variableScale[4][run1] = 1.0     ;
                 }
                 maxNumberOfEntries[4] = index_+1;
             }
             if( entryExists[4][index_] == BT_TRUE ){
                 return BT_FALSE;
             }
             entryExists[4][index_] = BT_TRUE;
             return BT_TRUE;


        case VT_INTEGER_PARAMETER:
             if( index_ >= maxNumberOfEntries[5] ){

                 entryExists[5]   = (BooleanType*)realloc(entryExists[5],
                                     (index_+1)*sizeof(BooleanType));
                 variableIndex[5] = (int*)realloc(variableIndex[5],
                                     (index_+1)*sizeof(int));
                 variableScale[5] = (double*)realloc(variableScale[5],
                                     (index_+1)*sizeof(double));

                 int run1;
                 for( run1 = maxNumberOfEntries[5]; run1 < index_+1; run1++ ){
                      entryExists[5]  [run1] = BT_FALSE;
                      variableIndex[5][run1] = -1      ;
                 }
                 maxNumberOfEntries[5] = index_+1;
             }
             if( entryExists[5][index_] == BT_TRUE ){
                 return BT_FALSE;
             }
             entryExists[5][index_] = BT_TRUE;
             return BT_TRUE;


        case VT_DISTURBANCE:
             if( index_ >= maxNumberOfEntries[6] ){

                 entryExists[6]   = (BooleanType*)realloc(entryExists[6],
                                     (index_+1)*sizeof(BooleanType));
                 variableIndex[6] = (int*)realloc(variableIndex[6],
                                     (index_+1)*sizeof(int));
                 variableScale[6] = (double*)realloc(variableScale[6],
                                     (index_+1)*sizeof(double));

                 int run1;
                 for( run1 = maxNumberOfEntries[6]; run1 < index_+1; run1++ ){
                      entryExists[6]  [run1] = BT_FALSE;
                      variableIndex[6][run1] = -1      ;
                      variableScale[6][run1] = 1.0     ;
                 }
                 maxNumberOfEntries[6] = index_+1;
             }
             if( entryExists[6][index_] == BT_TRUE ){
                 return BT_FALSE;
             }
             entryExists[6][index_] = BT_TRUE;
             return BT_TRUE;


        case VT_TIME:
             if( index_ >= maxNumberOfEntries[7] ){

                 entryExists[7]   = (BooleanType*)realloc(entryExists[7],
                                     (index_+1)*sizeof(BooleanType));
                 variableIndex[7] = (int*)realloc(variableIndex[7],
                                     (index_+1)*sizeof(int));
                 variableScale[7] = (double*)realloc(variableScale[7],
                                     (index_+1)*sizeof(double));

                 int run1;
                 for( run1 = maxNumberOfEntries[7]; run1 < index_+1; run1++ ){
                      entryExists[7]  [run1] = BT_FALSE;
                      variableIndex[7][run1] = -1      ;
                      variableScale[7][run1] = 1.0     ;
                 }
                 maxNumberOfEntries[7] = index_+1;
             }
             if( entryExists[7][index_] == BT_TRUE ){
                 return BT_FALSE;
             }
             entryExists[7][index_] = BT_TRUE;
             return BT_TRUE;


        case VT_INTERMEDIATE_STATE:
             if( index_ >= maxNumberOfEntries[8] ){

                 entryExists[8]   = (BooleanType*)realloc(entryExists[8],
                                     (index_+1)*sizeof(BooleanType));
                 variableIndex[8] = (int*)realloc(variableIndex[8],
                                     (index_+1)*sizeof(int));
                 variableScale[8] = (double*)realloc(variableScale[8],
                                     (index_+1)*sizeof(double));

                 int run1;
                 for( run1 = maxNumberOfEntries[8]; run1 < index_+1; run1++ ){
                      entryExists[8]  [run1] = BT_FALSE;
                      variableIndex[8][run1] = -1      ;
                      variableScale[8][run1] = 1.0     ;
                 }
                 maxNumberOfEntries[8] = index_+1;
             }
             if( entryExists[8][index_] == BT_TRUE ){
                 return BT_FALSE;
             }

             entryExists[8][index_] = BT_TRUE;
             return BT_TRUE;


        case VT_DDIFFERENTIAL_STATE:
             if( index_ >= maxNumberOfEntries[9] ){

                 entryExists[9]   = (BooleanType*)realloc(entryExists[9],
                                     (index_+1)*sizeof(BooleanType));
                 variableIndex[9] = (int*)realloc(variableIndex[9],
                                     (index_+1)*sizeof(int));
                 variableScale[9] = (double*)realloc(variableScale[9],
                                     (index_+1)*sizeof(double));

                 int run1;
                 for( run1 = maxNumberOfEntries[9]; run1 < index_+1; run1++ ){
                      entryExists[9]  [run1] = BT_FALSE;
                      variableIndex[9][run1] = -1      ;
                      variableScale[9][run1] = 1.0     ;
                 }
                 maxNumberOfEntries[9] = index_+1;
             }
             if( entryExists[9][index_] == BT_TRUE ){
                 return BT_FALSE;
             }
             entryExists[9][index_] = BT_TRUE;
             return BT_TRUE;

        case VT_ONLINE_DATA:
             if( index_ >= maxNumberOfEntries[10] ){

                 entryExists[10]   = (BooleanType*)realloc(entryExists[10],
                                     (index_+1)*sizeof(BooleanType));
                 variableIndex[10] = (int*)realloc(variableIndex[10],
                                     (index_+1)*sizeof(int));
                 variableScale[10] = (double*)realloc(variableScale[10],
                                     (index_+1)*sizeof(double));

                 int run1;
                 for( run1 = maxNumberOfEntries[10]; run1 < index_+1; run1++ ){
                      entryExists[10]  [run1] = BT_FALSE;
                      variableIndex[10][run1] = -1      ;
                      variableScale[10][run1] = 1.0     ;
                 }
                 maxNumberOfEntries[10] = index_+1;
             }
             if( entryExists[10][index_] == BT_TRUE ){
                 return BT_FALSE;
             }
             entryExists[10][index_] = BT_TRUE;
             return BT_TRUE;

         default: return BT_FALSE;

    }

    return BT_FALSE;
}

BooleanType SymbolicIndexList::determineCExpressionIndices( uint  dimension,
                                                    uint  ID       ,
                                                     int *idx        ){

    uint run1;

    if( nC <= (int) ID ){
        uint nnn = nC;
        nC = ID+1;
        cExist = (BooleanType*)realloc(cExist,(ID+1)*sizeof(BooleanType));
        cIdx   = (int**)realloc(cIdx,(ID+1)*sizeof(int*));
        cDim   = (uint*)realloc(cDim,(ID+1)*sizeof(uint));

        for( run1 = nnn; run1 < ID+1; run1++ ){
            cExist[run1] = BT_FALSE;
            cIdx[run1]   = 0;
            cDim  [run1] = 0;
        }
    }

    if( cExist[ID] == BT_FALSE ){
        cDim[ID] = dimension;
        cIdx[ID] = new int[dimension];
        for( run1 = 0; run1 < dimension; run1++ ){
            cIdx[ID][run1] = variableCounter;
            idx[run1]      = variableCounter;
            variableCounter++;
        }
        cExist[ID] = BT_TRUE;
        return BT_TRUE;
    }
    else{
        for( run1 = 0; run1 < cDim[ID]; run1++ )
            idx[run1] = cIdx[ID][run1];
        return BT_FALSE;
    }
}


int SymbolicIndexList::determineVariableIndex( VariableType variableType_, int index_, double scale_ ){

    switch(variableType_){

        case VT_DIFFERENTIAL_STATE:
             if( index_ >= maxNumberOfEntries[0] ){

                 ACADOERROR(RET_INDEX_OUT_OF_RANGE);
                 return -1;
             }
             if( variableIndex[0][index_] == -1 ){
                 variableIndex[0][index_] = variableCounter;
                 variableScale[0][index_] = scale_;
                 variableCounter++;
                 return variableIndex[0][index_];
             }
             return variableIndex[0][index_];

        case VT_ALGEBRAIC_STATE:
             if( index_ >= maxNumberOfEntries[1] ){

                 ACADOERROR(RET_INDEX_OUT_OF_RANGE);
                 return -1;
             }
             if( variableIndex[1][index_] == -1 ){
                 variableIndex[1][index_] = variableCounter;
                 variableScale[1][index_] = scale_;
                 variableCounter++;
                 return variableIndex[1][index_];
             }
             return variableIndex[1][index_];


        case VT_CONTROL:
             if( index_ >= maxNumberOfEntries[2] ){

                 ACADOERROR(RET_INDEX_OUT_OF_RANGE);
                 return -1;
             }
             if( variableIndex[2][index_] == -1 ){
                 variableIndex[2][index_] = variableCounter;
                 variableScale[2][index_] = scale_;
                 variableCounter++;
                 return variableIndex[2][index_];
             }
             return variableIndex[2][index_];


        case VT_INTEGER_CONTROL:
             if( index_ >= maxNumberOfEntries[3] ){

                 ACADOERROR(RET_INDEX_OUT_OF_RANGE);
                 return -1;
             }
             if( variableIndex[3][index_] == -1 ){
                 variableIndex[3][index_] = variableCounter;
                 variableScale[3][index_] = scale_;
                 variableCounter++;
                 return variableIndex[3][index_];
             }
             return variableIndex[3][index_];


        case VT_PARAMETER:
             if( index_ >= maxNumberOfEntries[4] ){

                 ACADOERROR(RET_INDEX_OUT_OF_RANGE);
                 return -1;
             }
             if( variableIndex[4][index_] == -1 ){
                 variableIndex[4][index_] = variableCounter;
                 variableScale[4][index_] = scale_;
                 variableCounter++;
                 return variableIndex[4][index_];
             }
             return variableIndex[4][index_];


        case VT_INTEGER_PARAMETER:
             if( index_ >= maxNumberOfEntries[5] ){

                 ACADOERROR(RET_INDEX_OUT_OF_RANGE);
                 return -1;
             }
             if( variableIndex[5][index_] == -1 ){
                 variableIndex[5][index_] = variableCounter;
                 variableScale[5][index_] = scale_;
                 variableCounter++;
                 return variableIndex[5][index_];
             }
             return variableIndex[5][index_];


        case VT_DISTURBANCE:
             if( index_ >= maxNumberOfEntries[6] ){

                 ACADOERROR(RET_INDEX_OUT_OF_RANGE);
                 return -1;
             }
             if( variableIndex[6][index_] == -1 ){
                 variableIndex[6][index_] = variableCounter;
                 variableScale[6][index_] = scale_;
                 variableCounter++;
                 return variableIndex[6][index_];
             }
             return variableIndex[6][index_];


        case VT_TIME:
             if( index_ >= maxNumberOfEntries[7] ){

                 ACADOERROR(RET_INDEX_OUT_OF_RANGE);
                 return -1;
             }
             if( variableIndex[7][index_] == -1 ){
                 variableIndex[7][index_] = variableCounter;
                 variableScale[7][index_] = scale_;
                 variableCounter++;
                 return variableIndex[7][index_];
             }
             return variableIndex[7][index_];


        case VT_INTERMEDIATE_STATE:
             if( index_ >= maxNumberOfEntries[8] ){

                 ACADOERROR(RET_INDEX_OUT_OF_RANGE);
                 return -1;
             }
             if( variableIndex[8][index_] == -1 ){
                 variableIndex[8][index_] = variableCounter;
                 variableScale[8][index_] = scale_;
                 variableCounter++;
                 return variableIndex[8][index_];
             }
             return variableIndex[8][index_];


        case VT_DDIFFERENTIAL_STATE:
             if( index_ >= maxNumberOfEntries[9] ){

                 ACADOERROR(RET_INDEX_OUT_OF_RANGE);
                 return -1;
             }
             if( variableIndex[9][index_] == -1 ){
                 variableIndex[9][index_] = variableCounter;
                 variableScale[9][index_] = scale_;
                 variableCounter++;
                 return variableIndex[9][index_];
             }
             return variableIndex[9][index_];

        case VT_ONLINE_DATA:
             if( index_ >= maxNumberOfEntries[10] ){

                 ACADOERROR(RET_INDEX_OUT_OF_RANGE);
                 return -1;
             }
             if( variableIndex[10][index_] == -1 ){
                 variableIndex[10][index_] = variableCounter;
                 variableScale[10][index_] = scale_;
                 variableCounter++;
                 return variableIndex[10][index_];
             }
             return variableIndex[10][index_];


         default: return -1;
    }

    return -1;
}


returnValue SymbolicIndexList::clearVariableIndexList(){

    int run1, run2                      ;
    const int numberOfVariableTypes = 11;

    variableCounter = 0;

    for( run1 = 0; run1 < numberOfVariableTypes; run1++ ){
        for( run2 = 0; run2 < maxNumberOfEntries[run1]; run2++ ){
           variableIndex [run1][run2] = -1;
        }
    }

    return SUCCESSFUL_RETURN;
}



CLOSE_NAMESPACE_ACADO

// end of file.
