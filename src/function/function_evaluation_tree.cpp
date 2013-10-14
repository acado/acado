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
 *    \file   src/function/function_evaluation_tree.cpp
 *    \author Boris Houska, Hans Joachim Ferreau
 *    \date   2010
 */


#include <acado/utils/acado_utils.hpp>
#include <acado/function/function_evaluation_tree.hpp>



BEGIN_NAMESPACE_ACADO



//
// PUBLIC MEMBER FUNCTIONS:
//

FunctionEvaluationTree::FunctionEvaluationTree( ){

    f         = NULL;
    sub       = NULL;
    lhs_comp  = NULL;
    indexList = new SymbolicIndexList();
    dim       =  0;
    n         =  0;

    globalExportVariableName = "acado_aux";
}

FunctionEvaluationTree::FunctionEvaluationTree( const FunctionEvaluationTree& arg ){

    int run1;

    dim = arg.dim;
    n   = arg.n  ;

    globalExportVariableName = arg.globalExportVariableName;

    if( arg.f == NULL ){
        f = NULL;
    }
    else{
        f = (Operator**)calloc(dim,sizeof(Operator*));

        for( run1 = 0; run1 < dim; run1++ ){
            f[run1] = arg.f[run1]->clone();
        }
    }

    indexList = new SymbolicIndexList(*arg.indexList);

    if( arg.sub == NULL ){
        sub       = NULL;
        lhs_comp  = NULL;
    }
    else{
        sub = (Operator**)calloc(n,sizeof(Operator*));
        lhs_comp = (int*)calloc(n,sizeof(int));

        for( run1 = 0; run1 < n; run1++ ){
            sub[run1] = arg.sub[run1]->clone();
            lhs_comp[run1] = arg.lhs_comp[run1];
        }
    }

    safeCopy = arg.safeCopy;
}


FunctionEvaluationTree::~FunctionEvaluationTree( ){

    int run1;

    for( run1 = 0; run1 < dim; run1++ ){
         delete f[run1];
    }
    if( f != NULL){
        free(f);
    }

    for( run1 = 0; run1 < n; run1++ ){

         delete sub[run1];
    }

    if( sub != NULL ){
        free(sub);
    }

    if( lhs_comp != NULL ){
        free(lhs_comp);
    }

    delete indexList;
}


FunctionEvaluationTree& FunctionEvaluationTree::operator=( const FunctionEvaluationTree& arg ){

    int run1;

    if( this != &arg ){

        for( run1 = 0; run1 < dim; run1++ ){
             delete f[run1];
        }
        if( f != NULL ){
            free(f);
        }

        for( run1 = 0; run1 < n; run1++ ){
             delete sub[run1];
        }
        if( sub != 0 ){
           free(sub);
        }

        if( lhs_comp != NULL ){
            free(lhs_comp);
        }

        delete indexList;

        dim = arg.dim;
        n   = arg.n  ;

        globalExportVariableName = arg.globalExportVariableName;

        if( arg.f == NULL ){
            f = NULL;
        }
        else{
            f = (Operator**)calloc(dim,sizeof(Operator*));

            for( run1 = 0; run1 < dim; run1++ ){
                f[run1] = arg.f[run1]->clone();
            }
        }

        indexList = new SymbolicIndexList(*arg.indexList);

        if( arg.sub == NULL ){
            sub = NULL;
            lhs_comp = NULL;
        }
        else{
            sub = (Operator**)calloc(n,sizeof(Operator*));
            lhs_comp = (int*)calloc(n,sizeof(int));

            for( run1 = 0; run1 < n; run1++ ){
                sub[run1] = arg.sub[run1]->clone();
                lhs_comp[run1] = arg.lhs_comp[run1];
            }
        }
        safeCopy = arg.safeCopy;
    }

    return *this;
}


returnValue FunctionEvaluationTree::getExpression( Expression& expression ) const{

    expression = safeCopy;
    return SUCCESSFUL_RETURN;
}


returnValue FunctionEvaluationTree::operator<<( const Expression& arg ){

    safeCopy << arg;

    uint run1;

    for( run1 = 0; run1 < arg.getDim(); run1++ ){

        int nn;

        nn = n;

        f = (Operator**)realloc(f,(dim+1)*sizeof(Operator*));

        f[dim] = arg.element[run1]->clone();
        f[dim]-> loadIndices  ( indexList );

        sub       = (Operator**)realloc(sub,
                    (indexList->getNumberOfOperators())*sizeof(Operator*));
        lhs_comp  = (int*)realloc(lhs_comp,
                    (indexList->getNumberOfOperators())*sizeof(int));

        indexList->getOperators( sub, lhs_comp, &n );

        while( nn < n ){

            sub[nn]-> enumerateVariables( indexList );
            nn++;
        }

        f[dim]-> enumerateVariables( indexList );

        dim++;
    }
    return SUCCESSFUL_RETURN;
}



returnValue FunctionEvaluationTree::evaluate( double *x, double *result ){

    int run1;

    for( run1 = 0; run1 < n; run1++ ){

        sub[run1]->evaluate( 0, x, &x[ indexList->index(VT_INTERMEDIATE_STATE,
                                                     lhs_comp[run1]         ) ] );
    }
    for( run1 = 0; run1 < dim; run1++ ){
        f[run1]->evaluate( 0, x, &result[run1] );
    }

    return SUCCESSFUL_RETURN;
}


returnValue FunctionEvaluationTree::evaluate( double *x, double *result, PrintLevel printL ){

    int run1;

    if( printL == MEDIUM || printL == HIGH ){
        acadoPrintf("symbolic expression evaluation:\n");
    }

    for( run1 = 0; run1 < n; run1++ ){

        sub[run1]->evaluate( 0, x, &x[ indexList->index(VT_INTERMEDIATE_STATE,
                                                     lhs_comp[run1]         ) ] );
        if( printL == HIGH ){
            acadoPrintf("sub[%d]  = %.16e \n", lhs_comp[run1],
                    x[ indexList->index(VT_INTERMEDIATE_STATE, lhs_comp[run1]) ] );
         }
    }

    for( run1 = 0; run1 < dim; run1++ ){

        f[run1]->evaluate( 0, x, &result[run1] );

        if( printL == HIGH ){
            acadoPrintf("f[%d]  = %.16e \n", run1, result[run1] );
        }
        if( printL == MEDIUM ){
            acadoPrintf("f[%d]  = %.16e \n", run1, result[run1] );
        }
    }

    return SUCCESSFUL_RETURN;
}



returnValue FunctionEvaluationTree::evaluate( int number, double *x, double *result ){

    int run1;

    for( run1 = 0; run1 < n; run1++ ){
        sub[run1]->evaluate( number, x, &x[ indexList->index(VT_INTERMEDIATE_STATE,
                                                             lhs_comp[run1]         ) ] );
    }
    for( run1 = 0; run1 < dim; run1++ ){
        f[run1]->evaluate( number, x, &result[run1] );
    }

    return SUCCESSFUL_RETURN;
}



FunctionEvaluationTree* FunctionEvaluationTree::differentiate( int index_ ){

    ACADOERROR(RET_NOT_IMPLEMENTED_YET);

    return this;
}


FunctionEvaluationTree FunctionEvaluationTree::substitute( VariableType variableType_, int index_, double sub_ ){

    int run1;
    FunctionEvaluationTree tmp;

    int sub_index = index(variableType_, index_ );

    tmp.dim = dim;
    tmp.n   = n  ;

    if( f == NULL ){
        tmp.f = NULL;
    }
    else{
        tmp.f = (Operator**)calloc(dim,sizeof(Operator*));

        Operator *temp;

        if( fabs( sub_ ) > 10.0*EPS ){
            if( fabs( 1.0 - sub_ ) < 10.0*EPS  ){
                temp = new DoubleConstant(1.0,NE_ONE);
                for( run1 = 0; run1 < dim; run1++ ){
                    tmp.f[run1] = f[run1]->substitute(sub_index,temp);
                }
                delete temp;
            }
            else{
                temp = new DoubleConstant(sub_,NE_NEITHER_ONE_NOR_ZERO);
                for( run1 = 0; run1 < dim; run1++ ){
                    tmp.f[run1] = f[run1]->substitute(sub_index,temp);
                }
                delete temp;
            }
        }
        else{
            temp = new DoubleConstant(0.0,NE_ZERO);
            for( run1 = 0; run1 < dim; run1++ ){
                tmp.f[run1] = f[run1]->substitute(sub_index,temp);
            }
            delete temp;
        }
    }

    if( sub == NULL ){
        tmp.sub       = NULL;
        tmp.lhs_comp  = NULL;
    }
    else{
        tmp.sub      = (Operator**)calloc(n,sizeof(Operator*));
        tmp.lhs_comp = (int*)calloc(n,sizeof(int));

        Operator *temp;

        if( fabs(sub_) > 10.0*EPS ){
            if( fabs( 1.0-sub_ ) < 10.0*EPS  ){
                temp = new DoubleConstant(1.0,NE_ONE);
                for( run1 = 0; run1 < n; run1++ ){
                    tmp.sub[run1]      = sub[run1]->substitute(sub_index,temp);
                    tmp.lhs_comp[run1] = lhs_comp[run1];
                }
                delete temp;
            }
            else{
                temp = new DoubleConstant(sub_,NE_NEITHER_ONE_NOR_ZERO);
                for( run1 = 0; run1 < n; run1++ ){
                    tmp.sub[run1]      = sub[run1]->substitute(sub_index,temp);
                    tmp.lhs_comp[run1] = lhs_comp[run1];
                }
                delete temp;
            }
        }
        else{
            temp = new DoubleConstant(0.0,NE_ZERO);
            for( run1 = 0; run1 < n; run1++ ){
                tmp.sub[run1]      = sub[run1]->substitute(sub_index,temp);
                tmp.lhs_comp[run1] = lhs_comp[run1];
            }
            delete temp;
        }
    }

    delete tmp.indexList;
    tmp.indexList = indexList->substitute(variableType_, index_);

    return tmp;
}



NeutralElement FunctionEvaluationTree::isOneOrZero(){

    int run1;

    NeutralElement e = NE_NEITHER_ONE_NOR_ZERO;

    if( dim > 0 ){
        if( f[0]->isOneOrZero() == NE_ZERO ){
            e = NE_ZERO;
        }
        if( f[0]->isOneOrZero() == NE_ONE ){
            e = NE_ONE;
        }
    }

    if( e == NE_NEITHER_ONE_NOR_ZERO ){
        return e;
    }

    for( run1 = 1; run1 < dim; run1++ ){

        if( e == NE_ONE ){
            if( f[run1]->isOneOrZero() != NE_ONE ){
                return NE_NEITHER_ONE_NOR_ZERO;
            }
        }
        if( e == NE_ZERO ){
            if( f[run1]->isOneOrZero() != NE_ZERO ){
                return NE_NEITHER_ONE_NOR_ZERO;
            }
        }
    }

    return e;
}



BooleanType FunctionEvaluationTree::isDependingOn( const Expression &variable ){

    int nn = variable.getDim();

    int                 run1;
    BooleanType *implicit_dep = new BooleanType [n ];
    VariableType *varType     = new VariableType[nn];
    int          *component   = new int         [nn];

    for( run1 = 0; run1 < nn; run1++ ){

        Operator *tmp2 = (variable.element[run1])->clone();

        if( tmp2->isVariable( varType[run1], component[run1] ) == BT_FALSE ){

             ACADOERROR(RET_INVALID_ARGUMENTS);
             delete   tmp2     ;
             delete[] varType  ;
             delete[] component;
             return   BT_TRUE  ;
        }

        if( varType[run1] == VT_INTERMEDIATE_STATE ){

             ACADOERROR(RET_INVALID_ARGUMENTS);
             delete   tmp2     ;
             delete[] varType  ;
             delete[] component;
             return   BT_TRUE  ;
        }
        delete tmp2;
    }


    for( run1 = 0; run1 < n; run1++ ){
        implicit_dep[run1] = sub[run1]->isDependingOn( 1, varType, component, implicit_dep );
    }
    for( run1 = 0; run1 < dim; run1++ ){
        if( f[run1]->isDependingOn( 1, varType, component, implicit_dep ) == BT_TRUE  ){
            delete[] implicit_dep;
            delete[] varType  ;
            delete[] component;
            return BT_TRUE;
        }
    }

    delete[] implicit_dep;
    delete[] varType  ;
    delete[] component;
    return   BT_FALSE;
}


BooleanType FunctionEvaluationTree::isLinearIn( const Expression &variable ){

    int nn = variable.getDim();

    int                 run1;
    BooleanType *implicit_dep = new BooleanType [n ];
    VariableType *varType     = new VariableType[nn];
    int          *component   = new int         [nn];

    for( run1 = 0; run1 < nn; run1++ ){

        Operator *tmp2 = (variable.element[run1])->clone();

        if( tmp2->isVariable( varType[run1], component[run1] ) == BT_FALSE ){

             ACADOERROR(RET_INVALID_ARGUMENTS);
             delete   tmp2     ;
             delete[] varType  ;
             delete[] component;
             return   BT_TRUE  ;
        }

        if( varType[run1] == VT_INTERMEDIATE_STATE ){

             ACADOERROR(RET_INVALID_ARGUMENTS);
             delete   tmp2     ;
             delete[] varType  ;
             delete[] component;
             return   BT_TRUE  ;
        }
        delete tmp2;
    }


    for( run1 = 0; run1 < n; run1++ ){
        implicit_dep[run1] = sub[run1]->isLinearIn( 1, varType, component, implicit_dep );
    }
    for( run1 = 0; run1 < dim; run1++ ){
        if( f[run1]->isLinearIn( 1, varType, component, implicit_dep ) == BT_FALSE  ){
            delete[] implicit_dep;
            delete[] varType  ;
            delete[] component;
            return BT_FALSE;
        }
    }

    delete[] implicit_dep;
    delete[] varType  ;
    delete[] component;
    return   BT_TRUE;
}


BooleanType FunctionEvaluationTree::isPolynomialIn( const Expression &variable ){

    int nn = variable.getDim();

    int                 run1;
    BooleanType *implicit_dep = new BooleanType [n ];
    VariableType *varType     = new VariableType[nn];
    int          *component   = new int         [nn];

    for( run1 = 0; run1 < nn; run1++ ){

        Operator *tmp2 = (variable.element[run1])->clone();

        if( tmp2->isVariable( varType[run1], component[run1] ) == BT_FALSE ){

             ACADOERROR(RET_INVALID_ARGUMENTS);
             delete   tmp2     ;
             delete[] varType  ;
             delete[] component;
             return   BT_TRUE  ;
        }

        if( varType[run1] == VT_INTERMEDIATE_STATE ){

             ACADOERROR(RET_INVALID_ARGUMENTS);
             delete   tmp2     ;
             delete[] varType  ;
             delete[] component;
             return   BT_TRUE  ;
        }
        delete tmp2;
    }


    for( run1 = 0; run1 < n; run1++ ){
        implicit_dep[run1] = sub[run1]->isPolynomialIn( 1, varType, component, implicit_dep );
    }
    for( run1 = 0; run1 < dim; run1++ ){
        if( f[run1]->isPolynomialIn( 1, varType, component, implicit_dep ) == BT_FALSE  ){
            delete[] implicit_dep;
            delete[] varType  ;
            delete[] component;
            return BT_FALSE;
        }
    }

    delete[] implicit_dep;
    delete[] varType  ;
    delete[] component;
    return   BT_TRUE;
}



BooleanType FunctionEvaluationTree::isRationalIn( const Expression &variable ){

    int nn = variable.getDim();

    int                 run1;
    BooleanType *implicit_dep = new BooleanType [n ];
    VariableType *varType     = new VariableType[nn];
    int          *component   = new int         [nn];

    for( run1 = 0; run1 < nn; run1++ ){

        Operator *tmp2 = (variable.element[run1])->clone();

        if( tmp2->isVariable( varType[run1], component[run1] ) == BT_FALSE ){

             ACADOERROR(RET_INVALID_ARGUMENTS);
             delete   tmp2     ;
             delete[] varType  ;
             delete[] component;
             return   BT_TRUE  ;
        }

        if( varType[run1] == VT_INTERMEDIATE_STATE ){

             ACADOERROR(RET_INVALID_ARGUMENTS);
             delete   tmp2     ;
             delete[] varType  ;
             delete[] component;
             return   BT_TRUE  ;
        }
        delete tmp2;
    }


    for( run1 = 0; run1 < n; run1++ ){
        implicit_dep[run1] = sub[run1]->isRationalIn( 1, varType, component, implicit_dep );
    }
    for( run1 = 0; run1 < dim; run1++ ){
        if( f[run1]->isRationalIn( 1, varType, component, implicit_dep ) == BT_FALSE  ){
            delete[] implicit_dep;
            delete[] varType  ;
            delete[] component;
            return BT_FALSE;
        }
    }

    delete[] implicit_dep;
    delete[] varType  ;
    delete[] component;
    return   BT_TRUE;
}


MonotonicityType FunctionEvaluationTree::getMonotonicity( ){

    int run1;
    MonotonicityType m = MT_CONSTANT;

    for( run1 = 0; run1 < dim; run1++ ){

        MonotonicityType mf = f[run1]->getMonotonicity();

        if( mf == MT_NONDECREASING ){

            if( m == MT_CONSTANT      )  m = MT_NONDECREASING;
            if( m == MT_NONINCREASING )  return MT_NONMONOTONIC;
        }

        if( mf == MT_NONINCREASING ){

            if( m == MT_CONSTANT      )  m = MT_NONINCREASING;
            if( m == MT_NONDECREASING )  return MT_NONMONOTONIC;
        }

        if( mf == MT_NONMONOTONIC ) return MT_NONMONOTONIC;
    }
    return m;
}



CurvatureType FunctionEvaluationTree::getCurvature( ){

    int run1;
    CurvatureType m = CT_CONSTANT;

    for( run1 = 0; run1 < dim; run1++ ){

        CurvatureType mf = f[run1]->getCurvature();

        if( mf == CT_AFFINE ){

            if( m == CT_CONSTANT )  m = CT_AFFINE;
        }

        if( mf == CT_CONVEX ){

            if( m == CT_CONSTANT      )  m = CT_CONVEX;
            if( m == CT_AFFINE        )  m = CT_CONVEX;
            if( m == CT_CONCAVE       )  return CT_NEITHER_CONVEX_NOR_CONCAVE;
        }

        if( mf == CT_CONCAVE ){

            if( m == CT_CONSTANT      )  m = CT_CONCAVE;
            if( m == CT_AFFINE        )  m = CT_CONCAVE;
            if( m == CT_CONVEX        )  return CT_NEITHER_CONVEX_NOR_CONCAVE;
        }

        if( mf == CT_NEITHER_CONVEX_NOR_CONCAVE ) return CT_NEITHER_CONVEX_NOR_CONCAVE;
    }
    return m;
}



returnValue FunctionEvaluationTree::AD_forward( double *x, double *seed, double *ff,
                                            double *df  ){

    int run1;

    for( run1 = 0; run1 < n; run1++ ){
        sub[run1]->AD_forward( 0, x, seed,
                         &x   [ indexList->index(VT_INTERMEDIATE_STATE, lhs_comp[run1])],
                         &seed[ indexList->index(VT_INTERMEDIATE_STATE, lhs_comp[run1])] );
    }
    for( run1 = 0; run1 < dim; run1++ ){
        f[run1]->AD_forward( 0, x, seed, &ff[run1], &df[run1] );
    }

    return SUCCESSFUL_RETURN;
}



returnValue FunctionEvaluationTree::AD_forward( int number, double *x, double *seed,
                                            double *ff, double *df  ){

    int run1;

    for( run1 = 0; run1 < n; run1++ ){
        sub[run1]->AD_forward( number, x, seed,
                         &x   [ indexList->index(VT_INTERMEDIATE_STATE, lhs_comp[run1])],
                         &seed[ indexList->index(VT_INTERMEDIATE_STATE, lhs_comp[run1])] );
    }
    for( run1 = 0; run1 < dim; run1++ ){
        f[run1]->AD_forward( number, x, seed, &ff[run1], &df[run1] );
    }

    return SUCCESSFUL_RETURN;
}


returnValue FunctionEvaluationTree::AD_forward( int number, double *seed, double *df  ){

    int run1;

    for( run1 = 0; run1 < n; run1++ ){
        sub[run1]->AD_forward( number, seed,
                         &seed[ indexList->index(VT_INTERMEDIATE_STATE, lhs_comp[run1])] );
    }
    for( run1 = 0; run1 < dim; run1++ ){
        f[run1]->AD_forward( number, seed, &df[run1] );
    }

    return SUCCESSFUL_RETURN;
}


returnValue FunctionEvaluationTree::AD_backward( double *seed, double  *df ){

    int run1;

    for( run1 = dim-1; run1 >= 0; run1-- ){
        f[run1]->AD_backward( 0, seed[run1], df );
    }


    for( run1 = n-1; run1 >= 0; run1-- ){

      sub[run1]->AD_backward( 0, df[ indexList->index(VT_INTERMEDIATE_STATE, lhs_comp[run1])],
                              df
                            );
    }

    return SUCCESSFUL_RETURN;
}



returnValue FunctionEvaluationTree::AD_backward( int number, double *seed, double  *df ){

    int run1;

    for( run1 = dim-1; run1 >= 0; run1-- ){
        f[run1]->AD_backward( number, seed[run1], df );
    }

    for( run1 = n-1; run1 >= 0; run1-- ){
      sub[run1]->AD_backward( number,
                              df[ indexList->index(VT_INTERMEDIATE_STATE, lhs_comp[run1])],
                              df );
    }

    return SUCCESSFUL_RETURN;
}


returnValue FunctionEvaluationTree::AD_forward2( int number, double *seed,
                                             double *dseed, double *df,
                                             double *ddf ){

    int run1;

    for( run1 = 0; run1 < n; run1++ ){
        sub[run1]->AD_forward2( number, seed, dseed,
                         &seed [ indexList->index(VT_INTERMEDIATE_STATE, lhs_comp[run1])],
                         &dseed[ indexList->index(VT_INTERMEDIATE_STATE, lhs_comp[run1])] );
    }
    for( run1 = 0; run1 < dim; run1++ ){
        f[run1]->AD_forward2( number, seed, dseed, &df[run1], &ddf[run1] );
    }

    return SUCCESSFUL_RETURN;
}


returnValue FunctionEvaluationTree::AD_backward2( int number, double *seed1, double *seed2,
                                              double *df, double  *ddf ){

    int run1;

    for( run1 = dim-1; run1 >= 0; run1-- ){
        f[run1]->AD_backward2( number, seed1[run1], seed2[run1], df, ddf );
    }


    for( run1 = n-1; run1 >= 0; run1-- ){
      sub[run1]->AD_backward2( number,
                              df  [ indexList->index(VT_INTERMEDIATE_STATE, lhs_comp[run1])],
                              ddf [ indexList->index(VT_INTERMEDIATE_STATE, lhs_comp[run1])],
                              df,
                              ddf
                            );
    }

    return SUCCESSFUL_RETURN;
}



returnValue FunctionEvaluationTree::C_print( 	FILE       *file     ,
												const char *fcnName  ,
												const char *realString,
												int         precision
												) const{

    acadoFPrintf(file,"/*\n");
    acadoFPrintf(file," *    This file was auto-generated by ACADO Toolkit.\n");
    acadoFPrintf(file," *\n");
    acadoFPrintf(file," *    ACADO Toolkit -- A Toolkit for Automatic Control and Dynamic Optimization.\n");
    acadoFPrintf(file," *    Copyright (C) 2008-2011 by Boris Houska and Hans Joachim Ferreau, K.U.Leuven.\n");
    acadoFPrintf(file," *    Developed within the Optimization in Engineering Center (OPTEC) under\n");
    acadoFPrintf(file," *    supervision of Moritz Diehl. All rights reserved.\n");
    acadoFPrintf(file," *\n");
    acadoFPrintf(file," */\n");

    exportHeader             ( file,fcnName,realString );
    exportForwardDeclarations( file,fcnName,realString );
    exportCode               ( file,fcnName,realString,precision );

    return SUCCESSFUL_RETURN;
}




returnValue FunctionEvaluationTree::exportHeader(	FILE *file,
													const char *fcnName,
													const char *realString
													) const{

    if( n > 0 )
    	acadoFPrintf(file,"%s %s[%d];\n", realString, globalExportVariableName.getName(), n );

    return SUCCESSFUL_RETURN;
}


returnValue FunctionEvaluationTree::exportForwardDeclarations(	FILE *file,
																const char *fcnName,
																const char *realString
																) const
{
	acadoFPrintf(file,
			"\n"
			"/** Export of an ACADO symbolic function.\n"
			" *\n"
			" *  \\param in Input to the exported function.\n"
			" *  \\param out Output of the exported function.\n"
			" */\n"
	);
	acadoFPrintf(file, "void %s(const %s* in, %s* out);\n", fcnName, realString, realString);

    return SUCCESSFUL_RETURN;
}


returnValue FunctionEvaluationTree::exportCode(	FILE *file,
												const char *fcnName,
												const char *realString,
												int         precision,
												uint        _numX,
												uint		_numXA,
												uint		_numU,
												uint		_numP,
												uint		_numDX
												) const{

    int run1;
    int nni = 0;

	for (run1 = 0; run1 < n; run1++)
		if (lhs_comp[run1] + 1 > nni)
			nni = lhs_comp[run1] + 1;
	
	unsigned numX = _numX > 0 ? _numX : getNX();
	unsigned numXA = _numXA > 0 ? _numXA : getNXA();
	unsigned numU = _numU > 0 ? _numU : getNU();
	unsigned numP = _numP > 0 ? _numP : getNP();
	unsigned numDX = _numDX > 0 ? _numDX : getNDX();

	acadoFPrintf(file, "void %s(const %s* in, %s* out)\n{\n", fcnName,
			realString, realString);
    if( numX > 0 ){
        acadoFPrintf(file,"const %s* xd = in;\n", realString );
    }
    if( numXA > 0 ){
        acadoFPrintf(file,"const %s* xa = in + %d;\n", realString,numX );
    }
    if( getNU() > 0 ){
        acadoFPrintf(file,"const %s* u  = in + %d;\n", realString,numX+numXA );
    }
    if( getNUI() > 0 ){
        acadoFPrintf(file,"const %s* v  = in + %d;\n", realString,numX+numXA+numU );
    }
    if( numP > 0 ){
        acadoFPrintf(file,"const %s* p  = in + %d;\n", realString,numX+numXA+numU+getNUI() );
    }
    if( getNPI() > 0 ){
        acadoFPrintf(file,"const %s* q  = in + %d;\n", realString,numX+numXA+numU+getNUI()+numP );
    }
    if( getNW() > 0 ){
        acadoFPrintf(file,"const %s* w  = in + %d;\n", realString,numX+numXA+numU+getNUI()+numP+getNPI() );
    }
    if( numDX > 0 ){
        acadoFPrintf(file,"const %s* dx = in + %d;\n", realString,numX+numXA+numU+getNUI()+numP+getNPI()+getNW() );
    }
    if( getNT() > 0 ){
        acadoFPrintf(file,"const %s* t = in + %d;\n", realString,numX+numXA+numU+getNUI()+numP+getNPI()+getNW()+numDX );
    }
    if (n > 0)
    {
    	acadoFPrintf(file, "/* Vector of auxiliary variables; number of elements: %d. */\n", n);
    	acadoFPrintf(file, "%s* a = %s;\n", realString, globalExportVariableName.getName() );
    	acadoFPrintf(file, "\n/* Compute intermediate quantities; */\n");
    }

    Stream *auxVarIndividualNames = new Stream[nni];
	for( run1 = 0; run1 < n; run1++ )
		auxVarIndividualNames[lhs_comp[run1]] << "a" << "[" << run1 << "]";

	// Export intermediate quantities
	for (run1 = 0; run1 < n; run1++)
	{
		// Convert the name for intermediate variables for subexpressions
		sub[run1]->setVariableExportName(VT_INTERMEDIATE_STATE, auxVarIndividualNames);

		acadoFPrintf(file, "%s[%d] = ", "a", run1);
		file << *sub[run1];
		acadoFPrintf(file, ";\n");
	}

    acadoFPrintf(file,"\n/* Compute outputs: */\n");

	// Export output quantities
	for (run1 = 0; run1 < dim; run1++)
	{
		// Convert names for interm. quantities for output expressions
		f[run1]->setVariableExportName(VT_INTERMEDIATE_STATE, auxVarIndividualNames);

		acadoFPrintf(file, "out[%d] = ", run1);
		file << *f[run1];
		acadoFPrintf(file, ";\n");
	}

    acadoFPrintf(file,"}\n\n");

    delete [] auxVarIndividualNames;

    return SUCCESSFUL_RETURN;
}


returnValue FunctionEvaluationTree::clearBuffer(){

    int run1;
    returnValue returnvalue;

    for( run1 = 0; run1 < n; run1++ ){
        returnvalue = sub[run1]->clearBuffer();
        if( returnvalue != SUCCESSFUL_RETURN ){
            return returnvalue;
        }
    }
    for( run1 = 0; run1 < dim; run1++ ){
        returnvalue = f[run1]->clearBuffer();
        if( returnvalue != SUCCESSFUL_RETURN ){
            return returnvalue;
        }
    }

    return SUCCESSFUL_RETURN;
}


returnValue FunctionEvaluationTree::makeImplicit(){

    return makeImplicit(dim);
}


returnValue FunctionEvaluationTree::makeImplicit( int dim_ ){

    int run1;
    int var_counter = indexList->makeImplicit(dim_);

    for( run1 = 0; run1 < dim_; run1++ ){

        Operator *tmp = f[run1]->clone();
        delete f[run1];
        Projection pp;
        pp.variableType   = VT_DDIFFERENTIAL_STATE;
        pp.vIndex         = run1                  ;
        pp.variableIndex  = var_counter-dim_+run1  ;

        f[run1] = new Subtraction( pp.clone(), tmp->clone() );
        delete tmp;
    }

    return SUCCESSFUL_RETURN;
}


int FunctionEvaluationTree::getNX   () const{

    return indexList->getNX();
}

int FunctionEvaluationTree::getNXA   () const{

    return indexList->getNXA();
}

int FunctionEvaluationTree::getNDX   () const{

    return indexList->getNDX();
}

int FunctionEvaluationTree::getNU   () const{

    return indexList->getNU();
}

int FunctionEvaluationTree::getNUI   () const{

    return indexList->getNUI();
}

int FunctionEvaluationTree::getNP   () const{

    return indexList->getNP();
}

int FunctionEvaluationTree::getNPI   () const{

    return indexList->getNPI();
}

int FunctionEvaluationTree::getNW   () const{

    return indexList->getNW();
}

int FunctionEvaluationTree::getNT   () const{

    return indexList->getNT();
}

int FunctionEvaluationTree::index( VariableType variableType_, int index_ ) const{

    return indexList->index( variableType_, index_ );
}

double FunctionEvaluationTree::scale( VariableType variableType_, int index_ ) const{

    return indexList->scale( variableType_, index_ );
}


int FunctionEvaluationTree::getNumberOfVariables() const{

    return indexList->getNumberOfVariables();
}


Operator* FunctionEvaluationTree::getExpression(	uint componentIdx
												) const
{
	if ( (int)componentIdx < getDim( ) )
		return f[componentIdx]->clone( );
	else
		return new DoubleConstant( 0.0,NE_ZERO );
}


BooleanType FunctionEvaluationTree::isSymbolic() const{

    int run1;
    for( run1 = 0; run1 < n; run1++ ){
        if( sub[run1]->isSymbolic() == BT_FALSE ) return BT_FALSE;
    }
    for( run1 = 0; run1 < dim; run1++ ){
        if( f[run1]->isSymbolic() == BT_FALSE ) return BT_FALSE;
    }
    return BT_TRUE;
}


returnValue FunctionEvaluationTree::setScale( double *scale_ )
{
    return ACADOERROR(RET_INVALID_USE_OF_FUNCTION);
}

returnValue FunctionEvaluationTree::setGlobalExportVariableName(const String& _name)
{
	if (_name.getLength() == 0)
		return ACADOERROR( RET_INVALID_ARGUMENTS );

	globalExportVariableName = _name;

	return SUCCESSFUL_RETURN;
}

String FunctionEvaluationTree::getGlobalExportVariableName() const
{
	return globalExportVariableName;
}

unsigned FunctionEvaluationTree::getGlobalExportVariableSize() const
{
	return n;
}

CLOSE_NAMESPACE_ACADO

// end of file.
