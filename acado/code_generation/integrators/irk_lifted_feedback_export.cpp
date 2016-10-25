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
 *    \file src/code_generation/integrators/irk_lifted_feedback_export.cpp
 *    \author Rien Quirynen
 *    \date 2016
 */

#include <math.h>
#include <acado/code_generation/integrators/irk_export.hpp>
#include <acado/code_generation/integrators/irk_lifted_feedback_export.hpp>

using namespace std;

BEGIN_NAMESPACE_ACADO

//
// PUBLIC MEMBER FUNCTIONS:
//

FeedbackLiftedIRKExport::FeedbackLiftedIRKExport(   UserInteraction* _userInteraction,
                                    const std::string& _commonHeaderName
                                    ) : ForwardIRKExport( _userInteraction,_commonHeaderName )
{
    NF = 0;
}

FeedbackLiftedIRKExport::FeedbackLiftedIRKExport( const FeedbackLiftedIRKExport& arg ) : ForwardIRKExport( arg )
{
    NF = 0;
}


FeedbackLiftedIRKExport::~FeedbackLiftedIRKExport( )
{
    if ( solver )
        delete solver;
    solver = 0;

    clear( );
}


FeedbackLiftedIRKExport& FeedbackLiftedIRKExport::operator=( const FeedbackLiftedIRKExport& arg ){

    if( this != &arg ){

        ForwardIRKExport::operator=( arg );
        copy( arg );
    }
    return *this;
}


ExportVariable FeedbackLiftedIRKExport::getAuxVariable() const
{
    ExportVariable max;
    if( NX1 > 0 ) {
        max = lin_input.getGlobalExportVariable();
    }
    if( NF > 0 ) {
        if( feedb.getGlobalExportVariable().getDim() >= max.getDim() ) {
            max = feedb.getGlobalExportVariable();
        }
        if( sens_input.getGlobalExportVariable().getDim() >= max.getDim() ) {
            max = sens_input.getGlobalExportVariable();
        }
        if( sens_fdb.getGlobalExportVariable().getDim() >= max.getDim() ) {
            max = sens_fdb.getGlobalExportVariable();
        }
    }
    return max;
}


returnValue FeedbackLiftedIRKExport::getDataDeclarations(   ExportStatementBlock& declarations,
                                                ExportStruct dataStruct
                                                ) const
{
    ForwardIRKExport::getDataDeclarations( declarations, dataStruct );

    declarations.addDeclaration( rk_seed,dataStruct );
    declarations.addDeclaration( rk_stageValues,dataStruct );

    declarations.addDeclaration( rk_Xprev,dataStruct );
    declarations.addDeclaration( rk_Uprev,dataStruct );
    declarations.addDeclaration( rk_delta,dataStruct );

    declarations.addDeclaration( rk_kTemp,dataStruct );

    declarations.addDeclaration( rk_dk1_tmp,dataStruct );
    declarations.addDeclaration( rk_dk2_tmp,dataStruct );

    declarations.addDeclaration( rk_sensF,dataStruct );

    return SUCCESSFUL_RETURN;
}


returnValue FeedbackLiftedIRKExport::getFunctionDeclarations(   ExportStatementBlock& declarations
                                                    ) const
{
    ForwardIRKExport::getFunctionDeclarations( declarations );
    declarations.addDeclaration( lin_input );
    declarations.addDeclaration( feedb );
    declarations.addDeclaration( sens_input );
    declarations.addDeclaration( sens_fdb );

    return SUCCESSFUL_RETURN;
}


returnValue FeedbackLiftedIRKExport::setNonlinearFeedback( const DMatrix& C, const Expression& feedb_ )
{
    LOG( LVL_DEBUG ) << "Integrator: setNonlinearFeedback... " << endl;
    if( C.isEmpty() || C.getNumCols() != feedb_.getDim() || C.getNumRows() != NX || NX != NX1 ) return ACADOERROR( RET_INVALID_OPTION );

    NF = C.getNumCols();
    C11 = C;

    OutputFcn feedb_expr;
    feedb_expr << feedb_;

    if( feedb_expr.getNP() > 0 || feedb_expr.getNXA() > 0 ) return RET_NOT_IMPLEMENTED_YET;

    setInputSystem( );

    NDX2 = feedb_expr.getNDX();
    if( NDX2 > 0 ) NDX2 = NX;

    OnlineData        dummy0;
    Control           dummy1;
    DifferentialState dummy2;
    dummy0.clearStaticCounters();
    dummy1.clearStaticCounters();
    dummy2.clearStaticCounters();
    x = DifferentialState("", NX, 1);
    u = Control("", NU, 1);
    od = OnlineData("", NOD, 1);

    DifferentialStateDerivative dummy4;
    dummy4.clearStaticCounters();

    dx = DifferentialStateDerivative("", NX, 1);

    DifferentialState f("", NF, 1);

    DifferentialEquation fun_input;
    fun_input << A11*x+B11*u+C11*f;
//    fun_input << A11*x+B11*u+C11*f - M11*dx;
    lin_input.init(fun_input, "acado_linear_input", NX+NF, NXA, NU, 0, 0, NOD);

    DifferentialState sX("", NX,NX+NU);
    DifferentialState A_row("", numStages, 1);
    Expression Gx = sX.getCols(0,NX);
    Expression Gu = sX.getCols(NX,NX+NU);

    Expression GKf;
    Expression GKx;
    Expression GKu;

    if( NDX2 > 0 ) {
        DifferentialState sKX("", NDX2,NX+NU);
        GKf = DifferentialState("", NDX2,numStages*NF);
        GKx = sKX.getCols(0,NX);
        GKu = sKX.getCols(NX,NX+NU);
    }


    Expression Gf(NX,numStages*NF);
    for( uint j = 0; j < numStages; j++ ) {
        Expression tmp = zeros<double>(NX,numStages*NF);
        for( uint k1 = 0; k1 < NX; k1++ ) {
            for( uint k2 = 0; k2 < numStages*NF; k2++ ) {
                tmp(k1,k2) += sensMat(j*NX+k1,k2)*A_row.getRow(j);
            }
        }
        Gf = Gf + tmp;
    }

    OutputFcn fdb_;
    for( uint i = 0; i < feedb_.getDim(); i++ ) {

        if( NDX2 > 0 ) {
            fdb_ << feedb_(i);
            fdb_ << multipleForwardDerivative( feedb_(i), x, Gx ) + multipleForwardDerivative( feedb_(i), dx, GKx );
            fdb_ << multipleForwardDerivative( feedb_(i), x, Gu ) + forwardDerivative( feedb_(i), u ) + multipleForwardDerivative( feedb_(i), dx, GKu );
            fdb_ << multipleForwardDerivative( feedb_(i), x, Gf ) + multipleForwardDerivative( feedb_(i), dx, GKf );
        }
        else {
            fdb_ << feedb_(i);
            fdb_ << multipleForwardDerivative( feedb_(i), x, Gx );
            fdb_ << multipleForwardDerivative( feedb_(i), x, Gu ) + forwardDerivative( feedb_(i), u );
            fdb_ << multipleForwardDerivative( feedb_(i), x, Gf );
        }
    }
    LOG( LVL_DEBUG ) << "done!" << endl;

    if( NDX2 == 0 ) {
        return feedb.init(fdb_, "acado_feedback", NX+NF+NX*(NX+NU)+numStages, NXA, NU, NP, NX, NOD);
    } else {
        return feedb.init(fdb_, "acado_feedback", NX+NF+NX*(NX+NU)+numStages+NDX2*(NX+NU+numStages*NF), NXA, NU, NP, NX, NOD);
    }
}


returnValue FeedbackLiftedIRKExport::setInputSystem( )
{
    LOG( LVL_DEBUG ) << "Integrator: setInputSystem... " << endl;
    if( NX1 > 0 ) {
        mat1 = formMatrix( M11, A11 );

        uint i, j;
        DMatrix Amat = A11;
        for( j = 1; j < numStages; j++ ) {
            Amat.appendRows(A11);
        }
        DMatrix sol = mat1*Amat;
        sensMat = sol;

        DMatrix Bmat = B11;
        for( j = 1; j < numStages; j++ ) {
            Bmat.appendRows(B11);
        }
        DMatrix solb = mat1*Bmat;
        sensMat.appendCols(solb);

        for( j = 0; j < numStages; j++ ) {
            DMatrix solc = mat1.getCols(j*NX1,(j+1)*NX1-1)*C11;
            sensMat.appendCols(solc);
        }

//        // sensMat: numStages*NX x NX+NU+numStages*NF
//        sensMat.
        for( i = 0; i < numStages*NX; i++ ) {
            for( j = 0; j < NX+NU+numStages*NF; j++ ) {
                if( fabs(sensMat(i,j)) < 1e-9 ) sensMat(i,j) = 0.0;
            }
        }

        // SENS_INPUT:
        DifferentialState dummy;
        dummy.clearStaticCounters();

        DifferentialState sX("", NX,NX+NU);

        DMatrix u_mat = zeros<double>(numStages*NX,NX);
        u_mat.appendCols(sensMat.getCols(NX,NX+NU-1));
        OutputFcn sens_input_;
        sens_input_ << sensMat.getCols(0,NX-1)*sX+u_mat;

        sens_input.init(sens_input_, "acado_sens_input", NX*(NX+NU), NXA, NU);

        // SENS_FEEDBACK:
        DifferentialState dummy2;
        dummy2.clearStaticCounters();

        DifferentialState sF("", numStages*NF,NX+NU);

        OutputFcn sens_fdb_;
        sens_fdb_ << sensMat.getCols(NX+NU,NX+NU+numStages*NF-1)*sF;

        sens_fdb.init(sens_fdb_, "acado_sens_fdb", numStages*NF*(NX+NU), NXA, NU);

        sensMat = sensMat.getCols(NX+NU,NX+NU+numStages*NF-1);
    }
    LOG( LVL_DEBUG ) << "done!" << endl;

    return SUCCESSFUL_RETURN;
}


returnValue FeedbackLiftedIRKExport::prepareInputSystem( ExportStatementBlock& code )
{
    LOG( LVL_DEBUG ) << "Integrator: prepareInputSystem... " << endl;
    if( NX1 > 0 ) {
        DMatrix mat1 = formMatrix( M11, A11 );
//      rk_mat1 = ExportVariable( "rk_mat1", mat1, STATIC_CONST_REAL );
//        code.addDeclaration( rk_mat1 );
//      // TODO: Ask Milan why this does NOT work properly !!
//      rk_mat1 = ExportVariable( "rk_mat1", numStages*NX1, numStages*NX1, STATIC_CONST_REAL, ACADO_LOCAL );

        uint j;
        DMatrix Amat = A11;
        for( j = 1; j < numStages; j++ ) {
            Amat.appendRows(A11);
        }
        DMatrix sol = mat1*Amat;
        DMatrix sens = sol;

        DMatrix Bmat = B11;
        for( j = 1; j < numStages; j++ ) {
            Bmat.appendRows(B11);
        }
        DMatrix solb = mat1*Bmat;
        sens.appendCols(solb);

        for( j = 0; j < numStages; j++ ) {
            DMatrix solc = mat1.getCols(j*NX1,(j+1)*NX1-1)*C11;
            sens.appendCols(solc);
        }
        if( NDX2 > 0 ) {
            rk_dk1 = ExportVariable( "rk_dk1", sens, STATIC_CONST_REAL );
            code.addDeclaration( rk_dk1 );
            // TODO: Ask Milan why this does NOT work properly !!
            rk_dk1 = ExportVariable( "rk_dk1", numStages*NX1, NX1+NU+numStages*NF, STATIC_CONST_REAL, ACADO_LOCAL );
        }
    }
    LOG( LVL_DEBUG ) << "done!" << endl;

    return SUCCESSFUL_RETURN;
}


returnValue FeedbackLiftedIRKExport::getCode(   ExportStatementBlock& code )
{
    int sensGen;
    get( DYNAMIC_SENSITIVITY, sensGen );
    int mode;
    get( IMPLICIT_INTEGRATOR_MODE, mode );
//  int liftMode;
//  get( LIFTED_INTEGRATOR_MODE, liftMode );
    if ( (ExportSensitivityType)sensGen != FORWARD && (ExportSensitivityType)sensGen != INEXACT ) ACADOERROR( RET_INVALID_OPTION );
    if( (ImplicitIntegratorMode)mode != LIFTED_FEEDBACK ) ACADOERROR( RET_INVALID_OPTION );
//  if( liftMode != 1 && liftMode != 4 ) ACADOERROR( RET_NOT_IMPLEMENTED_YET );
//  if( (ExportSensitivityType)sensGen == INEXACT && liftMode != 4 ) ACADOERROR( RET_INVALID_OPTION );

    int liftMode = 1;
    // liftMode == 1 --> EXACT LIFTING
    // liftMode == 2 --> INEXACT LIFTING
    if( (ExportSensitivityType)sensGen == INEXACT ) liftMode = 2;

    if( CONTINUOUS_OUTPUT || NX2 > 0 || NX3 > 0 || NXA > 0 || !equidistantControlGrid() ) ACADOERROR( RET_NOT_IMPLEMENTED_YET );

    int useOMP;
    get(CG_USE_OPENMP, useOMP);
    if ( useOMP ) {
        ACADOERROR( RET_NOT_IMPLEMENTED_YET );
    }

    if( NX1 == 0 || NF == 0 ) return ACADOERROR( RET_INVALID_OPTION );

    code.addFunction( lin_input );
    code.addStatement( "\n\n" );
    code.addFunction( feedb );
    code.addStatement( "\n\n" );

    solver->getCode( code );
    code.addLinebreak(2);

    // export RK scheme
    uint run5, run6, run7;
    std::string tempString;

    initializeDDMatrix();
    initializeCoefficients();

    double h = (grid.getLastTime() - grid.getFirstTime())/grid.getNumIntervals();
    DMatrix tmp = AA;
    ExportVariable Ah( "Ah_mat", tmp*=h, STATIC_CONST_REAL );
    code.addDeclaration( Ah );
    code.addLinebreak( 2 );
    // TODO: Ask Milan why this does NOT work properly !!
    Ah = ExportVariable( "Ah_mat", numStages, numStages, STATIC_CONST_REAL, ACADO_LOCAL );

    DVector BB( bb );
    ExportVariable Bh( "Bh_mat", DMatrix( BB*=h ) );

    DVector CC( cc );
    ExportVariable C;
    if( timeDependant ) {
        C = ExportVariable( "C_mat", DMatrix( CC*=(1.0/grid.getNumIntervals()) ), STATIC_CONST_REAL );
        code.addDeclaration( C );
        code.addLinebreak( 2 );
        C = ExportVariable( "C_mat", 1, numStages, STATIC_CONST_REAL, ACADO_LOCAL );
    }

    code.addComment(std::string("Fixed step size:") + toString(h));

    ExportVariable determinant( "det", 1, 1, REAL, ACADO_LOCAL, true );
    integrate.addDeclaration( determinant );

    ExportIndex i( "i" );
    ExportIndex j( "j" );
    ExportIndex k( "k" );
    ExportIndex run( "run" );
    ExportIndex run1( "run1" );
    ExportIndex tmp_index1("tmp_index1");
    ExportIndex tmp_index2("tmp_index2");
    ExportIndex tmp_index3("tmp_index3");
    ExportIndex k_index("k_index");
    ExportIndex shooting_index("shoot_index");
    ExportVariable tmp_meas("tmp_meas", 1, outputGrids.size(), INT, ACADO_LOCAL);

    ExportVariable numInt( "numInts", 1, 1, INT );
    if( !equidistantControlGrid() ) {
        ExportVariable numStepsV( "numSteps", numSteps, STATIC_CONST_INT );
        code.addDeclaration( numStepsV );
        code.addLinebreak( 2 );
        integrate.addStatement( std::string( "int " ) + numInt.getName() + " = " + numStepsV.getName() + "[" + rk_index.getName() + "];\n" );
    }

    prepareOutputEvaluation( code );

    integrate.addIndex( i );
    integrate.addIndex( j );
    integrate.addIndex( k );
    integrate.addIndex( run );
    integrate.addIndex( run1 );
    integrate.addIndex( tmp_index1 );
    integrate.addIndex( tmp_index2 );
    integrate.addIndex( tmp_index3 );
    integrate.addIndex( shooting_index );
    integrate.addIndex( k_index );
    ExportVariable time_tmp( "time_tmp", 1, 1, REAL, ACADO_LOCAL, true );
    integrate << shooting_index.getFullName() << " = " << rk_index.getFullName() << ";\n";
    integrate.addStatement( rk_ttt == DMatrix(grid.getFirstTime()) );
    if( NU > 0 || NOD > 0 ) {
        integrate.addStatement( rk_xxx.getCols( NX+NF,NX+NF+NU+NOD ) == rk_eta.getCols( NX+NXA+diffsDim,inputDim ) );
        integrate.addStatement( rk_seed.getCols( NX+NF+(NX+NDX2)*(NX+NU)+numStages+NDX2*(numStages*NF),NX+NF+(NX+NDX2)*(NX+NU)+numStages+NDX2*(numStages*NF)+NU+NOD ) == rk_eta.getCols( NX+NXA+diffsDim,inputDim ) );
    }
    integrate.addLinebreak( );
//  if( liftMode == 1 || liftMode == 4 ) {
        integrate.addStatement( rk_delta.getCols( 0,NX ) == rk_eta.getCols( 0,NX ) - rk_Xprev.getRow(shooting_index) );
        integrate.addStatement( rk_Xprev.getRow(shooting_index) == rk_eta.getCols( 0,NX ) );

        integrate.addStatement( rk_delta.getCols( NX,NX+NU ) == rk_eta.getCols( NX+NXA+diffsDim,NX+NXA+diffsDim+NU ) - rk_Uprev.getRow(shooting_index) );
        integrate.addStatement( rk_Uprev.getRow(shooting_index) == rk_eta.getCols( NX+NXA+diffsDim,NX+NXA+diffsDim+NU ) );
//  }

    // integrator loop:
    ExportForLoop tmpLoop( run, 0, grid.getNumIntervals() );
    ExportStatementBlock *loop;
    if( equidistantControlGrid() ) {
        loop = &tmpLoop;
    }
    else {
        loop = &integrate;
        loop->addStatement( std::string("for(") + run.getName() + " = 0; " + run.getName() + " < " + numInt.getName() + "; " + run.getName() + "++ ) {\n" );
    }

    //  if( grid.getNumIntervals() > 1 || !equidistantControlGrid() ) {
    // Set rk_diffsPrev:
    loop->addStatement( std::string("if( run > 0 ) {\n") );
    if( NX > 0 ) {
        ExportForLoop loopTemp1( i,0,NX );
        loopTemp1.addStatement( rk_diffsPrev1.getSubMatrix( i,i+1,0,NX ) == rk_eta.getCols( i*NX+NX+NXA,i*NX+NX+NXA+NX ) );
        if( NU > 0 ) loopTemp1.addStatement( rk_diffsPrev1.getSubMatrix( i,i+1,NX,NX+NU ) == rk_eta.getCols( i*NU+(NX+NXA)*(NX+1),i*NU+(NX+NXA)*(NX+1)+NU ) );
        loop->addStatement( loopTemp1 );
    }
    loop->addStatement( std::string("}\nelse{\n") );
    DMatrix eyeM = eye<double>(NX);
    eyeM.appendCols(zeros<double>(NX,NU));
    loop->addStatement( rk_diffsPrev1 == eyeM );
    loop->addStatement( std::string("}\n") );
    //  }

    loop->addStatement( k_index == (shooting_index*grid.getNumIntervals()+run)*NF );

    // FIRST update using term from optimization variables:
//  if( liftMode == 1 || (liftMode == 4 && (ExportSensitivityType)sensGen == INEXACT) ) {
        ExportForLoop loopTemp1( i,0,NF );
        loopTemp1.addStatement( j == k_index+i );
        loopTemp1.addStatement( tmp_index1 == j*(NX+NU) );
        ExportForLoop loopTemp2( run1,0,numStages );
        loopTemp2.addStatement( rk_kkk.getElement( j,run1 ) += rk_delta*rk_diffK.getSubMatrix( tmp_index1,tmp_index1+NX+NU,run1,run1+1 ) );
        loopTemp1.addStatement( loopTemp2 );
        loop->addStatement( loopTemp1 );
//  }

    // PART 1: The linear input system
    prepareInputSystem( code );
    code.addStatement( "\n\n" );
    code.addFunction( sens_input );
    code.addStatement( "\n\n" );
    code.addFunction( sens_fdb );
    code.addStatement( "\n\n" );

    solveInputSystem( loop, i, run1, j, k_index, Ah );
    evaluateAllStatesImplicitSystem( loop, k_index, Ah, C, run1, j, tmp_index1 );

    // propagate sensitivities:
    loop->addFunctionCall( sens_input.getName(), rk_diffsPrev1, rk_dk1_tmp );

    // IF INEXACT: CONDENSE DIFFK INTO RK_DK1:
    if( liftMode == 2 ) {
        ExportForLoop loopKsens02( j,0,NF );
        loopKsens02.addStatement( tmp_index1 == (k_index+j)*(NX+NU) );
        for( run6 = 0; run6 < numStages; run6++ ) {
            loopKsens02.addStatement( rk_sensF.getRow(run6*NF+j) == rk_diffK.getSubMatrix(tmp_index1,tmp_index1+NX+NU,run6,run6+1).getTranspose() );
        }
        loop->addStatement( loopKsens02 );
        loop->addFunctionCall( sens_fdb.getName(), rk_sensF, rk_dk2_tmp );

        ExportForLoop loopKsens04( i,0,NX );
        for( run5 = 0; run5 < numStages; run5++ ) {
            loopKsens04.addStatement( rk_dk2_tmp.getSubMatrix(run5*NX+i,run5*NX+i+1,0,NX+NU) += rk_dk1_tmp.getRow(run5*NX+i) );
        }
        loop->addStatement( loopKsens04 );
    }

    // PART 2: The static nonlinear system
    if( liftMode == 2 ) {
        loop->addStatement( std::string("if(") + run.getName() + " == 0) {\n" );
    }
    DMatrix eyeMF = eye<double>(numStages*NF);
    loop->addStatement( rk_A == eyeMF );
    if( liftMode == 2 ) loop->addStatement( std::string("}\n") );
    ExportForLoop loopF( i,0,numStages );
    loopF.addStatement( rk_seed.getCols(0,NX+NF) == rk_stageValues.getCols(i*(NX+NF),(i+1)*(NX+NF)) );

    loopF.addStatement( rk_seed.getCols(NX+NF+(NX+NDX2)*(NX+NU)+numStages+NDX2*(numStages*NF)+NU+NOD,NX+NF+(NX+NDX2)*(NX+NU)+numStages+NDX2*(numStages*NF)+NU+NOD+NX) == rk_kTemp.getRows(i*NX,(i+1)*NX).getTranspose() );

    DMatrix zeroV = zeros<double>(1,NX*(NX+NU));
    loopF.addStatement( rk_seed.getCols(NX+NF,NX+NF+NX*(NX+NU)) == zeroV );
//    }
    loopF.addStatement( rk_seed.getCols(NX+NF,NX+NF+NX*(NX+NU)) == rk_diffsPrev1.makeRowVector() );
    ExportForLoop loopF1( j,0,NX );
    for( run5 = 0; run5 < numStages; run5++ ) {
        if( liftMode == 1 ) {
            loopF1.addStatement( rk_seed.getCols(NX+NF+j*(NX+NU),NX+NF+(j+1)*(NX+NU)) += Ah.getElement(i,run5)*rk_dk1_tmp.getSubMatrix(run5*NX+j,run5*NX+j+1,0,NX+NU) );
        } else {
            loopF1.addStatement( rk_seed.getCols(NX+NF+j*(NX+NU),NX+NF+(j+1)*(NX+NU)) += Ah.getElement(i,run5)*rk_dk2_tmp.getSubMatrix(run5*NX+j,run5*NX+j+1,0,NX+NU) );
        }
    }
    loopF.addStatement( loopF1 );
        for( run5 = 0; run5 < numStages; run5++ ) {
                loopF.addStatement( rk_seed.getCol(NX+NF+NX*(NX+NU)+run5) == Ah.getElement(i,run5) );
//          }
        }
//  }

    if( NDX2 > 0 ) {
        ExportForLoop loopF12( j,0,NX );
        if( liftMode == 1 ) {
            loopF12.addStatement( tmp_index1 == i*NX+j );
            loopF12.addStatement( rk_seed.getCols(NX+NF+NX*(NX+NU)+numStages+j*(NX+NU),NX+NF+NX*(NX+NU)+numStages+(j+1)*(NX+NU)) == rk_dk1_tmp.getSubMatrix(tmp_index1,tmp_index1+1,0,NX+NU) );
        } else {
            loopF12.addStatement( tmp_index1 == i*NX+j );
            loopF12.addStatement( rk_seed.getCols(NX+NF+NX*(NX+NU)+numStages+j*(NX+NU),NX+NF+NX*(NX+NU)+numStages+(j+1)*(NX+NU)) == rk_dk2_tmp.getSubMatrix(tmp_index1,tmp_index1+1,0,NX+NU) );
        }
        loopF.addStatement( loopF12 );
        for( run6 = 0; run6 < NX; run6++ ) {
            for( run7 = 0; run7 < numStages*NF; run7++ ) {
                loopF.addStatement( rk_seed.getCol(NX+NF+NX*(NX+NU)+numStages+NX*(NX+NU)+run6*(numStages*NF)+run7) == rk_dk1.getElement(i*NX+run6,NX+NU+run7) );
            }
        }
    }

    loopF.addFunctionCall( feedb.getName(), rk_seed, rk_diffsTemp2.getAddress(i,0) );
    for( run5 = 0; run5 < NF; run5++ ) {
        loopF.addStatement( rk_b.getRow(i*NF+run5) == rk_diffsTemp2.getSubMatrix(i,i+1,run5*(1+NX+NU+numStages*NF),run5*(1+NX+NU+numStages*NF)+1+NX+NU) );
    }
    loopF.addStatement( rk_b.getSubMatrix(i*NF,(i+1)*NF,0,1) -= rk_kkk.getSubMatrix(k_index,k_index+NF,i,i+1) );

    // INEXACT UPDATE OF RHS SENSITIVITIES
    if( liftMode == 2 ) {
        ExportForLoop loopFsens( j,0,NF );
        loopFsens.addStatement( tmp_index1 == (k_index+j)*(NX+NU) );
        loopFsens.addStatement( tmp_index2 == i*NF+j );
        loopFsens.addStatement( rk_b.getSubMatrix(tmp_index2,tmp_index2+1,1,1+NX+NU) -= rk_diffK.getSubMatrix(tmp_index1,tmp_index1+NX+NU,i,i+1).getTranspose() );
        loopF.addStatement( loopFsens );
    }

    if( liftMode == 2 ) {
        loopF.addStatement( std::string("if(") + run.getName() + " == 0) {\n" );
    }
    for( run5 = 0; run5 < NF; run5++ ) {
        loopF.addStatement( rk_A.getRow(i*NF+run5) -= rk_diffsTemp2.getSubMatrix(i,i+1,run5*(1+NX+NU+numStages*NF)+1+NX+NU,(run5+1)*(1+NX+NU+numStages*NF)) );
    }
    if( liftMode == 2 ) loopF.addStatement( std::string("}\n") );
    loop->addStatement( loopF );

    // call the linear solver:
    if( liftMode == 2 ) {
        loop->addStatement( std::string("if(") + run.getName() + " == 0) {\n" );
    }
    loop->addStatement( determinant.getFullName() + " = " + ExportStatement::fcnPrefix + "_" + solver->getNameSolveFunction() + "( " + rk_A.getFullName() + ", " + rk_auxSolver.getFullName() + " );\n" );
    if( liftMode == 2 ) loop->addStatement( std::string("}\n") );
    loop->addFunctionCall( solver->getNameSolveReuseFunction(),rk_A.getAddress(0,0),rk_b.getAddress(0,0),rk_auxSolver.getAddress(0,0) );

    // update the F variables:
    for( run5 = 0; run5 < numStages; run5++ ) {
        loop->addStatement( rk_kkk.getSubMatrix(k_index,k_index+NF,run5,run5+1) += rk_b.getSubMatrix(run5*NF,(run5+1)*NF,0,1) );
    }
    ExportForLoop loopFsens( i,0,NF );
    loopFsens.addStatement( tmp_index1 == (k_index+i)*(NX+NU) );
    for( run5 = 0; run5 < numStages; run5++ ) {
        if( liftMode == 1 ) {
            loopFsens.addStatement( rk_diffK.getSubMatrix(tmp_index1,tmp_index1+NX+NU,run5,run5+1) == rk_b.getSubMatrix(run5*NF+i,run5*NF+i+1,1,1+NX+NU).getTranspose() );
        } else {
            loopFsens.addStatement( rk_diffK.getSubMatrix(tmp_index1,tmp_index1+NX+NU,run5,run5+1) += rk_b.getSubMatrix(run5*NF+i,run5*NF+i+1,1,1+NX+NU).getTranspose() );
        }
    }
    loop->addStatement( loopFsens );


    // PART 3: Condensing of the F variables to provide integrator output
    // update the K variables:
    for( run5 = 0; run5 < numStages; run5++ ) {
        loop->addStatement( rk_kTemp.getRows(run5*NX,(run5+1)*NX) += sensMat.getRows(run5*NX,(run5+1)*NX-1)*rk_b.getCol(0) );
    }

    ExportForLoop loopKsens03( j,0,NF );
    loopKsens03.addStatement( tmp_index2 == (k_index+j)*(NX+NU) );
    for( run6 = 0; run6 < numStages; run6++ ) {
        loopKsens03.addStatement( rk_sensF.getRow(run6*NF+j) == rk_diffK.getSubMatrix(tmp_index2,tmp_index2+NX+NU,run6,run6+1).getTranspose() );
    }
    loop->addStatement( loopKsens03 );

    loop->addFunctionCall( sens_fdb.getName(), rk_sensF, rk_dk2_tmp );
    for( run5 = 0; run5 < numStages; run5++ ) {
        ExportForLoop loopKsens( i,0,NX );
        loopKsens.addStatement( rk_dk1_tmp.getRow(run5*NX+i) += rk_dk2_tmp.getRow(run5*NX+i) );
        loop->addStatement( loopKsens );
    }

    // update rk_diffsNew with the new sensitivities:
    ExportForLoop loopKsens1( i,0,NX );
    ExportForLoop loopKsens2( j,0,NX+NU );
    loopKsens2.addStatement( rk_diffsNew1.getElement( i,j ) == rk_diffsPrev1.getElement( i,j ) );
    for( run5 = 0; run5 < numStages; run5++ ) {
        loopKsens2.addStatement( rk_diffsNew1.getElement( i,j ) += Bh(run5)*rk_dk1_tmp.getElement( run5*NX+i,j ) );
    }
    loopKsens1.addStatement( loopKsens2 );
    loop->addStatement( loopKsens1 );


    // update rk_eta:
    for( run6 = 0; run6 < numStages; run6++ ) {
        for( run5 = 0; run5 < NX; run5++ ) {
            loop->addStatement( rk_eta.getCol( run5 ) += rk_kTemp.getRow( run6*NX+run5 )*Bh(run6) );
        }
    }

    updateInputSystem(loop, i, j, tmp_index2);

    loop->addStatement( rk_ttt += DMatrix(1.0/grid.getNumIntervals()) );

    // end of the integrator loop.
    if( !equidistantControlGrid() ) {
        loop->addStatement( "}\n" );
    }
    else {
        integrate.addStatement( *loop );
    }

//    integrate.addStatement( std::string( "if( " ) + determinant.getFullName() + " < 1e-12 ) {\n" );
//    integrate.addStatement( error_code == 2 );
//    integrate.addStatement( std::string( "} else if( " ) + determinant.getFullName() + " < 1e-6 ) {\n" );
//    integrate.addStatement( error_code == 1 );
//    integrate.addStatement( std::string( "} else {\n" ) );
    integrate.addStatement( error_code == 0 );
//    integrate.addStatement( std::string( "}\n" ) );

    code.addFunction( integrate );
    code.addLinebreak( 2 );

    return SUCCESSFUL_RETURN;
}


returnValue FeedbackLiftedIRKExport::solveInputSystem( ExportStatementBlock* block, const ExportIndex& index1, const ExportIndex& index2, const ExportIndex& index3, const ExportIndex& k_index, const ExportVariable& Ah )
{
    if( NX1 > 0 ) {
        ExportForLoop loop( index1,0,numStages );
        loop.addStatement( rk_xxx.getCols(0,NX) == rk_eta.getCols(0,NX) );
        loop.addStatement( rk_xxx.getCols(NX,NX+NF) == rk_kkk.getSubMatrix(k_index,k_index+NF,index1,index1+1).getTranspose() );
        loop.addFunctionCall( lin_input.getName(), rk_xxx, rk_rhsTemp.getAddress(index1*NX,0) );
        block->addStatement(loop);

        block->addStatement( rk_kTemp == mat1*rk_rhsTemp );
    }

    return SUCCESSFUL_RETURN;
}


returnValue FeedbackLiftedIRKExport::evaluateAllStatesImplicitSystem( ExportStatementBlock* block, const ExportIndex& k_index, const ExportVariable& Ah, const ExportVariable& C, const ExportIndex& stage, const ExportIndex& i, const ExportIndex& tmp_index )
{
    ExportForLoop loop0( stage,0,numStages );
    ExportForLoop loop1( i, 0, NX );
    loop1.addStatement( rk_stageValues.getCol( stage*(NX+NF)+i ) == rk_eta.getCol( i ) );
    for( uint j = 0; j < numStages; j++ ) {
        loop1.addStatement( rk_stageValues.getCol( stage*(NX+NF)+i ) += Ah.getElement(stage,j)*rk_kTemp.getRow( j*NX+i ) );
    }
    loop0.addStatement( loop1 );

    ExportForLoop loop3( i, 0, NF );
    loop3.addStatement( tmp_index == k_index + i );
    loop3.addStatement( rk_stageValues.getCol( stage*(NX+NF)+NX+i ) == rk_kkk.getElement( tmp_index,stage ) );
    loop0.addStatement( loop3 );
    block->addStatement( loop0 );

    return SUCCESSFUL_RETURN;
}


returnValue FeedbackLiftedIRKExport::setup( )
{
    if( CONTINUOUS_OUTPUT ) return ACADOERROR( RET_NOT_YET_IMPLEMENTED );

//  int liftMode;
//  get( LIFTED_INTEGRATOR_MODE, liftMode );

    NVARS2 = 0;
    NVARS3 = 0;
    diffsDim = (NX+NXA)*(NX+NU);
    inputDim = (NX+NXA)*(NX+NU+1) + NU + NOD;

    int useOMP;
    get(CG_USE_OPENMP, useOMP);
    ExportStruct structWspace;
    structWspace = useOMP ? ACADO_LOCAL : ACADO_WORKSPACE;

    uint timeDep = 0;
    if( timeDependant ) timeDep = 1;

    rk_ttt = ExportVariable( "rk_ttt", 1, 1, REAL, structWspace, true );
    rk_xxx = ExportVariable( "rk_xxx", 1, NX+NF+NX+NU+NOD+timeDep, REAL, structWspace );
    rk_rhsTemp = ExportVariable( "rk_rhsTemp", numStages*NX, 1, REAL, structWspace );
    rk_kTemp = ExportVariable( "rk_kTemp", numStages*NX, 1, REAL, structWspace );
    rk_index = ExportVariable( "rk_index", 1, 1, INT, ACADO_LOCAL, true );
    rk_eta = ExportVariable( "rk_eta", 1, inputDim, REAL );
    integrate = ExportFunction( "integrate", rk_eta );
    integrate.addArgument( rk_index );
    integrate.setReturnValue( error_code );

    rk_eta.setDoc( "Working array of size " + toString( rk_eta.getDim() ) + " to pass the input values and return the results." );
    reset_int.setDoc( "The internal memory of the integrator can be reset." );
    rk_index.setDoc( "Number of the shooting interval." );
    error_code.setDoc( "Status code of the integrator." );
    integrate.doc( "Performs the integration and sensitivity propagation for one shooting interval." );
    integrate.addLinebreak( );  // TO MAKE SURE IT GETS EXPORTED

    // setup linear solver:
    int solverType;
    userInteraction->get( LINEAR_ALGEBRA_SOLVER,solverType );

    if ( solver )
        delete solver;
    solver = 0;

    switch( (LinearAlgebraSolver) solverType ) {
    case GAUSS_LU:
        solver = new ExportGaussElim( userInteraction,commonHeaderName );
        solver->init( NF*numStages, NX+NU+1 );
        solver->setReuse( true );   // IFTR method
        solver->setup();
        rk_auxSolver = solver->getGlobalExportVariable( 1 );
        break;
    default:
        return ACADOERROR( RET_INVALID_OPTION );
    }

    rk_diffsPrev1 = ExportVariable( "rk_diffsPrev1", NX, NX+NU, REAL, structWspace );
    rk_diffsNew1 = ExportVariable( "rk_diffsNew1", NX, NX+NU, REAL, structWspace );

    rk_seed = ExportVariable( "rk_seed", 1, 2*NX+NF+NX*(NX+NU)+numStages+NDX2*(NX+NU+numStages*NF)+NU+NOD+timeDep, REAL, structWspace );
    rk_diffsTemp2 = ExportVariable( "rk_diffsTemp", numStages, NF*(1+NX+NU+numStages*NF), REAL, structWspace );
    rk_stageValues = ExportVariable( "rk_stageValues", 1, numStages*(NX+NF), REAL, structWspace );
    rk_kkk = ExportVariable( "rk_Ktraj", N*grid.getNumIntervals()*NF, numStages, REAL, ACADO_VARIABLES );
//  if( liftMode == 1 || liftMode == 4 ) {
        rk_diffK = ExportVariable( "rk_diffKtraj", N*grid.getNumIntervals()*NF*(NX+NU), numStages, REAL, ACADO_VARIABLES );
//  }
//  else {
//      return ACADOERROR( RET_NOT_YET_IMPLEMENTED );
//  }

    rk_Xprev = ExportVariable( "rk_Xprev", N, NX, REAL, ACADO_VARIABLES );
    rk_Uprev = ExportVariable( "rk_Uprev", N, NU, REAL, ACADO_VARIABLES );
    rk_delta = ExportVariable( "rk_delta", 1, NX+NU, REAL, ACADO_WORKSPACE );

    rk_A = ExportVariable( "rk_A", numStages*NF, numStages*NF, REAL, structWspace );
    rk_b = ExportVariable( "rk_b", numStages*NF, 1+NX+NU, REAL, structWspace );

    rk_dk1_tmp = ExportVariable( "rk_dk1_tmp", numStages*NX1, NX+NU, REAL, ACADO_WORKSPACE );
    rk_dk2_tmp = ExportVariable( "rk_dk2_tmp", numStages*NX1, NX+NU, REAL, ACADO_WORKSPACE );

    rk_sensF = ExportVariable( "rk_sensF", numStages*NF, NX+NU, REAL, structWspace );

    return SUCCESSFUL_RETURN;
}



// PROTECTED:


CLOSE_NAMESPACE_ACADO

// end of file.
