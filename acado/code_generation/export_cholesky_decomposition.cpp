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
 *    \file src/code_generation/export_cholesky_decomposition.cpp
 *    \author Milan Vukov
 *    \date 2013
 */

#include <acado/code_generation/export_cholesky_decomposition.hpp>

#include <string>
#include <sstream>

using namespace std;

BEGIN_NAMESPACE_ACADO

ExportCholeskyDecomposition::ExportCholeskyDecomposition(	UserInteraction* _userInteraction,
															const std::string& _commonHeaderName
															) : ExportAlgorithm(_userInteraction, _commonHeaderName)
{
	init("choleskyDecomposition", 0, false);
}

returnValue ExportCholeskyDecomposition::init(	const std::string& _name,
												unsigned _dim,
												bool _unrolling
												)
{
	unrolling = _unrolling;

	A.setup("A", _dim, _dim, REAL, ACADO_LOCAL);
	fcn.init(_name, A);

	return SUCCESSFUL_RETURN;
}

returnValue ExportCholeskyDecomposition::setup()
{
	ASSERT( A.getDim() );

	stringstream s;

	ExportVariable ret("ret", 1, 1, INT, ACADO_LOCAL, true);
	fcn.setReturnValue( ret );

	string name( A.getName() );
	unsigned n = A.getNumRows();

	s 		<< "register unsigned i, j, k;" << endl
			<< "real_t inv;" << endl
			<< "for (i = 0; i < " << n << "; ++i)" << endl
			<< "{" << endl
			<< name << "[i * " << n << " + i] = "
				<< name << "[i * " << n << " + i] < 1e-8 ? 1e-8 : "
				<< "sqrt(" << name <<  "[i * " << n << " + i]);" << endl

			<< "inv = 1 / " << name << "[i * " << n << " + i];" << endl

			<< "for (j = i + 1; j < " << n << "; ++j)"  << endl
			<< name << "[j * " << n << " + i] = " << name << "[j * " << n << " + i] * inv;"  << endl

			<< "for (j = i + 1; j < " << n << "; ++j)"  << endl
			<< "for (k = j; k < " << n << "; ++k)"  << endl
			<< name << "[k * " << n << " + j] = " << name << "[k * " << n << " + j] - "
					<< name << "[k * " << n << " + i] * " << name << "[j * " << n << " + i];"  << endl
			<< "}"  << endl

			// Clear the upper triangular part
			<< "for (i = 0; i < " << n << "; ++i)"  << endl
			<< "for (j = i + 1; j < " << n << "; ++j)"  << endl
			<< name << "[i * " << n << " + j] = 0.0;" << endl
			<< ret.getName() << " = 0;" << endl;

	fcn.addStatement( s.str() );

	return SUCCESSFUL_RETURN;
}

returnValue ExportCholeskyDecomposition::getDataDeclarations(	ExportStatementBlock& declarations,
																ExportStruct dataStruct
																) const
{
	return SUCCESSFUL_RETURN;
}

returnValue ExportCholeskyDecomposition::getFunctionDeclarations(	ExportStatementBlock& declarations
																	) const
{
	if (A.getDim() != 0)
		return declarations.addFunction( fcn );

	return SUCCESSFUL_RETURN;
}

returnValue ExportCholeskyDecomposition::getCode(	ExportStatementBlock& code
													)
{
	if (A.getDim() != 0)
		return code.addFunction( fcn );

	return SUCCESSFUL_RETURN;
}

const std::string ExportCholeskyDecomposition::getName()
{
	return fcn.getName();
}

CLOSE_NAMESPACE_ACADO
