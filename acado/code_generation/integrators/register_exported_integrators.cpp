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
 *    \file src/code_generation/register_exported_integrators.cpp
 *    \author Milan Vukov
 *    \date 2014
 */

#include <acado/code_generation/integrators/register_exported_integrators.hpp>

#include <acado/code_generation/integrators/dirk3_export.hpp>
#include <acado/code_generation/integrators/dirk4_export.hpp>
#include <acado/code_generation/integrators/dirk5_export.hpp>

#include <acado/code_generation/integrators/discrete_export.hpp>
#include <acado/code_generation/integrators/narx_export.hpp>

#include <acado/code_generation/integrators/explicit_euler_export.hpp>
#include <acado/code_generation/integrators/erk2_export.hpp>
#include <acado/code_generation/integrators/erk3_export.hpp>
#include <acado/code_generation/integrators/erk4_export.hpp>

#include <acado/code_generation/integrators/gauss_legendre2_export.hpp>
#include <acado/code_generation/integrators/gauss_legendre4_export.hpp>
#include <acado/code_generation/integrators/gauss_legendre6_export.hpp>
#include <acado/code_generation/integrators/gauss_legendre8_export.hpp>

#include <acado/code_generation/integrators/radau_IIA1_export.hpp>
#include <acado/code_generation/integrators/radau_IIA3_export.hpp>
#include <acado/code_generation/integrators/radau_IIA5_export.hpp>

BEGIN_NAMESPACE_ACADO

//
// Solver registration
//

RegisterExportedIntegrators::RegisterExportedIntegrators()
{
	IntegratorExportFactory::instance().registerAlgorithm(INT_DIRK3, createDiagonallyIRK3Export);
	IntegratorExportFactory::instance().registerAlgorithm(INT_DIRK4, createDiagonallyIRK4Export);
	IntegratorExportFactory::instance().registerAlgorithm(INT_DIRK5, createDiagonallyIRK5Export);

	IntegratorExportFactory::instance().registerAlgorithm(INT_DT, createDiscreteTimeExport);
	IntegratorExportFactory::instance().registerAlgorithm(INT_NARX, createNARXExport);

	IntegratorExportFactory::instance().registerAlgorithm(INT_EX_EULER, createExplicitEulerExport);
	IntegratorExportFactory::instance().registerAlgorithm(INT_RK2, createExplicitRungeKutta2Export);
	IntegratorExportFactory::instance().registerAlgorithm(INT_RK3, createExplicitRungeKutta3Export);
	IntegratorExportFactory::instance().registerAlgorithm(INT_RK4, createExplicitRungeKutta4Export);

	IntegratorExportFactory::instance().registerAlgorithm(INT_IRK_GL2, createGaussLegendre2Export);
	IntegratorExportFactory::instance().registerAlgorithm(INT_IRK_GL4, createGaussLegendre4Export);
	IntegratorExportFactory::instance().registerAlgorithm(INT_IRK_GL6, createGaussLegendre6Export);
	IntegratorExportFactory::instance().registerAlgorithm(INT_IRK_GL8, createGaussLegendre8Export);

	IntegratorExportFactory::instance().registerAlgorithm(INT_IRK_RIIA1, createRadauIIA1Export);
	IntegratorExportFactory::instance().registerAlgorithm(INT_IRK_RIIA3, createRadauIIA3Export);
	IntegratorExportFactory::instance().registerAlgorithm(INT_IRK_RIIA5, createRadauIIA5Export);
}

CLOSE_NAMESPACE_ACADO
