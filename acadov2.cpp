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
 *    \file   examples/code_generation/getting_started.cpp
 *    \author Hans Joachim Ferreau, Boris Houska
 *    \date   2011-2013
 */

#include <acado_code_generation.hpp>

int main()
{
	USING_NAMESPACE_ACADO

		// Variables:
	DifferentialState   x;   // IDK
	DifferentialState   dx;  // chamber pressure
	Control             u;   // desired valve position


	// Model equations:
	DifferentialEquation f;

	f << dot(x) == -12.44*x + 0.7539*dx + 2.0*u;
	f << dot(dx) == -7.89*x;

	// Reference functions and weighting matrices:
	Function h, hN;
	h << x << dx  << u;
	hN << x << dx ;

	// Provide defined weighting matrices:
	DMatrix W = eye<double>(h.getDim());
	DMatrix WN = eye<double>(hN.getDim());

	//
	// Optimal Control Problem
	//

	const int N = 10;
	const double Ts = 0.1;

	OCP ocp(0.0, N*Ts, N);

	ocp.subjectTo(f);

	ocp.minimizeLSQ(W, h);
	ocp.minimizeLSQEndTerm(WN, hN);

	ocp.subjectTo(0.0 <= u <= 1.0);
	ocp.subjectTo(-5.0 <= dx <= 0);

	// Export the code:
	OCPexport mpc(ocp);

	mpc.set(HESSIAN_APPROXIMATION, GAUSS_NEWTON);
	mpc.set(DISCRETIZATION_TYPE, MULTIPLE_SHOOTING);
	mpc.set(SPARSE_QP_SOLUTION, FULL_CONDENSING);
	mpc.set(INTEGRATOR_TYPE, INT_IRK_RIIA5);
	mpc.set(NUM_INTEGRATOR_STEPS, 2*N);
	mpc.set(QP_SOLVER, QP_QPOASES3);
	mpc.set(HOTSTART_QP,                 YES             );
	mpc.set(LEVENBERG_MARQUARDT,         1.0e-10          );
	mpc.set(GENERATE_TEST_FILE, YES);
	mpc.set(GENERATE_MAKE_FILE, YES);
	mpc.set(GENERATE_MATLAB_INTERFACE, NO);
	mpc.set(GENERATE_SIMULINK_INTERFACE, NO);
	mpc.set(USE_SINGLE_PRECISION, NO);
	mpc.set(CG_HARDCORE_CONSTRAINT_VALUES, YES);
	mpc.set(CG_USE_VARIABLE_WEIGHTING_MATRIX, NO);
	

	if (mpc.exportCode("acadov2_export") != SUCCESSFUL_RETURN)
		exit(EXIT_FAILURE);

	mpc.printDimensionsQP();

	return EXIT_SUCCESS;
}