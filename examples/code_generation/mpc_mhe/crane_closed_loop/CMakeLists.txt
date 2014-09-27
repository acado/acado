################################################################################
#
# Description:
#	CMake scipt for building closed loop crane example to be used with
#	Simulink.
#
# Authors:
#	Milan Vukov, milan.vukov@esat.kuleuven.be
#
# Year:
#	2013 - 2014.
#
# Usage:
#	- This file should be called from the main CMake script for examples.
#
################################################################################

#
# Crane NMPC
#
ACADO_APPLICATION( crane_cl_nmpc crane_cl_nmpc.cpp )
ACADO_COPY_QPOASES_EMBEDDED( crane_cl_nmpc_export )

#
# Crane MHE
#
ACADO_APPLICATION( crane_cl_mhe crane_cl_mhe.cpp )
ACADO_COPY_QPOASES_EMBEDDED( crane_cl_mhe_export )
