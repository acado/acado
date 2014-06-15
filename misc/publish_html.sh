#!/bin/bash

################################################################################
#
# Description:
#	A script for updating of the Doxygen documentation
#
# Authors:
#	Milan Vukov, milan.vukov@esat.kuleuven.be
#
# Year:
#	2013 - 2014.
#
# Usage:
#	- This script is automatically called from Travis-CI script
#
################################################################################

# Update documentation only if:
# - the code is pushed to the "blessed" remote and
# - a g++ compiler is being used and
# - and if we pushed to the stable branch
if [ "$TRAVIS_REPO_SLUG" == "acado/acado" ] && [ "$CXX" == "g++" ] && [ "$TRAVIS_BRANCH" == "stable" ] && [ "$TRAVIS_OS_NAME" == "linux" ]; then
	# Move to the build folder
	cd build
	# Make documentation
	make -j2 doc
	# Change to doc folder
	cd ../doc
	# Synchronize the documentation folders
	sshpass -p "$SF_PASS" rsync -azP --delete -e "ssh -o StrictHostKeyChecking=no" html "$SF_USER,acado@web.sourceforge.net:/home/groups/a/ac/acado/htdocs/doc"
	# Move back to the root folder
	cd ..
fi
