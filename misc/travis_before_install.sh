#!/bin/bash

################################################################################
#
# Description:
#	Install necessary dependencies for Travis-CI
#
# Authors:
#	Milan Vukov, milan.vukov@esat.kuleuven.be
#
# Year:
#	2013.
#
# Usage:
#	- This script is automatically called from Travis-CI script
#
################################################################################

# Install boost by default
sudo apt-get update -qq
sudo apt-get install -qq libboost-all-dev

# Run only if:
# - the code is pushed to the "blessed" remote and
# - a g++ compiler is being used and
# - and if we pushed to the stable branch
if [ "$TRAVIS_REPO_SLUG" == "acado/acado" ] && [ "$CXX" == "g++" ] && [ "$TRAVIS_BRANCH" == "stable" ]; then
	# Install necessary deps
	sudo apt-get install -qq doxygen graphviz sshpass
fi
