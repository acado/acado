#!/bin/bash

################################################################################
#
# Description:
#	A script for upload of the source code to the acadotoolkit.org web-space
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

# Name of the archive
ZIP_FILE=acadotoolkit-current-$TRAVIS_BRANCH.zip

# Deploy only if:
# - the code is pushed to the "blessed" remote and
# - a g++ compiler is being used
if [ "$TRAVIS_REPO_SLUG" == "acado/acado" ] && [ "$CXX" == "g++" ]; then
	# Make an archive of the current travis-ci checked commit
	git archive -o $ZIP_FILE $TRAVIS_COMMIT
	# Deploy the archive to the website
	curl -T $ZIP_FILE -u $FTP_USER:$FTP_PASS ftp://ftp.acadotoolkit.org/zip/$ZIP_FILE
fi
