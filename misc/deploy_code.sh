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
#	2013 - 2014.
#
# Usage:
#	- This script is automatically called from Travis-CI script
#
################################################################################

# Name of the archive
ZIP_FILE=acadotoolkit-current-$TRAVIS_BRANCH.zip

# Print versioning info in a file
echo "Branch: $TRAVIS_BRANCH" >> VERSION.txt
echo "Commit: $TRAVIS_COMMIT" >> VERSION.txt
echo "Remote: $TRAVIS_REPO_SLUG" >> VERSION.txt

# Deploy only if:
# - the code is pushed to the "blessed" remote and
# - a g++ compiler is being used
if [ "$TRAVIS_REPO_SLUG" == "acado/acado" ] && [ "$CXX" == "g++" ] && [ "$TRAVIS_OS_NAME" == "linux" ]; then
	# Make an archive of the current travis-ci checked commit
	git archive -o $ZIP_FILE $TRAVIS_COMMIT
	# Add version information
	zip -g $ZIP_FILE VERSION.txt
	# Deploy the archive to the website
	curl -T $ZIP_FILE -u $FTP_USER:$FTP_PASS ftp://ftp.acadotoolkit.org/zip/$ZIP_FILE
fi
