#!/bin/bash

# Name of the archive
ZIP_FILE=acadotoolkit-current-$TRAVIS_BRANCH.zip

# Deploy only if the code is pushed to the "blessed" remote and if a g++ compiler is being used
if [ "$TRAVIS_REPO_SLUG" == "acado/acado" ] && [ "$CXX" == "g++" ]; then
	# Make an archive of the current travis-ci checked commit
	git archive -o $ZIP_FILE $TRAVIS_COMMIT
	# Deploy the archive to the website
	curl -T $ZIP_FILE -u $FTP_USER:$FTP_PASS ftp://ftp.acadotoolkit.org/zip/$ZIP_FILE
fi
