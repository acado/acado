#!/bin/bash

# Get the current branch name
BRANCH=$(git symbolic-ref --short HEAD)
# Get the current remote origin
ORIGIN=$(git ls-remote --get-url origin)
# Name of the archive
ZIP_FILE=acadotoolkit-current-$BRANCH.zip

# Deploy only if the code is pushed to the "blessed" remote
if [ "$ORIGIN" == "git@github.com:acado/acado.git" ] && [ "$CXX" == "g++" ]; then
	# Make an archive of the current HEAD
	git archive -o $ZIP_FILE HEAD
	# Deploy the archive to the website
	curl -T $ZIP_FILE -u $FTP_USER:$FTP_PASS ftp://ftp.acadotoolkit.org/zip/$ZIP_FILE
else
	echo "Dude, you cannot deploy, the code is not pushed to the blessed remote!"
fi
