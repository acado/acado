#!/bin/bash
echo "This script will synchronize the html doc/folder with http://acado.sourceforge.net/api/html/"
if [ -z "$sfaccount" ]; then
echo -n "Please type your sourceforge username:"
read username
else
username="$sfaccount"
fi
rsync -avP -e ssh html "$username,acado@web.sourceforge.net:/home/groups/a/ac/acado/htdocs/doc"

