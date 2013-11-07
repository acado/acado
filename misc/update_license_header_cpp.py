#!/usr/bin/python

import os
import datetime

exclude_folders = ["lib", "bin", "build", "qpoases", "templates"]

def update_license(filename, copyright):
    utfstr = chr(0xef) + chr(0xbb) + chr(0xbf)
    fdata = file(filename, "r+").read()
    
    isUTF = False
    if (fdata.startswith(utfstr)):
        isUTF = True
        fdata = fdata[3:]
    
    if fdata.startswith( "/*" ):
        # File starts with a header
        
        # Find end of the comment and trim the file
        fdata = fdata[fdata.find( "*/" ) + 2:]
        
    # Now write data to the file
    ndata = copyright[:-1] + fdata
    
    print "Updating: " + filename
    if (isUTF):
        file(filename, "w").write(utfstr + ndata)
    else:
        file(filename, "w").write(ndata)

def recursive_traversal(dir, copyright):
    global exclude_folders
    fns = os.listdir(dir)
    for fn in fns:
        fullfn = os.path.join(dir, fn)
        if (os.path.isdir(fullfn)):
            if (os.path.basename( fullfn ) in exclude_folders):
                continue
            recursive_traversal(fullfn, copyright)
        else:
            if (fullfn.endswith(".cpp") or fullfn.endswith(".hpp") or fullfn.endswith(".ipp")):
                update_license(fullfn, copyright)

# Read copyright data
copyright = file("license_header_cpp.txt", "r+").read() % datetime.datetime.today().year
# Update the licensing information
folders = ["../include",
		   "../src",
		   "../external_packages/include",
		   "../external_packages/src",
		   "../examples"]

for f in folders:
	recursive_traversal(f, copyright)
