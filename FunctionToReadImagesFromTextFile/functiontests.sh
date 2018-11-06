#!/bin/bash

COMPILER="g++"
SOURCE=$1
OUTPUTFILE="-o function.out"
ARGUMENT=$2
FILE_NOTEXISTING="no_file_exist"
FILE_EMPTY="file_empty"
FILE_WITH_ONE_LINE_ENTRY="file_single_entry"
FILE_INVALID_LINE="file_invalid_line"
FILE_READ_PROCESS="file_read"
FILE_WRITTEN_PROCESS="file_written"
FILE_SPECIAL_PERMISSION="/root/.bashrc"
DIR="dir"
 
# Compile test program, resulting in function.out executable.
$COMPILER $SOURCE $OUTPUTFILE
 
# Create test files / directories  
touch $FILE_EMPTY
if [[ ! -d $DIR ]]; then
    mkdir $DIR
fi
echo "rofl" > $FILE_WITH_ONE_LINE_ENTRY
echo -ne "validline\ninvalidline" > $FILE_INVALID_LINE
echo "opened to read from" > $FILE_READ_PROCESS
python -c 'import time; f = open("'$FILE_READ_PROCESS'"); time.sleep(4)' &
echo "opened to write to" > $FILE_WRITTEN_PROCESS
python -c 'import time; f = open("'$FILE_WRITTEN_PROCESS'", "a"); time.sleep(4)' & 
 
# Run each test case.
echo "testing on file whichdoes not exsit.." 
./function.out $ARGUMENT $FILE_NOTEXISTING
echo
echo "testing on empty file.."
./function.out $ARGUMENT $EMPTY_FILE
echo
echo "testing on valid file with one line "
./function.out $ARGUMENT $FILE_WITH_ONE_LINE_ENTRY
echo
echo "testing on a file with one valid  line and one invalid line"
./function.out $ARGUMENT $FILE_INVALID_LINE
echo
echo "testing on a file that is read by another process"
./function.out $ARGUMENT $FILE_READ_PROCESS
echo
echo "testing on a file that is written to by another process"
./function.out $ARGUMENT $FILE_WRITTEN_PROCESS
echo
echo "testing on a /root/.bashrc (access should be denied)"
./function.out $ARGUMENT $FILE_SPECIAL_PERMISSION
echo
echo "testing on a directory"
./function.out $ARGUMENT $DIR
