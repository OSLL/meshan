#------------------------------------------------------------------------------

SOURCE=psm.cpp modes.h
PSM=psm
MYINCLUDES=/home/nokia/Desktop/PSM/psm/

#MYLIBRARIES=fltk
CC=g++

#------------------------------------------------------------------------------



all: $(PSM)



$(PSM): $(SOURCE)

	$(CC) -I$(MYINCLUDES) $(SOURCE) -o$(PSM)

clean:

	rm -f $(PSM)



