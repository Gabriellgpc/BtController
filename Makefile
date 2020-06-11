CC = g++
CXX = g++
CPPFLAGS = -Wall -O2 -fopenmp
LDLIBS =  -lbluetooth

PROGRAMS	= btcontroller

all: ${PROGRAMS}

${PROGRAMS}:	bluetoothAction/bluetoothAction.h bluetoothAction/bluetoothAction.cpp

clean:
	-rm -f *.o *~	${PROGRAMS}
