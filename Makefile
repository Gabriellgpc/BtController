CC = g++
CXX = g++
CPPFLAGS = -Wall -O2 -fopenmp -std=c++11
LDLIBS =  -lbluetooth	-lpthread	-lbluetooth

PROGRAMS	= main

all: ${PROGRAMS}

${PROGRAMS}:	btcontroller.o	bluetoothAction/libbluetoothAction.a

btcontroller.o:	btcontroller.hpp btcontroller.cpp


clean:
	-rm -f *.o *~	${PROGRAMS}
