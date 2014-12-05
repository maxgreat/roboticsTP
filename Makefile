DEBUG=yes
CC=g++
ifeq ($(DEBUG),yes)
	CFLAGS= -W -Werror -Wall -g
else
	CFLAGS= -W -Werror -Wall
endif
LDFLAGS=-I/usr/local/include/opencv/ -L/usr/local/lib
LBLIBS=-lopencv_objdetect -lopencv_features2d -lopencv_imgproc -lopencv_highgui -lopencv_core
EXEC=example stereoVision

all: $(EXEC)

example : example.cpp
	echo compilation of example
	$(CC) $(LDFLAGS) -o $@ $< $(LBLIBS)

stereoVision : stereoVision.o
	echo ------------------------------------
	echo compilation of stereoVision
	echo ------------------------------------
	$(CC) $(LDFLAGS) -o $@ $< $(LBLIBS)

stereoVision.o: stereoVision.cpp
	echo ------------------------------------
	echo compilation of stereoVision.o
	echo ------------------------------------
	$(CC) -c $< $(CFLAGS)

clean:
	rm -rf *.o $(EXEC)
