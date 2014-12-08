DEBUG=yes
CC=g++
ifeq ($(DEBUG),yes)
	CFLAGS= -Wall -Wextra -g
else
	CFLAGS= -Werror -Wextra -Wall -O3
endif
LDFLAGS=-I/usr/local/include/opencv/ -L/usr/local/lib
LBLIBS=-lopencv_objdetect -lopencv_features2d -lopencv_calib3d -lopencv_imgproc -lopencv_highgui -lopencv_core
EXEC=example readLidarData stereoVision laser

all: $(EXEC)

example: example.cpp
	@echo compilation of example
	$(CC) $(LDFLAGS) -o $@ $< $(LBLIBS)

stereoVision: stereoVision.o tp_util.o
	@echo ----------------------------------------------
	@echo compilation of stereoVision
	@echo ----------------------------------------------
	$(CC) $(LDFLAGS) -o $@ $< $(LBLIBS)

readLidarData: readLidarData.o
	@echo ----------------------------------------------
	@echo compilation of readLidarData
	@echo ----------------------------------------------
	$(CC) $(LDFLAGS) -o $@ $< $(LBLIBS)

laser: laser.o
	@echo ----------------------------------------------
	@echo compilation of laser
	@echo ----------------------------------------------
	$(CC) $(LDFLAGS) -o $@ $< $(LBLIBS)
	
%.o: %.cpp
	@echo ----------------------------------------------
	@echo compilation of $<
	@echo ----------------------------------------------
	$(CC) -c $< $(CFLAGS)

clean:
	rm -rf *.o $(EXEC) *~
