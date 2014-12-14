DEBUG=yes
FAC=no
CC=g++
ifeq ($(DEBUG),yes)
	CFLAGS= -Wall -Wextra -g
else
	CFLAGS= -Werror -Wextra -Wall -O3
endif
ifeq ($(FAC),yes)
	LDFLAGS=-I/usr/local/opencv/2.4.5/include/ -L/usr/local/opencv/2.4.5/lib64
else
	LDFLAGS=-I/usr/local/include/opencv/ -L/usr/local/lib
endif
LBLIBS=-lopencv_objdetect -lopencv_features2d -lopencv_flann -lopencv_calib3d -lopencv_imgproc -lopencv_highgui -lopencv_core -lopencv_video
EXEC=readLidarData stereoVision laser

all: $(EXEC)

example: example.cpp
	@echo compilation of example
	$(CC) $(LDFLAGS) -o $@ $< $(LBLIBS)

stereoVision: stereoVision.o tp_util.o
	@echo ----------------------------------------------
	@echo compilation of stereoVision
	@echo ----------------------------------------------
	$(CC) $(LDFLAGS) -o $@ $^ $(LBLIBS)

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
	$(CC) $(LDFLAGS) -c $< $(CFLAGS)

clean:
	rm -rf *.o $(EXEC) *~
