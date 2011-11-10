#include $(shell rospack find mk)/cmake.mk

all: launch/start_map_eng2_vert.launch

launch/start_map_eng2_vert.launch:
	rosrun roseus roseus src/dump-map-info.l


clean:
	rm -rf euslisp *.pgm
