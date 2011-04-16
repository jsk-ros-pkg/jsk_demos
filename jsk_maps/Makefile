#include $(shell rospack find mk)/cmake.mk

all: eng2.pgm eng8.pgm euslisp/eng2-map.l euslisp/eng8-map.l

eng2.pgm: $(shell find raw_maps -name 'eng2*.pgm')
	convert +append raw_maps/eng2* eng2.pgm

eng8.pgm: $(shell find raw_maps -name 'eng8*.pgm')
	convert +append raw_maps/eng8* eng8.pgm

euslisp/eng2-map.l: eng2-map.yaml
	mkdir -p euslisp
	./src/map-convert.sh eng2-map.yaml euslisp/eng2-map.l

euslisp/eng8-map.l: eng8-map.yaml
	mkdir -p euslisp
	./src/map-convert.sh eng8-map.yaml euslisp/eng8-map.l

clean:
	rm -rf euslisp *.pgm
