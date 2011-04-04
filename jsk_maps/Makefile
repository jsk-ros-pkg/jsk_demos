#include $(shell rospack find mk)/cmake.mk

all: euslisp/eng2-map.l euslisp/eng8-map.l

euslisp/eng2-map.l: eng2-map.yaml
	mkdir -p euslisp
	./src/map-convert.sh eng2-map.yaml euslisp/eng2-map.l

euslisp/eng8-map.l: eng8-map.yaml
	mkdir -p euslisp
	./src/map-convert.sh eng8-map.yaml euslisp/eng8-map.l

clean:
	rm -rf euslisp
