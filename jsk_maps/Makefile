#include $(shell rospack find mk)/cmake.mk

all: euslisp/eng8-map.l

euslisp/eng8-map.l: eng8-map.yaml
	mkdir -p euslisp
	./src/map-convert.sh eng8-map.yaml euslisp/eng8-map.l

clean:
	rm -rf euslisp
