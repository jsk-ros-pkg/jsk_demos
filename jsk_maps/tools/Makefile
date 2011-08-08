all:
	./eus2owl.sh scene1
	./eus2owl.sh eng2
	cp -f *.owl.in `rospack find jsk_semantic_maps`/owl/
	(cd `rospack find jsk_semantic_maps`; make)