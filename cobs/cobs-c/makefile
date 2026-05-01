./build/objects/cobs_lib.o: src/cobs.c
	mkdir -p ./build/objects
	gcc -c -o ./build/objects/cobs_lib.o src/cobs.c
	
build: ./test/test.c ./build/objects/cobs_lib.o src/cobs.h
	mkdir -p ./build/
	gcc -o ./build/cobs test/test.c -I src ./build/objects/cobs_lib.o

clean:
	rm -r build