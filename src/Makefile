all: test_tools.o tools.o
		g++ test_tools.o tools.o -o test_tools

test_tools.o: test_tools.cpp tools.cpp
		g++ -c test_tools.cpp -o test_tools.o

tools.o: tools.cpp
		g++ -c tools.cpp -o tools.o
        
clean:
		rm -rf *.o

mrproper: clean
		rm -rf test_tools

