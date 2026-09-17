CXX ?= g++
CXXFLAGS ?= -std=c++17 -O2 -Wall -Wextra -Ivendor/chalao

chalaoos: src/chalaoos.cpp $(wildcard vendor/chalao/*.hpp)
	$(CXX) $(CXXFLAGS) src/chalaoos.cpp -o chalaoos

test: chalaoos
	bash tests/run_tests.sh

clean:
	rm -f chalaoos *.o
.PHONY: test clean
