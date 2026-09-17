CXX ?= g++
CXXFLAGS ?= -std=c++17 -O2 -Wall -Wextra -Werror -Ivendor/chalao

chalaoos: src/chalaoos.cpp $(wildcard vendor/chalao/*.hpp)
	$(CXX) $(CXXFLAGS) src/chalaoos.cpp -o chalaoos

test: chalaoos
	bash tests/run_tests.sh

clean:
	rm -f chalaoos *.o
.PHONY: test clean

.PHONY: san
san:
	$(CXX) -std=c++17 -O1 -g -Wall -Wextra -Werror -fsanitize=address,undefined -Ivendor/chalao src/chalaoos.cpp -o chalaoos
