CXX ?= g++
CXXFLAGS ?= -std=c++17 -O2 -Wall -Wextra -Werror -Ivendor/chalao -Isrc

zron: src/zron.cpp $(wildcard src/*.hpp) $(wildcard vendor/chalao/*.hpp)
	$(CXX) $(CXXFLAGS) src/zron.cpp -o zron

test: zron
	bash tests/run_tests.sh

clean:
	rm -f zron chalaoos *.o
.PHONY: test clean

.PHONY: san
san:
	$(CXX) -std=c++17 -O1 -g -Wall -Wextra -Werror -fsanitize=address,undefined -Ivendor/chalao -Isrc src/zron.cpp -o zron
