CXX ?= g++
CXXFLAGS ?= -std=c++17 -O2 -Wall -Wextra -Werror -Ivendor/chalao -Isrc
PREFIX ?= /usr/local
DESTDIR ?=

zron: src/zron.cpp $(wildcard src/*.hpp) $(wildcard vendor/chalao/*.hpp)
	$(CXX) $(CXXFLAGS) src/zron.cpp -o zron

test: zron
	bash tests/run_tests.sh

# static Linux binary for releases
static:
	$(CXX) $(CXXFLAGS) -static src/zron.cpp -o zron

san:
	$(CXX) -std=c++17 -O1 -g -Wall -Wextra -Werror -fsanitize=address,undefined -Ivendor/chalao -Isrc src/zron.cpp -o zron

# zron -> $(PREFIX)/bin, the MuJoCo bridge and the bundled robot -> $(PREFIX)/share/zronos
install: zron
	install -d $(DESTDIR)$(PREFIX)/bin $(DESTDIR)$(PREFIX)/share/zronos/bridge $(DESTDIR)$(PREFIX)/share/zronos/system $(DESTDIR)$(PREFIX)/share/zronos/examples
	install -m 755 zron $(DESTDIR)$(PREFIX)/bin/zron
	install -m 644 bridge/zron_mujoco.py bridge/compat.py bridge/requirements.txt $(DESTDIR)$(PREFIX)/share/zronos/bridge/
	install -m 644 system/* $(DESTDIR)$(PREFIX)/share/zronos/system/
	install -m 644 examples/* $(DESTDIR)$(PREFIX)/share/zronos/examples/

uninstall:
	rm -f $(DESTDIR)$(PREFIX)/bin/zron
	rm -rf $(DESTDIR)$(PREFIX)/share/zronos

clean:
	rm -f zron *.o

.PHONY: test static san install uninstall clean
