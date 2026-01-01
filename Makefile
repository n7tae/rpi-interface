all: spot

spot: spot.cpp
	g++ -std=c++17 -O2 -Wall -Wextra -Wno-unused-result spot.cpp -o spot -lm -lm17 -lgpiod

install: all
	sudo install spot /usr/local/bin

clean:
	rm -f spot
