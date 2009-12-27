

src     := ./src
sources := $(shell find ${src} -iname \*.cpp)
library  := erlybullet_drv.so

all:
	g++ -fpic -shared ${sources} -o ${library}