# Copyright 2009 Jason Wagner
# Released under the zlib license.  See LICENSE.txt

src     := ./src
sources := $(shell find ${src} -iname \*.cpp)
library  := erlybullet_drv.so
libs    := -lbulletdynamics -lbulletcollision -lbulletmath -I /usr/local/include/bullet

all:
	g++ -fpic -shared ${libs} ${sources} -o ${library}