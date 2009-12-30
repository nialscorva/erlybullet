appname = erlybullet
include ./build/make.mk

compile: include/erlybulletcommands.hrl

include/erlybulletcommands.hrl: priv/src/erlybulletcommands.hpp
	perl -ne '/#define\s+(\S+)\s+(.*)/ and print "-define($$1,$$2).\n"' $< > $@
