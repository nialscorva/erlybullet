#!/bin/sh
perl -ne '/#define\s+(\S+)\s+(.*)/ and print "-define($1,$2).\n"' c_src/erlybulletcommands.hpp > include/erlybulletcommands.hrl

