appname = erlybullet

# directories
src := src
ebin := ebin
doc := doc
edoc := doc/edoc
test := test
includedir := include

#erl config
erl_node := -sname e1
erl_path := 
erl_boot := 
erl_config := 
erlc_flags := +debug_info
#inputs 
sources = $(shell find ${src} -iname \*.erl)
test_src = $(shell find ${test} -iname \*_tests.erl)
includes := -I ${includedir}

#outputs
modules := $(basename $(notdir ${sources}))
beams := $(addprefix ${ebin}/,$(addsuffix .beam,${modules}))
products := ${beams} ${app_file} ${rel_file}

################################################################
# Rules
################################################################
.PHONY: all compile shell run
all: compile

compile: ${sources} include/erlybulletcommands.hrl 
	mkdir -p ${ebin}
	erlc ${erlc_flags} -o ${ebin} ${includes} ${sources}
	make -C priv

shell: compile
	-mkdir -p ${logs}
	erl ${erl_node} ${erl_path} -boot start_sasl ${erl_config}

test: compile
	erlc ${erlc_flags} -o ${ebin} ${includes} ${test_src}	
	erl ${erl_node} ${erl_path} -boot start_sasl ${erl_config} -noshell -s eunit test $(modules) -s init stop 

doc:
	-mkdir -p doc
	erl -noshell -run edoc_run application "'$(appname)'"

clean:
	rm -f ${products}

include/erlybulletcommands.hrl: priv/src/erlybulletcommands.hpp
	perl -ne '/#define\s+(\S+)\s+(.*)/ and print "-define($$1,$$2).\n"' $< > $@
