ifndef appname
$(error Must define appname)
endif

# directories
src := src
ebin := ebin
conf := conf
doc := doc
edoc := doc/edoc
logs := logs
test := test
includedir := include

#erl config
erl_node := -sname e1
erl_path := 
#-pz ${ebin}
erl_boot := -boot ${ebin}/${appname}
erl_config := -config conf/${appname}
erlc_flags := +debug_info
#inputs 
sources = $(shell find ${src} -iname \*.erl)
test_src = $(shell find ${test} -iname \*_tests.erl)
includes := -I ${includedir}
app_template := ${conf}/${appname}.app.tmpl
rel_template := ${conf}/${appname}.rel.tmpl
asn1dir := priv/asn.1
asn1src := ${asn1dir}/*.asn1
asn1out := src

#outputs
modules := $(basename $(notdir ${sources}))
app_file := ${ebin}/${appname}.app
rel_file := ${ebin}/${appname}.rel
beams := $(addprefix ${ebin}/,$(addsuffix .beam,${modules}))
products := ${beams} ${app_file} ${rel_file}

#derived variables
vsn := $(shell perl -ne '/\{vsn,\s*"(.*)"\}/ and print "$$1"' ${app_template})

################################################################
# Rules
################################################################
.PHONY: run doc clean run erlclean scripts test asn1build compile
all: compile

compile: ${sources} 
	mkdir -p ${ebin}
	erlc ${erlc_flags} -o ${ebin} ${includes} ${sources}
	make -C priv

scripts:  compile ${rel_file} ${app_file}
	mkdir -p ${ebin}
	erlc ${erl_path} -o ${ebin} ${rel_file}

shell: scripts
	-mkdir -p ${logs}
	erl ${erl_node} ${erl_path} -boot start_sasl ${erl_config}

run: scripts
	-mkdir -p ${logs}
	erl ${erl_node} ${erl_path}  ${erl_boot} ${erl_config}

runclean:
	-rm -rf ${logs}
	
test: compile
	erlc ${erlc_flags} -o ${ebin} ${includes} ${test_src}	
	erl ${erl_node} ${erl_path} -boot start_sasl ${erl_config} -noshell -s eunit test $(modules) -s init stop 

doc:
	-mkdir -p doc
	erl -noshell -run edoc_run application "'$(appname)'"

clean:
	rm -f ${products}

${app_file}: ${app_template}
	mkdir -p ${ebin}
	perl build/generate_app.pl $< > $@

${rel_file}: ${rel_template} ${app_file}
	mkdir -p ${ebin}
	perl -pe 's/\@\@VSN\@\@/${vsn}/g' $< > $@


	
