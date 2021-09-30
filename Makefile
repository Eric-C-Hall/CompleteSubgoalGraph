CPP_DIR = src
OBJ_DIR = obj
CPP_DIRECTORIES = Graph Preprocessing Run Test Utility Visualise Debug Time PathCompetition
CPP_FILES = $(foreach dir,$(CPP_DIRECTORIES),$(wildcard $(CPP_DIR)/$(dir)/*.cpp))
OBJ = $(patsubst $(CPP_DIR)/%.cpp,$(OBJ_DIR)/%.o,$(CPP_FILES))

MAP_DIR = maps
PREPROCESS_DIR = CompleteCornerGraph-$(MAP_DIR)
EXPERIMENT_MAP_DIRS = manually-created bgmaps da2 dao wc3maps512 sc1

OPTIMIZATION_LEVEL=-O3
CONCURRENCY_ARGUMENTS=-pthread

# Use this to disable assertions. "ASSERTION_ARGUMENTS=-DNDEBUG" should work.
#ASSERTION_ARGUMENTS=
ASSERTION_ARGUMENTS=-DNDEBUG

# -------------------------------
# List of commands:
#
# build (compile)
# debug (compile for debug purposes)
# pre (preprocess)
# predbg (prepreprocess using a debugger)
# run (run a scenario)
# rundbg (run a scenario using a debugger)
# test (test for correctness against Dijksrta)
# testdbg (test for correctness against Dijkstra using a debugger)
# testprof (test for correctness against Dijkstra using a profiler)
# time (time how long it takes to solve random queries)
# -------------------------------

build: $(OBJ_DIR)/main.o $(OBJ)
	g++ -Wall -o cornergraph $(OPTIMIZATION_LEVEL) $(ADDITIONAL_ARGUMENTS) $(OBJ_DIR)/main.o $(OBJ) $(CONCURRENCY_ARGUMENTS) $(ASSERTION_ARGUMENTS)
	g++ -Wall -o cornergraph_gppc $(OPTIMIZATION_LEVEL) $(ADDITIONAL_ARGUMENTS) $(OBJ_DIR)/main_gppc.o $(OBJ) $(CONCURRENCY_ARGUMENTS) $(ASSERTION_ARGUMENTS)

debug: OPTIMIZATION_LEVEL=-Og
debug: ADDITIONAL_ARGUMENTS=-g
debug: build

pre:
ifneq ($(MAP),)
	./cornergraph -pre $(MAP_DIR)/$(MAP).map $(MAP_DIR)/$(MAP).map.scen
else
	@echo "Pass in a map using 'make pre MAP=...'. Do not include the file extension."
endif

predbg:
ifneq ($(MAP),)
	gdb --args cornergraph -pre $(MAP_DIR)/$(MAP).map $(MAP_DIR)/$(MAP).map.scen
else
	@echo "Pass in a map using 'make predbg MAP=...'. Do not include the file extension."
endif

run:
ifneq ($(MAP),)
	./cornergraph_gppc -run $(MAP_DIR)/$(MAP).map $(MAP_DIR)/$(MAP).map.scen
else
	@echo "Pass in a map using 'make run MAP=...'. Do not include the file extension."
endif

rundbg:
ifneq ($(MAP),)
	gdb --args cornergraph -run $(MAP_DIR)/$(MAP).map $(MAP_DIR)/$(MAP).map.scen
else
	@echo "Pass in a map using 'make rundbg MAP=...'. Do not include the file extension."
endif

experiment_pre_all:
	$(foreach EXPERIMENT_MAP_DIR, $(EXPERIMENT_MAP_DIRS), time -f "Real: %E\nUser: %U\nSys: %S\n" -o $(MAP_DIR)/$(EXPERIMENT_MAP_DIR)/7ime.txt make experiment_pre DIR=$(EXPERIMENT_MAP_DIR);)

experiment_pre:
ifneq ($(DIR),)
	rm -f $(MAP_DIR)/$(DIR)/pre/*.map.pre
	rm -f $(MAP_DIR)/$(DIR)/pre/avg/*.avg
	rm -f $(MAP_DIR)/$(DIR)/pre/avg/*.cat
	rm -f $(PREPROCESS_DIR)/$(DIR)/*.map
	$(foreach EXPERIMENT_MAP, $(wildcard $(MAP_DIR)/$(DIR)/*.map), timeout 1h ./cornergraph_gppc -pre $(EXPERIMENT_MAP) $(EXPERIMENT_MAP).scen > $(patsubst $(MAP_DIR)/$(DIR)/%.map,$(MAP_DIR)/$(DIR)/pre/%.map.pre,$(EXPERIMENT_MAP));)
else
	@echo "Pass in a directory using 'make experiment_pre DIR=...'"
endif

experiment_run_all:
	$(foreach EXPERIMENT_MAP_DIR, $(EXPERIMENT_MAP_DIRS), make experiment_run DIR=$(EXPERIMENT_MAP_DIR);)

experiment_run:
ifneq ($(DIR),)
	rm -f $(MAP_DIR)/$(DIR)/run/*.map.run
	$(foreach EXPERIMENT_MAP, $(wildcard $(MAP_DIR)/$(DIR)/*.map), (./cornergraph_gppc -run $(EXPERIMENT_MAP) $(EXPERIMENT_MAP).scen) > $(patsubst $(MAP_DIR)/$(DIR)/%.map,$(MAP_DIR)/$(DIR)/run/%.map.run,$(EXPERIMENT_MAP));)
else
	@echo "Pass in a directory using 'make experiment_run DIR=...'"
endif

experiment_time_all:
	$(foreach EXPERIMENT_MAP_DIR, $(EXPERIMENT_MAP_DIRS), make experiment_time DIR=$(EXPERIMENT_MAP_DIR);)

experiment_time:
ifneq ($(DIR),)
	rm -f $(MAP_DIR)/$(DIR)/time/*.map.time
	rm -f $(MAP_DIR)/$(DIR)/time/avg/*.avg
	rm -f $(MAP_DIR)/$(DIR)/time/avg/*.cat
	rm -f $(MAP_DIR)/$(DIR)/time/avg/percentages.time
	$(foreach EXPERIMENT_MAP, $(wildcard $(MAP_DIR)/$(DIR)/*.map), (./cornergraph -time $(EXPERIMENT_MAP) $(EXPERIMENT_MAP).scen) > $(patsubst $(MAP_DIR)/$(DIR)/%.map,$(MAP_DIR)/$(DIR)/time/%.map.time,$(EXPERIMENT_MAP));)
else
	@echo "Pass in a directory using 'make experiment_time DIR=...'"
endif	

vis:
ifneq ($(MAP),)
	./cornergraph -vis $(MAP_DIR)/$(MAP).map $(MAP_DIR)/$(MAP).map.scen
else
	@echo "Pass in a map using 'make vis MAP=...'. Do not include the file extension."
endif

visdbg:
ifneq ($(MAP),)
	gdb --args ./cornergraph -vis $(MAP_DIR)/$(MAP).map $(MAP_DIR)/$(MAP).map.scen
else
	@echo "Pass in a map using 'make visdbg MAP=...'. Do not include the file extension."
endif

test:
ifneq ($(MAP),)
	./cornergraph -test $(MAP_DIR)/$(MAP).map $(MAP_DIR)/$(MAP).map.scen
else
	@echo "Pass in a map using 'make test MAP=...'. Do not include the file extension."
endif

testdbg:
ifneq ($(MAP),)
	gdb --args cornergraph -test $(MAP_DIR)/$(MAP).map $(MAP_DIR)/$(MAP).map.scen
else
	@echo "Pass in a map using 'make testdbg MAP=...'. Do not include the file extension."
endif

testprof:
ifneq ($(MAP),)
	@echo "Make sure to run kcachegrind after letting valgrind do it's thing"
	valgrind --tool=callgrind ./cornergraph -test $(MAP_DIR)/$(MAP).map $(MAP_DIR)/$(MAP).map.scen
	kcachegrind
else
	@echo "Pass in a map using 'make testprof MAP=...'. Do not include the file extension."
endif

time:
ifneq ($(MAP),)
	./cornergraph -time $(MAP_DIR)/$(MAP).map $(MAP_DIR)/$(MAP).map.scen
else
	@echo "Pass in a map using 'make time MAP=...'. Do not include the file extension."
endif

timedbg:
ifneq ($(MAP),)
	gdb --args cornergraph -time $(MAP_DIR)/$(MAP).map $(MAP_DIR)/$(MAP).map.scen
else
	@echo "Pass in a map using 'make timedbg MAP=...'. Do not include the file extension."
endif

$(OBJ_DIR)/%.o: $(CPP_DIR)/%.cpp $(CPP_DIR)/%.h
	g++ -Wall -c $< -std=c++11 $(OPTIMIZATION_LEVEL) $(ADDITIONAL_ARGUMENTS) $(CONCURRENCY_ARGUMENTS) $(ASSERTION_ARGUMENTS) -o $@

$(OBJ_DIR)/%.o: $(CPP_DIR)/%.cpp $(CPP_DIR)/%.hpp
	g++ -Wall -c $< -std=c++11 $(OPTIMIZATION_LEVEL) $(ADDITIONAL_ARGUMENTS) $(CONCURRENCY_ARGUMENTS) $(ASSERTION_ARGUMENTS) -o $@

$(OBJ_DIR)/%.o: $(CPP_DIR)/%.cpp
	g++ -Wall -c $< -std=c++11 $(OPTIMIZATION_LEVEL) $(ADDITIONAL_ARGUMENTS) $(CONCURRENCY_ARGUMENTS) $(ASSERTION_ARGUMENTS) -o $@

clean:
	rm -f $(OBJ)
	rm -f cornergraph
	rm -f cornergraph_gppc

cleanpre:
	rm -f $(MAP_DIR)/*.map.cornergraph
	rm -f $(PREPROCESS_DIR)/*.map

# eof
