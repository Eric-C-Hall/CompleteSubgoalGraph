CPP_DIR = src
OBJ_DIR = obj
MAP_DIR = maps
EXPERIMENT_MAP_DIR = $(MAP_DIR)/experiment_maps
EXPERIMENT_DIR = $(EXPERIMENT_MAP_DIR)/experiments
EXPERIMENT_MAPS = $(wildcard $(EXPERIMENT_MAP_DIR)/*.map)
CPP_DIRECTORIES = Graph Preprocessing Run/GetPath Run/RunScenario Test Utility Visualise Debug Time
CPP_FILES = $(wildcard $(CPP_DIR)/*.cpp) $(foreach dir,$(CPP_DIRECTORIES),$(wildcard $(CPP_DIR)/$(dir)/*.cpp))
OBJ = $(patsubst $(CPP_DIR)/%.cpp,$(OBJ_DIR)/%.o,$(CPP_FILES))

OPTIMIZATION_LEVEL=-O3
CONCURRENCY_ARGUMENTS=-pthread

# Use this to disable assertions. "ASSERTION_ARGUMENTS=-DNDEBUG" should work.
ASSERTION_ARGUMENTS=
#ASSERTION_ARGUMENTS=-DNDEBUG

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

build: $(OBJ)
	g++ -Wall -o cornergraph $(OPTIMIZATION_LEVEL) $(ADDITIONAL_ARGUMENTS) $(OBJ) $(CONCURRENCY_ARGUMENTS) $(ASSERTION_ARGUMENTS)

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
	./cornergraph -run $(MAP_DIR)/$(MAP).map $(MAP_DIR)/$(MAP).map.scen
else
	@echo "Pass in a map using 'make run MAP=...'. Do not include the file extension."
endif

rundbg:
ifneq ($(MAP),)
	gdb --args cornergraph -run $(MAP_DIR)/$(MAP).map $(MAP_DIR)/$(MAP).map.scen
else
	@echo "Pass in a map using 'make rundbg MAP=...'. Do not include the file extension."
endif

experiment_pre:
	rm -f $(EXPERIMENT_MAP_DIR)/*.map.cornergraph
	$(foreach EXPERIMENT_MAP, $(EXPERIMENT_MAPS), timeout 1h ./cornergraph -pre $(EXPERIMENT_MAP) $(EXPERIMENT_MAP).scen;)

experiment_run:
	rm -f $(EXPERIMENT_DIR)/*.map.results
	$(foreach EXPERIMENT_MAP, $(EXPERIMENT_MAPS), (./cornergraph -run $(EXPERIMENT_MAP) $(EXPERIMENT_MAP).scen) > $(patsubst $(EXPERIMENT_MAP_DIR)/%.map,$(EXPERIMENT_DIR)/%.map.results,$(EXPERIMENT_MAP));)


experiment_load_maps:
ifneq ($(DIR),)
	rm -f $(EXPERIMENT_MAP_DIR)/*.map
	rm -f $(EXPERIMENT_MAP_DIR)/*.map.scen
	cp $(MAP_DIR)/$(DIR)-map/*.map $(EXPERIMENT_MAP_DIR)
	cp $(MAP_DIR)/$(DIR)-scen/*.map.scen $(EXPERIMENT_MAP_DIR)
else
	@echo "Pass in a directory using 'make experiment_load_maps DIR=...'"
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

cleanpre:
	rm -f $(MAP_DIR)/*.map.cornergraph

# eof
