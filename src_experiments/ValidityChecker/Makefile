build:
	g++ -Wall main.cpp -o validity_checker

CORNERGRAPH_DIR = ../..
MAP_DIR = $(CORNERGRAPH_DIR)/maps
EXPERIMENT_MAP_DIRS = manually-created bgmaps da2 dao wc3maps512 sc1
RESULTS = $(foreach EXPERIMENT_MAP_DIR, $(EXPERIMENT_MAP_DIRS), $(wildcard $(MAP_DIR)/$(EXPERIMENT_MAP_DIR)/run/*.map.run))

experiment_valid:
	$(foreach RESULTS_FILE,$(RESULTS),./validity_checker $(RESULTS_FILE);)

clean:
	rm -f validity_checker

# eof
