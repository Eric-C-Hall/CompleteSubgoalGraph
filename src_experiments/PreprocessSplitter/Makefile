CORNER_GRAPH_DIR = ../..
MAP_DIR = $(CORNER_GRAPH_DIR)/maps
EXPERIMENT_MAP_DIRS = manually-created bgmaps da2 dao wc3maps512 sc1

PREPROCESS_EXTENSIONS = num_corners corner ccg smooth_graph nearby_corners relevant geometric_container push_corner

build:
	g++ -Wall preprocess_splitter.cpp -o preprocess_splitter

run:
	$(foreach EXPERIMENT_MAP_DIR, $(EXPERIMENT_MAP_DIRS), make run_dir DIR=$(EXPERIMENT_MAP_DIR);)

run_dir: CURR_PRE_DIR = $(MAP_DIR)/$(DIR)/pre
run_dir: CURR_PRE_AVG_DIR = $(CURR_PRE_DIR)/avg
run_dir:
ifneq ($(DIR),)
	$(foreach PRE_FILE, $(wildcard $(CURR_PRE_DIR)/*.map.pre), ./preprocess_splitter $(PRE_FILE);)
	$(foreach PREPROCESS_EXTENSION, $(PREPROCESS_EXTENSIONS), cat $(CURR_PRE_DIR)/*.map.pre.$(PREPROCESS_EXTENSION) > $(CURR_PRE_AVG_DIR)/$(PREPROCESS_EXTENSION).cat;)
	$(foreach PREPROCESS_EXTENSION, $(PREPROCESS_EXTENSIONS), rm -f $(CURR_PRE_DIR)/*.map.pre.$(PREPROCESS_EXTENSION);)
	$(foreach PREPROCESS_EXTENSION, $(PREPROCESS_EXTENSIONS), ./../Averager/averager $(CURR_PRE_AVG_DIR)/$(PREPROCESS_EXTENSION).cat > $(CURR_PRE_AVG_DIR)/$(PREPROCESS_EXTENSION).avg;)
else
	@echo "Pass in a directory using 'make run DIR=...'"
endif
