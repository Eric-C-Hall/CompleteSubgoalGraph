CORNERGRAPH_DIR = ../..
MAP_DIR = $(CORNERGRAPH_DIR)/maps

MAP_SUBDIRS = manually-created bgmaps da2 dao wc3maps512 sc1

build:
	g++ num_empty_counter.cpp -o num_empty_counter

num_empty:
	$(foreach MAP_SUBDIR, $(MAP_SUBDIRS), make num_empty_dir DIR=$(MAP_SUBDIR);)

num_empty_dir: CURR_MAP_DIR = $(MAP_DIR)/$(DIR)
num_empty_dir: CURR_STAT_DIR = $(CURR_MAP_DIR)/stats
num_empty_dir: CURR_NUM_EMPTY_DIR = $(CURR_STAT_DIR)/num_empty
num_empty_dir: CURR_CAT_FILE = $(CURR_NUM_EMPTY_DIR)/num_empty.cat
num_empty_dir: CURR_STATS_FILE = $(CURR_STAT_DIR)/num_empty.stats
num_empty_dir: CURR_MAPS = $(wildcard $(CURR_MAP_DIR)/*.map)
num_empty_dir:
ifneq ($(DIR),)
	rm -f $(CURR_STATS_FILE)
	mkdir -p $(CURR_NUM_EMPTY_DIR)
	rm -f $(CURR_NUM_EMPTY_DIR)/*.map.num_empty.stats
	$(foreach MAP, $(CURR_MAPS), ./num_empty_counter $(MAP) > $(patsubst $(CURR_MAP_DIR)/%.map,$(CURR_NUM_EMPTY_DIR)/%.map.num_empty.stats,$(MAP));)
	cat $(CURR_NUM_EMPTY_DIR)/*.map.num_empty.stats > $(CURR_CAT_FILE)
	./../Averager/averager $(CURR_CAT_FILE) > $(CURR_STATS_FILE)
else
	@echo "Pass in a directory using 'make num_empty_dir DIR=...'"
endif
