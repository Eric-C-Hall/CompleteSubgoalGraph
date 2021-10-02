MAP_DIR = ../../maps
EXPERIMENT_MAP_DIRS = manually-created bgmaps da2 dao wc3maps512 sc1

EXTENSIONS = .stats.num_corners\
             .time.corner .time.ccg .time.smooth_graph .time.nearby_corners .time.relevant .time.geometric_container .time.push_corner\
             .nearby.min .nearby.qt1 .nearby.med .nearby.qt2 .nearby.max .nearby.mean .nearby.mode\
             .with_relevant.min .with_relevant.qt1 .with_relevant.med .with_relevant.qt2 .with_relevant.max .with_relevant.mean .with_relevant.mode\
             .with_next.min .with_next.qt1 .with_next.med .with_next.qt2 .with_next.max .with_next.mean .with_next.mode

build:
	g++ -Wall preprocess_splitter.cpp -o preprocess_splitter

run_all:
	$(foreach EXPERIMENT_MAP_DIR, $(EXPERIMENT_MAP_DIRS), make --no-print-directory run DIR=$(EXPERIMENT_MAP_DIR);)

run: CURR_PRE_DIR = $(MAP_DIR)/$(DIR)/pre
run: CURR_PRE_FILES = $(wildcard $(CURR_PRE_DIR)/*.map.pre)
run:
ifneq ($(DIR),)
	$(foreach PRE_FILE, $(CURR_PRE_FILES), ./preprocess_splitter $(PRE_FILE);)
	@$(foreach EXTENSION, $(EXTENSIONS), make --no-print-directory run_ext AUTO=true CURR_PRE_DIR=$(CURR_PRE_DIR) EXTENSION=$(EXTENSION);)
else
	@echo "Pass in a directory using 'make run DIR=...'"
endif

run_ext: CURR_AVG_DIR = $(CURR_PRE_DIR)/avg
run_ext: EXTENSION_SPLIT = $(subst ., ,$(EXTENSION))
run_ext: EXTENSION_FIRST = $(firstword $(EXTENSION_SPLIT))
run_ext: EXTENSION_LAST = $(lastword $(EXTENSION_SPLIT))
run_ext:
ifneq ($(AUTO),)
	mkdir -p $(CURR_AVG_DIR)/$(EXTENSION_FIRST)
	rm -f $(CURR_AVG_DIR)/$(EXTENSION_FIRST)/$(EXTENSION_LAST).cat
	cat $(CURR_PRE_DIR)/*.map.pre.$(EXTENSION_FIRST).$(EXTENSION_LAST) > $(CURR_AVG_DIR)/$(EXTENSION_FIRST)/$(EXTENSION_LAST).cat
	rm -f $(CURR_PRE_DIR)/*.map.pre.$(EXTENSION_FIRST).$(EXTENSION_LAST)
	rm -f $(CURR_AVG_DIR)/$(EXTENSION_FIRST)/$(EXTENSION_LAST).avg
	./../Averager/averager $(CURR_AVG_DIR)/$(EXTENSION_FIRST)/$(EXTENSION_LAST).cat > $(CURR_AVG_DIR)/$(EXTENSION_FIRST)/$(EXTENSION_LAST).avg;
else
	@echo "This should not be called manually"
endif

