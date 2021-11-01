MAP_DIR = ../../maps
EXPERIMENT_MAP_DIRS = manually-created bgmaps da2 dao wc3maps512 sc1

AVERAGER_FILEPATH = ../Averager/averager

build:
	g++ -Wall run_splitter.cpp -o run_splitter

run_all:
	$(foreach EXPERIMENT_MAP_DIR, $(EXPERIMENT_MAP_DIRS), make --no-print-directory run DIR=$(EXPERIMENT_MAP_DIR);)

run: CURR_RUN_DIR = $(MAP_DIR)/$(DIR)/run
run: CURR_RUN_FILES = $(wildcard $(CURR_RUN_DIR)/*.map.run)
run:
ifneq ($(DIR),)
	$(foreach RUN_FILE, $(CURR_RUN_FILES), ./run_splitter $(RUN_FILE); mv $(RUN_FILE).runtime $(CURR_RUN_DIR)/runtimes;)
	cat $(CURR_RUN_DIR)/runtimes/*.map.run.runtime > $(CURR_RUN_DIR)/runtimes/6at.cat
	./$(AVERAGER_FILEPATH) $(CURR_RUN_DIR)/runtimes/6at.cat > $(CURR_RUN_DIR)/runtimes/4vg.avg
else
	@echo "Pass in a directory using 'make run DIR=...'"
endif

