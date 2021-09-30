CORNERGRAPH_DIR = ../..
CPP_DIR = $(CORNERGRAPH_DIR)/src
OBJ_DIR = $(CORNERGRAPH_DIR)/obj

CPP_DIRECTORIES = Graph Preprocessing Run Test Utility Visualise Debug Time PathCompetition
CPP_FILES = $(foreach dir,$(CPP_DIRECTORIES),$(wildcard $(CPP_DIR)/$(dir)/*.cpp))
OBJ = $(patsubst $(CPP_DIR)/%.cpp,$(OBJ_DIR)/%.o,$(CPP_FILES))

MAP_DIR_NAME = maps
MAP_DIR = $(CORNERGRAPH_DIR)/$(MAP_DIR_NAME)
PREPROCESS_DIR = $(CORNERGRAPH_DIR)/CompleteCornerGraph-$(MAP_DIR_NAME)

EXPERIMENT_MAP_DIRS = manually-created bgmaps da2 dao wc3maps512 sc1

MEMORY_EXTENSIONS = bb ccg corner_vector with_next with_relevant

# Note: Uses debug optimization by default, assumes object files are already built
build:
	g++ memory_splitter.cpp -o memory_splitter $(OBJ) -Og -g

run:
	$(foreach EXPERIMENT_MAP_DIR, $(EXPERIMENT_MAP_DIRS), make run_dir DIR=$(EXPERIMENT_MAP_DIR);)

run_dir: CURR_MAP_SUBDIR = $(MAP_DIR)/$(DIR)
run_dir: CURR_PREPROCESS_SUBDIR = $(PREPROCESS_DIR)/$(DIR)
run_dir:
ifneq ($(DIR),)
	
	$(foreach MAP, $(wildcard $(CURR_MAP_SUBDIR)/*.map), ./memory_splitter $(MAP);)
	$(foreach MEMORY_EXTENSION, $(MEMORY_EXTENSIONS), mkdir -p $(CURR_PREPROCESS_SUBDIR)/mem/$(MEMORY_EXTENSION);)
	$(foreach MEMORY_EXTENSION, $(MEMORY_EXTENSIONS), mv $(CURR_PREPROCESS_SUBDIR)/*.map.$(MEMORY_EXTENSION).mem $(CURR_PREPROCESS_SUBDIR)/mem/$(MEMORY_EXTENSION);)
else
	@echo "Pass in a directory using 'make run_dir DIR=...'"
endif
