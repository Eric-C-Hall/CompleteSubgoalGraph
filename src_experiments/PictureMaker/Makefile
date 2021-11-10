CORNERGRAPH_DIR = ../..
CPP_DIR = $(CORNERGRAPH_DIR)/src
OBJ_DIR = $(CORNERGRAPH_DIR)/obj

CPP_DIRECTORIES = Graph Preprocessing Run Test Utility Visualise Debug Time PathCompetition
CPP_FILES = $(foreach dir,$(CPP_DIRECTORIES),$(wildcard $(CPP_DIR)/$(dir)/*.cpp))
OBJ = $(patsubst $(CPP_DIR)/%.cpp,$(OBJ_DIR)/%.o,$(CPP_FILES))

# Note: Uses debug optimization by defaul
build:
	g++ picture_maker.cpp -o picture_maker $(OBJ) -Og -g
