FLAGS = -O3 -I include # -mavx -pthread -Wl,-rpath,'$$ORIGIN'
LIBS = -lmujoco -lglfw
CXX = g++
OBJ = obj
SRC = src
INC = include
TARGET = simulator

SOURCES = $(OBJ)/utils.o $(OBJ)/joint_state.o $(OBJ)/planner.o $(OBJ)/astar.o $(OBJ)/solution.o $(OBJ)/interactor.o $(OBJ)/logger.o $(OBJ)/taskset.o $(OBJ)/light_mujoco.o $(OBJ)/lazy_astar.o
INCLUDES = $(INC)/utils.h $(INC)/joint_state.h $(INC)/planner.h $(INC)/astar.h $(INC)/solution.h $(INC)/interactor.h $(INC)/logger.h $(INC)/taskset.h $(INC)/light_mujoco.h $(INC)/global_defs.h $(INC)/doctest.h $(INC)/lazy_astar.h

.PHONY: all clean unit_testing integration_testing simulator 

all: $(TARGET) tests/unit_tests/tests tests/integration_tests/tests

clean:
	rm -rf $(OBJ)
	rm -f $(TARGET)

unit_testing: tests/unit_tests/tests

integration_testing: tests/integration_tests/tests

$(TARGET): $(SOURCES) $(OBJ)/main.o
	$(CXX) $(SOURCES) $(OBJ)/main.o $(LIBS) -o $(TARGET)

tests/unit_tests/tests: $(SOURCES) tests/unit_tests/main.cpp $(INC)/interactor.h $(INC)/planner.h $(INC)/astar.h $(INC)/taskset.h $(INC)/doctest.h $(INC)/utils.h
	$(CXX) $(FLAGS) $(SOURCES) tests/unit_tests/main.cpp $(LIBS) -o tests/unit_tests/tests

tests/integration_tests/tests: $(SOURCES) tests/integration_tests/main.cpp $(INC)/interactor.h $(INC)/planner.h $(INC)/joint_state.h $(INC)/global_defs.h $(INC)/doctest.h
	$(CXX) $(FLAGS) $(SOURCES) tests/integration_tests/main.cpp $(LIBS) -o tests/integration_tests/tests

# compile commands
$(OBJ)/main.o: $(SRC)/main.cpp $(INC)/interactor.h $(INC)/planner.h $(INC)/joint_state.h $(INC)/global_defs.h
	mkdir -p $(OBJ)
	$(CXX) $(FLAGS) $(SRC)/main.cpp $(LIBS) -c -o $(OBJ)/main.o

$(OBJ)/joint_state.o: $(SRC)/joint_state.cpp $(INC)/joint_state.h $(INC)/global_defs.h
	mkdir -p $(OBJ)
	$(CXX) $(FLAGS) $(SRC)/joint_state.cpp $(LIBS) -c -o $(OBJ)/joint_state.o

$(OBJ)/planner.o: $(SRC)/planner.cpp $(INC)/planner.h $(INC)/joint_state.h $(INC)/light_mujoco.h $(INC)/global_defs.h
	mkdir -p $(OBJ)
	$(CXX) $(FLAGS) $(SRC)/planner.cpp $(LIBS) -c -o $(OBJ)/planner.o

$(OBJ)/astar.o: $(SRC)/astar.cpp $(INC)/astar.h $(INC)/utils.h $(INC)/global_defs.h
	mkdir -p $(OBJ)
	$(CXX) $(FLAGS) $(SRC)/astar.cpp $(LIBS) -c -o $(OBJ)/astar.o

$(OBJ)/lazy_astar.o: $(SRC)/lazy_astar.cpp $(INC)/lazy_astar.h $(INC)/utils.h $(INC)/global_defs.h $(INC)/astar.h
	mkdir -p $(OBJ)
	$(CXX) $(FLAGS) $(SRC)/lazy_astar.cpp $(LIBS) -c -o $(OBJ)/lazy_astar.o

$(OBJ)/solution.o: $(SRC)/solution.cpp $(INC)/solution.h $(INC)/utils.h $(INC)/global_defs.h
	mkdir -p $(OBJ)
	$(CXX) $(FLAGS) $(SRC)/solution.cpp $(LIBS) -c -o $(OBJ)/solution.o

$(OBJ)/utils.o: $(SRC)/utils.cpp $(INC)/utils.h
	mkdir -p $(OBJ)
	$(CXX) $(FLAGS) $(SRC)/utils.cpp $(LIBS) -c -o $(OBJ)/utils.o

$(OBJ)/light_mujoco.o: $(SRC)/light_mujoco.cpp $(INC)/light_mujoco.h
	mkdir -p $(OBJ)
	$(CXX) $(FLAGS) $(SRC)/light_mujoco.cpp $(LIBS) -c -o $(OBJ)/light_mujoco.o

$(OBJ)/logger.o: $(SRC)/logger.cpp $(INC)/logger.h $(INC)/solution.h $(INC)/global_defs.h
	mkdir -p $(OBJ)
	$(CXX) $(FLAGS) $(SRC)/logger.cpp $(LIBS) -c -o $(OBJ)/logger.o

$(OBJ)/taskset.o: $(SRC)/taskset.cpp $(INC)/taskset.h $(INC)/global_defs.h
	mkdir -p $(OBJ)
	$(CXX) $(FLAGS) $(SRC)/taskset.cpp $(LIBS) -c -o $(OBJ)/taskset.o

$(OBJ)/interactor.o: $(SRC)/interactor.cpp $(INC)/interactor.h $(INC)/logger.h $(INC)/joint_state.h $(INC)/planner.h $(INC)/taskset.h $(INC)/global_defs.h
	mkdir -p $(OBJ)
	$(CXX) $(FLAGS) $(SRC)/interactor.cpp $(LIBS) -c -o $(OBJ)/interactor.o
