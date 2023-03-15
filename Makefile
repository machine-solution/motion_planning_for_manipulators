FLAGS = -I include # -mavx -pthread -Wl,-rpath,'$$ORIGIN'
LIBS = -lmujoco -lglfw
CXX = g++
OBJ = obj
SRC = src
TARGET = simulator

.PHONY: all clean testing simulator

all: $(TARGET) tests/tests

clean:
	rm -rf $(OBJ)
	rm -f $(TARGET)

testing: tests/tests

simulator: $(TARGET)

$(TARGET): $(OBJ)/joint_state.o $(OBJ)/planner.o $(OBJ)/astar.o $(OBJ)/main.o $(OBJ)/solution.o
	$(CXX) $(OBJ)/joint_state.o $(OBJ)/planner.o $(OBJ)/astar.o $(OBJ)/main.o $(OBJ)/solution.o $(LIBS) -o $(TARGET)

tests/tests: $(OBJ)/catch_amalgamated.o $(OBJ)/joint_state.o $(OBJ)/planner.o $(OBJ)/astar.o $(OBJ)/solution.o tests/main.cpp
	$(CXX) $(FLAGS) $(OBJ)/catch_amalgamated.o $(OBJ)/joint_state.o $(OBJ)/planner.o $(OBJ)/astar.o $(OBJ)/solution.o tests/main.cpp $(LIBS) -o tests/tests

# compile commands
$(OBJ)/main.o: $(SRC)/main.cpp include/planner.h
	mkdir -p $(OBJ)
	$(CXX) $(FLAGS) $(SRC)/main.cpp $(LIBS) -c -o $(OBJ)/main.o

$(OBJ)/joint_state.o: $(SRC)/joint_state.cpp include/joint_state.h
	mkdir -p $(OBJ)
	$(CXX) $(FLAGS) $(SRC)/joint_state.cpp $(LIBS) -c -o $(OBJ)/joint_state.o

$(OBJ)/planner.o: $(SRC)/planner.cpp include/planner.h include/joint_state.h include/solution.h
	mkdir -p $(OBJ)
	$(CXX) $(FLAGS) $(SRC)/planner.cpp $(LIBS) -c -o $(OBJ)/planner.o

$(OBJ)/astar.o: $(SRC)/astar.cpp include/astar.h
	mkdir -p $(OBJ)
	$(CXX) $(FLAGS) $(SRC)/astar.cpp $(LIBS) -c -o $(OBJ)/astar.o

$(OBJ)/solution.o: $(SRC)/solution.cpp include/solution.h
	mkdir -p $(OBJ)
	$(CXX) $(FLAGS) $(SRC)/solution.cpp $(LIBS) -c -o $(OBJ)/solution.o

# test "lib" catch2
$(OBJ)/catch_amalgamated.o: tests/catch2/catch_amalgamated.cpp include/catch2/catch_amalgamated.hpp
	mkdir -p $(OBJ)
	$(CXX) $(FLAGS) tests/catch2/catch_amalgamated.cpp -c -o $(OBJ)/catch_amalgamated.o
