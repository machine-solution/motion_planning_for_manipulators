#pragma once

#include "solution.h"

class Logger
{
public:
    Logger(size_t dof, size_t arms);
    ~Logger();

    void prepareCspaceFile(const std::string& filename);
    void preparePathsFolder(const std::string& filename);
    // empty filename means stdout
    void prepareMainFile(const std::string& filename);
    void prepareRuntimeFile(const std::string& filename);
    void prepareScenFile(const std::string& filename);
    void prepareStatsFile(const std::string& filename);

    void printMainLog(Stats stats);
    void printRuntimeLog(const Solution& solution);
    void printStatsLog(Stats stats);
    void printScenLog(const MultiSolution& solution, const MultiState& startPos, const MultiState& goalPos);
    void printScenLog(const Solution& solution, const JointState& startPos, double goalX, double goalY);
    void printCSpace(const vector<string>& cSpace);
    // if number non-negative, print to file numbered by it,
    // else use auto increment from 0
    void printPath(const vector<string>& cSpacePath, int number = -1);

private:
    void printMainLog(FILE* file, Stats stats);

    void printRuntimeLogHeader(FILE* file, const Solution& solution);
    void printRuntimeLog(FILE* file, const Solution& solution);

    void printStatsLogHeader(FILE* file);
    void printStatsLog(FILE* file, Stats stats);

    void printScenLogHeader(FILE* file, size_t dof, size_t arms);
    void printScenLog(FILE* file, const MultiSolution& solution, const MultiState& startPos, const MultiState& goalPos);
    void printScenLog(FILE* file, const Solution& solution, const JointState& startPos, double goalX, double goalY);

    void printCspace(FILE* file, const vector<string>& cSpace);

    FILE* _cspaceFile = nullptr;
    FILE* _mainFile = nullptr;
    FILE* _runtimeFile = nullptr;
    FILE* _scenFile = nullptr;
    FILE* _statsFile = nullptr;

    std::string _pathsFolder = "";
    int _incrementalPathNumber = 0;

    bool _runtimeHaveHeader = false;

    size_t _dof, _arms;
};
