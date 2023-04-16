#pragma once

#include "solution.h"

class Logger
{
public:
    Logger(size_t dof);
    ~Logger();

    void prepareCspaceFile(const std::string& filename);
    // empty filename means stdout
    void prepareMainFile(const std::string& filename);
    void prepareRuntimeFile(const std::string& filename);
    void prepareScenFile(const std::string& filename);
    void prepareStatsFile(const std::string& filename);

    void printMainLog(const Solution& solution);
    void printRuntimeLog(const Solution& solution);
    void printStatsLog(const Solution& solution);
    void printScenLog(const Solution& solution, const JointState& startPos, const JointState& goalPos);
    void printCSpace(const vector<string>& cSpace);

private:
    void printMainLog(FILE* file, const Solution& solution);

    void printRuntimeLogHeader(FILE* file, const Solution& solution);
    void printRuntimeLog(FILE* file, const Solution& solution);

    void printStatsLogHeader(FILE* file);
    void printStatsLog(FILE* file, const Solution& solution);

    void printScenLogHeader(FILE* file, size_t dof);
    void printScenLog(FILE* file, const Solution& solution, const JointState& startPos, const JointState& goalPos);

    void printCspace(FILE* file, const vector<string>& cSpace);

    FILE* _cspaceFile = nullptr;
    FILE* _mainFile = nullptr;
    FILE* _runtimeFile = nullptr;
    FILE* _scenFile = nullptr;
    FILE* _statsFile = nullptr;

    size_t _dof;
};
