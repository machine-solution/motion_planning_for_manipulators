#include "logger.h"

#include <stdexcept>

Logger::Logger(size_t dof)
{
    _dof = dof;
}
Logger::~Logger()
{
    if (_cspaceFile)
    {
        fclose(_cspaceFile);
    }
    if (_mainFile && _mainFile != stdout)
    {
        fclose(_mainFile);
    }
    if (_runtimeFile)
    {
        fclose(_runtimeFile);
    }
    if (_scenFile)
    {
        fclose(_scenFile);
    }
    if (_statsFile)
    {
        fclose(_statsFile);
    }
}

void Logger::prepareCspaceFile(const std::string& filename)
{
    _cspaceFile = fopen(filename.c_str(), "w+");
    if (_cspaceFile == nullptr)
    {
        throw std::runtime_error("Logger::prepareMainFile: Could not open file " + filename);
    }
}
void Logger::prepareMainFile(const std::string& filename)
{
    if (filename == "")
    {
        _mainFile = stdout;
    }
    else
    {
        _mainFile = fopen(filename.c_str(), "w+");
        if (_mainFile == nullptr)
        {
            throw std::runtime_error("Logger::prepareMainFile: Could not open file " + filename);
        }
    }
}
void Logger::prepareRuntimeFile(const std::string& filename)
{
    _runtimeFile = fopen(filename.c_str(), "w+");
    if (_runtimeFile == nullptr)
    {
        throw std::runtime_error("Logger::prepareMainFile: Could not open file " + filename);
    }
    // TODO print header
}
void Logger::prepareScenFile(const std::string& filename)
{
    _scenFile = fopen(filename.c_str(), "w+");
    if (_scenFile == nullptr)
    {
        throw std::runtime_error("Logger::prepareMainFile: Could not open file " + filename);
    }
    printScenLogHeader(_scenFile, _dof);
}
void Logger::prepareStatsFile(const std::string& filename)
{
    _statsFile = fopen(filename.c_str(), "w+");
    if (_statsFile == nullptr)
    {
        throw std::runtime_error("Logger::prepareMainFile: Could not open file " + filename);
    }
    printStatsLogHeader(_statsFile);
}

void Logger::printMainLog(const Solution& solution)
{
    printMainLog(_mainFile, solution);
}
void Logger::printRuntimeLog(const Solution& solution)
{
    printRuntimeLog(_runtimeFile, solution);
}
void Logger::printStatsLog(const Solution& solution)
{
    printStatsLog(_statsFile, solution);
}
void Logger::printScenLog(const Solution& solution, const JointState& startPos, const JointState& goalPos)
{
    printScenLog(_scenFile, solution, startPos, goalPos);
}
void Logger::printCSpace(const vector<string>& cspace)
{
    printCspace(_cspaceFile, cspace);
}

void Logger::printMainLog(FILE* file, const Solution& solution)
{
    std::string yn[] = {"PATH FOUND", "PATH NOT FOUND", "PATH DOES NOT EXIST"};

    fprintf(file, "path verdict: %s\nexpansions: %zu\nmax tree size: %zu\ncost of path: %f\nruntime: %.3fs\n",
        yn[solution.stats.pathVerdict].c_str(),
        solution.stats.expansions,
        solution.stats.maxTreeSize,
        solution.stats.pathCost,
        solution.stats.runtime
    );
    fprintf(file, "---Planner Profile---\n");
    for (const ProfileInfo& info : solution.plannerProfile)
    {
        fprintf(file, "%.1fms\t%zu\t%s\n",
            info.runtime * 1000,
            info.calls,
            info.funcName.c_str()
        );
    }
    fprintf(file, "---Search Tree Profile---\n");
    for (const ProfileInfo& info : solution.searchTreeProfile)
    {
        fprintf(file, "%.1fms\t%zu\t%s\n",
            info.runtime * 1000,
            info.calls,
            info.funcName.c_str()
        );
    }
    fprintf(file, "\n");
}

void Logger::printRuntimeLogHeader(FILE* file, const Solution& solution)
{
    for (const ProfileInfo& info : solution.plannerProfile)
    {
        fprintf(file, "%s_time,", info.funcName.c_str());
        fprintf(file, "%s_calls,", info.funcName.c_str());
    }
    for (const ProfileInfo& info : solution.searchTreeProfile)
    {
        fprintf(file, "%s_time,", info.funcName.c_str());
        fprintf(file, "%s_calls,", info.funcName.c_str());
    }
    fprintf(file, "whole_runtime\n");
}
void Logger::printRuntimeLog(FILE* file, const Solution& solution)
{
    for (const ProfileInfo& info : solution.plannerProfile)
    {
        fprintf(file, "%f,", info.runtime * 1000);
        fprintf(file, "%ld,", info.calls);
    }
    for (const ProfileInfo& info : solution.searchTreeProfile)
    {
        fprintf(file, "%f,", info.runtime * 1000);
        fprintf(file, "%ld,", info.calls);
    }
    fprintf(file, "%f\n", solution.stats.runtime * 1000);
}

void Logger::printStatsLogHeader(FILE* file)
{
    fprintf(file, "expansions,runtime,maxTreeSize,pathCost,pathFound\n");
}
void Logger::printStatsLog(FILE* file, const Solution& solution)
{
    fprintf(file, "%zu,%f,%zu,%f,%d\n",
        solution.stats.expansions,
        solution.stats.runtime,
        solution.stats.maxTreeSize,
        solution.stats.pathCost,
        solution.stats.pathVerdict);
}

void Logger::printScenLogHeader(FILE* file, size_t dof)
{
    for (size_t i = 0; i < dof; ++i)
    {
        fprintf(file, "start_%zu,", i);
    }
    for (size_t i = 0; i < dof; ++i)
    {
        fprintf(file, "goal_%zu,", i);
    }
    fprintf(file, "path_cost,difficulty,runtime\n");
}
void Logger::printScenLog(FILE* file, const Solution& solution, const JointState& startPos, const JointState& goalPos)
{
    for (size_t i = 0; i < startPos.dof(); ++i)
    {
        fprintf(file, "%d,", startPos[i]);
    }
    for (size_t i = 0; i < goalPos.dof(); ++i)
    {
        fprintf(file, "%d,", goalPos[i]);
    }
    fprintf(file, "%f,%f,%f\n", solution.stats.pathCost,
        1.0 * solution.stats.pathCost / solution.stats.pathPotentialCost,
        solution.stats.runtime);
}

void Logger::printCspace(FILE* file, const vector<string>& cSpace)
{
    for (size_t i = 0; i < cSpace.size(); ++i)
    {
        fprintf(file, "%s\n", cSpace[i].c_str());
    }
}
