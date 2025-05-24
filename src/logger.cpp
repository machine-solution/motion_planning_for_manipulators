#include "logger.h"

#include <stdexcept>

Logger::Logger(size_t dof, size_t arms)
{
    _arms = arms;
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
void Logger::preparePathsFolder(const std::string& filename)
{
    _pathsFolder = filename;
    _incrementalPathNumber = 0;
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
    // Now header is printed when is printed first line of log
}
void Logger::prepareScenFile(const std::string& filename)
{
    _scenFile = fopen(filename.c_str(), "w+");
    if (_scenFile == nullptr)
    {
        throw std::runtime_error("Logger::prepareMainFile: Could not open file " + filename);
    }
    printScenLogHeader(_scenFile, _dof, _arms);
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

void Logger::printMainLog(Stats stats)
{
    printMainLog(_mainFile, stats);
}
void Logger::printRuntimeLog(const Solution& solution)
{
    printRuntimeLog(_runtimeFile, solution);
}
void Logger::printStatsLog(Stats stats)
{
    printStatsLog(_statsFile, stats);
}
void Logger::printScenLog(const MultiSolution& solution, const MultiState& startPos, const MultiState& goalPos)
{
    printScenLog(_scenFile, solution, startPos, goalPos);
}
void Logger::printScenLog(const Solution& solution, const JointState& startPos, double goalX, double goalY)
{
    printScenLog(_scenFile, solution, startPos, goalX, goalY);
}
void Logger::printCSpace(const vector<string>& cspace)
{
    printCspace(_cspaceFile, cspace);
}
void Logger::printPath(const vector<string>& cSpacePath, int number)
{
    std::string strNumber = std::to_string(number);
    if (number < 0)
    {
        strNumber = std::to_string(_incrementalPathNumber);
        ++_incrementalPathNumber;
    }
    std::string pathFilename = _pathsFolder + "path_" + strNumber + ".map";
    FILE* pathFile = fopen(pathFilename.c_str(), "w+");
    printCspace(pathFile, cSpacePath);
    fclose(pathFile);
}

void Logger::printMainLog(FILE* file, Stats stats)
{
    std::string yn[] = {"PATH NOT FOUND", "PATH FOUND", "PATH DOES NOT EXIST"};

    fprintf(file, "path verdict: %s\ncost of path: %f\nruntime: %.3fs\n",
        yn[stats.pathVerdict].c_str(),
        // solution.stats.expansions,
        // solution.stats.byteSize,
        stats.pathCost,
        stats.runtime
    );
    // fprintf(file, "---Planner Profile---\n");
    // for (const ProfileInfo& info : solution.plannerProfile)
    // {
    //     fprintf(file, "%.1fms\t%zu\t%s\n",
    //         info.runtime * 1000,
    //         info.calls,
    //         info.funcName.c_str()
    //     );
    // }
    // fprintf(file, "---Search Tree Profile---\n");
    // for (const ProfileInfo& info : solution.searchTreeProfile)
    // {
    //     fprintf(file, "%.1fms\t%zu\t%s\n",
    //         info.runtime * 1000,
    //         info.calls,
    //         info.funcName.c_str()
    //     );
    // }
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
    if (!_runtimeHaveHeader)
    {
        _runtimeHaveHeader = true;
        printRuntimeLogHeader(file, solution);
    }
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
    fprintf(file, "runtime,pathCost,pathTrivialCost,pathFound,pathTrivial\n");
}
void Logger::printStatsLog(FILE* file, Stats stats)
{
    fprintf(file, "%f,%f,%f,%d,%d\n",
        // solution.stats.expansions,
        stats.runtime,
        // solution.stats.preprocRuntime,
        // solution.stats.byteSize,
        // solution.stats.preprocByteSize,
        stats.pathCost,
        stats.pathTrivialCost,
        // solution.stats.pathPotentialCost,
        stats.pathVerdict,
        // solution.stats.consideredEdges,
        // solution.stats.evaluatedEdges
        stats.pathTrivial
    );
}

void Logger::printScenLogHeader(FILE* file, size_t dof, size_t arms)
{
    fprintf(file, "%ld %ld\n\n", dof, arms);
}
void Logger::printScenLog(FILE* file, const MultiSolution& solution, const MultiState& startPos, const MultiState& goalPos)
{
    for (size_t a = 0; a < startPos.arms(); ++a)
    {
        for (size_t i = 0; i < startPos.dof(); ++i)
        {
            fprintf(file, "%d ", startPos[a][i]);
        }
        fprintf(file, "\n");
    }
    fprintf(file, "\n");
    for (size_t a = 0; a < startPos.arms(); ++a)
    {
        for (size_t i = 0; i < startPos.dof(); ++i)
        {
            fprintf(file, "%d ", goalPos[a][i]);
        }
        fprintf(file, "\n");
    }
    fprintf(file, "\n\n");
}
void Logger::printScenLog(FILE* file, const Solution& solution, const JointState& startPos, double goalX, double goalY)
{
    for (size_t i = 0; i < startPos.dof(); ++i)
    {
        fprintf(file, "%d,", startPos[i]);
    }
    fprintf(file, "%f,%f,", goalX, goalY);
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
