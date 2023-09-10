#pragma once

#include <filesystem>
#include <string>
#include <time.h>
#include <unordered_map>
#include <vector>

using std::vector;
using std::string;
using std::unordered_map;

using recursive_directory_iterator = std::filesystem::recursive_directory_iterator;

struct ProfileInfo
{
    string funcName = "unnamed";
    double runtime = 0.0;
    size_t calls = 0;
};

bool operator<(const ProfileInfo& pi1, const ProfileInfo& pi2);
bool operator>(const ProfileInfo& pi1, const ProfileInfo& pi2);

/*
This is a class for profiling childs. It can show runtime for every profiled method and the number of calls.
To use this class you need to inherit this class and call functions 'startProfiling()' and 'stopProfiling()'
at the start and before end function inside function like this:

int func()
{
    startProfiling();
    ...
    some code
    ...
    stopProfiling();
    return 1;
}

*/
class Profiler
{
public:
    Profiler();

    void startProfiling(string funcName = __builtin_FUNCTION()) const;
    void stopProfiling(string funcName = __builtin_FUNCTION()) const;
    void clearAllProfiling() const;

    vector<ProfileInfo> getProfileInfo() const;
    // returns sorted by runtime vector of profile infos
    vector<ProfileInfo> getSortedProfileInfo() const;
    // returns sorted by name vector of profile infos
    vector<ProfileInfo> getNamedProfileInfo() const;

private:
    mutable unordered_map<string, clock_t> _timeMap;
    mutable unordered_map<string, size_t> _callMap;
};


// return lexical sorted vector of paths to files in directory
vector<string> filesInDirectory(const string& directoryPath);

// return lexical sorted vector of paths of only .json files in directory
vector<string> jsonFilesInDirectory(const string& directoryPath);
