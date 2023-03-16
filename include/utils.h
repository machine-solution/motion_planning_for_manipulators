#pragma once

#include <unordered_map>
#include <string>
#include <vector>
#include <time.h>

using std::vector;
using std::string;
using std::unordered_map;

struct ProfileInfo
{
    string funcName = "unnamed";
    double runtime = 0.0;
    size_t calls = 0;
};

bool operator<(const ProfileInfo& pi1, const ProfileInfo& pi2);
bool operator>(const ProfileInfo& pi1, const ProfileInfo& pi2);

class Profiler
{
public:
    Profiler();

    void startProfiling(string funcName = __builtin_FUNCTION());
    void endProfiling(string funcName = __builtin_FUNCTION());
    void clearAllProfiling();

    vector<ProfileInfo> getProfileInfo();
    // returns sorted by runtime vector of profile infos
    vector<ProfileInfo> getSortedProfileInfo();

private:
    unordered_map<string, clock_t> _timeMap;
    unordered_map<string, size_t> _callMap;
};
