#include "utils.h"

#include <algorithm>

bool operator<(const ProfileInfo& pi1, const ProfileInfo& pi2)
{
    return pi1.runtime < pi2.runtime;
}
bool operator>(const ProfileInfo& pi1, const ProfileInfo& pi2)
{
    return pi1.runtime > pi2.runtime;
}

Profiler::Profiler() {}

void Profiler::startProfiling(std::string funcName)
{
    ++_callMap[funcName];
    _timeMap[funcName] -= clock();
}
void Profiler::endProfiling(std::string funcName)
{
    _timeMap[funcName] += clock();
}
void Profiler::clearAllProfiling()
{
    _timeMap.clear();
    _callMap.clear();
}

vector<ProfileInfo> Profiler::getProfileInfo()
{
    vector<ProfileInfo> result;
    for (auto& mem : _timeMap)
    {
        ProfileInfo info;
        info.funcName = mem.first;
        info.runtime = mem.second;
        info.calls = _callMap[mem.first];
        result.push_back(info);
    }
    return result;
}
vector<ProfileInfo> Profiler::getSortedProfileInfo()
{
    vector<ProfileInfo> result = getProfileInfo();
    std::sort(result.begin(), result.end(), std::greater<ProfileInfo>());
    return result;
}
