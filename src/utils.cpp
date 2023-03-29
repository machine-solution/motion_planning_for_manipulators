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

class CmpByName
{
public:
    bool operator()(const ProfileInfo& pi1, const ProfileInfo& pi2)
    {
        return pi1.funcName < pi2.funcName;
    }
};

Profiler::Profiler() {}

void Profiler::startProfiling(std::string funcName) const
{
    ++_callMap[funcName];
    _timeMap[funcName] -= clock();
}
void Profiler::stopProfiling(std::string funcName) const
{
    _timeMap[funcName] += clock();
}
void Profiler::clearAllProfiling() const
{
    _timeMap.clear();
    _callMap.clear();
}

vector<ProfileInfo> Profiler::getProfileInfo() const
{
    vector<ProfileInfo> result;
    for (auto& mem : _timeMap)
    {
        ProfileInfo info;
        info.funcName = mem.first;
        info.runtime = (double)mem.second / CLOCKS_PER_SEC;
        info.calls = _callMap[mem.first];
        result.push_back(info);
    }
    return result;
}
vector<ProfileInfo> Profiler::getSortedProfileInfo() const
{
    vector<ProfileInfo> result = getProfileInfo();
    std::sort(result.begin(), result.end(), std::greater<ProfileInfo>());
    return result;
}

vector<ProfileInfo> Profiler::getNamedProfileInfo() const
{
    vector<ProfileInfo> result = getProfileInfo();
    std::sort(result.begin(), result.end(), CmpByName());
    return result;
}
