#ifndef ORB_SLAM3_VIZ_VIEWER_PROFILE_H
#define ORB_SLAM3_VIZ_VIEWER_PROFILE_H

#include <algorithm>
#include <chrono>
#include <cstdlib>
#include <iostream>
#include <vector>

namespace ORB_SLAM3
{
class VizViewerProfile
{
public:
    using Clock = std::chrono::steady_clock;

    VizViewerProfile(const char* name, std::initializer_list<const char*> labels): mName(name), mLabels(labels), mSum(labels.size(), 0), mMax(labels.size(), 0), mCount(0), mEnabled(std::getenv("ORB3_VIEWER_PROFILE") != nullptr)
    {
    }

    ~VizViewerProfile()
    {
        if(!mEnabled || !mCount)
            return;
        std::cerr << "[ORB3 VIEWER PROFILE] " << mName << " samples=" << mCount;
        for(size_t i = 0; i < mLabels.size(); ++i)
            std::cerr << " " << mLabels[i] << "_avg_ms=" << mSum[i] / mCount << " " << mLabels[i] << "_max_ms=" << mMax[i];
        std::cerr << std::endl;
    }

    static double Milliseconds(const Clock::time_point &begin, const Clock::time_point &end)
    {
        return std::chrono::duration<double, std::milli>(end - begin).count();
    }

    void Add(std::initializer_list<double> values)
    {
        if(!mEnabled)
            return;
        size_t i = 0;
        for(double value : values)
        {
            mSum[i] += value;
            mMax[i] = std::max(mMax[i], value);
            ++i;
        }
        ++mCount;
    }

private:
    const char* mName;
    std::vector<const char*> mLabels;
    std::vector<double> mSum, mMax;
    size_t mCount;
    bool mEnabled;
};
}

#endif
