#include <pcl/point_types.h>
#include <unordered_map>
#include <cmath>
#include <vector>
#include <ros/ros.h>

class EntropyCalculator
{
public:
    // 构造函数，初始化体素尺寸和帧参数
    EntropyCalculator(double Entropyleaf)
        : Entropyleaf_(Entropyleaf), min_entropy(2.5), max_entropy(8.0), current_richness_level_(2)
    {
        range = max_entropy - min_entropy;

        // 计算四个阈值
        P1_ = min_entropy + 0.10 * range; // 匮乏
        P2_ = min_entropy + 0.30 * range; // 普通
        P3_ = min_entropy + 0.70 * range; // 丰富
        P4_ = min_entropy + 0.90 * range; // 极丰富
    }
    int calculateRichnessLevel(const pcl::PointCloud<PointType>::Ptr &cloud)
    {

        std::unordered_map<int64_t, int> voxel_counts;
        for (const auto &point : cloud->points)
        {
            int voxel_x = static_cast<int>(point.x / Entropyleaf_);
            int voxel_y = static_cast<int>(point.y / Entropyleaf_);
            int voxel_z = static_cast<int>(point.z / Entropyleaf_);
            int64_t voxel_key = static_cast<int64_t>(voxel_x) * 1000000000LL +
                                static_cast<int64_t>(voxel_y) * 1000000LL +
                                static_cast<int64_t>(voxel_z);
            voxel_counts[voxel_key]++;
        }

        // 计算原始熵值
        double total_points = cloud->size();
        double entropy = 0.0;
        for (const auto &kv : voxel_counts)
        {
            double probability = kv.second / total_points;
            entropy -= probability * log2(probability);
        }

        if (entropy < P1_) // 极匮乏
        {
            if (current_richness_level_ != 4)
            {
                current_richness_level_ = 4;
            }
        }
        else if (entropy < P2_) // 匮乏
        {
            if (current_richness_level_ != 3)
            {
                current_richness_level_ = 3;
            }
        }
        else if (entropy < P3_) // 普通
        {
            if (current_richness_level_ != 2)
            {
                current_richness_level_ = 2;
            }
        }
        else if (entropy < P4_) // 丰富
        {
            if (current_richness_level_ != 1)
            {
                current_richness_level_ = 1;
            }
        }
        else // 极丰富
        {
            if (current_richness_level_ != 0)
            {
                current_richness_level_ = 0;
            }
        }

        // printf("current_richness_level_ = %d\n\n", current_richness_level_);

        return current_richness_level_; // 返回当前的环境丰富程度
    }

private:
    double Entropyleaf_;         // 体素尺寸
    int current_richness_level_; // 当前环境丰富程度
    double max_entropy, min_entropy;
    double P1_, P2_, P3_, P4_;
    double range;
};
