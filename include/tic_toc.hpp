#include <ctime>
#include <cstdlib>
#include <chrono>
#include <fstream>
#include <ros/ros.h>

int odometryframe_count = 0;

namespace odometry_timer
{
    class TicToc
    {
    public:
        TicToc(std::string time_file_path) : file_path(time_file_path)
        {
            tic(); // 默认构造函数调用时间开始成员函数

            // 打开文件，若不存在则创建
            out_file.open(file_path, std::ios::out | std::ios::app);
            if (!out_file.is_open())
            {
                ROS_ERROR("Failed to open file: %s", file_path.c_str());
            }

            if (out_file.tellp() == 0)
            { // 如果文件是空的，则写标题
                out_file << "frame_count,time(ms)\n";
            }
        }

        ~TicToc()
        {
            if (out_file.is_open())
            {
                out_file.close();
            }
        }

        void tic()
        {
            start = std::chrono::system_clock::now();
        }

        void toc()
        {
            end = std::chrono::system_clock::now();
            std::chrono::duration<double> elapsed_seconds = end - start;
            double elapsed_time_ms = elapsed_seconds.count() * 1000;

            out_file << odometryframe_count++ << "," << elapsed_time_ms << "\n";
        }

    private:
        std::chrono::time_point<std::chrono::system_clock> start, end;
        std::ofstream out_file;
        std::string file_path;
    };

}