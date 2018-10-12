#pragma once

#include <fstream>
#include <string>
#include <vector>
#include <experimental/filesystem>

#include "analytics.h"

using namespace std;


class MCLogger
{

public:
    MCLogger(string directory, string prefix);
    ~MCLogger();

    void log(const Analytics& a);

private:

    class MultiThreadStream
    {
    private:
        ofstream stream_;
        mutex mutex_;
    public:
        void open(string filename)
        {
            mutex_.lock();
            stream_.open(filename);
            mutex_.unlock();
        }
        void close()
        {
            mutex_.lock();
            stream_.close();
            mutex_.unlock();
        }
        void write(std::vector<double> data)
        {
            mutex_.lock();
            stream_.write((char*)data.data(), sizeof(double)*data.size());
            mutex_.unlock();
        }
    };

    MultiThreadStream total_RMS_;
    MultiThreadStream transform_RMS_;
    MultiThreadStream NEES_;
};
