#ifndef Tick_Tock_H
#define Tick_Tock_H
#include <chrono>
#include <map>
#include "ConsoleColor.h"

namespace tool
{
    class Duration
    {
        public:
        void Tick()
        {
            start_time = std::chrono::steady_clock::now();
        }
        void Tock()
        {
            end_time = std::chrono::steady_clock::now();
        }
        float Elapsed()
        {
            std::chrono::duration<float> diff = end_time-start_time;
            if(diff.count() < 0)
            {
                std::cout<<YELLOW<<"[TickTock]::[WARNING]::Elapsed time is less than zero." <<RESET<<std::endl;
            }
            return diff.count()*1000;
        }

        std::chrono::steady_clock::time_point start_time;
        std::chrono::steady_clock::time_point end_time;
        
    };

    class Timer
    {
        public:
        void Tick(const std::string &name)
        {
            durations[name].Tick();
        }
        void Tock(const std::string &name)
        {
            if(durations.find(name) == durations.end())
            {
                std::cout<<YELLOW<<"[TickTock]::[WARNING]::Tock without Tick!" <<RESET<<std::endl;
                return;
            }
            durations[name].Tock();
        }
        float Elapsed(const std::string &name)
        {
            return durations[name].Elapsed();
        }

        void Log(const std::string &name)
        {
            std::cout<<name+"::"<<durations[name].Elapsed()<<"ms" <<std::endl;
        }

        void LogAll()
        {
            for(auto i = durations.begin(); i!= durations.end(); ++i)
            {
                Log(i->first);
            }
        }

        void Reset()
        {
            durations.clear();
        }
        std::map<std::string, Duration> durations;
        
    };
}

#endif