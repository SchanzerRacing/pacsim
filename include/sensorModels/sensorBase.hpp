#ifndef PACSIMISENSORBASE_HPP
#define PACSIMISENSORBASE_HPP

#include <queue>
#include <random>

template <typename T> class SensorBase
{
public:
    double getRate() { return this->rate; }

    T getOldest()
    {
        T elem = this->deadTimeQueue.front();
        this->deadTimeQueue.pop();
        return elem;
    }

    bool availableDeadTime(double time)
    {
        if (this->deadTimeQueue.size() >= 1)
        {
            T elem = this->deadTimeQueue.front();
            if (time >= (elem.timestamp + this->deadTime))
            {
                return true;
            }
        }
        return false;
    }

    bool sampleReady(double time) { return (time >= (this->lastSampleTime + 1 / this->rate)); }

    void registerSampling()
    {
        this->lastSampleTime += 1.0 / this->rate;
        return;
    }

    Eigen::Vector3d getPosition() { return this->position; }

    Eigen::Vector3d getOrientation() { return this->orientation; }

    void setRandomSeed(int seed, unsigned int stream)
    {
        std::seed_seq sequence { static_cast<unsigned int>(seed), stream };
        this->randomGenerator.seed(sequence);
    }

protected:
    Eigen::Vector3d position;
    Eigen::Vector3d orientation;

    double rate;
    double lastSampleTime;
    double deadTime;
    std::queue<T> deadTimeQueue;
    int numFrames;
    std::mt19937 randomGenerator { 0 };
};

#endif /* PACSIMISENSORBASE_HPP */
