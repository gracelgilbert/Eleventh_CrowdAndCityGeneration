#ifndef SIMULATIONDRIVER_H
#define SIMULATIONDRIVER_H

class SimulationDriver {

private:
    float dt;
public:
    SimulationDriver();
    void run(const int frameCount);
    float getDt();


};

#endif // SIMULATIONDRIVER_H
