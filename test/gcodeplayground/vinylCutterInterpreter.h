#ifndef VINYLCUTTERINTERPRETER_H
#define VINYLCUTTERINTERPRETER_H

#include "gcodeLoader.h"


class VinylCutterInterpreter : public GCodeInterpreter {

public:
    GCodeProgram result;


    // GCodeInterpreter interface
public:
    virtual bool straightHook(int type, const MachineState &before, const MachineState &after) override;
    virtual bool arkHook(const MachineState &before, const MachineState &after, const PlaneFrame &frame, double maxArg) override;



};
#endif // VINYLCUTTERINTERPRETER_H
