#include "vinylCutterInterpreter.h"

bool VinylCutterInterpreter::straightHook(int type, const GCodeInterpreter::MachineState &before, const GCodeInterpreter::MachineState &after)
{
    return true;
}

bool VinylCutterInterpreter::arkHook(const GCodeInterpreter::MachineState &before, const GCodeInterpreter::MachineState &after, const PlaneFrame &frame, double maxArg)
{
    return true;
}
