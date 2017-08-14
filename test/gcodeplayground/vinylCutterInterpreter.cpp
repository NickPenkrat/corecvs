#include "vinylCutterInterpreter.h"

bool VinylCutterInterpreter::straightHook(int type, const GCodeInterpreter::MachineState &before, const GCodeInterpreter::MachineState &after)
{
    if (after.position.z() < touchZ)
    {

        double len = (after.position - before.position).l2Metric();
        Vector2dd moveTangent = (after.position - before.position).normalised().xy();
    }

    //result.program.





    return true;
}

bool VinylCutterInterpreter::arkHook(const GCodeInterpreter::MachineState &before, const GCodeInterpreter::MachineState &after, const PlaneFrame &frame, double maxArg)
{
    SYNC_PRINT(("VinylCutterInterpreter::arkHook(): not supported\n"));

    return true;
}
