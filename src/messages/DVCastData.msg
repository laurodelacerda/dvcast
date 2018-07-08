cplusplus {{
#include "veins/base/utils/Coord.h"
#include "veins/modules/messages/BasicSafetyMessage_m.h"
}}

class BasicSafetyMessage;
class noncobject Coord;

packet DVCastData extends BasicSafetyMessage
{
    double senderAngle;
    Coord roiUp;
    Coord roiDown;
}
