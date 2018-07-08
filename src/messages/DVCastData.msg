cplusplus {{
#include "veins/base/utils/Coord.h"
//#include "veins/modules/messages/BasicSafetyMessage_m.h"
#include "DVCastHello_m.h" 
}}

//class BasicSafetyMessage;
class DVCastHello;
class noncobject Coord;

packet DVCastData extends DVCastHello//BasicSafetyMessage
{
//    double senderAngle;
    Coord roiUp;
    Coord roiDown;
}
