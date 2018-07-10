cplusplus {{
#include "veins/base/utils/Coord.h"
#include "DVCastHello_m.h" 
}}

class DVCastHello;
class noncobject Coord;

packet DVCastData extends DVCastHello
{
    Coord roiUp;
    Coord roiDown;
}
