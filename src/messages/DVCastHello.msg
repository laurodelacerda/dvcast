cplusplus {{
#include "veins/modules/messages/BasicSafetyMessage_m.h"
}}
packet BasicSafetyMessage;

packet DVCastHello extends BasicSafetyMessage {
    double senderAngle;
}
