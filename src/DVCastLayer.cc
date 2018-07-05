
#include "DVCastLayer.h"

Define_Module(DVCastLayer);

void DVCastLayer::initialize(int stage)
{
    BaseWaveApplLayer::initialize(stage);
    if (stage == 0)
    {
        //Initializing members and pointers of your application goes here
        EV << "Initializing " << par("appName").stringValue() << std::endl;
        beaconUpdate = simTime();
        lastDroveAt = simTime();
        sentMessage = false;
    }
    else if (stage == 1)
    {
        //Initializing members that require initialized other modules goes here

    }
}

void DVCastLayer::finish() {
    BaseWaveApplLayer::finish();
    //statistics recording goes here

}

void DVCastLayer::onBSM(BasicSafetyMessage* bsm)
{
    //Your application has received a beacon message from another car or RSU
    //code for handling the message goes here

    if(DVCastHello* hello = dynamic_cast<DVCastHello*>(bsm))
    {
        /*
        EV_INFO << "[DVCAST] Got hello from " << hello->getSenderAddress()
                << "| Pos: "   << hello->getSenderPos()
                << "| Angle: " << hello->getSenderAngle()
                << endl;
        */
        updateTopology(hello);
    }
}

void DVCastLayer::onWSM(WaveShortMessage* wsm) {
    //Your application has received a data message from another car or RSU
    //code for handling the message goes here, see TraciDemo11p.cc for examples

}

void DVCastLayer::onWSA(WaveServiceAdvertisment* wsa) {
    //Your application has received a service advertisement from another car or RSU
    //code for handling the message goes here, see TraciDemo11p.cc for examples

}

void DVCastLayer::handleSelfMsg(cMessage* msg) {
    //this method is for self messages (mostly timers)
    //it is important to call the BaseWaveApplLayer function for BSM and WSM transmission
    BaseWaveApplLayer::handleSelfMsg(msg);
}

void DVCastLayer::handlePositionUpdate(cObject* obj) {
    BaseWaveApplLayer::handlePositionUpdate(obj);
    //the vehicle has moved. Code that reacts to new positions goes here.
    //member variables such as currentPosition and currentSpeed are updated in the parent class

    if (simTime() - beaconUpdate >= 1)
    {
        DVCastHello* hello = new DVCastHello("hello", 2);
        populateWSM(hello);
        hello->setSenderAngle(convertAngleToDegrees(mobility->getAngleRad()));
        sendDown(hello);
        beaconUpdate = simTime();
    }
}

void DVCastLayer::onHello(DVCastHello* msg){

    EV << "****onHello" << endl;

    EV << "My Position x:" << mobility->getCurrentPosition().x
       << " My Position y:" << mobility->getCurrentPosition().y
       << " My Id: " << getParentModule()->getIndex() << " angle: "
       << convertAngleToDegrees(mobility->getAngleRad()) << endl;

    EV << "Sender Position x:"  << msg->getSenderPos().x
       << " Sender Position y:" << msg->getSenderPos().y
       << " Sender id: "        << msg->getSenderAddress()
       << " angle: " << convertAngleToDegrees(msg->getSenderAngle()) << endl;

}

void DVCastLayer::updateTopology(DVCastHello* msg)
{
    Coord senderPos = msg->getSenderPos();
    double myAngle  = convertAngleToDegrees(mobility->getAngleRad());
    double nbAngle  = convertAngleToDegrees(msg->getSenderAngle());
    int angleDiff   = std::abs(myAngle - nbAngle);

    // Trata os casos de borda, principalmente na direção leste
    if (angleDiff > 180)
    {
        angleDiff = 360 - angleDiff;
    }

    if(angleDiff <= 45) // in the same direction
    {
          // EAST
        if (((nbAngle >= 0) && (nbAngle < 45)) || ((nbAngle >= 315) && (nbAngle < 360)))
        {
            if(mobility->getCurrentPosition().x <= senderPos.x)
            {
                // Vizinho a frente na direção leste
                updateTables(&nb_ahead, &nb_back, &nb_opposite, msg->getSenderAddress());
                EV_INFO << "[" << myId << "] " << " B E" << endl;
            }
            else
            {
                // Vizinho atrás na direção leste
                updateTables(&nb_back, &nb_ahead, &nb_opposite, msg->getSenderAddress());
                EV_INFO << "[" << myId << "] " << " A E" << endl;
            }
        } // NORTH
        else if (((nbAngle >= 45) && (nbAngle < 90)) || ((nbAngle >= 90) && (nbAngle < 135)))
        {
            if(mobility->getCurrentPosition().y <= senderPos.y)
            {
                // Vizinho a frente na direção norte
                updateTables(&nb_ahead, &nb_back, &nb_opposite, msg->getSenderAddress());
                EV_INFO << "[" << myId << "] " << " B N" << endl;
            }
            else
            {
                // Vizinho atrás na direção norte
                updateTables(&nb_back, &nb_ahead, &nb_opposite, msg->getSenderAddress());
                EV_INFO << "[" << myId << "] " << " A N" << endl;
            }

        } // WEST
        else if (((nbAngle >= 135) && (nbAngle < 180)) || ((nbAngle >= 180) && (nbAngle < 225)))
        {
            if(mobility->getCurrentPosition().x >= senderPos.x)
            {
                // Vizinho a frente na direção oeste
                updateTables(&nb_ahead, &nb_back, &nb_opposite, msg->getSenderAddress());
                EV_INFO << "[" << myId << "] " << " B W" << endl;
            }
            else
            {
                // Vizinho atrás na direção oeste
                updateTables(&nb_back, &nb_ahead, &nb_opposite, msg->getSenderAddress());
                EV_INFO << "[" << myId << "] " << " A W" << endl;
            }

        } // SOUTH
        else if (((nbAngle >= 225) && (nbAngle < 270)) || ((nbAngle >= 270 && nbAngle < 315)))
        {
            if(mobility->getCurrentPosition().y >= senderPos.y)
            {
                // Vizinho a frente na direção sul
                updateTables(&nb_ahead, &nb_back, &nb_opposite, msg->getSenderAddress());
                EV_INFO << "[" << myId << "] " << " B S" << endl;
            }
            else
            {
                // Vizinho atrás na direção sul
                updateTables(&nb_back, &nb_ahead, &nb_opposite, msg->getSenderAddress());
                EV_INFO << "[" << myId << "] " << " A S" << endl;
            }
        }
    }
    else if ((angleDiff > 90) && (angleDiff <= 180)) // Opposite direction
    {
        updateTables(&nb_ahead, &nb_back, &nb_opposite, msg->getSenderAddress());
    }
}

void DVCastLayer::updateTables(std::deque<int>* target, std::deque<int>* tRemove1,
                               std::deque<int>* tRemove2, int key){

    removeFromTable(target, key);
    removeFromTable(tRemove1, key);
    removeFromTable(tRemove2, key);

    target->push_back(key);

    while (target->size() > 5)
    {
        target->pop_front();
    }
}

void DVCastLayer::removeFromTable(std::deque<int>* target, int key) {

    for (auto it = target->begin(); it != target->end();)
    {
        if (*it == key)
        {
            it = target->erase(it);
            break;
        }
        else
        {
            ++it;
        }
    }
}

int DVCastLayer::convertAngleToDegrees(double angle){

    EV_INFO << angle << endl;
    angle = (180 / 3.14) * angle;
    angle = fmod(angle, 360);

    if (angle < 0)
    {
        angle += 360;
    }

    return angle;
}
