
#include "DVCastLayer.h"

Define_Module(DVCastLayer);

int senderRadius = 2500;

void DVCastLayer::initialize(int stage)
{
    BaseWaveApplLayer::initialize(stage);
    if (stage == 0)
    {
        //Initializing members and pointers of your application goes here
        EV << "Initializing " << par("appName").stringValue() << std::endl;
        lastDroveAt = simTime();
        checkpoint  = simTime();
        sentMessage = false;
        sendHello   = par("sendHello").boolValue();
        accident = par("accident").boolValue();
        checkTopology = par("checkTopology").boolValue();
        helloInterval = par("helloInterval").doubleValue();


        ODC = false;
        MDC = false;
        DFLG = false;

        msg_vector = new std::vector<DVCastData*>();

        timeout = 10.0;
        sendHelloEvt = new cMessage("helloEvent");
    }
    else if (stage == 1)
    {
        //Initializing members that require initialized other modules goes here
        simtime_t firstHello = simTime();
        simtime_t randomOffset = dblrand() * beaconInterval;
        firstHello = simTime() + randomOffset;

        if (sendHello) {
            scheduleAt(firstHello, sendHelloEvt);
        }
    }
}

void DVCastLayer::finish() {
    BaseWaveApplLayer::finish();
    //statistics recording goes here

}

void DVCastLayer::onBSM(BasicSafetyMessage* bsm)
{
    if(DVCastHello* hello = dynamic_cast<DVCastHello*>(bsm))
    {
        bool sameDirection = mapRelativePos(hello);

        // Nós em direção oposta e responsáveis pela retransmissão
        if (!sameDirection || MDC)
        {
            rebroadcast();
        }

        // Atualização das flags
        updateFlags(hello, sameDirection);

        // Tratamento de mensagem de dados
        if (DVCastData* data = dynamic_cast<DVCastData*>(hello))
        {
            // Nó em direção oposta retransmite a mensagem
            if (!MDC && ODC)
            {
                rebroadcast();

                if (!DFLG)
                {
                    cMessage* msg = new cMessage();
                    queue.insert(std::pair<cMessage*, DVCastData*>(msg, data->dup()));
                    scheduleAt(simTime() + timeout, msg);
                }
                else
                {
                    idle();
                }
            }
            else if(!MDC && !ODC)
            {
                cMessage* msg = new cMessage();
                queue.insert(std::pair<cMessage*, DVCastData*>(msg, data->dup()));
                scheduleAt(simTime() + timeout, msg);
            }
        }
    }

}

void DVCastLayer::onWSM(WaveShortMessage* wsm) {
}

void DVCastLayer::populateWSM(WaveShortMessage*  wsm, int rcvId, int serial)
{
    wsm->setWsmVersion(1);
    wsm->setTimestamp(simTime());
    wsm->setSenderAddress(myId);
    wsm->setRecipientAddress(rcvId);
    wsm->setSerial(serial);
    wsm->setBitLength(headerLength);

    wsm->setPsid(-1);
    wsm->setChannelNumber(Channels::CCH);

    if (DVCastData* data = dynamic_cast<DVCastData*>(wsm))
    {
        data->setSenderPos(curPosition);
        data->setSenderSpeed(curSpeed);

        data->setWsmData(mobility->getRoadId().c_str());
        data->setSenderAngle(convertAngleToDegrees(mobility->getAngleRad()));
        data->setRoiUp(getROIUp());
        data->setRoiDown(getROIDown());
    }
    else if (DVCastHello* hello = dynamic_cast<DVCastHello*>(wsm))
    {
        hello->setSenderPos(curPosition);
        hello->setSenderSpeed(curSpeed);

        hello->setSenderAngle(convertAngleToDegrees(mobility->getAngleRad()));
        hello->addBitLength(beaconLengthBits);
        hello->setUserPriority(beaconUserPriority);
    }

}

void DVCastLayer::handleSelfMsg(cMessage* msg) {

    if (msg == sendHelloEvt)
    {
        DVCastHello* hello = new DVCastHello("hello");
        populateWSM(hello);
        sendDown(hello);
        scheduleAt(simTime() + helloInterval, sendHelloEvt);
    }
    else
    {
        queue.erase(msg);
        delete msg;
    }

//    BaseWaveApplLayer::handleSelfMsg(hello);
}

void DVCastLayer::handlePositionUpdate(cObject* obj) {
    BaseWaveApplLayer::handlePositionUpdate(obj);

    // stopped for for at least 10s?
    if (mobility->getSpeed() < 1) {
        if (simTime() - lastDroveAt >= 10 && !sentMessage && accident)
        {
//            findHost()->getDisplayString().updateWith("r=16,red");
            sentMessage = true;

            DVCastData* data = new DVCastData("data", 1);
            populateWSM(data);
            sendDown(data);
            bubble("Me acidentei!");
        }
    }
    else {
        lastDroveAt = simTime();
    }

    if(simTime() - checkpoint >= 5 && checkTopology)
    {
        printTopology();
    }
    else
    {

    }

    if(!MDC) //  Último do bloco na direção da mensagem ou primeiro da direção oposta
    {
        findHost()->getDisplayString().updateWith("r=16,red");
    } else if(ODC)
    {
        findHost()->getDisplayString().updateWith("r=16,yellow");
    }
    else
    {
        findHost()->getDisplayString().updateWith("r=0,gray");
    }
}

bool DVCastLayer::mapRelativePos(DVCastHello* msg)
{
    assert(mobility);
    Coord senderPos = msg->getSenderPos();

    double myAngle  = convertAngleToDegrees(mobility->getAngleRad());
    double nbAngle  = convertAngleToDegrees(msg->getSenderAngle());
    int angleDiff   = std::abs(myAngle - nbAngle);

    bool sameDirection = angleDiff < 45;

    // Trata os casos de borda, principalmente na direção leste
    if (angleDiff > 180)
    {
        angleDiff = 360 - angleDiff;
    }

    if(sameDirection) // mesma direção
    {
          // EAST
        if (((nbAngle >= 0) && (nbAngle < 45)) || ((nbAngle >= 315) && (nbAngle < 360)))
        {
            if(mobility->getCurrentPosition().x <= senderPos.x)
            {
                // Vizinho a frente na direção leste
                updateTables(&nb_ahead, &nb_back, &nb_opposite, msg);
//                EV_INFO << "[" << myId << "] " << " B E" << endl;
            }
            else
            {
                // Vizinho atrás na direção leste
                updateTables(&nb_back, &nb_ahead, &nb_opposite, msg);
//                EV_INFO << "[" << myId << "] " << " A E" << endl;
            }
        } // NORTH
        else if (((nbAngle >= 45) && (nbAngle < 90)) || ((nbAngle >= 90) && (nbAngle < 135)))
        {
            if(mobility->getCurrentPosition().y <= senderPos.y)
            {
                // Vizinho a frente na direção norte
                updateTables(&nb_ahead, &nb_back, &nb_opposite, msg);
//                EV_INFO << "[" << myId << "] " << " B N" << endl;
            }
            else
            {
                // Vizinho atrás na direção norte
                updateTables(&nb_back, &nb_ahead, &nb_opposite, msg);
//                EV_INFO << "[" << myId << "] " << " A N" << endl;
            }

        } // WEST
        else if (((nbAngle >= 135) && (nbAngle < 180)) || ((nbAngle >= 180) && (nbAngle < 225)))
        {
            if(mobility->getCurrentPosition().x >= senderPos.x)
            {
                // Vizinho a frente na direção oeste
                updateTables(&nb_ahead, &nb_back, &nb_opposite, msg);
//                EV_INFO << "[" << myId << "] " << " B W" << endl;
            }
            else
            {
                // Vizinho atrás na direção oeste
                updateTables(&nb_back, &nb_ahead, &nb_opposite, msg);
//                EV_INFO << "[" << myId << "] " << " A W" << endl;
            }

        } // SOUTH
        else if (((nbAngle >= 225) && (nbAngle < 270)) || ((nbAngle >= 270 && nbAngle < 315)))
        {
            if(mobility->getCurrentPosition().y >= senderPos.y)
            {
                // Vizinho a frente na direção sul
                updateTables(&nb_ahead, &nb_back, &nb_opposite, msg);
//                EV_INFO << "[" << myId << "] " << " B S" << endl;
            }
            else
            {
                // Vizinho atrás na direção sul
                updateTables(&nb_back, &nb_ahead, &nb_opposite, msg);
//                EV_INFO << "[" << myId << "] " << " A S" << endl;
            }
        }
    }
    else if ((angleDiff > 90) && (angleDiff <= 180)) // Opposite direction
    {
        updateTables(&nb_ahead, &nb_back, &nb_opposite, msg);
    }

    return sameDirection;
}

void DVCastLayer::updateFlags(DVCastHello* msg, bool sameDirection)
{
    ODC = !nb_opposite.empty();

    MDC = !((nb_back.empty() && sameDirection) || (nb_ahead.empty() && !sameDirection));

    if (DVCastData* data = dynamic_cast<DVCastData*>(msg))
        DFLG = inROI(data->getRoiUp(), data->getRoiDown());
    else
        DFLG = false;


}

Coord DVCastLayer::getROIUp()
{
    Coord p1(curPosition.x - senderRadius/2, curPosition.y + senderRadius/2);
    return p1;
}

Coord DVCastLayer::getROIDown()
{
    Coord p2(curPosition.x + senderRadius/2, curPosition.y - senderRadius/2);
    return p2;
}

bool DVCastLayer::inROI(Coord up, Coord down)
{
    if ((curPosition.x > up.x) && (curPosition.x < down.x) &&
        (curPosition.y > up.y) && (curPosition.y < down.y))
    {
        return true;
    }
    else
    {
        return false;
    }
}

void DVCastLayer::updateTables(std::map<int, simtime_t>* target, std::map<int, simtime_t>* tRemove1,
                               std::map<int, simtime_t>* tRemove2, DVCastHello* msg){

    int key = msg->getSenderAddress();
    simtime_t timestamp = msg->getTimestamp();

    target->erase(key);
    tRemove1->erase(key);
    tRemove2->erase(key);

    target->insert(std::pair<int, simtime_t>(key, timestamp));

    simtime_t currentTime = simTime();
    std::map<int, simtime_t>::iterator it;
    for (it=target->begin(); it != target->end(); it++)
    {
        if (currentTime - it->second > timeout)
        {
            target->erase(it);
        }
    }

//  Talvez atualizar outras tabelas dos outros vizinhos.
}

int DVCastLayer::convertAngleToDegrees(double angleRad){

    double angleDeg;
    angleDeg = (180 / 3.14) * angleRad;
    angleDeg = fmod(angleDeg, 360);

    if (angleDeg < 0)
    {
        angleDeg += 360;
    }
    EV_INFO << angleRad << " " << angleDeg << endl;
    return angleDeg;
}

void DVCastLayer::rebroadcast()
{
    std::vector<DVCastData*>::iterator it;

    // Nós retransmitem a mensagem constantemente
    for (it=msg_vector->begin(); it!=msg_vector->end(); it++)
    {
        DVCastData* relayMsg = *it;
        relayMsg->setSerial(relayMsg->getSerial() + 1);
        relayMsg->getId();

        sendDown(relayMsg->dup());
    }
}

void DVCastLayer::idle()
{
    queue.clear();
}

void DVCastLayer::printTopology()
{
    EV_INFO << "[DVCAST Flags] MDC: " << MDC << " ODC: " << ODC << " DFLG: " << DFLG << endl;

    EV_INFO << "[DVCAST AHEAD] [";
    for (auto& t : nb_ahead)
        EV_INFO << t.first << " " ;
    EV_INFO << "]" << endl;

    EV_INFO << "[DVCAST BACK] [";
    for (auto& t : nb_back)
        EV_INFO << t.first << " " ;
    EV_INFO << "]" << endl;

    EV_INFO << "[DVCAST OPPOSITE] [";
    for (auto& t : nb_opposite)
        EV_INFO << t.first << " " ;
    EV_INFO << "]" << endl;
}
