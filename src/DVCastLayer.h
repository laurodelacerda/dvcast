
#ifndef DVCASTLAYER_H_
#define DVCASTLAYER_H_

#include <omnetpp.h>
#include "messages/DVCastHello_m.h"
#include "veins/modules/application/ieee80211p/BaseWaveApplLayer.h"

using namespace omnetpp;

class DVCastLayer : public BaseWaveApplLayer
{

public:
    virtual void initialize(int stage);
    virtual void finish();

protected:

    virtual void onBSM(BasicSafetyMessage* bsm);
    virtual void onWSM(WaveShortMessage* wsm);
    virtual void onWSA(WaveServiceAdvertisment* wsa);

    virtual void handleSelfMsg(cMessage* msg);
    virtual void handlePositionUpdate(cObject* obj);

    void onHello(DVCastHello* msg);

private:

    std::deque<int> nb_ahead;
    std::deque<int> nb_back;
    std::deque<int> nb_opposite;

    virtual void updateTopology(DVCastHello* msg);

    virtual void updateTables(std::deque<int>* target,
                              std::deque<int>* t_remove1,
                              std::deque<int>* t_remove2,
                              int key);

    virtual void removeFromTable(std::deque<int>* target, int key);

    virtual int convertAngleToDegrees(double angle);

    simtime_t lastDroveAt;
    simtime_t beaconUpdate;
    bool sentMessage;

};

#endif /* DVCASTLAYER_H_ */
