
#ifndef DVCASTLAYER_H_
#define DVCASTLAYER_H_

#include <omnetpp.h>
#include "messages/DVCastHello_m.h"
#include "messages/DVCastData_m.h"
#include "veins/modules/application/ieee80211p/BaseWaveApplLayer.h"

using namespace omnetpp;

class DVCastLayer : public BaseWaveApplLayer
{

public:
    virtual void initialize(int stage);
    virtual void finish();

    enum DVCastMessageKinds {
        SEND_HELLO_EVT,
        SEND_DATA_EVT
    };

protected:

    // Função redefinida para lidar com DVCastHello
    virtual void onBSM(BasicSafetyMessage* bsm);

    // Função redefinida para lidar com DVCastData
    virtual void onWSM(WaveShortMessage* wsm);

    // Função redefinida para popular DVCastHello e DVCastData
    virtual void populateWSM(WaveShortMessage*  wsm, int rcvId=-1, int serial=0);

    // Função para agendar envio de DVCastHello
    virtual void handleSelfMsg(cMessage* msg);

    // Atualiza informações de posição de nó
    virtual void handlePositionUpdate(cObject* obj);

    // Imprime mensagens de Hello recebidas
    void onHello(DVCastHello* msg);

private:

    // Mapeia a posição do nó emissor em relação este nó
    virtual bool mapRelativePos(DVCastHello* msg);

    // Atualiza a topologia da rede
    virtual void updateTables(std::map<int, simtime_t>* target,
                              std::map<int, simtime_t>* t_remove1,
                              std::map<int, simtime_t>* t_remove2,
                              DVCastHello* msg);

    // Converte o ângulo de radianos para graus
    virtual int convertAngleToDegrees(double angleRad);

    virtual void rebroadcast();

    virtual void idle();

    virtual void printTopology();

    // Atualiza as flags após hello ou data
    virtual void updateFlags(DVCastHello* msg, bool sameDirection);

    virtual bool inROI(Coord up, Coord down);

    virtual Coord getROIUp();
    virtual Coord getROIDown();

    // Armazena nós a frente
    std::map<int, simtime_t> nb_ahead;
    // Armazena nós atrás
    std::map<int, simtime_t> nb_back;
    // Armazena nós em direção oposta
    std::map<int, simtime_t> nb_opposite;

    std::vector<DVCastData*> *msg_vector;
    std::map<cMessage*, DVCastData*> queue;

    simtime_t lastDroveAt;
    simtime_t helloInterval;

    bool sentMessage;
    bool sendHello;

    // Flag é ativada se o nó não for o último de um cluster que está na direção da ROI
    bool MDC;
    // Flag é ativada se o nó receber mensagem de um vizinho em direção oposta
    bool ODC;
    // Flag é ativada se o nó estiver na ROI
    bool DFLG;

    // Evento de Hello, usado como beacon do DVCast
    cMessage* sendHelloEvt;

    // Timeout
    simtime_t timeout;
    simtime_t checkpoint;
    bool accident;
    bool checkTopology;

};

#endif /* DVCASTLAYER_H_ */
