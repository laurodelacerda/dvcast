
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

protected:

    // Função redefinida para lidar com DVCastHello e DVCastData
    virtual void onBSM(BasicSafetyMessage* bsm);

    // Função redefinida para popular DVCastHello e DVCastData
    virtual void populateWSM(WaveShortMessage*  wsm, int rcvId=-1, int serial=0);

    // Função para agendar envio de DVCastHello
    virtual void handleSelfMsg(cMessage* msg);

    // Atualiza informações de posição de nó
    virtual void handlePositionUpdate(cObject* obj);

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

    // Retransmite mensagens na fila de saída
    virtual void rebroadcast();

    // Apaga mensagens da fila de saída
    virtual void idle();

    // Mostra a topologia atual
    virtual void printTopology();

    // Atualiza as flags após hello ou data
    virtual void updateFlags(DVCastHello* msg, bool sameDirection);

    // Verifica se nó está na região de interesse
    virtual bool inROI(Coord up, Coord down);

    virtual Coord getROIUp();
    virtual Coord getROIDown();

    // Armazena nós a frente
    std::map<int, simtime_t> nb_ahead;
    // Armazena nós atrás
    std::map<int, simtime_t> nb_back;
    // Armazena nós em direção oposta
    std::map<int, simtime_t> nb_opposite;

    // Fila de mensagens de saída
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
    simtime_t timeoutMessage;
    simtime_t timeoutTopology;
    simtime_t checkpoint;
    bool accident;
    bool checkTopology;

};

#endif /* DVCASTLAYER_H_ */
