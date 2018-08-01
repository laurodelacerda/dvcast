
#ifndef DVCASTLAYER_H_
#define DVCASTLAYER_H_

#include "messages/DVCastHello_m.h"
#include "messages/DVCastData_m.h"
#include "veins/modules/application/ieee80211p/BaseWaveApplLayer.h"
#include <omnetpp.h>

using namespace omnetpp;

class DVCastLayer : public BaseWaveApplLayer
{

public:
    virtual void initialize(int stage);
    virtual void finish();

protected:

    // Lida com recebimento de DVCastHello e DVCastData
    virtual void onBSM(BasicSafetyMessage* bsm);

    // Popula DVCastHello e DVCastData
    virtual void populateWSM(WaveShortMessage*  wsm, int rcvId=-1, int serial=0);

    // Agendar envio de DVCastHello
    virtual void handleSelfMsg(cMessage* msg);

    // Atualiza informações de posição de nó
    virtual void handlePositionUpdate(cObject* obj);

    // Guarda estatísticas de mensagens recebidas
    virtual void handleLowerMsg(cMessage* msg);

    // Guarda estatísticas de mensagens enviadas
    virtual void checkAndTrackPacket(cMessage* msg);


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

    virtual void printFlags();

    // Atualiza as flags após hello ou data
    virtual void updateFlags(DVCastHello* msg, bool sameDirection);

    // Verifica se nó está na região de interesse
    virtual bool inROI(Coord up, Coord down);

    virtual Coord getROIUp();
    virtual Coord getROIDown();

    std::map<int, simtime_t> nb_ahead;     // Nós a frente
    std::map<int, simtime_t> nb_back;      // Nós atrás
    std::map<int, simtime_t> nb_opposite;  // Nós em direção oposta

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

    // Estatísticas
    uint32_t generatedDVHello;
    uint32_t generatedDVData;
    uint32_t receivedDVHello;
    uint32_t receivedDVData;

};

#endif /* DVCASTLAYER_H_ */
