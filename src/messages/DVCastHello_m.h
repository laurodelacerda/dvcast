//
// Generated file, do not edit! Created by nedtool 5.3 from messages/DVCastHello.msg.
//

#if defined(__clang__)
#  pragma clang diagnostic ignored "-Wreserved-id-macro"
#endif
#ifndef __DVCASTHELLO_M_H
#define __DVCASTHELLO_M_H

#include <omnetpp.h>

// nedtool version check
#define MSGC_VERSION 0x0503
#if (MSGC_VERSION!=OMNETPP_VERSION)
#    error Version mismatch! Probably this file was generated by an earlier version of nedtool: 'make clean' should help.
#endif



// cplusplus {{
#include "veins/modules/messages/BasicSafetyMessage_m.h"
// }}

/**
 * Class generated from <tt>messages/DVCastHello.msg:7</tt> by nedtool.
 * <pre>
 * packet DVCastHello extends BasicSafetyMessage
 * {
 *     double senderAngle;
 * }
 * </pre>
 */
class DVCastHello : public ::BasicSafetyMessage
{
  protected:
    double senderAngle;

  private:
    void copy(const DVCastHello& other);

  protected:
    // protected and unimplemented operator==(), to prevent accidental usage
    bool operator==(const DVCastHello&);

  public:
    DVCastHello(const char *name=nullptr, short kind=0);
    DVCastHello(const DVCastHello& other);
    virtual ~DVCastHello();
    DVCastHello& operator=(const DVCastHello& other);
    virtual DVCastHello *dup() const override {return new DVCastHello(*this);}
    virtual void parsimPack(omnetpp::cCommBuffer *b) const override;
    virtual void parsimUnpack(omnetpp::cCommBuffer *b) override;

    // field getter/setter methods
    virtual double getSenderAngle() const;
    virtual void setSenderAngle(double senderAngle);
};

inline void doParsimPacking(omnetpp::cCommBuffer *b, const DVCastHello& obj) {obj.parsimPack(b);}
inline void doParsimUnpacking(omnetpp::cCommBuffer *b, DVCastHello& obj) {obj.parsimUnpack(b);}


#endif // ifndef __DVCASTHELLO_M_H

