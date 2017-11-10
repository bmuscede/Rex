//
// Created by bmuscede on 10/04/17.
//

#ifndef REX_REXEDGE_H
#define REX_REXEDGE_H

#include "RexNode.h"

class RexEdge {
public:
    enum EdgeType {CONTAINS, COMP_CONTAINS, REFERENCES, CALLS, READS, WRITES, ADVERTISE, SUBSCRIBE, PUBLISH};
    static std::string typeToString(EdgeType type);

    RexEdge(RexNode* src, RexNode* dst, EdgeType type);
    RexEdge(std::string src, std::string dst, EdgeType type);
    RexEdge(RexNode* src, std::string dst, EdgeType type);
    RexEdge(std::string src, RexNode* dst, EdgeType type);
    ~RexEdge();

    bool isEstablished();

    void setSource(RexNode* src);
    void setDestination(RexNode* dst);
    void setSourceID(std::string ID);
    void setDestinationID(std::string ID);
    void setType(EdgeType type);

    RexNode* getSource();
    RexNode* getDestination();
    EdgeType getType();
    std::string getSourceID();
    std::string getDestinationID();
    int getNumAttributes();

    void addSingleAttribute(std::string key, std::string value);
    void addMultiAttribute(std::string key, std::string value);
    std::string getSingleAttribute(std::string key);
    std::vector<std::string> getMultiAttribute(std::string key);

    std::string generateTAEdge();
    std::string generateTAAttribute();

private:
    RexNode* sourceNode;
    RexNode* destNode;

    std::string sourceID;
    std::string destID;

    EdgeType type;

    std::map<std::string, std::string> singleAttributes;
    std::map<std::string, std::vector<std::string>> multiAttributes;
};


#endif //REX_REXEDGE_H
