//
// Created by bmuscede on 10/04/17.
//

#ifndef REX_TAGRAPH_H
#define REX_TAGRAPH_H

#include <unordered_map>
#include <string>
#include <vector>
#include "RexNode.h"
#include "RexEdge.h"

class TAGraph {
public:
    TAGraph();
    ~TAGraph();

    void addNode(RexNode* node);
    void addEdge(RexEdge* edge);

    void removeNode(std::string nodeID);
    void removeEdge(std::string srcID, std::string dstID, RexEdge::EdgeType type, bool hashed = false);

    RexNode* findNode(std::string nodeID);
    RexNode* findNodeByName(std::string nodeName);
    RexEdge* findEdge(std::string srcID, std::string dstID, RexEdge::EdgeType type);
    std::vector<RexEdge*> findEdgesBySrc(std::string srcID, bool md5 = true);
    std::vector<RexEdge*> findEdgesByDst(std::string dstID, bool md5 = true);

    bool doesNodeExist(std::string nodeID);
    bool doesEdgeExist(std::string srcID, std::string dstID, RexEdge::EdgeType type);

    void resolveUnestablishedEdges();
    void purgeUnestablishedEdges(bool resolveFirst = true);

    std::string getTAModel();

    RexNode* generateSubscriberNode(std::string parentID, std::string parentName);
    RexNode* generatePublisherNode(std::string parentID, std::string parentName);

private:
    std::unordered_map<std::string, RexNode*> idList;
    std::unordered_map<std::string, std::vector<RexEdge*>> edgeSrcList;
    std::unordered_map<std::string, std::vector<RexEdge*>> edgeDstList;

    //ROS Functionality
    std::string const PUB_NAME = "Publisher";
    std::string const SUB_NAME = "Subscriber";
    std::string const ROS_NUM = "rosNumber";

    std::string const TA_SCHEMA = "//Rex Extraction\n//Author: Jingwei Wu & Bryan J Muscedere\n\nSCHEME TUPLE :\n//Node"
            "s\n$INHERIT\tcArchitecturalNds\tcRoot\n$INHERIT\tcAsgNds\t\t\tcRoot\n$INHERIT\trosMsg\t\t\tcRoot\n$INHERIT"
            "\tcSubSystem\t\tcArchitecturalNds\n$INHERIT\tcFile\t\t\tcArchitecturalNds\n$INHERIT\tcClass\t\t\tcAsgNds\n"
            "$INHERIT\tcFunction\t\tcAsgNds\n$INHERIT\tcVariable\t\tcAsgNds\n$INHERIT\tcLang\t\t\tcAsgNds\n$INHERIT\tro"
            "sTopic\t\trosMsg\n$INHERIT\trosPublisher\t\trosMsg\n$INHERIT\trosSubscriber\t\trosMsg\n$INHERIT\tcEnum\t\t"
            "\tcLang\n$INHERIT\tcEnumConst\t\tcLang\n$INHERIT\tcStruct\t\t\tcLang\n$INHERIT\tcUnion\t\t\tcLang\n\n//Rel"
            "ationships\ncontain\t\tcRoot\t\tcRoot\npublish\t\tcFunction\trosTopic\nsubscribe\tcFunction\trosTopic\ncal"
            "l\t\tcFunction\tcFunction\nreference\tcAsgNds\t\tcAsgNds\ninherits\tcClass\t\tcClass\nadvertise\tcRoot\t\t"
            "cRoot\nsubscribe\tcRoot\t\tcRoot\n\nSCHEME ATTRIBUTE :\n$ENTITY {\n\tx\n\ty\n\twidth\n\theight\n\tlabel\n}"
            "\n\ncRoot {\n\telision = contain\n\tcolor = (0.0 0.0 0.0)\n\tfile\n\tline\n\tname\n}\n\ncAsgNds {\n\tbeg\n"
            "\tend\n\tfile\n\tline\n\tvalue\n\tcolor = (0.0 0.0 0.0)\n}\n\ncArchitecturalNds {\n\tclass_style = 4\n\tco"
            "lor = (0.0 0.0 1.0)\n\tcolor = (0.0 0.0 0.0)\n}\n\ncSubSystem {\n\tclass_style = 4\n\tcolor = (0.0 0.0 1.0"
            ")\n}\n\ncFile {\n\tclass_style = 2\n\tcolor = (0.9 0.9 0.9)\n\tlabelcolor = (0.0 0.0 0.0)\n}\n\ncFunction "
            "{\n\tfilename\n\tisStatic\n\tisConst\n\tisVolatile\n\tisVariadic\n\tvisibility\n\tcolor = (1.0 0.0 0.0)\n"
            "\tlabelcolor = (0.0 0.0 0.0)\n}\n\ncVariable {\n\tfilename\n\tscopeType\n\tisStatic\n\tisPublisher = 0\n\t"
            "isSubscriber = 0\n}\n\ncClass {\n\tfilename\n\tbaseNum\n\tcolor = (0.2 0.4 0.1)\n\tlabelcolor = (0.0 0.0 0"
            ".0)\n}\n\ncEnum {\n\tfilename\n\tcolor = (0.9 0.2 0.5)\n\tlabelcolor = (0.0 0.0 0.0)\n}\n\ncEnumConst {\n"
            "\tfilename\n\tcolor = (0.9 0.2 0.5)\n\tlabelcolor = (0.0 0.0 0.0)\n}\n\nrosSubscriber {\n\tbufferSize\n\tn"
            "umAttributes\n\trosNumber\t\n\tcallbackFunc\n}\n\nrosPublisher {\n\tnumAttributes\n\trosNumber\n}\n\nrosTo"
            "pic {\n\t\n}\n\n(reference) {\n\taccess\n}\n\n";
    int const MD5_LENGTH = 33;

    std::string getMD5(std::string ID);

    bool resolveEdge(RexEdge* edge);

    RexNode* generateROSNode(std::string parentID, std::string parentName, RexNode::NodeType type);
    int getLastROSNumber(std::string rosID);
};


#endif //REX_TAGRAPH_H
