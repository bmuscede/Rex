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
    TAGraph(RexEdge::EdgeType edgeType = RexEdge::EdgeType::CONTAINS);
    ~TAGraph();

    void setMinMode(bool minMode);

    void addNode(RexNode* node);
    void addEdge(RexEdge* edge);

    void removeNode(std::string nodeID);
    void removeEdge(std::string srcID, std::string dstID, RexEdge::EdgeType type, bool hashed = false);

    RexNode* findNode(std::string nodeID);
    RexNode* findNodeByName(std::string nodeName, bool MD5Check = false);
    RexNode* findNodeByEndName(std::string endName, bool MD5Check = false);
    RexEdge* findEdge(std::string srcID, std::string dstID, RexEdge::EdgeType type);
    std::vector<RexEdge*> findEdgesBySrc(std::string srcID, bool md5 = true);
    std::vector<RexEdge*> findEdgesByDst(std::string dstID, bool md5 = true);

    bool doesNodeExist(std::string nodeID);
    bool doesEdgeExist(std::string srcID, std::string dstID, RexEdge::EdgeType type);

    std::string checkCorrectness();
    bool resolveComponents(std::map<std::string, std::vector<std::string>> databaseMap);
    void resolveUnestablishedEdges();
    void purgeUnestablishedEdges(bool resolveFirst = true);

    std::string getTAModel();

    RexNode* generateSubscriberNode(std::string parentID, std::string parentName);
    RexNode* generatePublisherNode(std::string parentID, std::string parentName);

private:
    std::unordered_map<std::string, RexNode*> idList;
    std::unordered_map<std::string, std::vector<RexEdge*>> edgeSrcList;
    std::unordered_map<std::string, std::vector<RexEdge*>> edgeDstList;
    bool minMode;
    RexEdge::EdgeType forestEdgeType;

    //ROS Functionality
    std::string const PUB_NAME = "Publisher";
    std::string const SUB_NAME = "Subscriber";
    std::string const ROS_NUM = "rosNumber";
    std::string const FILENAME_ATTR = "filename";

    std::string const FULL_TA_SCHEMA = "//Full Rex Extraction\n//Author: Bryan J Muscedere\n\nSCHEME TUPLE :\n//Nodes\n"
            "$INHERIT\tcArchitecturalNds\tcRoot\n$INHERIT\tcAsgNds\t\t\tcRoot\n$INHERIT\trosMsg\t\t\tcRoot\n$INHERIT\tc"
            "Component\t\tcAsgNds\n$INHERIT\tcClass\t\t\tcAsgNds\n$INHERIT\tcFunction\t\tcAsgNds\n$INHERIT\tcVariable\t"
            "\tcAsgNds\n$INHERIT\trosTopic\t\trosMsg\n$INHERIT\trosPublisher\t\trosMsg\n$INHERIT\trosSubscriber\t\trosM"
            "sg\n$INHERIT\trosNodeHandle\t\trosMsg\n\n//Relationships\ncompContain\tcComponent\tcClass\ncontain\t\tcRoo"
            "t\t\tcRoot\npublish\t\trosPublisher\trosTopic\nsubscribe\trosTopic\trosSubscriber\nreference\trosNodeHandl"
            "e\trosMsg\nadvertise\trosPublisher\trosTopic\ncall\t\tcRoot\t\tcRoot\nwrite\t\tcFunction\tcVariable\nread"
            "\t\tcVariable\tcFunction\t\t\n\nSCHEME ATTRIBUTE :\n$ENTITY {\n\tx\n\ty\n\twidth\n\theight\n\tlabel\n}\n\n"
            "cRoot {\n\telision = contain\n\tcolor = (0.0 0.0 0.0)\n\tfile\n\tline\n\tname\n}\n\ncAsgNds {\n\tbeg\n\ten"
            "d\n\tfile\n\tline\n\tvalue\n\tcolor = (0.0 0.0 0.0)\n}\n\ncArchitecturalNds {\n\tclass_style = 4\n\tcolor "
            "= (0.0 0.0 1.0)\n\tcolor = (0.0 0.0 0.0)\n}\n\nrosMsg {\n\tclass_style = 4\n\tcolor = (0.0 0.0 1.0)\n}\n\n"
            "cComponent {\n\tcolor = (0.7 0.3 0.1)\n\tlabelcolor = (0.0 0.0 0.0)\n\tclass_style = 2\n}\n\ncFunction {\n"
            "\tfilename\n\tisStatic\n\tisConst\n\tisVolatile\n\tisVariadic\n\tvisibility\n\tcolor = (1.0 0.0 0.0)\n\tla"
            "belcolor = (0.0 0.0 0.0)\n}\n\ncVariable {\n\tfilename\n\tscopeType\n\tisStatic\n\tisControlFlow = 0\n\tis"
            "Publisher = 0\n\tisSubscriber = 0\n}\n\ncClass {\n\tfilename\n\tbaseNum\n\tcolor = (0.2 0.4 0.1)\n\tlabelc"
            "olor = (0.0 0.0 0.0)\n\tclass_style = 2\n}\n\nrosSubscriber {\n\tcolor = (0.4 1.0 0.4)\n\tlabelcolor = (0."
            "0 0.0 0.0)\n\tclass_style = 6\n\tbufferSize\n\tnumAttributes\n\trosNumber\t\n\tcallbackFunc\n}\n\nrosPubli"
            "sher {\n\tcolor = (1.0 0.0 0.8)\n\tlabelcolor = (1.0 1.0 1.0)\n\tclass_style = 6\n\tnumAttributes\n\trosNu"
            "mber\n\tisUnderControl = 0\n}\n\nrosTopic {\n\tcolor = (1.0 1.0 0.6)\n\tlabelcolor = (0.0 0.0 0.0)\n\tclas"
            "s_style = 5\n}\n\n(call) {\n\tisUnderControl = 0\n}\n";
    std::string const MINI_TA_SCHEMA = "//Minimal Rex Extraction\n//Author: Bryan J Muscedere\n\nSCHEME TUPLE :\n//Node"
            "s\n$INHERIT\tcArchitecturalNds\tcRoot\n$INHERIT\tcAsgNds\t\t\tcRoot\n$INHERIT\trosMsg\t\t\tcRoot\n$INHERIT"
            "\tcComponent\t\tcAsgNds\n$INHERIT\tcClass\t\t\tcAsgNds\n$INHERIT\trosTopic\t\trosMsg\n$INHERIT\trosPublish"
            "er\t\trosMsg\n$INHERIT\trosSubscriber\t\trosMsg\n$INHERIT\trosNodeHandle\t\trosMsg\n\n//Relationships\ncom"
            "pContain\tcComponent\tcClass\ncontain\t\tcRoot\t\tcRoot\npublish\t\trosPublisher\trosTopic\nsubscribe\tros"
            "Topic\trosSubscriber\nreference\trosNodeHandle\trosMsg\nadvertise\trosPublisher\trosTopic\nsubscribe\trosS"
            "ubscriber\trosTopic\n\nSCHEME ATTRIBUTE :\n$ENTITY {\n\tx\n\ty\n\twidth\n\theight\n\tlabel\n}\n\ncRoot {\n"
            "\telision = contain\n\tcolor = (0.0 0.0 0.0)\n\tfile\n\tline\n\tname\n}\n\ncAsgNds {\n\tbeg\n\tend\n\tfile"
            "\n\tline\n\tvalue\n\tcolor = (0.0 0.0 0.0)\n}\n\ncArchitecturalNds {\n\tclass_style = 4\n\tcolor = (0.0 0."
            "0 1.0)\n\tcolor = (0.0 0.0 0.0)\n}\n\nrosMsg {\n\tclass_style = 4\n\tcolor = (0.0 0.0 1.0)\n}\n\ncComponen"
            "t {\n\tcolor = (0.7 0.3 0.1)\n\tlabelcolor = (0.0 0.0 0.0)\n\tclass_style = 2\n}\n\ncClass {\n\tfilename\n"
            "\tbaseNum\n\tcolor = (0.2 0.4 0.1)\n\tlabelcolor = (0.0 0.0 0.0)\n\tclass_style = 2\n}\n\nrosSubscriber {"
            "\n\tcolor = (0.4 1.0 0.4)\n\tlabelcolor = (0.0 0.0 0.0)\n\tclass_style = 6\n\tbufferSize\n\tnumAttributes"
            "\n\trosNumber\t\n\tcallbackFunc\n}\n\nrosPublisher {\n\tcolor = (1.0 0.0 0.8)\n\tlabelcolor = (1.0 1.0 1.0"
            ")\n\tclass_style = 6\n\tnumAttributes\n\trosNumber\n}\n\nrosTopic {\n\tcolor = (1.0 1.0 0.6)\n\tlabelcolor"
            " = (0.0 0.0 0.0)\n\tclass_style = 5\n}\n";

    int const MD5_LENGTH = 33;

    std::string getMD5(std::string ID);

    bool resolveEdge(RexEdge* edge);
    bool resolveEdgeByName(RexEdge* edge);

    RexNode* generateROSNode(std::string parentID, std::string parentName, RexNode::NodeType type);
    int getLastROSNumber(std::string rosID);

    bool hasEnding(std::string const &fullString, std::string const &ending){
        if (fullString.length() >= ending.length()) {
            return (0 == fullString.compare (fullString.length() - ending.length(), ending.length(), ending));
        } else {
            return false;
        }
    }
};


#endif //REX_TAGRAPH_H
