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
    void resolveUnestablishedEdges();
    void resolveUnestablishedCallbackEdges();
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

    std::string const FULL_TA_SCHEMA ="//Full Rex Extraction\n//Author: Bryan J Muscedere\n\nSCHEME TUPLE :\n//Nodes\n$"
            "INHERIT\tcArchitecturalNds\tcRoot\n$INHERIT\tcAsgNds\t\t\tcRoot\n$INHERIT\trosMsg\t\t\tcRoot\n$INHERIT\tcCl"
            "ass\t\t\tcAsgNds\n$INHERIT\tcFunction\t\tcAsgNds\n$INHERIT\tcVariable\t\tcAsgNds\n$INHERIT\trosTopic\t\tr"
            "osMsg\n$INHERIT\trosPublisher\t\trosMsg\n$INHERIT\trosSubscriber\t\trosMsg\n$INHERIT\trosNodeHandle\t\tros"
            "Msg\n\n//Relationships\ncontain\t\tcRoot\t\tcRoot\npublish\t\trosPublisher\trosTopic\nsubscribe\trosTopic\t"
            "rosSubscriber\nreference\trosNodeHandle\trosMsg\nadvertise\trosPublisher\trosTopic\nsubscribe\trosSubscribe"
            "r\trosTopic\ncall\t\tcAsgNds\t\tcAsgNds\nwrite\t\tcFunction\tcVariable\nread\t\tcVariable\tcFunction\t\t\n"
            "\nSCHEME ATTRIBUTE :\n$ENTITY {\n\tx\n\ty\n\twidth\n\theight\n\tlabel\n}\n\ncRoot {\n\telision = contain\n"
            "\tcolor = (0.0 0.0 0.0)\n\tfile\n\tline\n\tname\n}\n\ncAsgNds {\n\tbeg\n\tend\n\tfile\n\tline\n\tvalue\n\tc"
            "olor = (0.0 0.0 0.0)\n}\n\ncArchitecturalNds {\n\tclass_style = 4\n\tcolor = (0.0 0.0 1.0)\n\tcolor = (0.0 "
            "0.0 0.0)\n}\n\nrosMsg {\n\tclass_style = 4\n\tcolor = (0.0 0.0 1.0)\n}\n\ncFunction {\n\tfilename\n\tisSta"
            "tic\n\tisConst\n\tisVolatile\n\tisVariadic\n\tvisibility\n\tcolor = (1.0 0.0 0.0)\n\tlabelcolor = (0.0 0.0"
            " 0.0)\n}\n\ncVariable {\n\tfilename\n\tscopeType\n\tisStatic\n\tisPublisher = 0\n\tisSubscriber = 0\n}\n\nc"
            "Class {\n\tfilename\n\tbaseNum\n\tcolor = (0.2 0.4 0.1)\n\tlabelcolor = (0.0 0.0 0.0)\n}\n\nrosSubscriber "
            "{\n\tbufferSize\n\tnumAttributes\n\trosNumber\t\n\tcallbackFunc\n}\n\nrosPublisher {\n\tnumAttributes\n\tr"
            "osNumber\n}\n";
            std::string const MINI_TA_SCHEMA ="//Minimal Rex Extraction\n//Author: Bryan J Muscedere\n\nSCHEME TUPLE :\n"
            "//Nodes\n$INHERIT\tcArchitecturalNds\tcRoot\n$INHERIT\tcAsgNds\t\t\tcRoot\n$INHERIT\trosMsg\t\t\tcRoot\n$IN"
            "HERIT\tcClass\t\t\tcAsgNds\n$INHERIT\trosTopic\t\trosMsg\n$INHERIT\trosPublisher\t\trosMsg\n$INHERI"
            "T\trosSubscr"
            "iber\t\trosMsg\n$INHERIT\trosNodeHandle\t\trosMsg\n\n//Relationships\ncontain\t\tcRoot\t\tcRoot\npublish\t"
            "\trosPublisher\trosTopic\nsubscribe\trosTopic\trosSubscriber\nreference\trosNodeHandle\trosMsg\nadvertise"
            "\trosPublisher\trosTopic\nsubscribe\trosSubscriber\trosTopic\n\nSCHEME ATTRIBUTE :\n$ENTITY {\n\tx\n\ty\n"
            "\twidth\n\theight\n\tlabel\n}\n\ncRoot {\n\telision = contain\n\tcolor = (0.0 0.0 0.0)\n\tfile\n\tline\n\t"
            "name\n}\n\ncAsgNds {\n\tbeg\n\tend\n\tfile\n\tline\n\tvalue\n\tcolor = (0.0 0.0 0.0)\n}\n\ncArchitecturalN"
            "ds {\n\tclass_style = 4\n\tcolor = (0.0 0.0 1.0)\n\tcolor = (0.0 0.0 0.0)\n}\n\nrosMsg {\n\tclass_style = "
            "4\n\tcolor = (0.0 0.0 1.0)\n}\n\ncFile {\n\tclass_style = 2\n\tcolor = (0.9 0.9 0.9)\n\tlabelcolor = (0.0 "
            "0.0 0.0)\n}\n\ncFunction {\n\tfilename\n\tisStatic\n\tisConst\n\tisVolatile\n\tisVariadic\n\tvisibility\n"
            "\tcolor = (1.0 0.0 0.0)\n\tlabelcolor = (0.0 0.0 0.0)\n}\n\ncVariable {\n\tfilename\n\tscopeType\n\tisStat"
            "ic\n\tisPublisher = 0\n\tisSubscriber = 0\n}\n\ncClass {\n\tfilename\n\tbaseNum\n\tcolor = (0.2 0.4 0.1)\n"
            "\tlabelcolor = (0.0 0.0 0.0)\n}\n\ncEnum {\n\tfilename\n\tcolor = (0.9 0.2 0.5)\n\tlabelcolor = (0.0 0.0 0"
            ".0)\n}\n\ncEnumConst {\n\tfilename\n\tcolor = (0.9 0.2 0.5)\n\tlabelcolor = (0.0 0.0 0.0)\n}\n\nrosSubscri"
            "ber {\n\tbufferSize\n\tnumAttributes\n\trosNumber\t\n\tcallbackFunc\n}\n\nrosPublisher {\n\tnumAttributes"
            "\n\trosNumber\n}\n\n";
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
