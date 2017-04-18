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
    RexEdge* findEdge(std::string srcID, std::string dstID, RexEdge::EdgeType type);
    std::vector<RexEdge*> findEdgesBySrc(std::string srcID);
    std::vector<RexEdge*> findEdgesByDst(std::string dstID);

    bool doesNodeExist(std::string nodeID);
    bool doesEdgeExist(std::string srcID, std::string dstID, RexEdge::EdgeType type);

    void resolveUnestablishedEdges();
    void purgeUnestablishedEdges(bool resolveFirst = true);

    std::string getTAModel();

private:
    std::unordered_map<std::string, RexNode*> idList;
    std::unordered_map<std::string, std::vector<RexEdge*>> edgeSrcList;
    std::unordered_map<std::string, std::vector<RexEdge*>> edgeDstList;

    std::string const TA_SCHEMA = "//Rex Extraction\n//Author: Jingwei Wu & Bryan J Muscedere\n\nSCHEME TUPLE :\n//Node"
            "s\n$INHERIT\tcArchitecturalNds\tcRoot\n$INHERIT\tcAsgNds\t\t\tcRoot\n$INHERIT\trosMsg\t\t\tcRoot\n$INHERIT"
            "\tcSubSystem\t\tcArchitecturalNds\n$INHERIT\tcFile\t\t\tcArchitecturalNds\n$INHERIT\tcClass\t\t\tcAsgNds\n"
            "$INHERIT\tcFunction\t\tcAsgNds\n$INHERIT\tcVariable\t\tcAsgNds\n$INHERIT\tcLang\t\t\tcAsgNds\n$INHERIT\tro"
            "sTopic\t\trosMsg\n$INHERIT\tcEnum\t\t\tcLang\n$INHERIT\tcEnumConst\t\tcLang\n$INHERIT\tcStruct\t\t\tcLang"
            "\n$INHERIT\tcUnion\t\t\tcLang\n\n//Relationships\ncontain\t\tcRoot\t\tcRoot\npublish\t\tcFunction\trosTopi"
            "c\nsubscribe\tcFunction\trosTopic\ncall\t\tcFunction\tcFunction\nreference\tcAsgNds\t\tcAsgNds\ninherits\t"
            "cClass\t\tcClass\n\nSCHEME ATTRIBUTE :\n$ENTITY {\n\tx\n\ty\n\twidth\n\theight\n\tlabel\n}\n\ncRoot {\n\te"
            "lision = contain\n\tcolor = (0.0 0.0 0.0)\n\tfile\n\tline\n\tname\n}\n\ncAsgNds {\n\tbeg\n\tend\n\tfile\n\t"
            "line\n\tvalue\n\tcolor = (0.0 0.0 0.0)\n}\n\ncArchitecturalNds {\n\tclass_style = 4\n\tcolor = (0.0 0.0 1."
            "0)\n\tcolor = (0.0 0.0 0.0)\n}\n\ncSubSystem {\n\tclass_style = 4\n\tcolor = (0.0 0.0 1.0)\n}\n\ncFile {\n"
            "\tclass_style = 2\n\tcolor = (0.9 0.9 0.9)\n\tlabelcolor = (0.0 0.0 0.0)\n}\n\ncFunction {\n\tfilename\n\t"
            "isStatic\n\tisConst\n\tisVolatile\n\tisVariadic\n\tvisibility\n\tcolor = (1.0 0.0 0.0)\n\tlabelcolor = (0."
            "0 0.0 0.0)\n}\n\ncVariable {\n\tfilename\n\tscopeType\n\tisStatic\n}\n\ncClass {\n\tfilename\n\tbaseNum\n\t"
            "color = (0.2 0.4 0.1)\n\tlabelcolor = (0.0 0.0 0.0)\n}\n\ncEnum {\n\tfilename\n\tcolor = (0.9 0.2 0.5)\n\t"
            "labelcolor = (0.0 0.0 0.0)\n}\n\ncEnumConst {\n\tfilename\n\tcolor = (0.9 0.2 0.5)\n\tlabelcolor = (0.0 0."
            "0 0.0)\n}\n\n(reference) {\n\taccess\n}\n\n";
    int const MD5_LENGTH = 33;

    std::string getMD5(std::string ID);

    bool resolveEdge(RexEdge* edge);
};


#endif //REX_TAGRAPH_H
