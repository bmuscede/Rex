/////////////////////////////////////////////////////////////////////////////////////////////////////////
// TAGraph.h
//
// Created By: Bryan J Muscedere
// Date: 10/04/17.
//
// Master system for maintaining the in-memory
// digraph system. This is important to store
// C++ entities and then output to TA.
//
// Copyright (C) 2017, Bryan J. Muscedere
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <http://www.gnu.org/licenses/>.
/////////////////////////////////////////////////////////////////////////////////////////////////////////

#ifndef REX_TAGRAPH_H
#define REX_TAGRAPH_H

#include <unordered_map>
#include <string>
#include <vector>
#include "RexNode.h"
#include "RexEdge.h"

class TAGraph {
public:
    //Constructor/Destructor
    TAGraph(RexEdge::EdgeType edgeType = RexEdge::EdgeType::CONTAINS);
    ~TAGraph();

    //Setters
    void setMinMode(bool minMode);

    //Node/Edge Adders
    void addNode(RexNode* node);
    void addEdge(RexEdge* edge);

    //Node/Edge Removers
    void removeNode(std::string nodeID);
    void removeEdge(std::string srcID, std::string dstID, RexEdge::EdgeType type, bool hashed = false);

    //Find Methods
    RexNode* findNode(std::string nodeID);
    RexNode* findNodeByName(std::string nodeName, bool MD5Check = false);
    RexNode* findNodeByEndName(std::string endName, bool MD5Check = false);
    RexEdge* findEdge(std::string srcID, std::string dstID, RexEdge::EdgeType type);
    std::vector<RexEdge*> findEdgesBySrc(std::string srcID, bool md5 = true);
    std::vector<RexEdge*> findEdgesByDst(std::string dstID, bool md5 = true);

    //Element Exist Methods
    bool doesNodeExist(std::string nodeID);
    bool doesEdgeExist(std::string srcID, std::string dstID, RexEdge::EdgeType type);

    //Graph Clean Methods
    std::string checkCorrectness();
    bool resolveComponents(std::map<std::string, std::vector<std::string>> databaseMap);
    void resolveUnestablishedEdges();
    void purgeUnestablishedEdges(bool resolveFirst = true);

    //TA Generators
    std::string getTAModel();

    //ROS Node Generators
    RexNode* generateSubscriberNode(std::string parentID, std::string parentName);
    RexNode* generatePublisherNode(std::string parentID, std::string parentName);

private:
    //Member variables
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

    //Rex TA Schemas
    std::string const FULL_TA_SCHEMA = "//Full Rex Extraction\n//Author: Bryan J Muscedere\n\nSCHEME TUPLE :\n//Nodes\n"
            "$INHERIT\tcArchitecturalNds\tcRoot\n$INHERIT\tcAsgNds\t\t\tcRoot\n$INHERIT\trosMsg\t\t\tcRoot\n$INHERIT\tc"
            "Component\t\tcAsgNds\n$INHERIT\tcClass\t\t\tcAsgNds\n$INHERIT\tcFunction\t\tcAsgNds\n$INHERIT\tcVariable\t"
            "\tcAsgNds\n$INHERIT\trosTopic\t\trosMsg\n$INHERIT\trosPublisher\t\trosMsg\n$INHERIT\trosSubscriber\t\trosM"
            "sg\n$INHERIT\trosNodeHandle\t\trosMsg\n\n//Relationships\ncompContain\tcComponent\tcClass\ncontain\t\tcRoo"
            "t\t\tcRoot\npublish\t\trosPublisher\trosTopic\nsubscribe\trosTopic\trosSubscriber\nreference\trosNodeHandl"
            "e\trosMsg\nadvertise\trosPublisher\trosTopic\ncall\t\tcRoot\t\tcRoot\nwrite\t\tcFunction\tcVariable\nread"
            "\t\tcVariable\tcFunction\t\t\nvarWrite\tcVariable\tcVariable\nvarInfluence\tcVariable\trosMsg\nvarInfFunc"
            "\tcVariable\tcFunction\n\nSCHEME ATTRIBUTE :\n$ENTITY {\n\tx\n\ty\n\twidth\n\theight\n\tlabel\n}\n\ncRoot "
            "{\n\telision = contain\n\tcolor = (0.0 0.0 0.0)\n\tfile\n\tline\n\tname\n}\n\ncAsgNds {\n\tbeg\n\tend\n\tf"
            "ile\n\tline\n\tvalue\n\tcolor = (0.0 0.0 0.0)\n}\n\ncArchitecturalNds {\n\tclass_style = 4\n\tcolor = (0.0"
            " 0.0 1.0)\n\tcolor = (0.0 0.0 0.0)\n}\n\nrosMsg {\n\tclass_style = 4\n\tcolor = (0.0 0.0 1.0)\n}\n\ncCompo"
            "nent {\n\tcolor = (0.7 0.3 0.1)\n\tlabelcolor = (0.0 0.0 0.0)\n\tclass_style = 2\n}\n\ncFunction {\n\tfile"
            "name\n\tisStatic\n\tisConst\n\tisVolatile\n\tisVariadic\n\tvisibility\n\tcolor = (1.0 0.0 0.0)\n\tlabelcol"
            "or = (0.0 0.0 0.0)\n}\n\ncVariable {\n\tfilename\n\tscopeType\n\tisStatic\n\tisControlFlow = 0\n\tisPublis"
            "her = 0\n\tisSubscriber = 0\n}\n\ncClass {\n\tfilename\n\tbaseNum\n\tcolor = (0.2 0.4 0.1)\n\tlabelcolor ="
            " (0.0 0.0 0.0)\n\tclass_style = 2\n}\n\nrosSubscriber {\n\tcolor = (0.4 1.0 0.4)\n\tlabelcolor = (0.0 0.0 "
            "0.0)\n\tclass_style = 6\n\tbufferSize\n\tnumAttributes\n\trosNumber\t\n\tcallbackFunc\n}\n\nrosPublisher {"
            "\n\tcolor = (1.0 0.0 0.8)\n\tlabelcolor = (1.0 1.0 1.0)\n\tclass_style = 6\n\tnumAttributes\n\trosNumber\n"
            "\tisUnderControl = 0\n}\n\nrosTopic {\n\tcolor = (1.0 1.0 0.6)\n\tlabelcolor = (0.0 0.0 0.0)\n\tclass_styl"
            "e = 5\n}\n\n(call) {\n\tisUnderControl = 0\n}\n";
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

    //MD5 Generator
    std::string getMD5(std::string ID);

    //Edge Resolvers
    bool resolveEdge(RexEdge* edge);
    bool resolveEdgeByName(RexEdge* edge);

    //ROS Helper Functions
    RexNode* generateROSNode(std::string parentID, std::string parentName, RexNode::NodeType type);
    int getLastROSNumber(std::string rosID);

    //Helper Function
    bool hasEnding(std::string const &fullString, std::string const &ending);
};


#endif //REX_TAGRAPH_H
