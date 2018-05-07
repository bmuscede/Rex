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
    virtual ~TAGraph();

    void clearGraph();

    bool isEmpty();

    //Setters
    bool getMinMode();
    void setMinMode(bool minMode);

    //Node/Edge Adders
    virtual void addNode(RexNode* node);
    virtual void addEdge(RexEdge* edge);

    //Node/Edge Removers
    void hierarchyRemove(RexNode* toRemove);
    void removeNode(std::string nodeID);
    void removeEdge(std::string srcID, std::string dstID, RexEdge::EdgeType type, bool hashed = false);

    //Find Methods
    RexNode* findNode(std::string nodeID);
    RexNode* findNodeByName(std::string nodeName, bool MD5Check = false);
    RexNode* findNodeByEndName(std::string endName, bool MD5Check = false);
    std::vector<RexNode*> findNodesByType(RexNode::NodeType type);
    RexEdge* findEdge(std::string srcID, std::string dstID, RexEdge::EdgeType type);
    std::vector<RexEdge*> findEdgesBySrc(std::string srcID, bool md5 = true);
    std::vector<RexEdge*> findEdgesByDst(std::string dstID, bool md5 = true);

    //Element Exist Methods
    bool doesNodeExist(std::string nodeID);
    bool doesEdgeExist(std::string srcID, std::string dstID, RexEdge::EdgeType type);

    //Graph Clean Methods
    virtual bool resolveComponents(std::map<std::string, std::vector<std::string>> databaseMap);
    void resolveUnestablishedEdges();
    virtual void purgeUnestablishedEdges(bool resolveFirst = true);
    bool keepFeatures(std::vector<std::string> features);

    //TA Generators
    virtual std::string getTAModel();

protected:
    bool minMode;

    //Rex TA Schemas
    std::string const FULL_TA_SCHEMA = "//Full Rex Extraction\n//Author: Bryan J Muscedere\n\nSCHEME TUPLE :\n//Nodes\n"
                                       "$INHERIT\tcArchitecturalNds\tcRoot\n$INHERIT\tcAsgNds\t\t\tcRoot\n$INHERIT\trosMsg\t\t\tcRoot\n$INHERIT\tc"
                                       "Component\t\tcAsgNds\n$INHERIT\tcClass\t\t\tcAsgNds\n$INHERIT\tcFunction\t\tcAsgNds\n$INHERIT\tcVariable\t"
                                       "\tcAsgNds\n$INHERIT\trosTopic\t\trosMsg\n$INHERIT\trosPublisher\t\trosMsg\n$INHERIT\trosSubscriber\t\trosM"
                                       "sg\n$INHERIT\trosNodeHandle\t\trosMsg\n$INHERIT\trosTimer\t\trosMsg\n\n//Relationships\ncompContain\tcComp"
                                       "onent\tcClass\ncontain\t\tcRoot\t\tcRoot\npublish\t\trosPublisher\trosTopic\nsubscribe\trosTopic\trosSubsc"
                                       "riber\nreference\trosNodeHandle\trosMsg\nadvertise\trosPublisher\trosTopic\ncall\t\tcRoot\t\tcRoot\nwrite"
                                       "\t\tcFunction\tcVariable\nread\t\tcVariable\tcFunction\t\t\nvarWrite\tcVariable\tcVariable\nvarInfluence\t"
                                       "cVariable\trosMsg\nvarInfFunc\tcVariable\tcFunction\ntime\t\trosTimer\tcFunction\n\nSCHEME ATTRIBUTE :\n$E"
                                       "NTITY {\n\tx\n\ty\n\twidth\n\theight\n\tlabel\n}\n\ncRoot {\n\telision = contain\n\tcolor = (0.0 0.0 0.0)"
                                       "\n\tfile\n\tline\n\tname\n}\n\ncAsgNds {\n\tbeg\n\tend\n\tfile\n\tline\n\tvalue\n\tcolor = (0.0 0.0 0.0)\n"
                                       "}\n\ncArchitecturalNds {\n\tclass_style = 4\n\tcolor = (0.0 0.0 1.0)\n\tcolor = (0.0 0.0 0.0)\n}\n\nrosMsg"
                                       " {\n\tclass_style = 4\n\tcolor = (0.0 0.0 1.0)\n}\n\ncComponent {\n\tcolor = (0.7 0.3 0.1)\n\tlabelcolor ="
                                       " (0.0 0.0 0.0)\n\tclass_style = 2\n}\n\ncFunction {\n\tfilename\n\tisStatic\n\tisConst\n\tisVolatile\n\tis"
                                       "Variadic\n\tvisibility\n\tcolor = (1.0 0.0 0.0)\n\tlabelcolor = (0.0 0.0 0.0)\n}\n\ncVariable {\n\tfilenam"
                                       "e\n\tscopeType\n\tisStatic\n\tisControlFlow = 0\n\tisPublisher = 0\n\tisSubscriber = 0\n}\n\ncClass {\n\tf"
                                       "ilename\n\tbaseNum\n\tcolor = (0.2 0.4 0.1)\n\tlabelcolor = (0.0 0.0 0.0)\n\tclass_style = 2\n}\n\nrosSubs"
                                       "criber {\n\tcolor = (0.4 1.0 0.4)\n\tlabelcolor = (0.0 0.0 0.0)\n\tclass_style = 6\n\tbufferSize\n\tnumAtt"
                                       "ributes\n\trosNumber\t\n\tcallbackFunc\n}\n\nrosPublisher {\n\tcolor = (1.0 0.0 0.8)\n\tlabelcolor = (1.0 "
                                       "1.0 1.0)\n\tclass_style = 6\n\tnumAttributes\n\trosNumber\n\tisUnderControl = 0\n}\n\nrosTopic {\n\tcolor "
                                       "= (1.0 1.0 0.6)\n\tlabelcolor = (0.0 0.0 0.0)\n\tclass_style = 5\n}\n\n(call) {\n\tisUnderControl = 0\n}\n";
    std::string const MINI_TA_SCHEMA = "//Minimal Rex Extraction\n//Author: Bryan J Muscedere\n\nSCHEME TUPLE :\n//Node"
                                       "s\n$INHERIT\tcArchitecturalNds\tcRoot\n$INHERIT\tcAsgNds\t\t\tcRoot\n$INHERIT\trosMsg\t\t\tcRoot\n$INHERIT"
                                       "\tcComponent\t\tcAsgNds\n$INHERIT\tcClass\t\t\tcAsgNds\n$INHERIT\trosTopic\t\trosMsg\n$INHERIT\trosPublish"
                                       "er\t\trosMsg\n$INHERIT\trosSubscriber\t\trosMsg\n$INHERIT\trosNodeHandle\t\trosMsg\n$INHEIRT\trosTimer\t\t"
                                       "rosMsg\n\n//Relationships\ncompContain\tcComponent\tcClass\ncontain\t\tcRoot\t\tcRoot\npublish\t\trosPubli"
                                       "sher\trosTopic\nsubscribe\trosTopic\trosSubscriber\nreference\trosNodeHandle\trosMsg\nadvertise\trosPublis"
                                       "her\trosTopic\nsubscribe\trosSubscriber\trosTopic\n\nSCHEME ATTRIBUTE :\n$ENTITY {\n\tx\n\ty\n\twidth\n\th"
                                       "eight\n\tlabel\n}\n\ncRoot {\n\telision = contain\n\tcolor = (0.0 0.0 0.0)\n\tfile\n\tline\n\tname\n}\n\nc"
                                       "AsgNds {\n\tbeg\n\tend\n\tfile\n\tline\n\tvalue\n\tcolor = (0.0 0.0 0.0)\n}\n\ncArchitecturalNds {\n\tclas"
                                       "s_style = 4\n\tcolor = (0.0 0.0 1.0)\n\tcolor = (0.0 0.0 0.0)\n}\n\nrosMsg {\n\tclass_style = 4\n\tcolor ="
                                       " (0.0 0.0 1.0)\n}\n\ncComponent {\n\tcolor = (0.7 0.3 0.1)\n\tlabelcolor = (0.0 0.0 0.0)\n\tclass_style = "
                                       "2\n}\n\ncClass {\n\tfilename\n\tbaseNum\n\tcolor = (0.2 0.4 0.1)\n\tlabelcolor = (0.0 0.0 0.0)\n\tclass_st"
                                       "yle = 2\n}\n\nrosSubscriber {\n\tcolor = (0.4 1.0 0.4)\n\tlabelcolor = (0.0 0.0 0.0)\n\tclass_style = 6\n"
                                       "\tbufferSize\n\tnumAttributes\n\trosNumber\t\n\tcallbackFunc\n}\n\nrosPublisher {\n\tcolor = (1.0 0.0 0.8)"
                                       "\n\tlabelcolor = (1.0 1.0 1.0)\n\tclass_style = 6\n\tnumAttributes\n\trosNumber\n}\n\nrosTopic {\n\tcolor "
                                       "= (1.0 1.0 0.6)\n\tlabelcolor = (0.0 0.0 0.0)\n\tclass_style = 5\n}\n\n";

    //Member variables
    std::unordered_map<std::string, RexNode*> idList;
    std::unordered_map<std::string, std::vector<RexEdge*>> edgeSrcList;
    std::unordered_map<std::string, std::vector<RexEdge*>> edgeDstList;
    RexEdge::EdgeType forestEdgeType;

    std::string generateInstances();
    std::string generateRelations();
    std::string generateAttributes();

    //Helper Function
    bool hasEnding(std::string const &fullString, std::string const &ending);
private:
    //ROS Functionality
    std::string const FILENAME_ATTR = "filename";

    int const MD5_LENGTH = 33;

    //MD5 Generator
    std::string getMD5(std::string ID);

    //Edge Resolvers
    bool resolveEdge(RexEdge* edge);
    bool resolveEdgeByName(RexEdge* edge);
};


#endif //REX_TAGRAPH_H
