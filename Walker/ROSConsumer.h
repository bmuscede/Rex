/////////////////////////////////////////////////////////////////////////////////////////////////////////
// ROSConsumer.h
//
// Created By: Bryan J Muscedere
// Date: 08/07/17.
//
// Sets up the AST walker components that
// Rex uses to walk through Clang's AST.
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

#ifndef REX_ROSCONSUMER_H
#define REX_ROSCONSUMER_H

#include <string>
#include <vector>
#include "ROSWalker.h"
#include "MinimalROSWalker.h"

class ROSConsumer : public ASTConsumer {
public:
    //Constructor/Destructor
    explicit ROSConsumer(ASTContext *Context);
    virtual void HandleTranslationUnit(ASTContext &Context);

    //Mode Functions
    enum Mode {FULL, MINIMAL};
    static void setMode(Mode curMode);
    static bool setLowMemory(bool low, std::string lowMemPath = "");

    //Library Functions
    static void setLibrariesToIgnore(std::vector<std::string> libraries);

private:
    ROSWalker ROSVisitor;
    MinimalROSWalker MROSVisitor;

    static Mode currentMode;
    static std::vector<std::string> libraries;
};

class ROSAction : public ASTFrontendAction {
public:
    //Consumer Functions
    virtual std::unique_ptr<ASTConsumer> CreateASTConsumer(CompilerInstance &Compiler, StringRef InFile);
};

#endif //REX_ROSCONSUMER_H
