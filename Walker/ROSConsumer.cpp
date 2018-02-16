/////////////////////////////////////////////////////////////////////////////////////////////////////////
// ROSConsumer.cpp
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

#include "ROSConsumer.h"

using namespace std;

ROSConsumer::Mode ROSConsumer::currentMode = ROSConsumer::FULL;
vector<string> ROSConsumer::libraries = vector<string>();

/**
 * Creates a ROS consumer.
 * @param Context The AST context.
 */
ROSConsumer::ROSConsumer(ASTContext *Context) : ROSVisitor(Context), MROSVisitor(Context) {}

/**
 * Handles the AST context's translation unit. Tells Clang to traverse AST.
 * @param Context The AST context.
 */
void ROSConsumer::HandleTranslationUnit(ASTContext &Context) {
    //Checks which mode we're running in.
    if (currentMode == ROSConsumer::FULL){
        if (ROSConsumer::libraries.size() != 0) ROSVisitor.addLibrariesToIgnore(ROSConsumer::libraries);
        ROSVisitor.TraverseDecl(Context.getTranslationUnitDecl());
    } else {
        if (ROSConsumer::libraries.size() != 0) MROSVisitor.addLibrariesToIgnore(ROSConsumer::libraries);
        MROSVisitor.TraverseDecl(Context.getTranslationUnitDecl());
    }
}

/**
 * Sets the processing mode.
 * @param curMode The mode to process.
 */
void ROSConsumer::setMode(Mode curMode){
    currentMode = curMode;

    if (curMode == Mode::MINIMAL){
        ParentWalker::setCurrentGraphMinMode(true);
    } else {
        ParentWalker::setCurrentGraphMinMode(false);
    }
}

/**
 * Sets the libraries to ignore.
 * @param libraries The libraries to ignore.
 */
void ROSConsumer::setLibrariesToIgnore(vector<string> libraries){
    ROSConsumer::libraries = libraries;
}

/**
 * Creates an AST consumer to "eat" the AST.
 * @param Compiler The compiler instance to process.
 * @param InFile The input file.
 * @return A pointer to the AST consumer.
 */
std::unique_ptr<ASTConsumer> ROSAction::CreateASTConsumer(CompilerInstance &Compiler, StringRef InFile) {
    return std::unique_ptr<ASTConsumer>(new ROSConsumer(&Compiler.getASTContext()));
}