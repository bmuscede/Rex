//
// Created by bmuscede on 08/07/17.
//

#include "ROSConsumer.h"

using namespace std;

ROSConsumer::Mode ROSConsumer::currentMode = ROSConsumer::FULL;
vector<string> ROSConsumer::libraries = vector<string>();

ROSConsumer::ROSConsumer(ASTContext *Context) : ROSVisitor(Context), MROSVisitor(Context) {}

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

void ROSConsumer::setMode(Mode curMode){
    currentMode = curMode;

    if (curMode == Mode::MINIMAL){
        ParentWalker::setCurrentGraphMinMode(true);
    } else {
        ParentWalker::setCurrentGraphMinMode(false);
    }
}

void ROSConsumer::setLibrariesToIgnore(vector<string> libraries){
    ROSConsumer::libraries = libraries;
}

std::unique_ptr<ASTConsumer> ROSAction::CreateASTConsumer(CompilerInstance &Compiler, StringRef InFile) {
    return std::unique_ptr<ASTConsumer>(new ROSConsumer(&Compiler.getASTContext()));
}