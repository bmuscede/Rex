//
// Created by bmuscede on 08/07/17.
//

#include "ROSConsumer.h"

ROSConsumer::Mode ROSConsumer::currentMode = ROSConsumer::FULL;

ROSConsumer::ROSConsumer(ASTContext *Context) : ROSVisitor(Context), MROSVisitor(Context) {}

void ROSConsumer::HandleTranslationUnit(ASTContext &Context) {
    //Checks which mode we're running in.
    if (currentMode == ROSConsumer::FULL){
        ROSVisitor.TraverseDecl(Context.getTranslationUnitDecl());
    } else {
        MROSVisitor.TraverseDecl(Context.getTranslationUnitDecl());
    }
}

void ROSConsumer::setMode(Mode curMode){
    currentMode = curMode;
}

std::unique_ptr<ASTConsumer> ROSAction::CreateASTConsumer(CompilerInstance &Compiler, StringRef InFile) {
    return std::unique_ptr<ASTConsumer>(new ROSConsumer(&Compiler.getASTContext()));
}