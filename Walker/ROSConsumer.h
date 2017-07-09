//
// Created by bmuscede on 08/07/17.
//

#ifndef REX_ROSCONSUMER_H
#define REX_ROSCONSUMER_H

#include "ROSWalker.h"
#include "MinimalROSWalker.h"

class ROSConsumer : public ASTConsumer {
public:
    explicit ROSConsumer(ASTContext *Context);
    virtual void HandleTranslationUnit(ASTContext &Context);

    enum Mode {FULL, MINIMAL};
    static void setMode(Mode curMode);

private:
    ROSWalker ROSVisitor;
    MinimalROSWalker MROSVisitor;

    static Mode currentMode;
};

class ROSAction : public ASTFrontendAction {
public:
    virtual std::unique_ptr<ASTConsumer> CreateASTConsumer(CompilerInstance &Compiler, StringRef InFile);
};

#endif //REX_ROSCONSUMER_H
