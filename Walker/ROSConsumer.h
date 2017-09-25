//
// Created by bmuscede on 08/07/17.
//

#ifndef REX_ROSCONSUMER_H
#define REX_ROSCONSUMER_H

#include <string>
#include <vector>
#include "ROSWalker.h"
#include "MinimalROSWalker.h"

class ROSConsumer : public ASTConsumer {
public:
    explicit ROSConsumer(ASTContext *Context);
    virtual void HandleTranslationUnit(ASTContext &Context);

    enum Mode {FULL, MINIMAL};
    static void setMode(Mode curMode);

    static void setLibrariesToIgnore(std::vector<std::string> libraries);

private:
    ROSWalker ROSVisitor;
    MinimalROSWalker MROSVisitor;

    static Mode currentMode;
    static std::vector<std::string> libraries;

};

class ROSAction : public ASTFrontendAction {
public:
    virtual std::unique_ptr<ASTConsumer> CreateASTConsumer(CompilerInstance &Compiler, StringRef InFile);
};

#endif //REX_ROSCONSUMER_H
