//
// Created by bmuscede on 07/04/17.
//

#ifndef REX_ROSWALKER_H
#define REX_ROSWALKER_H

#include "clang/ASTMatchers/ASTMatchers.h"
#include "clang/ASTMatchers/ASTMatchFinder.h"

using namespace clang::ast_matchers;

class ROSWalker : public MatchFinder::MatchCallback {
public:
    ROSWalker();
    ~ROSWalker();

    virtual void run(const MatchFinder::MatchResult &result);
    void setMatchers(MatchFinder *finder);

private:
    const std::string FUNCTION_DECL = "fd";

};


#endif //REX_ROSWALKER_H
