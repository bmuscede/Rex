//
// Created by bmuscede on 07/04/17.
//

#include "ROSWalker.h"

ROSWalker::ROSWalker(){

}

ROSWalker::~ROSWalker(){

}

void ROSWalker::run(const MatchFinder::MatchResult &result){
    //Check what the
}

void ROSWalker::setMatchers(MatchFinder *finder){
    finder->addMatcher(functionDecl().bind(FUNCTION_DECL), this);
}