//
// Created by bmuscede on 15/05/17.
//

#ifndef REX_REXHANDLER_H
#define REX_REXHANDLER_H

#include <string>

class RexHandler {
public:
    RexHandler();
    ~RexHandler();

    int getNumGraphs();

    bool processClangToolCode(std::string argument);
    bool processAllROSProjects();
    bool processROSProject(std::string projName);

private:

};


#endif //REX_REXHANDLER_H
