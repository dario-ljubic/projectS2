#ifndef PROJECTS2_INPUTPARSER_H
#define PROJECTS2_INPUTPARSER_H

#include <iostream>
#include <vector>
#include <algorithm>

class InputParser{
public:
  InputParser(int &argc, char **argv);

  const std::string GetCmdOption(const std::string &option) const;

  bool CmdOptionExists(const std::string &option) const;

private:
  std::vector <std::string> tokens;
  const std::string empty_string;

};

#endif //PROJECTS2_INPUTPARSER_H
