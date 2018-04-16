#include "InputParser.h"

InputParser::InputParser(int &argc, char **argv) : empty_string("") {

  for (int i = 1; i < argc; ++i)
    tokens.emplace_back(std::string(argv[i]));
}

const std::string InputParser::GetCmdOption(const std::string &option) const {

  std::vector<std::string>::const_iterator itr;

  itr = std::find(tokens.begin(), tokens.end(), option);

  if (itr != tokens.end() && ++itr != tokens.end()) {
    return *itr;
  }

  return empty_string;
}

bool InputParser::CmdOptionExists(const std::string &option) const {

  return std::find(tokens.begin(), tokens.end(), option) != tokens.end();
}