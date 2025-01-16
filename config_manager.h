#pragma once
#include <string>
#include <any>


//Appends a named variable and its value to a binary file.
void save_variable(const std::string& var_name, std::any value);

//Reads the value of a specific variable from the binary file.
std::any load_variable(const std::string& var_name);

