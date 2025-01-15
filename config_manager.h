#pragma once
#include <string>
#include <cstdint>


//Appends a named variable and its value to a binary file.
void save_variable(const std::string& var_name, std::int32_t value);

//Reads the value of a specific variable from the binary file.
std::int32_t load_variable(const std::string& var_name);

