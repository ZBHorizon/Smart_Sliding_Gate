#ifndef CONFIG_MANAGER_H
#define CONFIG_MANAGER_H

#include <string>
#include <cstdint>


//brief Appends a named variable and its value to a binary file.
void save_variable(const std::string& var_name, std::int32_t value);

//brief Reads the value of a specific variable from the binary file.
std::int32_t load_variable(const std::string& var_name);

#endif 
