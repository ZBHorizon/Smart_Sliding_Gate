#include "config_manager.h"
#include <fstream>
#include <stdexcept>

/*!
 * \brief Appends a named variable and its value to a binary file.
 * \param var_name The name of the variable to save.
 * \param value    The value of the variable to save.
 * \throws std::runtime_error if the file cannot be opened or written.
 */
void save_variable(const std::string& var_name, std::int32_t value)
{
    std::ofstream file("config.bin", std::ios::binary | std::ios::app);
    if (!file.is_open()) {
        throw std::runtime_error("Failed to open config.bin for writing.");
    }

    // Save the length of the variable name
    std::uint32_t name_length = var_name.size();
    file.write(reinterpret_cast<const char*>(&name_length), sizeof(name_length));

    // Save the variable name
    file.write(var_name.data(), name_length);

    // Save the value
    file.write(reinterpret_cast<const char*>(&value), sizeof(value));

    if (!file.good()) {
        throw std::runtime_error("Failed to write to config.bin.");
    }

    file.close();
}

/*!
 * \brief Reads the value of a specific variable from the binary file.
 * \param var_name The name of the variable to read.
 * \return The value of the variable.
 * \throws std::runtime_error if the file cannot be opened or the variable is not found.
 */
std::int32_t load_variable(const std::string& var_name)
{
    std::ifstream file("config.bin", std::ios::binary);
    if (!file.is_open()) {
        throw std::runtime_error("Failed to open config.bin for reading.");
    }

    while (file.good()) {
        // Read the length of the variable name
        std::uint32_t name_length;
        file.read(reinterpret_cast<char*>(&name_length), sizeof(name_length));
        if (!file.good()) break;

        // Read the variable name
        std::string name(name_length, '\0');
        file.read(name.data(), name_length);
        if (!file.good()) break;

        // Read the value
        std::int32_t value;
        file.read(reinterpret_cast<char*>(&value), sizeof(value));
        if (!file.good()) break;

        // Check if the variable name matches
        if (name == var_name) {
            file.close();
            return value;
        }
    }

    file.close();
    throw std::runtime_error("Variable not found: " + var_name);
}
