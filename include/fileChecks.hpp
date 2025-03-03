#ifndef FILECHECKS_HPP
#define FILECHECKS_HPP

#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <rapidjson/document.h>
#include <rapidjson/error/en.h>


int configFileChecks(const std::string& filename) {
    std::ifstream ifs(filename);

    if (!ifs.is_open()) {
        std::cerr << "Error: Unable to open file '" << filename << "'" << std::endl;
        return 1; // Return a non-zero status code to indicate failure
    }

    // Check if the file is empty
    if (ifs.peek() == std::ifstream::traits_type::eof()) {
        std::cerr << "Error: File '" << filename << "' is empty." << std::endl;
        return 1;
    }

    // Attempt to read a character to check if it's a text file
    char firstChar;
    if (!ifs.get(firstChar)) {
        std::cerr << "Error: Unable to read from file '" << filename << "'." << std::endl;
        return 1;
    }
    ifs.unget(); // Put the character back

    if (!isprint(firstChar) && !isspace(firstChar)) {
        std::cerr << "Error: File '" << filename << "' does not appear to be a text file." << std::endl;
        return 1;
    }


    std::stringstream buffer;
    buffer << ifs.rdbuf();
    std::string content = buffer.str();
    rapidjson::Document document;
    document.Parse(content.c_str());
    if (document.HasParseError()) {
        std::cerr << "Error: Failed to parse JSON from file '" << filename
                  << "'; Error was: " << rapidjson::GetParseError_En(document.GetParseError())
                  << " at offset " << document.GetErrorOffset() << std::endl;
        return 1;
    }

    return 0;
}

#endif