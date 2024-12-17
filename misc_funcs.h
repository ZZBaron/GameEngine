#pragma once 
#define NOMINMAX  // Prevent Windows.h from defining min/max macros
#ifdef _WIN32
#include <windows.h>
#endif
#include <filesystem>
#include "GameEngine.h"

namespace fs = std::filesystem;


// Cross - platform solution to getting project root using std::filesystem
std::string getProjectRoot() {
#ifdef _WIN32
	wchar_t buffer[MAX_PATH];
	GetModuleFileNameW(NULL, buffer, MAX_PATH);
	fs::path executablePath = fs::path(buffer).parent_path();

	// Look for the "GameEngine" directory in the path
	while (executablePath.has_parent_path() && executablePath.filename() != "GameEngine") {
		executablePath = executablePath.parent_path();
	}

	// Check if we found the GameEngine directory
	if (executablePath.filename() != "GameEngine") {
		throw std::runtime_error("Could not find GameEngine directory in path");
	}

	return executablePath.string();
#elif defined(__linux__) || defined(__APPLE__)
	char buffer[PATH_MAX];
	ssize_t count = readlink("/proc/self/exe", buffer, PATH_MAX);
	fs::path executablePath = fs::path(std::string(buffer, (count > 0) ? count : 0)).parent_path();

	while (executablePath.has_parent_path() && executablePath.filename() != "GameEngine") {
		executablePath = executablePath.parent_path();
	}

	if (executablePath.filename() != "GameEngine") {
		throw std::runtime_error("Could not find GameEngine directory in path");
	}

	return executablePath.string();
#endif
}

// Misclanious functions
std::string vec3_to_string(glm::vec3 v, int decimal_places = 2) { // assuming vec3 is floats
	std::stringstream x_comp_stream;
	x_comp_stream << std::fixed << std::setprecision(decimal_places) << v.x;
	std::string x_str = x_comp_stream.str();

	std::stringstream y_comp_stream;
	y_comp_stream << std::fixed << std::setprecision(decimal_places) << v.y;
	std::string y_str = y_comp_stream.str();

	std::stringstream z_comp_stream;
	z_comp_stream << std::fixed << std::setprecision(decimal_places) << v.z;
	std::string z_str = z_comp_stream.str();

	std::string s = std::string("(") + x_str + std::string(", ") + y_str + std::string(", ") + z_str + std::string(")");

	return s;

}

class debugLog {
public:
	std::vector<std::string> log;

	void add_to_debugLog(std::string s, bool print = true) {
		if (print) {
			std::cout << s << std::endl;
		}

		log.push_back(s);
	}

	void save_to_file() { //  saves each string in log to a new line in a txt file
		// Create the Debug Logs directory if it doesn't exist
		fs::path debugLogsPath = fs::path(getProjectRoot()) / "Debug Logs";
		if (!fs::exists(debugLogsPath)) {
			fs::create_directory(debugLogsPath);
		}

		// Open file for writing (will overwrite existing file)
		fs::path logFile = debugLogsPath / "log.txt";
		std::ofstream file(logFile, std::ios::trunc);

		if (!file.is_open()) {
			throw std::runtime_error("Failed to open log file for writing: " + logFile.string());
		}

		// Write each log entry to the file
		for (const auto& entry : log) {
			file << entry << std::endl;
		}

		file.close();
	}

};

