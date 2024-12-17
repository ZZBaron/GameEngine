// console.h
#pragma once

#include "GameEngine.h"


class Console {
public:
    static Console& getInstance();
    void toggleVisibility();
    bool isVisible() const;
    void render();
    void addInput(char c);
    void removeLastChar();
    void executeCommand();
    void registerCommand(const std::string& name, std::function<void(const std::vector<std::string>&)> func);
    std::vector<std::string> tokenizeInput(const std::string& input);
    void addCommandHistory(const std::string& entry); // New method to ad

private:
    Console() : visible(false) {}
    Console(const Console&) = delete;
    Console& operator=(const Console&) = delete;

    bool visible;
    std::string inputBuffer;
    std::vector<std::string> commandHistory;
    std::map<std::string, std::function<void(const std::vector<std::string>&)>> commands;

};

void initializeConsole();