#pragma once

#include <Utils/Astra.h>
#include <State/DefaultState.h>
#include <Vector.h>


using namespace astra;

// Max number of lines the loader can store
#define MAX_LINES 50

// Max number of characters per line
#define MAX_LINE_LEN 64



class FileLoader {
    public:

    FileLoader();



    bool load(const char* filename);

    const char* getLine(int index);

    int countLine();



    private:

    char lines[MAX_LINES][MAX_LINE_LEN];

    int linecount;
};