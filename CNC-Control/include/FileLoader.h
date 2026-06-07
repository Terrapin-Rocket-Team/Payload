#pragma once

#include <RecordData/Storage/IStorage.h>

// Max number of lines the loader can store
#define MAX_LINES 50

// Max number of characters per line
#define MAX_LINE_LEN 64



class FileLoader {
    public:

    FileLoader();



    bool load(astra::IStorage& storage, const char* filename);

    const char* getLine(int index) const;

    int countLine() const;



    private:

    char lines[MAX_LINES][MAX_LINE_LEN];

    int linecount;
};
