#include "FileLoader.h"
#include <Arduino.h>
#include <SPI.h>
#include <SD.h>
#include "FileLoader.h"



FileLoader::FileLoader() {
    linecount = 0;
    memset(lines, 0, sizeof(lines));
}

bool FileLoader::load(const char* filename) {
    File f = SD.open(filename, FILE_READ);
    if (!f) return false;

    linecount = 0;

    while (f.available() && linecount < MAX_LINES) {
        String line = f.readStringUntil('\n');
        line.trim();

        if (line.length() == 0) continue;
        if (line.length() >= MAX_LINE_LEN) continue;

        line.toCharArray(lines[linecount], MAX_LINE_LEN);
        linecount++;
    }

    f.close();
    return true;
}