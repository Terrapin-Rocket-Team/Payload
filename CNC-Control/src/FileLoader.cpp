#include "FileLoader.h"
#include "Arduino.h"
#include <string.h>

namespace {
bool normalizeLine(char* line) {
    char* comment = strchr(line, ';');
    if (comment) {
        *comment = '\0';
    }

    char* start = line;
    while (*start == ' ' || *start == '\t') {
        start++;
    }

    char* end = start + strlen(start);
    while (end > start && (end[-1] == ' ' || end[-1] == '\t')) {
        end--;
    }
    *end = '\0';

    if (start != line) {
        memmove(line, start, strlen(start) + 1);
    }

    return line[0] != '\0';
}
}

FileLoader::FileLoader() {
    linecount = 0;
    memset(lines, 0, sizeof(lines));
}

bool FileLoader::load(astra::IStorage& storage, const char* filename) {
    astra::IFile* f = storage.openRead(filename);
    if (!f) {
        Serial.print("ERROR: System could not open file: ");
        Serial.println(filename);
        return false;
    }

    linecount = 0;
    int charIdx = 0;

    // Read byte-by-byte directly into the static array (Zero-Heap Allocation)
    while (f->available() && linecount < MAX_LINES) {
        char c = f->read();

        if (c == '\n' || c == '\r') {
            // End of line reached
            if (charIdx > 0) {
                lines[linecount][charIdx] = '\0'; // Null-terminate the string

                if (normalizeLine(lines[linecount])) {
                    linecount++;
                }
                charIdx = 0;
            }
        } else {
            // Guard against buffer overflow on a single long line
            if (charIdx < MAX_LINE_LEN - 1) {
                lines[linecount][charIdx] = c;
                charIdx++;
            } else {
                Serial.print("WARNING: Line ");
                Serial.print(linecount + 1);
                Serial.println(" exceeded MAX_LINE_LEN! Truncating remaining characters on this line.");

                // Fast-forward through the file until the actual end of this line
                while (f->available() && c != '\n' && c != '\r') {
                    c = f->read();
                }

                lines[linecount][charIdx] = '\0'; // Null-terminate what fits
                if (normalizeLine(lines[linecount])) {
                    linecount++;
                }
                charIdx = 0;
            }
        }
    }

    // Handle a file that doesn't end with a trailing newline
    if (charIdx > 0 && linecount < MAX_LINES) {
        lines[linecount][charIdx] = '\0';
        if (normalizeLine(lines[linecount])) {
            linecount++;
        }
    }

    if (f->available() && linecount >= MAX_LINES) {
        Serial.println("WARNING: SD file contains more lines than MAX_LINES. Remaining lines ignored.");
    }

    f->close();
    delete f;
    return true;
}

const char* FileLoader::getLine(int index) const {
    if (index < 0 || index >= linecount) {
        return nullptr;
    }
    return lines[index];
}

int FileLoader::countLine() const {
    return linecount;
}
