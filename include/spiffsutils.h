#include "FS.h"
#include "SPIFFS.h"

#define FORMAT_SPIFFS_IF_FAILED true
#define AM270 256
#define AM650 650

#define MAXSIGS 10

double default_freq = 433.92; // Change the default frequency here
int default_bandwidth = AM270;
double default_drate = 512;
double used_frequency;
int used_bandwidth;
double used_drate;
bool fileStatus[MAXSIGS] = {0};

void listSPIFFS(const char *dirname, uint8_t levels)
{ // Function that lists all SPIFFS files, signals and saved frequencies. It is called on setup
    Serial.printf("Listing directory: %s\n", dirname);

    File root = SPIFFS.open(dirname);
    if (!root)
    {
        Serial.printf("Open failed for %s\n", dirname);
        return;
    }
    if (!root.isDirectory())
    {
        Serial.printf("%s is not a directory\n", dirname);
        return;
    }

    File file = root.openNextFile();
    while (file)
    {
        if (file.isDirectory())
        {
            Serial.print(" DIR : ");
            Serial.println(file.name());
            if (levels)
            {
                listSPIFFS(file.name(), levels - 1);
            }
        }
        else
        {
            Serial.print(" FILE: ");
            Serial.print(file.name());
            Serial.print("\tSIZE: ");
            Serial.println(file.size());
        }
        file = root.openNextFile();
    }
}

bool loadSPIFFS(const char *path, uint16_t *signal433_store, uint16_t size)
{ // Function that loads all 10 recorded signals. It is called on setup
    Serial.printf("Reading file: %s\n", path);
    File file = SPIFFS.open(path);
    if (!file || file.isDirectory())
    {
        Serial.printf("Load failed for %s\n", path);
        return 0;
    }
    int i = 0;
    while (file.available() && i < size)
    {
        uint16_t value = ((unsigned char)file.read() << 8) | (unsigned char)file.read();
        signal433_store[i++] = value;
    }
    return 1;
}

void storeSPIFFS(const char *path, uint16_t *signal433_store, uint16_t size)
{ // Function that stores the recorded signals using SPIFFS
    Serial.printf("Writing file: %s\r\n", path);
    File file = SPIFFS.open(path, FILE_WRITE);
    if (!file)
    {
        Serial.printf("Store failed for %s\n", path);
        return;
    }
    int i = 0;
    while (i < size)
    {
        file.write((unsigned char)(signal433_store[i] >> 8 & 0xff));
        file.write((unsigned char)(signal433_store[i] & 0xff));
        i++;
    }
}

void saveFrequency(String freqname, double frequency)
{ // Function to save in storage the selected frequency for a specific page
    File freqFile = SPIFFS.open(freqname, "w");
    if (!SPIFFS.exists(freqname))
    {
        Serial.println("Failed to open frequency file for writing");
        return;
    }
    freqFile.print(frequency);
    freqFile.close();
}

void readFrequency(String freqname)
{ // Function that reads the saved frequency for a specific page
    File freqFile = SPIFFS.open(freqname, "r");
    if (!SPIFFS.exists(freqname))
    {
        Serial.println("Creating file with default frequency");
        saveFrequency(freqname, default_freq); // avoids having 0.00 by default when recording a single for the first time on a page
        return;
    }

    used_frequency = freqFile.readString().toDouble();
    freqFile.close();
}

void saveBandwidth(String filename, int bandwidth)
{
    File file = SPIFFS.open(filename, "w");
    if (!SPIFFS.exists(filename))
    {
        Serial.println("Failed to open bandwidth file for writing");
        return;
    }
    file.print(bandwidth);
    file.close();
}

void readBandwidth(String filename)
{
    File file = SPIFFS.open(filename, "r");
    if (!SPIFFS.exists(filename))
    {
        Serial.println("Creating file with default bandwidth");
        saveBandwidth(filename, default_bandwidth);
        return;
    }
    used_bandwidth = file.readString().toInt();
    file.close();
}

template <typename T>
void saveValueToFile(String filename, T value)
{
    File file = SPIFFS.open(filename, "w");
    if (!file)
    {
        Serial.println("Failed to open file for writing");
        return;
    }
    file.print(value);
    file.close();
}

void readDataRrate(String filename)
{
    File file = SPIFFS.open(filename, "r");
    if (!SPIFFS.exists(filename))
    {
        Serial.println("Creating file with default bandwidth");
        saveValueToFile(filename, default_drate);
        return;
    }
    used_drate = file.readString().toDouble();
    file.close();
}