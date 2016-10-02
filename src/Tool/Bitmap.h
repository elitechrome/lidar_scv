
#pragma once

#include <stdio.h>
#include <memory.h>
#include <vector>
#include <fstream>
using namespace std;

namespace SCV
{

class Bitmap
{

    #pragma pack(push, 1)
    struct fileHeader
    {
        unsigned short bfType;
        unsigned int bfSize;
        unsigned short bfReserved1;
        unsigned short bfReserved2;
        unsigned int bfOffBits;
    };
    #pragma pack(pop)

    struct infoHeader
    {
        unsigned int biSize;
        unsigned int biWidth;
        unsigned int biHeight;
        unsigned short biPlanes;
        unsigned short biBitCount;
        unsigned int biCompression;
        unsigned int biSizeImage;
        unsigned int biXPelsPerMeter;
        unsigned int biYPelsPerMeter;
        unsigned int biClrUsed;
        unsigned int biClrImportant;
    };


public:
    Bitmap() {}
    ~Bitmap() {}

private:
    fileHeader bmpFileHeader;
    infoHeader bmpInfoHeader;
    vector <unsigned char> bmpPalette;
    vector <unsigned char> bmpRawData;

public:
    void initForSLAM(vector <unsigned char> &map, int sizeX, int sizeY)
    {
        // file header
        bmpFileHeader.bfType = 0x4D42;
        bmpFileHeader.bfSize = 54 + 1024 + map.size();
        bmpFileHeader.bfReserved1 = 0;
        bmpFileHeader.bfReserved2 = 0;
        bmpFileHeader.bfOffBits = 54 + 1024;

        // info header
        bmpInfoHeader.biSize = 40;
        bmpInfoHeader.biWidth = sizeX;
        bmpInfoHeader.biHeight = sizeY;
        bmpInfoHeader.biPlanes = 1;
        bmpInfoHeader.biBitCount = 8;
        bmpInfoHeader.biCompression = 0;
        bmpInfoHeader.biSizeImage = 0;
        bmpInfoHeader.biXPelsPerMeter = 1;
        bmpInfoHeader.biYPelsPerMeter = 1;
        bmpInfoHeader.biClrUsed = 0;
        bmpInfoHeader.biClrImportant = 0;

        // palatte
        bmpPalette.resize(1024);
        int x, y = 0;
        for (x = 0; x < 256; x++)
        {
            bmpPalette[y] = x;
            bmpPalette[y + 1] = x;
            bmpPalette[y + 2] = x;
            bmpPalette[y + 3] = 0;
            y = y + 4;
        }

        // data
        bmpRawData = map;
    }

    void save(string filePath)
    {
        ofstream fout;

        fout.open(filePath.c_str(), ios_base::binary);
        fout.write(reinterpret_cast<char*>(&bmpFileHeader), sizeof(bmpFileHeader));
        fout.write(reinterpret_cast<char*>(&bmpInfoHeader), sizeof(bmpInfoHeader));
        fout.write(reinterpret_cast<char*>(bmpPalette.data()), bmpPalette.size() * sizeof(unsigned char));
        fout.write(reinterpret_cast<char*>(bmpRawData.data()), bmpRawData.size() * sizeof(unsigned char));

        fout.close();
    }

    void load(string filePath, vector <unsigned char> &map)
    {
        ifstream fin;

        fin.open(filePath.c_str(), ios_base::binary);
        fin.read(reinterpret_cast<char*>(&bmpFileHeader), sizeof(bmpFileHeader));
        fin.read(reinterpret_cast<char*>(&bmpInfoHeader), sizeof(bmpInfoHeader));
        bmpPalette.resize(bmpFileHeader.bfOffBits - 54);
        bmpRawData.resize(bmpFileHeader.bfSize - bmpFileHeader.bfOffBits);
        fin.read(reinterpret_cast<char*>(bmpPalette.data()), bmpPalette.size() * sizeof(unsigned char));
        fin.read(reinterpret_cast<char*>(bmpRawData.data()), bmpRawData.size() * sizeof(unsigned char));

        fin.close();

        /// val < 128 => occ
        int size = bmpRawData.size();
        for (int i = 0; i < size; i++) if (bmpRawData[i] < 128) bmpRawData[i] = 0;

        map = bmpRawData;
    }


};


}
