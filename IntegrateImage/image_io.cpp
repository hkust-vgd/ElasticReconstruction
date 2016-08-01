#include "image.h"
#include "lodepng/lodepng.h"

BEGIN_INCREON_SPACE

void ImageIO::loadDepth(std::string file, Image<ushort> &depth) {
    std::vector<uchar> pixels;
    uint width, height;

    unsigned error = lodepng::decode(pixels, width, height, file.c_str(), LCT_RGBA, 16);
    if (error) std::cout << "PNG read error " << error << ": " << lodepng_error_text(error) << std::endl;

    depth.alloc(width, height);
    for (uint i = 0; i < width * height; ++i) {
        ushort high = pixels[8 * i + 1];             // 2 bytes that make the 16-bit pixel. Big endian.
        ushort low  = pixels[8 * i + 0];
        depth.data()[i] = (low << 8) + high;
    }
}

void ImageIO::loadColor(std::string file, Image<Color3b> &image) {
    std::vector<uchar> pixels;
    uint width, height;

    unsigned error = lodepng::decode(pixels, width, height, file.c_str());
    if (error) std::cout << "PNG read error " << error << ": " << lodepng_error_text(error) << std::endl;

    image.alloc(width, height);
    for (uint i = 0; i < width * height; ++i) {
        image.data()[i].r = pixels[4 * i];
        image.data()[i].g = pixels[4 * i + 1];
        image.data()[i].b = pixels[4 * i + 2];
    }
}

void ImageIO::saveDepth(std::string file, const Image<ushort> &image) {
    uint width = image.width();
    uint height = image.height();
    std::vector<uchar> pixels(height * width * sizeof(ushort));
    
    uint k = 0;
    for (uint i = 0; i < height; ++i) {
        for (uint j = 0; j < width; ++j) {
            uint index = i * width + j;
            ushort p = image(j, i);
            pixels[2 * index    ] = (p & 0xff00) >> 8;
            pixels[2 * index + 1] = (p & 0x00ff);
        }
    }

    unsigned error = lodepng::encode(file.c_str(), pixels, width, height, LCT_GREY, 16);
    if (error) std::cout << "PNG write error " << error << ": " << lodepng_error_text(error) << std::endl;
}

void ImageIO::saveColor(std::string file, const Image<Color3b> &image) {
    uint width = image.width();
    uint height = image.height();
    std::vector<uchar> pixels(height * width * 4);
    
    uint k = 0;
    for (uint i = 0; i < height; ++i) {
        for (uint j = 0; j < width; ++j) {
            uint index = i * width + j;
            Color3b p = image(j, i);
            pixels[4 * index    ] = p.r;
            pixels[4 * index + 1] = p.g;
            pixels[4 * index + 2] = p.b;
            pixels[4 * index + 3] = 255;
        }
    }

    unsigned error = lodepng::encode(file.c_str(), pixels, width, height);
    if (error) std::cout << "PNG write error " << error << ": " << lodepng_error_text(error) << std::endl;
}

void ImageIO::saveIntensityGrey(std::string file, const Image<float> &image) {
    uint width = image.width();
    uint height = image.height();
    std::vector<uchar> pixels(height * width * 4);
    
    uint k = 0;
    for (uint i = 0; i < height; ++i) {
        for (uint j = 0; j < width; ++j) {
            uint index = i * width + j;
            float p = image(j, i);
            pixels[4 * index    ] = p * 255;
            pixels[4 * index + 1] = p * 255;
            pixels[4 * index + 2] = p * 255;
            pixels[4 * index + 3] = 255;
        }
    }

    unsigned error = lodepng::encode(file.c_str(), pixels, width, height);
    if (error) std::cout << "PNG write error " << error << ": " << lodepng_error_text(error) << std::endl;
}

void ImageIO::loadMask(std::string file, Image<bool> &image) { 
    std::vector<uchar> pixels;
    uint width, height;

    unsigned error = lodepng::decode(pixels, width, height, file.c_str(), LCT_GREY, 8);
    if (error) std::cout << "PNG read error " << error << ": " << lodepng_error_text(error) << std::endl;

    image.alloc(width, height);
    for (uint i = 0; i < width * height; ++i) {
        image.data()[i] = (bool)pixels[i];
    }
}

void ImageIO::saveMask(std::string file, const Image<bool> &image) {
    uint width = image.width();
    uint height = image.height();
    std::vector<uchar> pixels(height * width);
    
    uint k = 0;
    for (uint i = 0; i < height; ++i) {
        for (uint j = 0; j < width; ++j) {
            uint index = i * width + j;
            pixels[index] = image(j, i) * 255;
        }
    }

    unsigned error = lodepng::encode(file.c_str(), pixels, width, height, LCT_GREY, 8);
    if (error) std::cout << "PNG write error " << error << ": " << lodepng_error_text(error) << std::endl;
}

END_INCREON_SPACE
