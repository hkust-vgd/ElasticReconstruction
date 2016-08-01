#pragma once

#include "increon.h"

BEGIN_INCREON_SPACE

struct Size2 {
  uint width, height;

  Size2(uint width, uint height) : width(width), height(height) {}
  Size2(uint s) : width(s), height(s) {}
  Size2() : width(0), height(0) {}
};

template <typename T>
class Image {
public:
  Image() : vdata(NULL) {
    
  }

  Image(const Image<T> &img) : vdata(NULL) {
    this->operator=(img);
  }

  Image(const Size2 & s) : vdata(NULL) {
    alloc(s);
  }

  Image(const uint& cols, const uint& rows) : vdata(NULL) {
    alloc(cols, rows);
    SetZero();
  }

  void alloc(const Size2 & s) {
    if (s.width == dim.width && s.height == dim.height)
      return;
    if (vdata) free(vdata);
    vdata = malloc(s.width * s.height * sizeof(T));
    dim  = s;
  }

  void alloc(const uint cols, const uint rows) {
    if (cols == dim.width && rows == dim.height)
      return;
    if (vdata) free(vdata);
    vdata = malloc(cols * rows * sizeof(T));
    dim  = Size2(cols, rows);
  }

  uint width() const {
    return dim.width;
  }

  uint height() const {
    return dim.height;
  }

  Size2 size() const {
    return dim;
  }

  T& operator()(const uint &c, const uint &r) {
    return static_cast<T*>(vdata)[c + dim.width * r];
  }

  const T& operator()(const uint &c, const uint &r) const {
    return static_cast<const T*>(vdata)[c + dim.width * r];
  }

  T& operator()(const Eigen::Vector2i &p) {
    return static_cast<T*>(vdata)[p.x() + dim.width * p.y()];
  }

  const T& operator()(const Eigen::Vector2i &p) const {
    return static_cast<const T*>(vdata)[p.x() + dim.width * p.y()];
  }

  T& operator[](const Eigen::Vector2i &p) {
    return static_cast<T*>(vdata)[p.x() + dim.width * p.y()];
  }

  const T& operator[](const Eigen::Vector2i &p) const {
    return static_cast<const T*>(vdata)[p.x() + dim.width * p.y()];
  }

  T& operator[](const uint &i) {
    return static_cast<T*>(vdata)[i];
  }

  const T& operator[](const uint &i) const {
    return static_cast<const T*>(vdata)[i];
  }

  T& operator()(const uint &i) {
    return static_cast<T*>(vdata)[i];
  }

  const T& operator()(const uint &i) const {
    return static_cast<const T*>(vdata)[i];
  }

  Image<T>& operator=(const Image<T> &other) {
    if (vdata == NULL) {
      dim = other.dim;
      vdata = malloc(dim.width * dim.height * sizeof(T));
    }
    memcpy(vdata, other.vdata, dim.width * dim.height * sizeof(T));
    return *this;
  }

  T* data() {
    return static_cast<T *>(vdata);
  }

  const T* data() const {
    return static_cast<const T *>(vdata);
  }

  void SetZero() {
    memset(vdata, 0, dim.width * dim.height * sizeof(T));
  }

  void set(T val) {
    for (uint i = 0; i < dim.width * dim.height; ++i) {
        data()[i] = val;
    }
  }

  void Free() {
    if (vdata)
      free(vdata);
    vdata = NULL;
  }

private:
  Size2 dim;
  void *vdata;
};

class ImageIO {
public:
	void loadColor(std::string file, Image<Color3b> &img);
	void loadDepth(std::string file, Image<ushort> &img);
	void loadMask(std::string file, Image<bool> &img);
	void saveColor(std::string file, const Image<Color3b> &img);
	void saveDepth(std::string file, const Image<ushort> &img);
	void saveIntensityGrey(std::string file, const Image<float> &img);
	void saveMask(std::string file, const Image<bool> &img);
};

END_INCREON_SPACE
