#pragma once

#include "increon.h"
#include "image.h"
#include "util.h"

#include <boost/filesystem.hpp>

namespace fs = boost::filesystem;

BEGIN_INCREON_SPACE

struct Frame {
    int index;
    ulong timestamp;
};

struct RawDepthFrame : Frame {
    Image<ushort> buffer;
};

struct RawColorFrame : Frame {
    Image<Color3b> buffer;
};

struct DepthFrame : Frame {
    Image<float> buffer;

    void copy(RawDepthFrame &raw) {
        if (buffer.height() != raw.buffer.height() ||
            buffer.width() != raw.buffer.width()) {
            buffer.alloc(raw.buffer.width(), raw.buffer.height());
        }

        for (uint i = 0; i < buffer.height() * buffer.width(); ++i) {
            buffer[i] = raw.buffer[i] / 1000.0f;
        }

        index = raw.index;
        timestamp = raw.timestamp;
    }
};

struct ColorFrame : Frame {
    Image<Eigen::Vector3f> buffer;
};

template <typename Frame>
class Stream {

};

class DepthStreamListener {
public:
	virtual void onDepthStreamStart() {}
	virtual void onDepthFrame(DepthFrame &frame) {}
	virtual void onDepthStreamEnd() {}
};

class ColorStreamListener {
public:
    virtual void onColorFrame(ColorFrame &frame) {}
};

class DepthStream : public Stream<DepthFrame> {
public:
    void registerDepthListener(DepthStreamListener *listener) {
        listeners.push_back(listener);
    }

    void notifyDepthFrame(DepthFrame &frame) {
        for (uint i = 0; i < listeners.size(); ++i) {
            listeners[i]->onDepthFrame(frame);
        }
    }

	void notifyDepthStreamStart() {
		for (uint i = 0; i < listeners.size(); ++i) {
			listeners[i]->onDepthStreamStart();
		}
	}

	void notifyDepthStreamEnd() {
		for (uint i = 0; i < listeners.size(); ++i) {
			listeners[i]->onDepthStreamEnd();
		}
	}

protected:
    std::vector<DepthStreamListener *> listeners;
};

class ColorStream : public Stream<ColorFrame> {
public:
    void registerColorListener(ColorStreamListener *listener) {
        listeners.push_back(listener);
    }

    void notifyColorFrame(ColorFrame &frame) {
        for (uint i = 0; i < listeners.size(); ++i) {
            listeners[i]->onColorFrame(frame);
        }
    }

protected:
    std::vector<ColorStreamListener *> listeners;
};

class RGBDStream : public DepthStream, public ColorStream {
public:
    virtual void start() = 0;
};

class ImageFileStream : public RGBDStream
{
public:
    ImageFileStream() {
    }

	bool openFolder(std::string folder) {
        // home folder that contains a depth and image subfolder
        getFileList(folder + "/raw_depth", depth_list);
		if (depth_list.size() <= 0) 
			getFileList(folder + "/depth", depth_list);

        //getFileList(folder + "/image", image_list);
        if (depth_list.size() > 0)
            return true;
        return false;
    }

    virtual void start() {
        if (depth_list.size() == 0) return;

		this->notifyDepthStreamStart();

        ImageIO iio;

        cur_index = 0;
        iio.loadDepth(depth_list[cur_index], raw_depth_frame.buffer);
        depth_width = raw_depth_frame.buffer.width();
        depth_height = raw_depth_frame.buffer.height();
        float_depth_frame.buffer.alloc(depth_width, depth_height);

        raw_depth_frame.index = cur_index;
        raw_depth_frame.timestamp = cur_index * 33000;        // default timestamp
        float_depth_frame.copy(raw_depth_frame);
        this->notifyDepthFrame(float_depth_frame);

        for (cur_index = 1; cur_index < depth_list.size(); ++cur_index) {
            iio.loadDepth(depth_list[cur_index], raw_depth_frame.buffer);
            raw_depth_frame.index = cur_index;
            raw_depth_frame.timestamp = cur_index * 33000;
            float_depth_frame.copy(raw_depth_frame);
            this->notifyDepthFrame(float_depth_frame);
        }

		this->notifyDepthStreamEnd();
    }

private:
    static void getFileList(std::string folder, std::vector<std::string> &list) {
        fs::path p(folder);
		if (fs::exists(p) == false) return;

        list.clear();
		fs::directory_iterator end_itr;
		for (fs::directory_iterator itr(p); itr != end_itr; ++itr)
        {
             if (is_regular_file(itr->path())) {
                list.push_back(itr->path().string());
            }
        }
		std::sort(list.begin(), list.end());
    }

private:
    std::vector<std::string> image_list;
    std::vector<std::string> depth_list;
    int cur_index;

    RawDepthFrame raw_depth_frame;
    DepthFrame float_depth_frame;

    int depth_width, depth_height, color_width, color_height;
};

///////////////////////////////////////////////////////////////////////////////

/**
 * A set of frames that is not necessarily continuous.
 * When loop closure is discovered, all similar frames can be merged into a fragment.
 */
struct FrameRange {
    int start, end;         // [start, end]
};

struct FrameSet {
	std::vector<FrameRange> ranges;

    void add(FrameRange &range) {
        ranges.push_back(range);
    }

    void clear() {
        ranges.clear();
    }

    int count() {
        int c = 0;
        for (int i = 0; i < ranges.size(); ++i) {
            FrameRange &r = ranges[i];
            c += r.end - r.start + 1;
        }
        return c;
    }
};

class FrameSetIterator {
public:
    FrameSetIterator(FrameSet &set) : set(set) {
        reset();
    };

    void reset() {
        r = 0;
        f = set.ranges[r].start - 1;
    }

    bool hasNext() {
        return r < set.ranges.size();
    }

    int getNextIndex() {
        if (! hasNext()) return -1;

        f++;
        if (f <= set.ranges[r].end) return f;

        r++;
        if (r >= set.ranges.size()) return -1;

        f = set.ranges[r].start;
        return f;
    }

private:
    int r, f;
    FrameSet &set;
};

END_INCREON_SPACE
