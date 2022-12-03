#ifndef TAGSH
#define TAGSH

#include "apriltag.h"
#include "apriltag/tag16h5.h"
#include <mutex>

class TagDetector{
public:
    TagDetector();
    ~TagDetector();
    void getTags(zarray_t& detections);
    void giveFrame(image_u8_t& im);
    void detectTags();
private:
    std::mutex frame_lock;
    std::mutex data_lock;

    image_u8_t* im;
    // tag detector
    apriltag_detector_t* td;
    // tag family
    apriltag_family_t* tf;
    zarray_t* detections;
};

#endif // TAGSH