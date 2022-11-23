#include "tags.hpp"

TagDetector::TagDetector(){
    td = apriltag_detector_create();
    tf = tag16h5_create();
    apriltag_detector_add_family(td, tf);
    zarray_t *detections = apriltag_detector_detect(td, im);
}
image_u8_t* im = image_u8_create_from_pnm("test.png");

for (int i = 0; i < zarray_size(detections); i++) {
    apriltag_detection_t *det;
    zarray_get(detections, i, &det);

    // Do stuff with detections here.
}

TagDetector::~TagDetector(){
    // Cleanup.
    tagStandard41h12_destroy(tf);
    apriltag_detector_destroy(td);
}