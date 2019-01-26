#ifndef CLEAN_CAMERA_MODEL_H
#define CLEAN_CAMERA_MODEL_H

#include <image_geometry/pinhole_camera_model.h>

//NOTE: originally used in egoyclindrical package; should be moved somewhere common
//Inherits from PinholeCamerModel in order to access protected member function initRectificationMaps
        class CleanCameraModel : public image_geometry::PinholeCameraModel
        {
        public:
            void init()
            {
                //Some of the camera model's functions don't work unless this has been called first
                PinholeCameraModel::initRectificationMaps();
            }
        };

#endif
