/*
 * Copyright (c) 2012, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

/* 
 * Author: Chad Rockey
 */

#ifndef DEPTH_IMAGE_TO_LASERSCAN
#define DEPTH_IMAGE_TO_LASERSCAN

#include <sensor_msgs/Image.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/image_encodings.h>
#include <image_geometry/pinhole_camera_model.h>
#include <depthimage_to_laserscan/depth_traits.h>
#include <sstream>
//#include <limits.h>
//#include <math.h>
#include <cmath>
//#include <algorithm>
#include <depthimage_to_laserscan/clean_camera_model.h>
#include <boost/make_shared.hpp>

#include <ros/ros.h>



namespace depthimage_to_laserscan
{ 
  
  struct ConversionCache
  {
    float floor_dist, overhead_dist;
    
    sensor_msgs::ImageConstPtr limits;
  };
  
  class DepthImageToLaserScan
  {
  public:
    DepthImageToLaserScan();
    ~DepthImageToLaserScan();

    /**
     * Converts the information in a depth image (sensor_msgs::Image) to a sensor_msgs::LaserScan.
     * 
     * This function converts the information in the depth encoded image (UInt16 or Float32 encoding) into
     * a sensor_msgs::LaserScan as accurately as possible.  To do this, it requires the synchornized Image/CameraInfo
     * pair associated with the image.
     * 
     * @param depth_msg UInt16 or Float32 encoded depth image.
     * @param info_msg CameraInfo associated with depth_msg
     * @return sensor_msgs::LaserScanPtr for the center row(s) of the depth image.
     * 
     */
    sensor_msgs::LaserScanPtr convert_msg(const sensor_msgs::ImageConstPtr& depth_msg,
                                          const sensor_msgs::CameraInfoConstPtr& info_msg, int approach, sensor_msgs::ImageConstPtr& image);
    
    /**
     * Sets the scan time parameter.
     * 
     * This function stores the desired value for scan_time.  In sensor_msgs::LaserScan, scan_time is defined as 
     * "time between scans [seconds]".  This value is not easily calculated from consquetive messages, and is thus
     * left to the user to set correctly.
     * 
     * @param scan_time The value to use for outgoing sensor_msgs::LaserScan.
     * 
     */
    void set_scan_time(const float scan_time);
    
    /**
     * Sets the minimum and maximum range for the sensor_msgs::LaserScan.
     * 
     * range_min is used to determine how close of a value to allow through when multiple radii correspond to the same
     * angular increment.  range_max is used to set the output message.
     * 
     * @param range_min Minimum range to assign points to the laserscan, also minimum range to use points in the output scan.
     * @param range_max Maximum range to use points in the output scan.
     * 
     */
    void set_range_limits(const float range_min, const float range_max);
    
    /**
     * Sets the number of image rows to use in the output LaserScan.
     * 
     * scan_height is the number of rows (pixels) to use in the output.  This will provide scan_height number of radii for each
     * angular increment.  The output scan will output the closest radius that is still not smaller than range_min.  This function
     * can be used to vertically compress obstacles into a single LaserScan.
     * 
     * @param scan_height Number of pixels centered around the center of the image to compress into the LaserScan.
     * 
     */
    void set_scan_height(const int scan_height);
    
    /**
     * Sets the frame_id for the output LaserScan.
     * 
     * Output frame_id for the LaserScan.  Will probably NOT be the same frame_id as the depth image.
     * Example: For OpenNI cameras, this should be set to 'camera_depth_frame' while the camera uses 'camera_depth_optical_frame'.
     * 
     * @param output_frame_id Frame_id to use for the output sensor_msgs::LaserScan.
     * 
     */
    void set_output_frame(const std::string output_frame_id);
    
    void set_filtering_limits(const float floor_dist, const float overhead_dist);
    

  private:
    /**
     * Computes euclidean length of a cv::Point3d (as a ray from origin)
     * 
     * This function computes the length of a cv::Point3d assumed to be a vector starting at the origin (0,0,0).
     * 
     * @param ray The ray for which the magnitude is desired.
     * @return Returns the magnitude of the ray.
     * 
     */
    double magnitude_of_ray(const cv::Point3d& ray) const;

    /**
     * Computes the angle between two cv::Point3d
     * 
     * Computes the angle of two cv::Point3d assumed to be vectors starting at the origin (0,0,0).
     * Uses the following equation: angle = arccos(a*b/(|a||b|)) where a = ray1 and b = ray2.
     * 
     * @param ray1 The first ray
     * @param ray2 The second ray
     * @return The angle between the two rays (in radians)
     * 
     */
    double angle_between_rays(const cv::Point3d& ray1, const cv::Point3d& ray2) const;
    
    /**
     * Determines whether or not new_value should replace old_value in the LaserScan.
     * 
     * Uses the values of range_min, and range_max to determine if new_value is a valid point.  Then it determines if
     * new_value is 'more ideal' (currently shorter range) than old_value.
     * 
     * @param new_value The current calculated range.
     * @param old_value The current range in the output LaserScan.
     * @param range_min The minimum acceptable range for the output LaserScan.
     * @param range_max The maximum acceptable range for the output LaserScan.
     * @return If true, insert new_value into the output LaserScan.
     * 
     */
    bool use_point_old(const float new_value, const float old_value, const float range_min, const float range_max) const;
    bool use_point_new(const float new_value, const float old_value, const float range_min, const float range_max) const;
    
    void updateCache(const sensor_msgs::ImageConstPtr& depth_msg, const sensor_msgs::CameraInfoConstPtr& info_msg);
    
    template <typename T>
    void update_limits(const sensor_msgs::ImageConstPtr& depth_msg)
    {
      sensor_msgs::ImagePtr new_msg_ptr = boost::make_shared<sensor_msgs::Image>();
      
      sensor_msgs::Image &new_msg = *new_msg_ptr;
      new_msg.header = depth_msg->header;
      new_msg.height = depth_msg->height;
      new_msg.width = depth_msg->width;
      new_msg.encoding = depth_msg->encoding;
      new_msg.is_bigendian = false; //image->is_bigendian;
      new_msg.step = depth_msg->step;
      size_t size = new_msg.step * new_msg.height;
      new_msg.data.resize(size);
      
      T* send_data = (T*)new_msg.data.data(); 
      
      float unit_scaling=depthimage_to_laserscan::DepthTraits<T>::toMeters( T(1) );
      
      
      //NOTE: This really only needs to be checked (and therefore generated) up to the horizon (assuming level camera).
      for(int v=0,i=0; v< depth_msg->height; ++v)
      {
        //TODO: These values will be the same along a row, so no need to compute an entire image worth.
        for(int u=0; u<depth_msg->width; ++u,++i)
        {
          cv::Point2d pt;
          pt.x = u;
          pt.y = v;
          
          cv::Point3f world_pnt = cam_model_.projectPixelTo3dRay(pt);
          float ratio;
          if(world_pnt.y>0)
          {
            ratio=floor_dist_/world_pnt.y;
          }
          else
          {
            ratio=-overhead_dist_/world_pnt.y;
          }
          float z = world_pnt.z*ratio*unit_scaling; //NOTE: world_pnt.z is always 1, and can precompute unit_scaling/d_floor;
          //TODO: add check for out of range number for 16U, convert to 0?
          
          send_data[i] = z;
        }
      }
      cache_.limits = (sensor_msgs::ImageConstPtr)new_msg_ptr;
    }
    
    /**
    * Converts the depth image to a laserscan using the DepthTraits to assist.
    * 
    * This uses a method to inverse project each pixel into a LaserScan angular increment.  This method first projects the pixel
    * forward into Cartesian coordinates, then calculates the range and angle for this point.  When multiple points coorespond to
    * a specific angular measurement, then the shortest range is used.
    * 
    * @param depth_msg The UInt16 or Float32 encoded depth message.
    * @param cam_model The image_geometry camera model for this image.
    * @param scan_msg The output LaserScan.
    * @param scan_height The number of vertical pixels to feed into each angular_measurement.
    * 
    */
    template<typename T>
    void convert_old(const sensor_msgs::ImageConstPtr& depth_msg, const image_geometry::PinholeCameraModel& cam_model, 
		 const sensor_msgs::LaserScanPtr& scan_msg, const int& scan_height) const
    {
      // Use correct principal point from calibration
      float center_x = cam_model.cx();
      float center_y = cam_model.cy();

      // Combine unit conversion (if necessary) with scaling by focal length for computing (X,Y)
      double unit_scaling = depthimage_to_laserscan::DepthTraits<T>::toMeters( T(1) );
      float constant_x = unit_scaling / cam_model.fx();
      float constant_y = unit_scaling / cam_model.fy();
      
      const T* depth_row = reinterpret_cast<const T*>(&depth_msg->data[0]);
      int row_step = depth_msg->step / sizeof(T);

      int offset = (int)(cam_model.cy()-scan_height/2);
      depth_row += offset*row_step; // Offset to center of image

      for(int v = offset; v < offset+scan_height_; v++, depth_row += row_step)
      {
        for (int u = 0; u < (int)depth_msg->width; u++) // Loop over each pixel in row
        {	
          T depth = depth_row[u];
          
          double r = depth; // Assign to pass through NaNs and Infs
          double th = -atan2((double)(u - center_x) * constant_x, unit_scaling); // Atan2(x, z), but depth divides out
          int index = (th - scan_msg->angle_min) / scan_msg->angle_increment;
          
          if (depthimage_to_laserscan::DepthTraits<T>::valid(depth)){ // Not NaN or Inf
            // Calculate in XYZ
            double x = (u - center_x) * depth * constant_x;
            double z = depthimage_to_laserscan::DepthTraits<T>::toMeters(depth);
            
            // Calculate actual distance
            r = sqrt(pow(x, 2.0) + pow(z, 2.0));
          }
        
          // Determine if this point should be used.
          if(use_point_old(r, scan_msg->ranges[index], scan_msg->range_min, scan_msg->range_max)){
            scan_msg->ranges[index] = r;
          }
        }
      }
    }
    
    //We don't distinguish between infs and Nans
    template<typename T>
    void __attribute__((optimize ("-ffast-math"))) convert_new(const sensor_msgs::ImageConstPtr& depth_msg, const image_geometry::PinholeCameraModel& cam_model, 
                                                               const sensor_msgs::LaserScanPtr& scan_msg, const int& scan_height, const ConversionCache& cache) const
    {
      // Use correct principal point from calibration
      float center_x = cam_model.cx();
      float center_y = cam_model.cy();
      
      // Combine unit conversion (if necessary) with scaling by focal length for computing (X,Y)
      double unit_scaling = depthimage_to_laserscan::DepthTraits<T>::toMeters( T(1) );
      float constant_x = unit_scaling / cam_model.fx();
      float constant_y = unit_scaling / cam_model.fy();
      
      const T* depth_row = reinterpret_cast<const T*>(depth_msg->data.data());
      int row_step = depth_msg->step / sizeof(T); //is this the same as image width?
      
      int offset = (int)(cam_model.cy()-scan_height/2);
      depth_row += offset*row_step; // Offset to center of image
      
      const T* limits_row = reinterpret_cast<const T*>(cache.limits->data.data());
      limits_row += offset*row_step;
      
      int ranges_size = depth_msg->width;
      
      std::vector<float> range_ratios(ranges_size);
      //precomputation, only redo if camera matrix changes
      //TODO: calculate this for each angle
      float min_range = DepthTraits<T>::fromMeters(scan_msg->range_min);
      std::vector<T> min_depth_limits(ranges_size);
      
      constexpr float big_val = 1000; //std::numeric_limits<T>::max(); //std::numeric_limits<float>::quiet_NaN()
      
      for(int u = 0; u < ranges_size; ++u)
      {
        cv::Point2d pt;
        pt.x = u;
        pt.y = 0;
        
        cv::Point3f world_pnt = cam_model.projectPixelTo3dRay(pt);
        float ratio = std::sqrt(world_pnt.x*world_pnt.x + 1); //making use of the fact that z=1 and y is irrelevant
        range_ratios[u] = ratio;
        min_depth_limits[u] = min_range/ratio;
      }
      
      
      std::vector<T> min_depths;//(ranges_size, std::numeric_limits<T>::max());
      
      {
        int num_rows = scan_height_;
        const T* source=depth_row;
        const T* safe_mins=limits_row; //reinterpret_cast<const T*>(cache.limits->data.data() );
        
        {
          int region_size = num_rows*ranges_size;
          
          min_depths.resize(region_size, big_val);
          
          for(int u=0; u<region_size; ++u)
          {
            T depth = source[u];
            T safe_min = safe_mins[u];
            T filtered_depth = !(depth==depth && depth < safe_min) ? big_val : depth;
            min_depths[u] = filtered_depth;
            //ROS_INFO_STREAM("Depth: " << depth << ", safe_min: " << safe_min << ", big_val: " << big_val << ", result: " << filtered_depth);
          }

        }
        
        source=min_depths.data();
        
        while(num_rows>1)
        {
          
          {
            int region_size = num_rows*ranges_size;
            num_rows/=2;
            
            int half_size= num_rows*ranges_size;
            int remainder = region_size-half_size*2;
            
            int start,mid,end;
            
            start=0;
            mid=half_size;
            end = region_size;
            
            for(int u=start; u<half_size; ++u)
            {
              T min_val = std::fmin(source[u], source[u+half_size]);
              min_depths[u] = min_val;
              //ROS_INFO_STREAM("A: " << source[u] << ", B: " << source[u+half_size] << ", result: " << min_val);
            }
            for(int i=0;i<remainder;++i)
            {
              min_depths[i] = std::fmin(min_depths[i], source[half_size*2 +i]);
            }
            min_depths.resize(half_size); //NOTE: this is probably not necessary
          }
        }
      
      }
      
      
      for(int u = 0; u < ranges_size; ++u)
      {
        T depth = min_depths[u];
        float range = range_ratios[u]*depth;
        
        
        double r = depth; // Assign to pass through NaNs and Infs
        double th = -atan2((double)(u - center_x) * constant_x, unit_scaling); // Atan2(x, z), but depth divides out
        int index = (th - scan_msg->angle_min) / scan_msg->angle_increment;
        //ROS_INFO_STREAM("u=" << u << ", index=" << index);
        ROS_INFO_STREAM("Depth: " << depth << ", range: " << range << ", th: " << th << ", index: " << index);
        
        if(range < scan_msg->range_max)
        {
          float cur_ind_range = scan_msg->ranges[index];
          
          if(!(cur_ind_range < range))
          {
            scan_msg->ranges[index] = range;
          }
        }
      }
      
      
    }
    
    CleanCameraModel cam_model_; ///< image_geometry helper class for managing sensor_msgs/CameraInfo messages.
    ConversionCache cache_;
    
    float scan_time_; ///< Stores the time between scans.
    float range_min_; ///< Stores the current minimum range to use.
    float range_max_; ///< Stores the current maximum range to use.
    int scan_height_; ///< Number of pixel rows to use when producing a laserscan from an area.
    float floor_dist_, overhead_dist_;
    std::string output_frame_id_; ///< Output frame_id for each laserscan.  This is likely NOT the camera's frame_id.
  };
  
  
}; // depthimage_to_laserscan

#endif
