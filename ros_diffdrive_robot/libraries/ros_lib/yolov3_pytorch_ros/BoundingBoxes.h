#ifndef _ROS_yolov3_pytorch_ros_BoundingBoxes_h
#define _ROS_yolov3_pytorch_ros_BoundingBoxes_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"
#include "yolov3_pytorch_ros/BoundingBox.h"

namespace yolov3_pytorch_ros
{

  class BoundingBoxes : public ros::Msg
  {
    public:
      typedef std_msgs::Header _header_type;
      _header_type header;
      typedef std_msgs::Header _image_header_type;
      _image_header_type image_header;
      uint32_t bounding_boxes_length;
      typedef yolov3_pytorch_ros::BoundingBox _bounding_boxes_type;
      _bounding_boxes_type st_bounding_boxes;
      _bounding_boxes_type * bounding_boxes;

    BoundingBoxes():
      header(),
      image_header(),
      bounding_boxes_length(0), st_bounding_boxes(), bounding_boxes(nullptr)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      offset += this->image_header.serialize(outbuffer + offset);
      *(outbuffer + offset + 0) = (this->bounding_boxes_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->bounding_boxes_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->bounding_boxes_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->bounding_boxes_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->bounding_boxes_length);
      for( uint32_t i = 0; i < bounding_boxes_length; i++){
      offset += this->bounding_boxes[i].serialize(outbuffer + offset);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      offset += this->image_header.deserialize(inbuffer + offset);
      uint32_t bounding_boxes_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      bounding_boxes_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      bounding_boxes_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      bounding_boxes_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->bounding_boxes_length);
      if(bounding_boxes_lengthT > bounding_boxes_length)
        this->bounding_boxes = (yolov3_pytorch_ros::BoundingBox*)realloc(this->bounding_boxes, bounding_boxes_lengthT * sizeof(yolov3_pytorch_ros::BoundingBox));
      bounding_boxes_length = bounding_boxes_lengthT;
      for( uint32_t i = 0; i < bounding_boxes_length; i++){
      offset += this->st_bounding_boxes.deserialize(inbuffer + offset);
        memcpy( &(this->bounding_boxes[i]), &(this->st_bounding_boxes), sizeof(yolov3_pytorch_ros::BoundingBox));
      }
     return offset;
    }

    virtual const char * getType() override { return "yolov3_pytorch_ros/BoundingBoxes"; };
    virtual const char * getMD5() override { return "c03e7499c2e5b938e301fea76459b092"; };

  };

}
#endif
