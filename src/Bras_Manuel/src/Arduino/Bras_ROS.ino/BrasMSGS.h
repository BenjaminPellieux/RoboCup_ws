#ifndef _ROS_Bras_Vers_Arduino_msgs_BrasMSGS_h
#define _ROS_Bras_Vers_Arduino_msgs_BrasMSGS_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace Bras_Vers_Arduino_msgs
{

  class BrasMSGS : public ros::Msg
  {
    public:
      uint32_t pos_length;
      typedef int32_t _pos_type;
      _pos_type st_pos;
      _pos_type * pos;
      uint32_t vit_length;
      typedef int32_t _vit_type;
      _vit_type st_vit;
      _vit_type * vit;

    BrasMSGS():
      pos_length(0), pos(NULL),
      vit_length(0), vit(NULL)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->pos_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->pos_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->pos_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->pos_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->pos_length);
      for( uint32_t i = 0; i < pos_length; i++){
      union {
        int32_t real;
        uint32_t base;
      } u_posi;
      u_posi.real = this->pos[i];
      *(outbuffer + offset + 0) = (u_posi.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_posi.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_posi.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_posi.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->pos[i]);
      }
      *(outbuffer + offset + 0) = (this->vit_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->vit_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->vit_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->vit_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->vit_length);
      for( uint32_t i = 0; i < vit_length; i++){
      union {
        int32_t real;
        uint32_t base;
      } u_viti;
      u_viti.real = this->vit[i];
      *(outbuffer + offset + 0) = (u_viti.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_viti.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_viti.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_viti.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->vit[i]);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      uint32_t pos_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      pos_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      pos_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      pos_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->pos_length);
      if(pos_lengthT > pos_length)
        this->pos = (int32_t*)realloc(this->pos, pos_lengthT * sizeof(int32_t));
      pos_length = pos_lengthT;
      for( uint32_t i = 0; i < pos_length; i++){
      union {
        int32_t real;
        uint32_t base;
      } u_st_pos;
      u_st_pos.base = 0;
      u_st_pos.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_st_pos.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_st_pos.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_st_pos.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->st_pos = u_st_pos.real;
      offset += sizeof(this->st_pos);
        memcpy( &(this->pos[i]), &(this->st_pos), sizeof(int32_t));
      }
      uint32_t vit_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      vit_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      vit_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      vit_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->vit_length);
      if(vit_lengthT > vit_length)
        this->vit = (int32_t*)realloc(this->vit, vit_lengthT * sizeof(int32_t));
      vit_length = vit_lengthT;
      for( uint32_t i = 0; i < vit_length; i++){
      union {
        int32_t real;
        uint32_t base;
      } u_st_vit;
      u_st_vit.base = 0;
      u_st_vit.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_st_vit.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_st_vit.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_st_vit.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->st_vit = u_st_vit.real;
      offset += sizeof(this->st_vit);
        memcpy( &(this->vit[i]), &(this->st_vit), sizeof(int32_t));
      }
     return offset;
    }

    const char * getType(){ return "Bras_Vers_Arduino_msgs/BrasMSGS"; };
    const char * getMD5(){ return "795df452dd7e88ff6cccda00bb341d7b"; };

  };

}
#endif
