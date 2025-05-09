// Generated by gencpp from file pick_place_msgs/PickStep.msg
// DO NOT EDIT!


#ifndef PICK_PLACE_MSGS_MESSAGE_PICKSTEP_H
#define PICK_PLACE_MSGS_MESSAGE_PICKSTEP_H

#include <ros/service_traits.h>


#include <pick_place_msgs/PickStepRequest.h>
#include <pick_place_msgs/PickStepResponse.h>


namespace pick_place_msgs
{

struct PickStep
{

typedef PickStepRequest Request;
typedef PickStepResponse Response;
Request request;
Response response;

typedef Request RequestType;
typedef Response ResponseType;

}; // struct PickStep
} // namespace pick_place_msgs


namespace ros
{
namespace service_traits
{


template<>
struct MD5Sum< ::pick_place_msgs::PickStep > {
  static const char* value()
  {
    return "efb821ff9d5a3f3a02dcfb089da0dd4e";
  }

  static const char* value(const ::pick_place_msgs::PickStep&) { return value(); }
};

template<>
struct DataType< ::pick_place_msgs::PickStep > {
  static const char* value()
  {
    return "pick_place_msgs/PickStep";
  }

  static const char* value(const ::pick_place_msgs::PickStep&) { return value(); }
};


// service_traits::MD5Sum< ::pick_place_msgs::PickStepRequest> should match
// service_traits::MD5Sum< ::pick_place_msgs::PickStep >
template<>
struct MD5Sum< ::pick_place_msgs::PickStepRequest>
{
  static const char* value()
  {
    return MD5Sum< ::pick_place_msgs::PickStep >::value();
  }
  static const char* value(const ::pick_place_msgs::PickStepRequest&)
  {
    return value();
  }
};

// service_traits::DataType< ::pick_place_msgs::PickStepRequest> should match
// service_traits::DataType< ::pick_place_msgs::PickStep >
template<>
struct DataType< ::pick_place_msgs::PickStepRequest>
{
  static const char* value()
  {
    return DataType< ::pick_place_msgs::PickStep >::value();
  }
  static const char* value(const ::pick_place_msgs::PickStepRequest&)
  {
    return value();
  }
};

// service_traits::MD5Sum< ::pick_place_msgs::PickStepResponse> should match
// service_traits::MD5Sum< ::pick_place_msgs::PickStep >
template<>
struct MD5Sum< ::pick_place_msgs::PickStepResponse>
{
  static const char* value()
  {
    return MD5Sum< ::pick_place_msgs::PickStep >::value();
  }
  static const char* value(const ::pick_place_msgs::PickStepResponse&)
  {
    return value();
  }
};

// service_traits::DataType< ::pick_place_msgs::PickStepResponse> should match
// service_traits::DataType< ::pick_place_msgs::PickStep >
template<>
struct DataType< ::pick_place_msgs::PickStepResponse>
{
  static const char* value()
  {
    return DataType< ::pick_place_msgs::PickStep >::value();
  }
  static const char* value(const ::pick_place_msgs::PickStepResponse&)
  {
    return value();
  }
};

} // namespace service_traits
} // namespace ros

#endif // PICK_PLACE_MSGS_MESSAGE_PICKSTEP_H
