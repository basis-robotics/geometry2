#pragma once

#include <basis/core/time.h>
#include <foxglove/FrameTransform.pb.h>
#include <tf2/buffer_core.h>

namespace tf2_basis
{
tf2::TimePoint fromBasis(const basis::core::MonotonicTime & time);
basis::core::MonotonicTime toBasis(const tf2::TimePoint & time);
foxglove::FrameTransform toFoxglove(const tf2::Stamped<tf2::Transform>& in, const std::string& child_frame_id);

class Buffer : public tf2::BufferCore {
public:
/**
   * \brief Get the transform between two frames by frame ID.
   * \param target_frame The frame to which data should be transformed.
   * \param source_frame The frame where the data originated.
   * \param time The time at which the value of the transform is desired (0 will get the latest).
   * \return The transform between the frames as a ROS type.
   */
  TF2_PUBLIC
  foxglove::FrameTransform
  lookupTransform(
    const std::string & target_frame,
    const std::string & source_frame,
    const basis::core::MonotonicTime & time) const;


  /** \brief Add transform information to the tf data structure
   * \param transform The transform to store
   * \param authority The source of the information for this transform
   * \param is_static Record this transform as a static transform.  It will be good across all time.  (This cannot be changed after the first call.)
   * \return True unless an error occured
   */
  TF2_PUBLIC
  bool setTransform(
    const foxglove::FrameTransform & transform,
    const std::string & authority, bool is_static = false);

#if BASIS_TF2_LOOKUP_VELOCITY
  TF2_PUBLIC
  geometry_msgs::msg::VelocityStamped lookupVelocity(
    const std::string & tracking_frame, const std::string & observation_frame,
    const basis::core::MonotonicTime & time, const tf2::Duration & averaging_interval) const = 0;

  /** \brief Lookup the velocity of the moving_frame in the reference_frame
   * \param reference_frame The frame in which to track
   * \param moving_frame The frame to track
   * \param time The time at which to get the velocity
   * \param duration The period over which to average
   * \param velocity The velocity output as a ROS type
   *
   * Possible exceptions TransformReference::LookupException, TransformReference::ConnectivityException,
   * TransformReference::MaxDepthException
   */
  TF2_PUBLIC
  geometry_msgs::msg::VelocityStamped lookupVelocity(
    const std::string & tracking_frame, const std::string & observation_frame,
    const std::string & reference_frame, const tf2::Vector3 & reference_point,
    const std::string & reference_point_frame,
    const basis::core::MonotonicTime & time, const tf2::Duration & duration) const = 0;
#endif
};

}