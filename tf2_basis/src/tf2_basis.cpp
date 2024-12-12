#include <tf2_basis/tf2_basis.h>
#include <chrono>
#include <google/protobuf/util/time_util.h>

namespace tf2_basis {
  // TODO: work out of tf2 template functions can handle some of these conversions

  tf2::TimePoint fromBasis(const basis::core::MonotonicTime & time) {
    return tf2::TimePoint(std::chrono::nanoseconds(time.nsecs));
  };

  basis::core::MonotonicTime toBasis(const tf2::TimePoint & time) {
    return basis::core::MonotonicTime::FromNanoseconds(time.time_since_epoch().count());
  };

  foxglove::FrameTransform toFoxglove(tf2::Vector3 origin, tf2::Quaternion rotation, tf2::TimePoint time, const std::string& parent_frame_id, const std::string& child_frame_id) {
    foxglove::FrameTransform out;

    *out.mutable_timestamp() = google::protobuf::util::TimeUtil::NanosecondsToTimestamp(time.time_since_epoch().count());
    out.set_parent_frame_id(parent_frame_id);
    out.set_child_frame_id(child_frame_id);

    const auto& translation_out = out.mutable_translation();
    translation_out->set_x(origin.getX());
    translation_out->set_y(origin.getY());
    translation_out->set_z(origin.getZ());
    
    const auto& rotation_out = out.mutable_rotation();
    rotation_out->set_x(rotation.getX());
    rotation_out->set_y(rotation.getY());
    rotation_out->set_z(rotation.getZ());
    rotation_out->set_w(rotation.getW());

    return out;
  };

  void fromFoxglove(const foxglove::FrameTransform& in, tf2::Vector3 & origin_out, tf2::Quaternion & rotation_out, tf2::TimePoint & time_out, std::string& frame_id_out) {
    time_out = tf2::TimePoint(std::chrono::nanoseconds(google::protobuf::util::TimeUtil::TimestampToNanoseconds(in.timestamp())));
    frame_id_out = in.parent_frame_id();
    origin_out = {
      in.translation().x(), in.translation().y(), in.translation().z()
    };
    rotation_out = {
      in.rotation().x(), in.rotation().y(), in.rotation().z(), in.rotation().w()
    };
  };

  foxglove::FrameTransform
  Buffer::lookupTransform(
    const std::string & target_frame,
    const std::string & source_frame,
    const basis::core::MonotonicTime & time) const {
      tf2::Vector3 origin;
      tf2::Quaternion rotation;
      tf2::TimePoint new_time;
      lookupTransformTf2(
            target_frame,
            source_frame, fromBasis(time), origin, rotation, new_time);

      foxglove::FrameTransform msg = toFoxglove(origin, rotation, new_time, target_frame, source_frame);

      return msg;
  }

  /** \brief Add transform information to the tf data structure
   * \param transform The transform to store
   * \param authority The source of the information for this transform
   * \param is_static Record this transform as a static transform.  It will be good across all time.  (This cannot be changed after the first call.)
   * \return True unless an error occured
   */
  TF2_PUBLIC
  bool Buffer::setTransform(
    const foxglove::FrameTransform & transform,
    const std::string & authority, bool is_static) {
      tf2::Vector3 origin;
      tf2::Quaternion rotation;
      tf2::TimePoint time;
      std::string frame_id;
      fromFoxglove(transform, origin, rotation, time, frame_id);
      return setTransformTf2(origin, rotation, frame_id, transform.child_frame_id(), time, authority, is_static);
    }
}