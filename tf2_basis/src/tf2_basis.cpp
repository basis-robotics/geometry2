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

  foxglove::FrameTransform toFoxglove(const tf2::Stamped<tf2::Transform>& in, const std::string& child_frame_id) {
    foxglove::FrameTransform out;

    *out.mutable_timestamp() = google::protobuf::util::TimeUtil::NanosecondsToTimestamp(in.stamp_.time_since_epoch().count());
    out.set_parent_frame_id(in.frame_id_);
    out.set_child_frame_id(child_frame_id);

    const auto& origin = in.getOrigin();
    const auto& translation_out = out.mutable_translation();
    translation_out->set_x(origin.getX());
    translation_out->set_y(origin.getY());
    translation_out->set_z(origin.getZ());
    
    const auto& rotation = in.getRotation();
    const auto& rotation_out = out.mutable_rotation();
    rotation_out->set_x(rotation.getX());
    rotation_out->set_y(rotation.getY());
    rotation_out->set_z(rotation.getZ());
    rotation_out->set_w(rotation.getW());

    return out;
  };

  tf2::Stamped<tf2::Transform> fromFoxglove(const foxglove::FrameTransform& in) {
    tf2::Stamped<tf2::Transform> out;

    out.stamp_ = tf2::TimePoint(std::chrono::nanoseconds(google::protobuf::util::TimeUtil::TimestampToNanoseconds(in.timestamp())));
    out.frame_id_ = in.parent_frame_id();
    out.setOrigin({
      in.translation().x(), in.translation().y(), in.translation().z()
    });
    out.setRotation({
      in.rotation().x(), in.rotation().y(), in.rotation().z(), in.rotation().w()
    });

    return out;
  };

  foxglove::FrameTransform
  Buffer::lookupTransform(
    const std::string & target_frame,
    const std::string & source_frame,
    const basis::core::MonotonicTime & time) const {
        const tf2::Stamped<tf2::Transform> stamped_transform = lookupTransformTf2(
            target_frame,
            source_frame, fromBasis(time));

        foxglove::FrameTransform msg = toFoxglove(stamped_transform, source_frame);

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
      auto tf2_transform = fromFoxglove(transform);
      return setTransformTf2(tf2_transform, tf2_transform.frame_id_, transform.child_frame_id(), tf2_transform.stamp_, authority, is_static);
    }
}