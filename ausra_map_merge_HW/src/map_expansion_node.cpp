// ============================================================================
// map_expansion_node.cpp
// AUSRA Multi-Robot Map Expansion Node  —  Heartbeat Timer Architecture
// HARDWARE VARIANT (ausra_map_merge_HW)
//
// PURPOSE:
//   Translates each robot's dynamic slam_toolbox OccupancyGrid (shifting size
//   and origin) into a fixed-size, globally-aligned canvas that
//   multirobot_map_merge can consume without crashing or misaligning.
//
// HARDWARE ADAPTATION NOTES:
//   - Default input_topic is "map" (relative; namespaced at launch time).
//   - Default output_topic is "/ausra_1/map_fixed".
//   - robot_offset_x/y represent physical tape-measured spawn offsets, not
//     Gazebo spawn coordinates.
//   - All spatial math is identical to the simulation variant.
//
// ARCHITECTURE: Decoupled Publisher / Subscriber
//
//   ┌─────────────────────────────────────────────────────────────────────┐
//   │  mapCallback  (SLAM-driven, fires whenever slam_toolbox publishes)  │
//   │  └─► Updates canvas_data_ in-place. NEVER publishes.               │
//   │                                                                     │
//   │  publishCanvas  (Timer-driven, fires at 1 Hz unconditionally)       │
//   │  └─► Publishes the current canvas_data_ state. ALWAYS fires.       │
//   └─────────────────────────────────────────────────────────────────────┘
//
// WHY THIS SOLVES THE SEGFAULT:
//   multirobot_map_merge segfaults when it boots with 0 or 1 maps because
//   its OpenCV pipeline dereferences a null matrix. By publishing a valid
//   1000×1000 canvas of Unknown (-1) cells IMMEDIATELY at startup — before
//   any SLAM data arrives — we always give the merger a legally initialised
//   array to work with. There is no longer any race between launch order
//   and the merger's memory accesses.
//
// FAULT TOLERANCE LIFECYCLE:
//
//   [Before SLAM]      Timer fires → publishes all-Unknown canvas.
//                      Merger initialises safely. No segfault.
//
//   [SLAM online]      mapCallback updates canvas with real cell data.
//                      Timer fires → publishes real map data.
//                      Merger works normally.
//
//   [Robot dies]        mapCallback stops firing. Partial-reset never runs.
//                      Last known SLAM data remains frozen in canvas_data_.
//                      Timer continues firing → merger keeps last known map.
//                      *** This preserves the desired "ghost map" behaviour. ***
//
//   [New robot joins]  Its expansion node has been publishing Unknown since
//                      launch. Once its SLAM starts, mapCallback populates
//                      real data. Merger seamlessly picks it up next tick.
//
// SPATIAL MATH (preserves existing architecture from Code Explanation doc):
//
//   slam_toolbox publishes info.origin in the robot's LOCAL SLAM frame.
//   robot_offset_x/y is the robot's GLOBAL spawn position (tape-measured).
//
//   Step 1:  global_origin = local_origin + robot_offset
//            (converts local SLAM bounds → global world coordinates)
//
//   Step 2:  canvas_offset = (global_origin - canvas_origin) / resolution
//            (converts global position → fixed canvas pixel offset)
//
//   Invariant proof (why moving floor is eliminated):
//     canvas_col = (global_origin_x / resolution) + col
//               = (local_origin_x + robot_offset_x - canvas_origin_x) / resolution + col
//               = (world_x_of_cell - canvas_origin_x) / resolution
//
//   world_x_of_cell is the physical position of a fixed wall — constant.
//   canvas_origin_x is -25.0 — constant.
//   resolution is 0.05 — constant.
//   Therefore canvas_col is constant regardless of slam_toolbox origin drift.
//
// INIT_POSE CONFIGURATION (map_merge_HW_params.yaml):
//   Because this node bakes the spawn offset into the canvas pixel positions,
//   all init_pose_* values in YAML must be 0.0 for all robots.
//   Setting them to the spawn coordinates would double-shift every pixel.
//
// PARAMETERS:
//   input_topic       — Raw SLAM map topic  (default: /map for hardware)
//   output_topic      — Fixed canvas topic  (e.g. /ausra_1/map_fixed)
//   canvas_width      — Canvas width  in cells    (default: 1000)
//   canvas_height     — Canvas height in cells    (default: 1000)
//   canvas_resolution — Metres per cell           (default: 0.05)
//   canvas_origin_x   — Canvas SW corner X in m  (default: -25.0)
//   canvas_origin_y   — Canvas SW corner Y in m  (default: -25.0)
//   robot_offset_x    — Robot spawn X in m        (default:   0.0)
//   robot_offset_y    — Robot spawn Y in m        (default:   0.0)
//   publish_rate_hz   — Heartbeat publish rate Hz (default:   1.0)
// ============================================================================

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>

#include <chrono>
#include <cmath>
#include <mutex>
#include <string>
#include <vector>


class MapExpansionNode : public rclcpp::Node
{
public:
  // ==========================================================================
  // Constructor — initialise everything BEFORE the timer starts firing
  // ==========================================================================
  MapExpansionNode()
  : Node("map_expansion_node"),
    slam_received_(false),
    overflow_count_(0),
    frame_id_("map")         // safe default; overwritten on first SLAM message
  {
    // ── Parameter declarations ──────────────────────────────────────────────
    // HARDWARE DEFAULT: input_topic is "map" (relative, namespaced at launch)
    this->declare_parameter<std::string>("input_topic",      "map");
    this->declare_parameter<std::string>("output_topic",     "map_fixed");
    this->declare_parameter<int>   ("canvas_width",           1000);
    this->declare_parameter<int>   ("canvas_height",          1000);
    this->declare_parameter<double>("canvas_resolution",      0.05);
    this->declare_parameter<double>("canvas_origin_x",       -25.0);
    this->declare_parameter<double>("canvas_origin_y",       -25.0);
    this->declare_parameter<double>("robot_offset_x",         0.0);
    this->declare_parameter<double>("robot_offset_y",         0.0);
    this->declare_parameter<double>("publish_rate_hz",        1.0);

    // ── Read parameters ─────────────────────────────────────────────────────
    input_topic_       = this->get_parameter("input_topic").as_string();
    output_topic_      = this->get_parameter("output_topic").as_string();
    canvas_width_      = this->get_parameter("canvas_width").as_int();
    canvas_height_     = this->get_parameter("canvas_height").as_int();
    canvas_resolution_ = this->get_parameter("canvas_resolution").as_double();
    canvas_origin_x_   = this->get_parameter("canvas_origin_x").as_double();
    canvas_origin_y_   = this->get_parameter("canvas_origin_y").as_double();
    robot_offset_x_    = this->get_parameter("robot_offset_x").as_double();
    robot_offset_y_    = this->get_parameter("robot_offset_y").as_double();
    const double publish_rate_hz = this->get_parameter("publish_rate_hz").as_double();

    // ── Validate canvas origin lies on a grid line ──────────────────────────
    // If canvas_origin is not an integer multiple of resolution, every cell
    // offset calculation accumulates a sub-cell floating-point error.
    auto check_grid_alignment = [&](double origin, double res, const char* name) {
      double remainder = std::fmod(std::abs(origin), res);
      if (remainder > 1e-6 && remainder < res - 1e-6) {
        RCLCPP_WARN(this->get_logger(),
          "PARAM WARNING: %s (%.5f) is not an integer multiple of "
          "canvas_resolution (%.5f). Sub-cell offset errors will accumulate "
          "over distance. Adjust canvas_origin to a grid-aligned value.",
          name, origin, res);
      }
    };
    check_grid_alignment(canvas_origin_x_, canvas_resolution_, "canvas_origin_x");
    check_grid_alignment(canvas_origin_y_, canvas_resolution_, "canvas_origin_y");

    // ── Pre-allocate canvas — ALL cells initialised to -1 (Unknown) ─────────
    // This MUST happen in the constructor, before the timer fires.
    // The heartbeat timer will publish this buffer immediately at startup,
    // giving multirobot_map_merge a legally-sized array to read even when
    // no SLAM data has arrived yet.
    const size_t canvas_size =
      static_cast<size_t>(canvas_width_) *
      static_cast<size_t>(canvas_height_);
    canvas_data_.assign(canvas_size, static_cast<int8_t>(-1));

    // Reserve index tracker for the partial-reset optimisation.
    // Worst-case capacity: the entire incoming SLAM map in one callback.
    last_written_indices_.reserve(canvas_size);

    RCLCPP_INFO(this->get_logger(),
      "MapExpansionNode initialised: [%s] → [%s] | "
      "Canvas %d×%d @ %.3f m/cell | "
      "Origin (%.1f, %.1f) | "
      "Robot spawn offset (%.2f, %.2f) | "
      "Publish rate %.1f Hz",
      input_topic_.c_str(), output_topic_.c_str(),
      canvas_width_, canvas_height_, canvas_resolution_,
      canvas_origin_x_, canvas_origin_y_,
      robot_offset_x_, robot_offset_y_,
      publish_rate_hz);

    // ── Publisher (transient_local) ─────────────────────────────────────────
    // transient_local ensures late-joining subscribers (e.g. multirobot_map_merge
    // launched after this node) receive the most recent canvas immediately.
    publisher_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>(
      output_topic_, rclcpp::QoS(1).transient_local());

    // ── Subscriber (transient_local) ────────────────────────────────────────
    // transient_local ensures we receive the last SLAM map even if slam_toolbox
    // published it before this node started.
    subscription_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
      input_topic_,
      rclcpp::QoS(1).transient_local(),
      std::bind(&MapExpansionNode::mapCallback, this, std::placeholders::_1));

    // ── Heartbeat Timer ──────────────────────────────────────────────────────
    // Created LAST so that all other members are fully initialised before
    // the first timer callback can fire.
    //
    // Converts the Hz rate to a std::chrono duration with microsecond precision
    // to avoid integer rounding for non-integer Hz rates.
    const auto period_us = std::chrono::microseconds(
      static_cast<int64_t>(1'000'000.0 / publish_rate_hz));

    heartbeat_timer_ = this->create_wall_timer(
      period_us,
      std::bind(&MapExpansionNode::publishCanvas, this));

    RCLCPP_INFO(this->get_logger(),
      "Heartbeat timer armed at %.1f Hz. "
      "Publishing all-Unknown canvas until first SLAM message arrives.",
      publish_rate_hz);
  }

private:
  // ==========================================================================
  // publishCanvas  — Timer callback (1 Hz, unconditional)
  //
  // Responsibility: publish the current canvas_data_ state.
  // Does NOT know or care whether mapCallback has ever run.
  // Does NOT modify canvas_data_.
  // ==========================================================================
  void publishCanvas()
  {
    nav_msgs::msg::OccupancyGrid output_map;

    // ── Header ────────────────────────────────────────────────────────────
    // Use current ROS time so the merged map's timestamp stays fresh.
    // frame_id_ is set from the first SLAM message. Until then it defaults
    // to "map", which is valid for an all-Unknown canvas.
    output_map.header.stamp    = this->now();
    output_map.header.frame_id = frame_id_;

    // ── Map metadata (fixed — never changes after construction) ───────────
    output_map.info.resolution    = static_cast<float>(canvas_resolution_);
    output_map.info.width         = static_cast<uint32_t>(canvas_width_);
    output_map.info.height        = static_cast<uint32_t>(canvas_height_);
    output_map.info.map_load_time = this->now();

    // Canvas origin is hard-locked. multirobot_map_merge receives the same
    // (-25.0, -25.0) value on every tick — this is what prevents the moving
    // floor: a stable reference point the merger can always trust.
    output_map.info.origin.position.x    = canvas_origin_x_;
    output_map.info.origin.position.y    = canvas_origin_y_;
    output_map.info.origin.position.z    = 0.0;
    output_map.info.origin.orientation.w = 1.0;
    output_map.info.origin.orientation.x = 0.0;
    output_map.info.origin.orientation.y = 0.0;
    output_map.info.origin.orientation.z = 0.0;

    // ── Thread-safe data copy ──────────────────────────────────────────────
    // canvas_data_ may be written by mapCallback concurrently if a
    // multi-threaded executor is used. The mutex guarantees a consistent
    // snapshot for the publish operation.
    {
      std::lock_guard<std::mutex> lock(canvas_mutex_);
      output_map.data = canvas_data_;
    }

    publisher_->publish(output_map);

    RCLCPP_DEBUG(this->get_logger(),
      "Heartbeat published: %d×%d canvas | SLAM active: %s",
      canvas_width_, canvas_height_,
      slam_received_ ? "YES" : "NO — publishing all-Unknown");
  }


  // ==========================================================================
  // mapCallback  — Subscription callback (SLAM-driven)
  //
  // Responsibility: validate incoming SLAM data and update canvas_data_.
  // Does NOT publish anything. Decoupled from the publisher entirely.
  //
  // When this callback STOPS firing (robot dies / SLAM crashes), the partial
  // reset below never triggers, and the last SLAM data stays frozen in
  // canvas_data_. The heartbeat then continues broadcasting that last-known
  // state — exactly the desired fault-tolerance behaviour.
  // ==========================================================================
  void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr incoming_map)
  {
    // ── Guard 1: resolution must match canvas ──────────────────────────────
    // A resolution mismatch causes every cell to be stamped at the wrong
    // pixel. There is no silent degradation — the merged map would be
    // geometrically wrong. Reject the frame and log clearly.
    if (std::abs(static_cast<double>(incoming_map->info.resolution)
                 - canvas_resolution_) > 1e-5)
    {
      RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
        "RESOLUTION MISMATCH: incoming=%.5f m/cell, canvas=%.5f m/cell. "
        "Update slam_toolbox 'resolution' param or canvas_resolution to match. "
        "Frame dropped.",
        static_cast<double>(incoming_map->info.resolution),
        canvas_resolution_);
      return;
    }

    // ── Guard 2: data array size must match declared width × height ────────
    // slam_toolbox can publish a partially-initialised message on startup or
    // immediately after a loop closure when width/height expand before the
    // data vector is resized. Accessing beyond data.size() is UB / segfault.
    const size_t expected =
      static_cast<size_t>(incoming_map->info.width) *
      static_cast<size_t>(incoming_map->info.height);

    if (incoming_map->data.size() != expected) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
        "MALFORMED MAP: data.size()=%zu but width(%u)×height(%u)=%zu. "
        "Frame dropped. This is normal during SLAM cold-start.",
        incoming_map->data.size(),
        incoming_map->info.width,
        incoming_map->info.height,
        expected);
      return;
    }

    // ── Spatial math: local SLAM frame → global canvas pixel ──────────────
    //
    // slam_toolbox publishes info.origin in the robot's LOCAL SLAM frame.
    // robot_offset_x/y is the robot's physical spawn position measured from
    // the global tape-measure origin.
    //
    // Step 1 — Add spawn offset to convert local bounds → global world coords.
    //   global_origin_x = incoming_origin_x + robot_offset_x_
    //
    // Step 2 — Convert global world position → canvas pixel offset.
    //   offset_x = round((global_origin_x - canvas_origin_x_) / resolution)
    //
    // Combined (proof of moving-floor fix):
    //   canvas_col = offset_x + col
    //              = (local_origin_x + robot_offset_x_ + col*res - canvas_origin_x_) / res
    //              = (world_x_of_cell - canvas_origin_x_) / res
    //
    // world_x_of_cell is the physical global X of any fixed wall — a constant.
    // canvas_origin_x_ is -25.0 — a constant.
    // resolution is 0.05 — a constant.
    // canvas_col is therefore CONSTANT regardless of how slam_toolbox shifts
    // its internal origin as the map grows. The sliding is eliminated.

    const double local_origin_x = incoming_map->info.origin.position.x;
    const double local_origin_y = incoming_map->info.origin.position.y;

    const double global_origin_x = local_origin_x + robot_offset_x_;
    const double global_origin_y = local_origin_y + robot_offset_y_;

    const int offset_x = static_cast<int>(
      std::round((global_origin_x - canvas_origin_x_) / canvas_resolution_));
    const int offset_y = static_cast<int>(
      std::round((global_origin_y - canvas_origin_y_) / canvas_resolution_));

    const int inc_w = static_cast<int>(incoming_map->info.width);
    const int inc_h = static_cast<int>(incoming_map->info.height);

    // ── Update canvas (thread-safe, partial reset) ─────────────────────────
    {
      std::lock_guard<std::mutex> lock(canvas_mutex_);

      // Partial reset: only clear pixels written in the PREVIOUS callback.
      // Cost: O(slam_map_width × slam_map_height)  ≈ 10,000–100,000 ops.
      // vs full std::fill at O(canvas_width × canvas_height) = 1,000,000 ops.
      //
      // Critical for fault tolerance: if mapCallback STOPS firing (robot dies),
      // this reset never triggers, so the last SLAM data remains frozen in the
      // canvas. The heartbeat then keeps broadcasting that last-known map.
      for (const int idx : last_written_indices_) {
        canvas_data_[idx] = static_cast<int8_t>(-1);
      }
      last_written_indices_.clear();

      // Stamp incoming SLAM cells onto globally-aligned canvas positions
      for (int row = 0; row < inc_h; ++row) {
        const int canvas_row = offset_y + row;

        // Row-level boundary check (avoids repeated col-level checks per row)
        if (canvas_row < 0 || canvas_row >= canvas_height_) {
          if ((++overflow_count_ % 500) == 1) {
            RCLCPP_WARN(this->get_logger(),
              "CANVAS OVERFLOW [row] (%d total cells dropped): "
              "canvas_row=%d is outside [0, %d). "
              "Robot is exploring beyond the canvas boundary. "
              "Increase canvas_height or shift canvas_origin_y.",
              overflow_count_, canvas_row, canvas_height_);
          }
          // Skip all columns in this row — they would all overflow anyway
          overflow_count_ += (inc_w - 1);
          continue;
        }

        for (int col = 0; col < inc_w; ++col) {
          const int canvas_col = offset_x + col;

          if (canvas_col < 0 || canvas_col >= canvas_width_) {
            if ((++overflow_count_ % 500) == 1) {
              RCLCPP_WARN(this->get_logger(),
                "CANVAS OVERFLOW [col] (%d total cells dropped): "
                "canvas_col=%d is outside [0, %d). "
                "Increase canvas_width or shift canvas_origin_x.",
                overflow_count_, canvas_col, canvas_width_);
            }
            continue;
          }

          const int inc_idx    = row * inc_w + col;
          const int canvas_idx = canvas_row * canvas_width_ + canvas_col;

          canvas_data_[canvas_idx] = incoming_map->data[inc_idx];
          last_written_indices_.push_back(canvas_idx);
        }
      }
    } // mutex released — publishCanvas() may now safely copy canvas_data_

    // ── First-message bookkeeping ──────────────────────────────────────────
    // Record the SLAM frame_id once. It stays stable for the node's lifetime.
    // The heartbeat uses this to set the published map's header.frame_id.
    if (!slam_received_) {
      frame_id_     = incoming_map->header.frame_id;
      slam_received_ = true;
      RCLCPP_INFO(this->get_logger(),
        "First SLAM map received from frame '%s' at offset (%d, %d) px. "
        "Canvas now carries real map data.",
        frame_id_.c_str(), offset_x, offset_y);
    }

    RCLCPP_DEBUG(this->get_logger(),
      "Canvas updated: incoming %d×%d | canvas offset (%d, %d) | "
      "local_origin (%.3f, %.3f) | global_origin (%.3f, %.3f)",
      inc_w, inc_h, offset_x, offset_y,
      local_origin_x, local_origin_y,
      global_origin_x, global_origin_y);
  }


  // ==========================================================================
  // Member variables
  // ==========================================================================

  // ── Parameters ─────────────────────────────────────────────────────────────
  std::string input_topic_;
  std::string output_topic_;
  int         canvas_width_;
  int         canvas_height_;
  double      canvas_resolution_;
  double      canvas_origin_x_;
  double      canvas_origin_y_;
  double      robot_offset_x_;    ///< Robot's physical spawn X (tape-measured)
  double      robot_offset_y_;    ///< Robot's physical spawn Y (tape-measured)

  // ── Canvas state ───────────────────────────────────────────────────────────
  std::vector<int8_t> canvas_data_;           ///< Shared pixel buffer
  std::vector<int>    last_written_indices_;  ///< Cells written last callback (partial reset)
  std::mutex          canvas_mutex_;          ///< Guards canvas_data_ between timer + callback

  // ── Runtime state ──────────────────────────────────────────────────────────
  bool        slam_received_;    ///< True after first valid SLAM message
  int         overflow_count_;   ///< Count of cells dropped due to canvas boundary
  std::string frame_id_;         ///< Populated from first SLAM message header

  // ── ROS 2 interfaces ───────────────────────────────────────────────────────
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr    publisher_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr subscription_;
  rclcpp::TimerBase::SharedPtr                                  heartbeat_timer_;
};


// ============================================================================
int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MapExpansionNode>());
  rclcpp::shutdown();
  return 0;
}
