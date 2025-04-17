#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include "coordinate_transformer/coordinate_transformer.hpp"

using namespace std::chrono_literals;

class CoordinateTransformerTest : public ::testing::Test {
protected:
  void SetUp() override {
    // Initialize ROS node
    if (!rclcpp::ok()) rclcpp::init(0, nullptr);
    node_ = rclcpp::Node::make_shared("test_transformer_node");
    transformer_ = std::make_shared<CoordinateTransformer>(node_);

    // Add a known transform: frame_a -> frame_b : translation (1,2,3), identity rotation
    geometry_msgs::msg::TransformStamped tf_ab;
    tf_ab.header.frame_id = "frame_a";
    tf_ab.child_frame_id = "frame_b";
    tf_ab.transform.translation.x = 1.0;
    tf_ab.transform.translation.y = 2.0;
    tf_ab.transform.translation.z = 3.0;
    tf_ab.transform.rotation.w = 1.0;
    transformer_->addTransform(tf_ab);

    // Set bounds for frame_b: allow only within [0,5] in each axis
    geometry_msgs::msg::Point min_pt{}, max_pt{};
    min_pt.x = 0; min_pt.y = 0; min_pt.z = 0;
    max_pt.x = 5; max_pt.y = 5; max_pt.z = 5;
    transformer_->setBounds("frame_b", min_pt, max_pt);
  }

  std::shared_ptr<rclcpp::Node> node_;
  std::shared_ptr<CoordinateTransformer> transformer_;
};

TEST_F(CoordinateTransformerTest, DirectTransformWithinBounds) {
  // Input pose at origin in frame_a
  geometry_msgs::msg::PoseStamped input, output;
  input.header.frame_id = "frame_a";
  input.pose.position.x = 0.0;
  input.pose.position.y = 0.0;
  input.pose.position.z = 0.0;
  input.pose.orientation.w = 1.0;

  auto status = transformer_->convert(input, output, "frame_b");
  EXPECT_EQ(status, ResultStatus::SUCCESS);
  EXPECT_NEAR(output.pose.position.x, 1.0, 1e-6);
  EXPECT_NEAR(output.pose.position.y, 2.0, 1e-6);
  EXPECT_NEAR(output.pose.position.z, 3.0, 1e-6);
}

TEST_F(CoordinateTransformerTest, DirectTransformOutOfBounds) {
  // Input pose that will map outside bounds in frame_b
  geometry_msgs::msg::PoseStamped input, output;
  input.header.frame_id = "frame_a";
  input.pose.position.x = 10.0;  // maps to 11, outside max=5
  input.pose.position.y = 0.0;
  input.pose.position.z = 0.0;
  input.pose.orientation.w = 1.0;

  auto status = transformer_->convert(input, output, "frame_b");
  EXPECT_EQ(status, ResultStatus::OUT_OF_BOUNDS);
}

TEST_F(CoordinateTransformerTest, InverseTransform) {
  // First do direct to get output, then inverse
  geometry_msgs::msg::PoseStamped in, mid, out;
  in.header.frame_id = "frame_a";
  in.pose.position.x = 1.0;
  in.pose.position.y = 1.0;
  in.pose.position.z = 1.0;
  in.pose.orientation.w = 1.0;

  auto s1 = transformer_->convert(in, mid, "frame_b");
  ASSERT_EQ(s1, ResultStatus::SUCCESS);
  auto s2 = transformer_->inverseConvert(mid, out, "frame_a");
  EXPECT_EQ(s2, ResultStatus::SUCCESS);
  EXPECT_NEAR(out.pose.position.x, in.pose.position.x, 1e-6);
  EXPECT_NEAR(out.pose.position.y, in.pose.position.y, 1e-6);
  EXPECT_NEAR(out.pose.position.z, in.pose.position.z, 1e-6);
}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  int ret = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return ret;
}
