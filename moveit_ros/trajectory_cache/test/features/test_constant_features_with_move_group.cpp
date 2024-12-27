// Copyright 2024 Intrinsic Innovation LLC.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

/** @file
 * @author methylDragon
 */

#include <memory>

#include <gtest/gtest.h>
#include <warehouse_ros/message_collection.h>

#include <geometry_msgs/msg/point.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>
#include <moveit/trajectory_cache/features/constant_features.hpp>

#include "../fixtures/move_group_fixture.hpp"

namespace
{

using ::geometry_msgs::msg::Point;

using ::warehouse_ros::MessageCollection;
using ::warehouse_ros::Metadata;
using ::warehouse_ros::Query;

using ::moveit_ros::trajectory_cache::QueryOnlyEqFeature;
using ::moveit_ros::trajectory_cache::QueryOnlyGTEFeature;
using ::moveit_ros::trajectory_cache::QueryOnlyLTEFeature;
using ::moveit_ros::trajectory_cache::QueryOnlyRangeInclusiveWithToleranceFeature;

using ::moveit_ros::trajectory_cache::MetadataOnlyFeature;

TEST_F(MoveGroupFixture, MetadataOnlyFeature)
{
  MessageCollection<Point> coll = db_->openCollection<Point>("test_db", "test_collection");

  Query::Ptr query = coll.createQuery();
  Metadata::Ptr metadata = coll.createMetadata();
  ASSERT_EQ(metadata->lookupFieldNames().size(), 0);

  Point msg;

  /// MetadataOnlyFeature.

  MetadataOnlyFeature<std::string, Point> string_metadata_feature("string_metadata", "test_string");
  MetadataOnlyFeature<double, Point> double_metadata_feature("double_metadata", 1.0);
  MetadataOnlyFeature<int, Point> int_metadata_feature("int_metadata", 2);
  MetadataOnlyFeature<bool, Point> bool_metadata_feature("bool_metadata", true);

  // Names.
  EXPECT_EQ(string_metadata_feature.getName(), "MetadataOnlyFeature.string_metadata");
  EXPECT_EQ(double_metadata_feature.getName(), "MetadataOnlyFeature.double_metadata");
  EXPECT_EQ(int_metadata_feature.getName(), "MetadataOnlyFeature.int_metadata");
  EXPECT_EQ(bool_metadata_feature.getName(), "MetadataOnlyFeature.bool_metadata");

  // Expect no-ops.
  EXPECT_EQ(string_metadata_feature.appendFeaturesAsFuzzyFetchQuery(*query, msg, *move_group_, 0.0),
            moveit::core::MoveItErrorCode::SUCCESS);
  EXPECT_EQ(string_metadata_feature.appendFeaturesAsExactFetchQuery(*query, msg, *move_group_, 0.0),
            moveit::core::MoveItErrorCode::SUCCESS);

  // Fetch ok.
  string_metadata_feature.appendFeaturesAsInsertMetadata(*metadata, msg, *move_group_);
  EXPECT_EQ(metadata->lookupFieldNames().size(), 1);
  EXPECT_TRUE(metadata->lookupField("string_metadata"));
  EXPECT_EQ(metadata->lookupString("string_metadata"), "test_string");

  double_metadata_feature.appendFeaturesAsInsertMetadata(*metadata, msg, *move_group_);
  EXPECT_EQ(metadata->lookupFieldNames().size(), 2);
  EXPECT_TRUE(metadata->lookupField("double_metadata"));
  EXPECT_EQ(metadata->lookupDouble("double_metadata"), 1.0);

  int_metadata_feature.appendFeaturesAsInsertMetadata(*metadata, msg, *move_group_);
  EXPECT_EQ(metadata->lookupFieldNames().size(), 3);
  EXPECT_TRUE(metadata->lookupField("int_metadata"));
  EXPECT_EQ(metadata->lookupInt("int_metadata"), 2);

  bool_metadata_feature.appendFeaturesAsInsertMetadata(*metadata, msg, *move_group_);
  EXPECT_EQ(metadata->lookupFieldNames().size(), 4);
  EXPECT_TRUE(metadata->lookupField("bool_metadata"));
  EXPECT_TRUE(metadata->lookupBool("bool_metadata"));
}

TEST_F(MoveGroupFixture, QueryOnlyEqFeature)
{
  MessageCollection<Point> coll = db_->openCollection<Point>("test_db", "test_collection");

  Metadata::Ptr metadata = coll.createMetadata();
  metadata->append("test_metadata", "test_metadata");

  Point msg;
  coll.insert(msg, metadata);

  QueryOnlyEqFeature<std::string, Point> eq_feature("test_metadata", "test_metadata");
  QueryOnlyEqFeature<std::string, Point> unrelated_eq_feature("unrelated", "test_metadata");
  QueryOnlyEqFeature<std::string, Point> mismatched_eq_feature("test_metadata", "mismatched");

  // Names.
  EXPECT_EQ(eq_feature.getName(), "QueryOnlyEqFeature.test_metadata");
  EXPECT_EQ(unrelated_eq_feature.getName(), "QueryOnlyEqFeature.unrelated");

  // Expect no-ops.
  {
    Metadata::Ptr noop_metadata = coll.createMetadata();
    eq_feature.appendFeaturesAsInsertMetadata(*noop_metadata, msg, *move_group_);
    EXPECT_TRUE(noop_metadata->lookupFieldNames().empty());
  }

  // Match.
  {
    Query::Ptr fuzzy_query = coll.createQuery();
    eq_feature.appendFeaturesAsFuzzyFetchQuery(*fuzzy_query, msg, *move_group_, 0.0);
    EXPECT_EQ(coll.queryList(fuzzy_query).size(), 1);

    Query::Ptr exact_query = coll.createQuery();
    eq_feature.appendFeaturesAsExactFetchQuery(*exact_query, msg, *move_group_, 0.0);
    EXPECT_EQ(coll.queryList(exact_query).size(), 1);
  }

  // Unrelated. No match.
  {
    Query::Ptr fuzzy_query = coll.createQuery();
    unrelated_eq_feature.appendFeaturesAsFuzzyFetchQuery(*fuzzy_query, msg, *move_group_, 0.0);
    EXPECT_TRUE(coll.queryList(fuzzy_query).empty());

    Query::Ptr exact_query = coll.createQuery();
    unrelated_eq_feature.appendFeaturesAsExactFetchQuery(*exact_query, msg, *move_group_, 0.0);
    EXPECT_TRUE(coll.queryList(exact_query).empty());
  }

  // Mismatched. No match.
  {
    Query::Ptr fuzzy_query = coll.createQuery();
    mismatched_eq_feature.appendFeaturesAsFuzzyFetchQuery(*fuzzy_query, msg, *move_group_, 0.0);
    EXPECT_TRUE(coll.queryList(fuzzy_query).empty());

    Query::Ptr exact_query = coll.createQuery();
    mismatched_eq_feature.appendFeaturesAsExactFetchQuery(*exact_query, msg, *move_group_, 0.0);
    EXPECT_TRUE(coll.queryList(exact_query).empty());
  }
}

TEST_F(MoveGroupFixture, QueryOnlyGTEFeature)
{
  MessageCollection<Point> coll = db_->openCollection<Point>("test_db", "test_collection");

  Metadata::Ptr metadata = coll.createMetadata();
  metadata->append("test_metadata", 5.0);

  Point msg;
  coll.insert(msg, metadata);

  metadata = coll.createMetadata();
  metadata->append("unrelated", 5.0);
  coll.insert(msg, metadata);

  QueryOnlyGTEFeature<double, Point> gte_feature("test_metadata", 4.0);
  QueryOnlyGTEFeature<double, Point> gte_eq_feature("test_metadata", 5.0);
  QueryOnlyGTEFeature<double, Point> unrelated_gte_feature("unrelated", 6.0);
  QueryOnlyGTEFeature<double, Point> mismatched_gte_feature("test_metadata", 6.0);

  // Names.
  EXPECT_EQ(gte_feature.getName(), "QueryOnlyGTEFeature.test_metadata");
  EXPECT_EQ(unrelated_gte_feature.getName(), "QueryOnlyGTEFeature.unrelated");

  // Expect no-ops.
  {
    Metadata::Ptr metadata = coll.createMetadata();
    gte_feature.appendFeaturesAsInsertMetadata(*metadata, msg, *move_group_);
    EXPECT_TRUE(metadata->lookupFieldNames().empty());
  }

  // Match.
  {
    Query::Ptr fuzzy_query = coll.createQuery();
    gte_feature.appendFeaturesAsFuzzyFetchQuery(*fuzzy_query, msg, *move_group_, 0.0);
    EXPECT_EQ(coll.queryList(fuzzy_query).size(), 1);

    Query::Ptr exact_query = coll.createQuery();
    gte_feature.appendFeaturesAsExactFetchQuery(*exact_query, msg, *move_group_, 0.0);
    EXPECT_EQ(coll.queryList(exact_query).size(), 1);
  }

  {
    Query::Ptr fuzzy_query = coll.createQuery();
    gte_eq_feature.appendFeaturesAsFuzzyFetchQuery(*fuzzy_query, msg, *move_group_, 0.0);
    EXPECT_EQ(coll.queryList(fuzzy_query).size(), 1);

    Query::Ptr exact_query = coll.createQuery();
    gte_eq_feature.appendFeaturesAsExactFetchQuery(*exact_query, msg, *move_group_, 0.0);
    EXPECT_EQ(coll.queryList(exact_query).size(), 1);
  }

  // Unrelated. No match.
  {
    Query::Ptr unrelated_fuzzy_query = coll.createQuery();
    unrelated_gte_feature.appendFeaturesAsFuzzyFetchQuery(*unrelated_fuzzy_query, msg, *move_group_, 0.0);
    EXPECT_TRUE(coll.queryList(unrelated_fuzzy_query).empty());

    Query::Ptr exact_query = coll.createQuery();
    unrelated_gte_feature.appendFeaturesAsExactFetchQuery(*exact_query, msg, *move_group_, 0.0);
    EXPECT_TRUE(coll.queryList(exact_query).empty());
  }

  // Mismatched. No match.
  {
    Query::Ptr fuzzy_query = coll.createQuery();
    mismatched_gte_feature.appendFeaturesAsFuzzyFetchQuery(*fuzzy_query, msg, *move_group_, 0.0);
    EXPECT_TRUE(coll.queryList(fuzzy_query).empty());

    Query::Ptr exact_query = coll.createQuery();
    mismatched_gte_feature.appendFeaturesAsExactFetchQuery(*exact_query, msg, *move_group_, 0.0);
    EXPECT_TRUE(coll.queryList(exact_query).empty());
  }
}

TEST_F(MoveGroupFixture, QueryOnlyLTEFeature)
{
  MessageCollection<Point> coll = db_->openCollection<Point>("test_db", "test_collection");

  Metadata::Ptr metadata = coll.createMetadata();
  metadata->append("test_metadata", 5.0);

  Point msg;
  coll.insert(msg, metadata);

  QueryOnlyLTEFeature<double, Point> lte_feature("test_metadata", 6.0);
  QueryOnlyLTEFeature<double, Point> lte_eq_feature("test_metadata", 5.0);
  QueryOnlyLTEFeature<double, Point> unrelated_lte_feature("unrelated", 6.0);
  QueryOnlyLTEFeature<double, Point> mismatched_lte_feature("test_metadata", 4.0);

  // Names.
  EXPECT_EQ(lte_feature.getName(), "QueryOnlyLTEFeature.test_metadata");
  EXPECT_EQ(unrelated_lte_feature.getName(), "QueryOnlyLTEFeature.unrelated");

  // Expect no-ops.
  {
    Metadata::Ptr metadata = coll.createMetadata();
    lte_feature.appendFeaturesAsInsertMetadata(*metadata, msg, *move_group_);
    EXPECT_TRUE(metadata->lookupFieldNames().empty());
  }

  // Match.
  {
    Query::Ptr fuzzy_query = coll.createQuery();
    lte_feature.appendFeaturesAsFuzzyFetchQuery(*fuzzy_query, msg, *move_group_, 0.0);
    EXPECT_EQ(coll.queryList(fuzzy_query).size(), 1);

    Query::Ptr exact_query = coll.createQuery();
    lte_feature.appendFeaturesAsExactFetchQuery(*exact_query, msg, *move_group_, 0.0);
    EXPECT_EQ(coll.queryList(exact_query).size(), 1);
  }

  {
    Query::Ptr fuzzy_query = coll.createQuery();
    lte_eq_feature.appendFeaturesAsFuzzyFetchQuery(*fuzzy_query, msg, *move_group_, 0.0);
    EXPECT_EQ(coll.queryList(fuzzy_query).size(), 1);

    Query::Ptr exact_query = coll.createQuery();
    lte_eq_feature.appendFeaturesAsExactFetchQuery(*exact_query, msg, *move_group_, 0.0);
    EXPECT_EQ(coll.queryList(exact_query).size(), 1);
  }

  // Unrelated. No match.
  {
    Query::Ptr fuzzy_query = coll.createQuery();
    unrelated_lte_feature.appendFeaturesAsFuzzyFetchQuery(*fuzzy_query, msg, *move_group_, 0.0);
    EXPECT_TRUE(coll.queryList(fuzzy_query).empty());

    Query::Ptr exact_query = coll.createQuery();
    unrelated_lte_feature.appendFeaturesAsExactFetchQuery(*exact_query, msg, *move_group_, 0.0);
    EXPECT_TRUE(coll.queryList(exact_query).empty());
  }

  // Mismatched. No match.
  {
    Query::Ptr fuzzy_query = coll.createQuery();
    mismatched_lte_feature.appendFeaturesAsFuzzyFetchQuery(*fuzzy_query, msg, *move_group_, 0.0);
    EXPECT_TRUE(coll.queryList(fuzzy_query).empty());

    Query::Ptr exact_query = coll.createQuery();
    mismatched_lte_feature.appendFeaturesAsExactFetchQuery(*exact_query, msg, *move_group_, 0.0);
    EXPECT_TRUE(coll.queryList(exact_query).empty());
  }
}

TEST_F(MoveGroupFixture, QueryOnlyRangeInclusiveWithToleranceFeature)
{
  MessageCollection<Point> coll = db_->openCollection<Point>("test_db", "test_collection");

  Metadata::Ptr metadata = coll.createMetadata();
  metadata->append("test_metadata", 5.0);

  Point msg;
  coll.insert(msg, metadata);

  QueryOnlyRangeInclusiveWithToleranceFeature<double, Point> exact_range_feature("test_metadata", 5.0, 5.0);
  QueryOnlyRangeInclusiveWithToleranceFeature<double, Point> lower_range_feature("test_metadata", 4.0, 5.0);
  QueryOnlyRangeInclusiveWithToleranceFeature<double, Point> upper_range_feature("test_metadata", 5.0, 6.0);
  QueryOnlyRangeInclusiveWithToleranceFeature<double, Point> over_range_feature("test_metadata", 4.5, 5.5);

  QueryOnlyRangeInclusiveWithToleranceFeature<double, Point> unrelated_range_feature("unrelated", 5.5, 6.0);
  QueryOnlyRangeInclusiveWithToleranceFeature<double, Point> mismatched_range_feature("test_metadata", 5.5, 6.0);

  // Names.
  EXPECT_EQ(exact_range_feature.getName(), "QueryOnlyRangeInclusiveWithToleranceFeature.test_metadata");
  EXPECT_EQ(unrelated_range_feature.getName(), "QueryOnlyRangeInclusiveWithToleranceFeature.unrelated");

  // Expect no-ops.
  {
    Metadata::Ptr metadata = coll.createMetadata();
    exact_range_feature.appendFeaturesAsInsertMetadata(*metadata, msg, *move_group_);
    EXPECT_TRUE(metadata->lookupFieldNames().empty());
  }

  // Match.
  {
    Query::Ptr fuzzy_query = coll.createQuery();
    exact_range_feature.appendFeaturesAsFuzzyFetchQuery(*fuzzy_query, msg, *move_group_, 0.0);
    EXPECT_EQ(coll.queryList(fuzzy_query).size(), 1);

    Query::Ptr exact_query = coll.createQuery();
    exact_range_feature.appendFeaturesAsExactFetchQuery(*exact_query, msg, *move_group_, 0.0);
    EXPECT_EQ(coll.queryList(exact_query).size(), 1);
  }

  {
    Query::Ptr fuzzy_query = coll.createQuery();
    lower_range_feature.appendFeaturesAsFuzzyFetchQuery(*fuzzy_query, msg, *move_group_, 0.0);
    EXPECT_EQ(coll.queryList(fuzzy_query).size(), 1);

    Query::Ptr exact_query = coll.createQuery();
    lower_range_feature.appendFeaturesAsExactFetchQuery(*exact_query, msg, *move_group_, 0.0);
    EXPECT_EQ(coll.queryList(exact_query).size(), 1);
  }

  {
    Query::Ptr fuzzy_query = coll.createQuery();
    upper_range_feature.appendFeaturesAsFuzzyFetchQuery(*fuzzy_query, msg, *move_group_, 0.0);
    EXPECT_EQ(coll.queryList(fuzzy_query).size(), 1);

    Query::Ptr exact_query = coll.createQuery();
    upper_range_feature.appendFeaturesAsExactFetchQuery(*exact_query, msg, *move_group_, 0.0);
    EXPECT_EQ(coll.queryList(exact_query).size(), 1);
  }

  {
    Query::Ptr fuzzy_query = coll.createQuery();
    over_range_feature.appendFeaturesAsFuzzyFetchQuery(*fuzzy_query, msg, *move_group_, 0.0);
    EXPECT_EQ(coll.queryList(fuzzy_query).size(), 1);

    Query::Ptr exact_query = coll.createQuery();
    over_range_feature.appendFeaturesAsExactFetchQuery(*exact_query, msg, *move_group_, 0.0);
    EXPECT_EQ(coll.queryList(exact_query).size(), 1);
  }

  // Unrelated. No match.
  {
    Query::Ptr fuzzy_query = coll.createQuery();
    unrelated_range_feature.appendFeaturesAsFuzzyFetchQuery(*fuzzy_query, msg, *move_group_, 0.0);
    EXPECT_TRUE(coll.queryList(fuzzy_query).empty());

    Query::Ptr exact_query = coll.createQuery();
    unrelated_range_feature.appendFeaturesAsExactFetchQuery(*exact_query, msg, *move_group_, 0.0);
    EXPECT_TRUE(coll.queryList(exact_query).empty());
  }

  // Mismatched. No match.
  {
    Query::Ptr fuzzy_query = coll.createQuery();
    mismatched_range_feature.appendFeaturesAsFuzzyFetchQuery(*fuzzy_query, msg, *move_group_, 0.0);
    EXPECT_TRUE(coll.queryList(fuzzy_query).empty());

    Query::Ptr exact_query = coll.createQuery();
    mismatched_range_feature.appendFeaturesAsExactFetchQuery(*exact_query, msg, *move_group_, 0.0);
    EXPECT_TRUE(coll.queryList(exact_query).empty());
  }
}

}  // namespace

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  ::testing::InitGoogleTest(&argc, argv);
  int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
