#include <gtest/gtest.h>
#include <gmock/gmock.h>
#include <moveit/planning_request_adapter/planning_request_adapter.h>

using namespace planning_request_adapter;
using namespace planning_interface;

// A simple callback function that is used for recording the call order of adapters and planner
using RegisterNameCallbackFn = std::function<void(const std::string&)>;

/*
 * A PlanningContext mock that allows setting planning success and planner name, which is returned on solve()
 * using the RegisterNameCallbackFn.
 */
class MockPlanningContext : public PlanningContext
{
public:
  /** \brief Construct a planning context named \e name for the group \e group */
  MockPlanningContext(const std::string& name, bool succeeds, RegisterNameCallbackFn callback_fn)
    : PlanningContext(name, "group"), succeeds_(succeeds), callback_fn_(callback_fn)
  {
  }

  /** \brief Set the planning scene for this context */
  void setPlanningScene(const planning_scene::PlanningSceneConstPtr& planning_scene)
  {
    planning_scene_ = planning_scene;
  }

  /** \brief Set the planning request for this context */
  void setMotionPlanRequest(const MotionPlanRequest& request)
  {
    request_ = request;
  }

  bool solve(MotionPlanResponse& res) override
  {
    res.error_code.val =
        succeeds_ ? moveit_msgs::msg::MoveItErrorCodes::SUCCESS : moveit_msgs::msg::MoveItErrorCodes::FAILURE;
    callback_fn_(name_);
    return true;
  }

  // Mocks
  MOCK_METHOD(bool, solve, (MotionPlanDetailedResponse & res), (override));
  MOCK_METHOD(bool, terminate, (), (override));
  MOCK_METHOD(void, clear, (), (override));

private:
  const bool succeeds_;
  RegisterNameCallbackFn callback_fn_;
};

/*
 * A PlannerManager mock that produces MockPlanningContext instances with the provided name and success results.
 * The success of the planner can be set using setSucceedsFlag(). The RegisterNameCallbackFn will be called with
 * the specified planner name on every solve() attempt.
 */
class MockPlannerManager : public PlannerManager
{
public:
  MockPlannerManager(const std::string& name, RegisterNameCallbackFn callback_fn)
    : name_(name), callback_fn_(callback_fn)
  {
  }
  MOCK_METHOD(bool, initialize,
              (const moveit::core::RobotModelConstPtr& model, const rclcpp::Node::SharedPtr& node,
               const std::string& parameter_namespace),
              (override));
  MOCK_METHOD(std::string, getDescription, (), (const, override));
  MOCK_METHOD(void, getPlanningAlgorithms, (std::vector<std::string> & algs), (const, override));
  MOCK_METHOD(void, setPlannerConfigurations, (const PlannerConfigurationMap& pcs), (override));

  PlanningContextPtr getPlanningContext(const planning_scene::PlanningSceneConstPtr& planning_scene,
                                        const MotionPlanRequest& req,
                                        moveit_msgs::msg::MoveItErrorCodes& /* error_code */) const override
  {
    return getPlanningContext(planning_scene, req);
  }

  PlanningContextPtr getPlanningContext(const planning_scene::PlanningSceneConstPtr& /* planning_scene */,
                                        const MotionPlanRequest& /* req */) const
  {
    return std::make_shared<MockPlanningContext>(name_, succeeds_, callback_fn_);
  }

  /// \brief Determine whether this plugin instance is able to represent this planning request
  bool canServiceRequest(const MotionPlanRequest& /*req*/) const override
  {
    return true;
  }

  void terminate() const
  {
  }

  void setSucceedsFlag(bool succeeds)
  {
    succeeds_ = succeeds;
  }

private:
  const std::string name_;
  bool succeeds_{ true };
  RegisterNameCallbackFn callback_fn_;
};

/*
 * A PlanningRequestAdapter mock that can be positioned before or after a planner using AdapterMode,
 * that has a predefined result for the adaptAndPlan() function, and that offers a callback for returning
 * the name when it is being called in the adapter chain.
 */
enum AdapterMode
{
  BEFORE_PLANNER,
  AFTER_PLANNER
};
class MockAdapter : public PlanningRequestAdapter
{
public:
  MockAdapter(const std::string& description, AdapterMode mode, bool succeeds, RegisterNameCallbackFn callback_fn)
    : description_(description), mode_(mode), succeeds_(succeeds), callback_fn_(callback_fn)
  {
  }

  void initialize(const rclcpp::Node::SharedPtr& /* node */, const std::string& /* parameter_namespace */) override
  {
  }

  std::string getDescription() const override
  {
    return description_;
  }

  bool adaptAndPlan(const PlannerFn& planner, const planning_scene::PlanningSceneConstPtr& planning_scene,
                    const MotionPlanRequest& req, MotionPlanResponse& res,
                    std::vector<std::size_t>& /* added_path_index */) const override
  {
    if (mode_ == AdapterMode::BEFORE_PLANNER)
    {
      adapt(req, res);
    }

    planner(planning_scene, req, res);

    if (mode_ == AdapterMode::AFTER_PLANNER)
    {
      adapt(req, res);
    }

    return true;
  }

  void adapt(const MotionPlanRequest& /* req */, MotionPlanResponse& res) const
  {
    res.error_code.val =
        succeeds_ ? moveit_msgs::msg::MoveItErrorCodes::SUCCESS : moveit_msgs::msg::MoveItErrorCodes::FAILURE;
    callback_fn_(description_);
  }

private:
  const std::string description_;
  const AdapterMode mode_;
  const bool succeeds_;
  RegisterNameCallbackFn callback_fn_;
};

/**
 * Tests the call sequence of planner and adapters configured in an AdapterChain in succeeding and failing scenarios
 */
TEST(TestPlanningRequestAdapterChain, SequenceOK)
{
  constexpr bool SUCCEEDS = true;
  std::vector<std::string> called_adapters;
  RegisterNameCallbackFn register_name_on_call = [&](const std::string& adapter) { called_adapters.push_back(adapter); };

  // Simple adapter chain that should result in this pipeline sequence: {A, B, PLANNER, C, D}
  PlanningRequestAdapterChain chain;
  chain.addAdapter(std::make_shared<MockAdapter>("A", BEFORE_PLANNER, SUCCEEDS, register_name_on_call));
  chain.addAdapter(std::make_shared<MockAdapter>("B", BEFORE_PLANNER, SUCCEEDS, register_name_on_call));
  chain.addAdapter(std::make_shared<MockAdapter>("C", AFTER_PLANNER, SUCCEEDS, register_name_on_call));
  chain.addAdapter(std::make_shared<MockAdapter>("D", AFTER_PLANNER, SUCCEEDS, register_name_on_call));

  auto planner = std::make_shared<MockPlannerManager>("PLANNER", register_name_on_call);
  planning_scene::PlanningSceneConstPtr scene = nullptr;
  MotionPlanRequest req;
  MotionPlanResponse res;

  // CASE: Planning and adapters succeed
  // EXPECTED: All elements should be called in the correct sequence - planning fails
  planner->setSucceedsFlag(true);
  EXPECT_TRUE(chain.adaptAndPlan(planner, scene, req, res));
  EXPECT_TRUE(bool(res));
  EXPECT_THAT(called_adapters, ::testing::ElementsAre("A", "B", "PLANNER", "C", "D"));

  // CASE: Planner fails
  // EXPECTED: No adapters should be called after a failing planner - planning fails
  called_adapters.clear();
  planner->setSucceedsFlag(false);
  EXPECT_TRUE(chain.adaptAndPlan(planner, scene, req, res));
  EXPECT_FALSE(bool(res));
  EXPECT_THAT(called_adapters, ::testing::ElementsAre("A", "B", "PLANNER"));

  // CASE: Failing adapter after a succeeding plan
  // EXPECTED: No adapters should be called after a failing adapter (here "E") - planning fails
  chain.addAdapter(std::make_shared<MockAdapter>("fails_after", AFTER_PLANNER, !SUCCEEDS, register_name_on_call));
  chain.addAdapter(std::make_shared<MockAdapter>("E", AFTER_PLANNER, SUCCEEDS, register_name_on_call));

  called_adapters.clear();
  planner->setSucceedsFlag(true);
  EXPECT_TRUE(chain.adaptAndPlan(planner, scene, req, res));
  EXPECT_FALSE(bool(res));
  EXPECT_THAT(called_adapters, ::testing::ElementsAre("A", "B", "PLANNER", "C", "D", "fails_after"));

  // CASE: Failing adapter before the planner
  // EXPECTED: No adapters or planner should be called after a failing adapter - planning fails
  chain.addAdapter(std::make_shared<MockAdapter>("fails_before", BEFORE_PLANNER, !SUCCEEDS, register_name_on_call));

  called_adapters.clear();
  planner->setSucceedsFlag(true);
  EXPECT_TRUE(chain.adaptAndPlan(planner, scene, req, res));
  EXPECT_FALSE(bool(res));
  EXPECT_THAT(called_adapters, ::testing::ElementsAre("A", "B", "fails_before"));
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
