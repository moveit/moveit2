/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2012, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Author: Dave Coleman */

// SA
#include <moveit_setup_assistant/setup_assistant_widget.hpp>

// Qt
#include <QApplication>
#include <QCloseEvent>
#include <QHBoxLayout>
#include <QMessageBox>
#include <QPushButton>
#include <QSplitter>
#include <QStackedWidget>
#include <QString>
#include <pluginlib/class_loader.hpp>  // for loading all avail kinematic planners

namespace moveit_setup
{
namespace assistant
{
// ******************************************************************************************
// Outer User Interface for MoveIt Configuration Assistant
// ******************************************************************************************
SetupAssistantWidget::SetupAssistantWidget(const rviz_common::ros_integration::RosNodeAbstractionIface::WeakPtr& node,
                                           QWidget* parent, const boost::program_options::variables_map& args)
  : QWidget(parent)
  , node_abstraction_(node)
  , node_(node_abstraction_.lock()->get_raw_node())
  , widget_loader_("moveit_setup_framework", "moveit_setup::SetupStepWidget")
{
  // Create object to hold all MoveIt configuration data
  config_data_ = std::make_shared<DataWarehouse>(node_);

  // Set debug mode flag if necessary
  if (args.count("debug"))
    config_data_->debug = true;

  // Setting the window icon
  auto icon_path = getSharePath("moveit_ros_visualization") / "icons/classes/MotionPlanning.png";
  setWindowIcon(QIcon(icon_path.c_str()));

  // Basic widget container -----------------------------------------
  QHBoxLayout* layout = new QHBoxLayout();
  layout->setAlignment(Qt::AlignTop);

  // Create main content stack for various screens
  main_content_ = new QStackedWidget();
  current_index_ = -1;

  // Setup Steps --------------------------------------------------------
  std::vector<std::string> setup_steps;
  // TODO: The list of setup steps should be read from a parameter with some as the default
  // TODO: (or be configured dynamically in some other step)
  setup_steps.push_back("moveit_setup::core::StartScreenWidget");
  setup_steps.push_back("moveit_setup::srdf_setup::DefaultCollisionsWidget");
  setup_steps.push_back("moveit_setup::srdf_setup::VirtualJointsWidget");
  setup_steps.push_back("moveit_setup::srdf_setup::PlanningGroupsWidget");
  setup_steps.push_back("moveit_setup::srdf_setup::RobotPosesWidget");
  setup_steps.push_back("moveit_setup::srdf_setup::EndEffectorsWidget");
  setup_steps.push_back("moveit_setup::srdf_setup::PassiveJointsWidget");
  setup_steps.push_back("moveit_setup::controllers::UrdfModificationsWidget");
  setup_steps.push_back("moveit_setup::controllers::ROS2ControllersWidget");
  setup_steps.push_back("moveit_setup::controllers::MoveItControllersWidget");
  // setup_steps.push_back("moveit_setup::simulation::SimulationWidget");
  setup_steps.push_back("moveit_setup::app::PerceptionWidget");
  setup_steps.push_back("moveit_setup::app::LaunchesWidget");
  setup_steps.push_back("moveit_setup::core::AuthorInformationWidget");
  setup_steps.push_back("moveit_setup::core::ConfigurationFilesWidget");
  node_->declare_parameter("setup_steps", setup_steps);
  setup_steps = node_->get_parameter("setup_steps").as_string_array();

  // Rviz View Right Pane ---------------------------------------------------
  rviz_panel_ = new RVizPanel(this, node_abstraction_, config_data_);
  rviz_panel_->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Preferred);
  rviz_panel_->hide();  // do not show until after the start screen

  for (const std::string& step_class : setup_steps)
  {
    auto widget = widget_loader_.createSharedInstance(step_class);
    widget->initialize(node_, this, rviz_panel_, config_data_);

    connect(widget.get(), SIGNAL(dataUpdated()), this, SLOT(onDataUpdate()));
    connect(widget.get(), SIGNAL(advanceRequest()), this, SLOT(onAdvanceRequest()));
    connect(widget.get(), SIGNAL(setModalMode(bool)), this, SLOT(onModalModeUpdate(bool)));
    const std::string name = widget->getSetupStep().getName();
    steps_.push_back(widget);

    main_content_->addWidget(widget.get());

    nav_name_list_ << name.c_str();
  }

  // Pass command arg values to start screen and show appropriate part of screen
  if (args.count("urdf_path"))
  {
    config_data_->preloadWithURDFPath(args["urdf_path"].as<std::filesystem::path>());
  }
  if (args.count("config_pkg"))
  {
    config_data_->preloadWithFullConfig(args["config_pkg"].as<std::string>());
  }

  // Navigation Left Pane --------------------------------------------------
  navs_view_ = new NavigationWidget(this);
  navs_view_->setNavs(nav_name_list_);
  if (!steps_.empty())
  {
    navs_view_->setEnabled(0, true);
    moveToScreen(0);
  }

  // Split screen -----------------------------------------------------
  splitter_ = new QSplitter(Qt::Horizontal, this);
  splitter_->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
  splitter_->addWidget(navs_view_);
  splitter_->addWidget(main_content_);
  splitter_->addWidget(rviz_panel_);
  splitter_->setHandleWidth(6);
  // splitter_->setCollapsible( 0, false ); // don't let navigation collapse
  layout->addWidget(splitter_);

  // Add event for switching between screens -------------------------
  connect(navs_view_, SIGNAL(clicked(const QModelIndex&)), this, SLOT(navigationClicked(const QModelIndex&)));

  // Final Layout Setup ---------------------------------------------
  setLayout(layout);

  // Title
  setWindowTitle("MoveIt Setup Assistant");  // title of window

  // Show screen before message
  QApplication::processEvents();
}

// ******************************************************************************************
// Change screens of Setup Assistant
// ******************************************************************************************
void SetupAssistantWidget::navigationClicked(const QModelIndex& index)
{
  // Convert QModelIndex to int
  moveToScreen(index.row());
}

void SetupAssistantWidget::onDataUpdate()
{
  for (size_t index = 0; index < steps_.size(); index++)
  {
    bool ready = steps_[index]->isReady();
    navs_view_->setEnabled(index, ready);
  }

  if (rviz_panel_->isReadyForInitialization())
  {
    rviz_panel_->initialize();
    // Replace logo with Rviz screen
    rviz_panel_->show();
  }
}

void SetupAssistantWidget::onAdvanceRequest()
{
  if (static_cast<unsigned int>(current_index_ + 1) < steps_.size())
  {
    moveToScreen(current_index_ + 1);
  }
}

// ******************************************************************************************
// Change screens
// ******************************************************************************************
void SetupAssistantWidget::moveToScreen(const int index)
{
  std::scoped_lock slock(change_screen_lock_);
  if (!navs_view_->isEnabled(index))
  {
    return;
  }

  if (current_index_ != index)
  {
    // Send the focus lost command to the screen widget
    if (current_index_ >= 0)
    {
      auto ssw = steps_[current_index_];
      if (!ssw->focusLost())
      {
        navs_view_->setSelected(current_index_);
        return;  // switching not accepted
      }
    }

    current_index_ = index;

    // Unhighlight anything on robot
    rviz_panel_->unhighlightAll();

    // Change screens
    main_content_->setCurrentIndex(index);

    // Send the focus given command to the screen widget
    steps_[current_index_]->focusGiven();

    // Change navigation selected option
    navs_view_->setSelected(index);
  }
}

// ******************************************************************************************
// Ping ROS on interval
// ******************************************************************************************
void SetupAssistantWidget::updateTimer()
{
  // TODO: Figure out if there's a ROS 2 equivalent of this that needs to be run
  // ros::spinOnce();  // keep ROS node alive
}

// ******************************************************************************************
// Qt close event function for reminding user to save
// ******************************************************************************************
void SetupAssistantWidget::closeEvent(QCloseEvent* event)
{
  // Only prompt to close if not in debug mode
  if (!config_data_->debug)
  {
    if (QMessageBox::question(this, "Exit Setup Assistant",
                              QString("Are you sure you want to exit the MoveIt Setup Assistant?"),
                              QMessageBox::Ok | QMessageBox::Cancel) == QMessageBox::Cancel)
    {
      event->ignore();
      return;
    }
  }

  // Shutdown app
  event->accept();
}

// ******************************************************************************************
// Qt Error Handling - TODO
// ******************************************************************************************
bool SetupAssistantWidget::notify(QObject* /*receiver*/, QEvent* /*event*/)
{
  QMessageBox::critical(this, "Error", "An error occurred and was caught by Qt notify event handler.", QMessageBox::Ok);

  return false;
}

// ******************************************************************************************
// Change the widget modal state based on subwidgets state
// ******************************************************************************************
void SetupAssistantWidget::onModalModeUpdate(bool isModal)
{
  navs_view_->setDisabled(isModal);

  for (int i = 0; i < nav_name_list_.count(); ++i)
  {
    navs_view_->setEnabled(i, !isModal && steps_[i]->isReady());
  }
}

}  // namespace assistant
}  // namespace moveit_setup
