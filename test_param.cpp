#include "image_subscriber.hpp"
#include "instantiate_component.hpp"
#include "log_client.hpp"
#include "param_client.hpp"
#include <gtest/gtest.h>
#include <libcamera/camera_manager.h>
#include <rclcpp/rclcpp.hpp>


class ParamTest : public testing::Test
{
protected:
  void
  SetUp() override
  {
    {
      // skip tests when no camera is available
      libcamera::CameraManager camera_manager;
      camera_manager.start();
      if (camera_manager.cameras().empty())
        GTEST_SKIP() << "No cameras available. Skipping tests." << std::endl;
    }

    rclcpp::get_logger(CAMERA_NODE_NAME).set_level(rclcpp::Logger::Level::Debug);

    exec = rclcpp::executors::SingleThreadedExecutor::make_shared();

    log_client = std::make_unique<LogClient>(rclcpp::NodeOptions {}.use_intra_process_comms(true), CAMERA_NODE_NAME);

    param_client = std::make_unique<ParamClient>(rclcpp::NodeOptions {}.use_intra_process_comms(true), CAMERA_NODE_NAME, exec);

    image_subscriber = std::make_unique<ImageSubscriber>(rclcpp::NodeOptions {}.use_intra_process_comms(true), CAMERA_NODE_NAME);

    exec->add_node(log_client->get_node_base_interface());
    exec->add_node(image_subscriber->get_node_base_interface());
  }

  void
  TearDown() override
  {
    if (exec) {
      exec->remove_node(camera.get_node_base_interface());
      exec->remove_node(log_client->get_node_base_interface());
    }
  }

  void
  spin_all()
  {
    // spin log nodes until all work is done
    exec->spin_all(std::chrono::nanoseconds {0});
  }

  bool
  wait_image()
  {
    return image_subscriber->wait();
  }

  void
  instantiate_camera(const std::vector<rclcpp::Parameter> &parameter_overrides)
  {
    if (!exec) {
      FAIL() << "Executor is not initialised";
    }

    camera = instantiate_component(
      "camera::CameraNode",
      rclcpp::NodeOptions {}
        .use_intra_process_comms(true)
        .arguments({"--ros-args", "--remap", "__node:=" + CAMERA_NODE_NAME})
        .parameter_overrides(parameter_overrides));

    exec->add_node(camera.get_node_base_interface());
  }

  void
  set_check_all_successful(const std::vector<rclcpp::Parameter> &parameters)
  {
    const std::vector<rcl_interfaces::msg::SetParametersResult> res =
      param_client->set_parameters(parameters);

    for (size_t i = 0; i < parameters.size(); i++) {
      ASSERT_EQ(res[i].reason, std::string {});
      ASSERT_TRUE(res[i].successful);
      ASSERT_EQ(param_client->get_parameters({parameters[i].get_name()}).front().get_parameter_value(),
                parameters[i].get_parameter_value());
    }
  }

  const std::string CAMERA_NODE_NAME = "camera";

  const std::string conflict_reason = "ExposureTimeMode and ExposureTime must not be set simultaneously";
  const std::string declare_ExposureTimeMode = "declare 'ExposureTimeMode' \\(type: integer\\)";
  const std::string declare_ExposureTime = "declare 'ExposureTime' \\(type: integer\\)";

  const int exp_init = 15600;

  rclcpp::Executor::SharedPtr exec;
  rclcpp_components::NodeInstanceWrapper camera;
  std::unique_ptr<ParamClient> param_client;
  std::unique_ptr<LogClient> log_client;
  std::unique_ptr<ImageSubscriber> image_subscriber;
};


TEST_F(ParamTest, override_default)
{
  instantiate_camera({});

  spin_all();

  // expect: declare 'ExposureTimeMode' and 'ExposureTime'
  ASSERT_TRUE(log_client->regex_search(declare_ExposureTimeMode));
  ASSERT_TRUE(log_client->regex_search(declare_ExposureTime));

  // expect: only 'ExposureTimeMode' is set
  ASSERT_TRUE(log_client->regex_search("setting integer parameter ExposureTimeMode to 0"));
  ASSERT_FALSE(log_client->regex_search("setting integer parameter ExposureTime to (.*)"));

  // check that camera has parameter 'ExposureTimeMode'
  ASSERT_TRUE(param_client->has_parameter("ExposureTimeMode"));
  // check that camera has parameter 'ExposureTime'
  ASSERT_TRUE(param_client->has_parameter("ExposureTime"));

  // 'ExposureTimeMode' is set to default value
  ASSERT_TRUE(param_client->is_set_parameter("ExposureTimeMode"));
  ASSERT_EQ(param_client->get_parameters({"ExposureTimeMode"}).front().as_int(), 0);
  // 'ExposureTime' is not set
  ASSERT_FALSE(param_client->is_set_parameter("ExposureTime"));
}

TEST_F(ParamTest, override_ae_disabled)
{
  instantiate_camera({{"ExposureTimeMode", 1}});

  spin_all();

  // expect: declare 'ExposureTimeMode' and 'ExposureTime'
  ASSERT_TRUE(log_client->regex_search(declare_ExposureTimeMode));
  ASSERT_TRUE(log_client->regex_search(declare_ExposureTime));

  // expect: 'ExposureTimeMode' is set to override value
  ASSERT_TRUE(log_client->regex_search("setting integer parameter ExposureTimeMode to 1"));
  // expect: 'ExposureTime' is restored
  ASSERT_TRUE(log_client->regex_search("setting integer parameter ExposureTime to (.*)"));

  ASSERT_TRUE(param_client->is_set_parameter("ExposureTimeMode"));
  ASSERT_EQ(param_client->get_parameters({"ExposureTimeMode"}).front().as_int(), 1);
}

TEST_F(ParamTest, override_exposure)
{
  instantiate_camera({{"ExposureTime", 15600}});

  spin_all();

  // expect: declare 'ExposureTimeMode' and 'ExposureTime'
  ASSERT_TRUE(log_client->regex_search(declare_ExposureTimeMode));
  ASSERT_TRUE(log_client->regex_search(declare_ExposureTime));

  // expect: 'ExposureTimeMode' adjusted to manual
  // expect: 'ExposureTime' set to override value
  ASSERT_TRUE(log_client->regex_search("setting integer parameter ExposureTimeMode to 1"));
  ASSERT_TRUE(log_client->regex_search("setting integer parameter ExposureTime to 15600"));

  ASSERT_TRUE(param_client->is_set_parameter("ExposureTimeMode"));
  ASSERT_EQ(param_client->get_parameters({"ExposureTimeMode"}).front().as_int(), 1);
  ASSERT_TRUE(param_client->is_set_parameter("ExposureTime"));
  ASSERT_EQ(param_client->get_parameters({"ExposureTime"}).front().as_int(), 15600);
}

TEST_F(ParamTest, override_ae_disabled_exposure)
{
  instantiate_camera({{"ExposureTimeMode", 1}, {"ExposureTime", 15600}});

  spin_all();

  // expect: declare 'ExposureTimeMode' and 'ExposureTime'
  ASSERT_TRUE(log_client->regex_search(declare_ExposureTimeMode));
  ASSERT_TRUE(log_client->regex_search(declare_ExposureTime));

  // expect: 'ExposureTimeMode' and 'ExposureTime' are set
  ASSERT_TRUE(log_client->regex_search("setting integer parameter ExposureTimeMode to 1"));
  ASSERT_TRUE(log_client->regex_search("setting integer parameter ExposureTime to 15600"));

  ASSERT_TRUE(param_client->is_set_parameter("ExposureTimeMode"));
  ASSERT_EQ(param_client->get_parameters({"ExposureTimeMode"}).front().as_int(), 1);
  ASSERT_TRUE(param_client->is_set_parameter("ExposureTime"));
  ASSERT_EQ(param_client->get_parameters({"ExposureTime"}).front().as_int(), 15600);
}

TEST_F(ParamTest, override_ae_enabled_exposure)
{
  instantiate_camera({{"ExposureTimeMode", 0}, {"ExposureTime", 15600}});

  spin_all();

  // expect: declare 'ExposureTimeMode' and 'ExposureTime'
  ASSERT_TRUE(log_client->regex_search(declare_ExposureTimeMode));
  ASSERT_TRUE(log_client->regex_search(declare_ExposureTime));

  // expect: 'ExposureTime' override with default 'ExposureTimeMode' shows warning
  ASSERT_TRUE(log_client->regex_search("ExposureTimeMode and ExposureTime must not be enabled at the same time. 'ExposureTimeMode' will be set to off."));

  // expect: 'ExposureTimeMode' and 'ExposureTime' are set
  // expect: 'ExposureTimeMode' adjusted to manual
  ASSERT_TRUE(log_client->regex_search("setting integer parameter ExposureTimeMode to 1"));
  ASSERT_TRUE(log_client->regex_search("setting integer parameter ExposureTime to 15600"));

  ASSERT_TRUE(param_client->is_set_parameter("ExposureTimeMode"));
  ASSERT_EQ(param_client->get_parameters({"ExposureTimeMode"}).front().as_int(), 1);
  ASSERT_TRUE(param_client->is_set_parameter("ExposureTime"));
  ASSERT_EQ(param_client->get_parameters({"ExposureTime"}).front().as_int(), 15600);
}

TEST_F(ParamTest, override_default_set_exposure)
{
  instantiate_camera({});

  ASSERT_TRUE(param_client->is_set_parameter("ExposureTimeMode"));
  ASSERT_FALSE(param_client->is_set_parameter("ExposureTime"));

  // by default, auto exposure is active
  ASSERT_EQ(param_client->get_parameters({"ExposureTimeMode"}).front().as_int(), 0);

  // setting 'ExposureTime' with 'ExposureTimeMode' enabled by default causes conflict
  const int exp_tar1 = exp_init + 100;
  const std::vector<rcl_interfaces::msg::SetParametersResult> res_exposure =
    param_client->set_parameters({{"ExposureTime", exp_tar1}});
  ASSERT_FALSE(res_exposure[0].successful);
  ASSERT_EQ(res_exposure[0].reason, "ExposureTimeMode and ExposureTime must not be set simultaneously");
  // parameter updates are not applied
  ASSERT_FALSE(param_client->is_set_parameter("ExposureTime"));
}

TEST_F(ParamTest, override_ae_disabled_set_exposure)
{
  // set 'ExposureTimeMode' to manual
  instantiate_camera({{"ExposureTimeMode", 1}});

  ASSERT_TRUE(param_client->is_set_parameter("ExposureTimeMode"));
  ASSERT_TRUE(param_client->is_set_parameter("ExposureTime"));

  // 'ExposureTimeMode' takes the override value
  ASSERT_EQ(param_client->get_parameters({"ExposureTimeMode"}).front().as_int(), 1);

  // setting 'ExposureTime' does not cause conflict and is applied
  const int exp_tar1 = exp_init + 100;
  const std::vector<rcl_interfaces::msg::SetParametersResult> res_exposure =
    param_client->set_parameters({{"ExposureTime", exp_tar1}});
  ASSERT_TRUE(res_exposure[0].successful);
  ASSERT_EQ(res_exposure[0].reason, std::string {});
  // parameter updates are applied
  ASSERT_EQ(param_client->get_parameters({"ExposureTime"}).front().as_int(), exp_tar1);
}

TEST_F(ParamTest, override_ae_disabled_set_atom_ae_enabled_exposure)
{
  // set 'ExposureTimeMode' to manual
  instantiate_camera({{"ExposureTimeMode", 1}});

  ASSERT_TRUE(param_client->is_set_parameter("ExposureTimeMode"));
  ASSERT_TRUE(param_client->is_set_parameter("ExposureTime"));

  // setting 'ExposureTimeMode' and 'ExposureTime' at once will fail
  // no parameter updates are applied
  const std::vector<rclcpp::Parameter> initial_param_vals =
    param_client->get_parameters({"ExposureTimeMode", "ExposureTime"});
  const int exp_tar1 = exp_init + 100;
  const rcl_interfaces::msg::SetParametersResult res_atom =
    param_client->set_parameters_atomically({{"ExposureTimeMode", 0}, {"ExposureTime", exp_tar1}});
  ASSERT_FALSE(res_atom.successful);
  ASSERT_EQ(res_atom.reason, conflict_reason);
  // parameters do not change
  ASSERT_EQ(initial_param_vals, param_client->get_parameters({"ExposureTimeMode", "ExposureTime"}));
}

TEST_F(ParamTest, override_ae_disabled_set_atom_exposure_ae_enabled)
{
  // set 'ExposureTimeMode' to manual
  instantiate_camera({{"ExposureTimeMode", 1}});

  ASSERT_TRUE(param_client->is_set_parameter("ExposureTimeMode"));
  ASSERT_TRUE(param_client->is_set_parameter("ExposureTime"));

  // setting 'ExposureTime' and 'ExposureTimeMode' at once will fail
  // no parameter updates are applied
  const std::vector<rclcpp::Parameter> initial_param_vals =
    param_client->get_parameters({"ExposureTimeMode", "ExposureTime"});
  const int exp_tar1 = exp_init + 100;
  const rcl_interfaces::msg::SetParametersResult res_atom =
    param_client->set_parameters_atomically({{"ExposureTime", exp_tar1}, {"ExposureTimeMode", 0}});
  ASSERT_FALSE(res_atom.successful);
  ASSERT_EQ(res_atom.reason, conflict_reason);
  // parameters do not change
  ASSERT_EQ(initial_param_vals, param_client->get_parameters({"ExposureTimeMode", "ExposureTime"}));
}

TEST_F(ParamTest, override_ae_disabled_set_indiv_ae_enabled_exposure)
{
  // set 'ExposureTimeMode' to manual
  instantiate_camera({{"ExposureTimeMode", 1}});

  ASSERT_TRUE(param_client->is_set_parameter("ExposureTimeMode"));
  ASSERT_TRUE(param_client->is_set_parameter("ExposureTime"));

  // setting 'ExposureTimeMode' and 'ExposureTime' individually one-by-one will fail evenetually
  const int exp_tar1 = exp_init + 100;
  const std::vector<rcl_interfaces::msg::SetParametersResult> res_indiv =
    param_client->set_parameters({{"ExposureTimeMode", 0}, {"ExposureTime", exp_tar1}});
  // first parameter 'ExposureTimeMode' does not cause conflicts
  ASSERT_TRUE(res_indiv[0].successful);
  ASSERT_EQ(res_indiv[0].reason, std::string {});
  // second parameter 'ExposureTime' causes conflict with previous 'ExposureTimeMode'
  ASSERT_FALSE(res_indiv[1].successful);
  ASSERT_EQ(res_indiv[1].reason, "parameter 'ExposureTime' cannot be set because it was not declared");
  // only the parameter update for 'ExposureTimeMode' will have been applied
  ASSERT_EQ(param_client->get_parameters({"ExposureTimeMode"}).front().as_int(), 0);
  ASSERT_FALSE(param_client->is_set_parameter("ExposureTime"));

  // setting 'ExposureTime' again will fail since 'ExposureTimeMode' has already been applied
  const int exp_tar2 = exp_init + 200;
  const std::vector<rcl_interfaces::msg::SetParametersResult> res_exposure =
    param_client->set_parameters({{"ExposureTime", exp_tar2}});
  ASSERT_FALSE(res_exposure[0].successful);
  ASSERT_EQ(res_exposure[0].reason, "parameter 'ExposureTime' cannot be set because it was not declared");
  ASSERT_FALSE(param_client->is_set_parameter("ExposureTime"));
}

TEST_F(ParamTest, override_ae_disabled_set_indiv_exposure_ae_enabled)
{
  // set 'ExposureTimeMode' to manual
  instantiate_camera({{"ExposureTimeMode", 1}});

  ASSERT_TRUE(param_client->is_set_parameter("ExposureTimeMode"));
  ASSERT_TRUE(param_client->is_set_parameter("ExposureTime"));

  // setting 'ExposureTime' and 'ExposureTimeMode' individually one-by-one will fail evenetually
  const int exp_tar1 = exp_init + 100;
  const std::vector<rcl_interfaces::msg::SetParametersResult> res_indiv =
    param_client->set_parameters({{"ExposureTime", exp_tar1}, {"ExposureTimeMode", 0}});
  // first parameter 'ExposureTime' does not cause conflicts
  ASSERT_TRUE(res_indiv[0].successful);
  ASSERT_EQ(res_indiv[0].reason, std::string {});
  // second parameter 'ExposureTimeMode' takes precedence over previous 'ExposureTime'
  ASSERT_TRUE(res_indiv[1].successful);
  ASSERT_EQ(res_indiv[1].reason, std::string {});
  // both parameters will have been updated individually, but 'ExposureTime' will have been unset due to 'ExposureTimeMode'
  ASSERT_FALSE(param_client->is_set_parameter("ExposureTime"));
  ASSERT_EQ(param_client->get_parameters({"ExposureTimeMode"}).front().as_int(), 0);
}

TEST_F(ParamTest, override_ae_disabled_restore_exposure)
{
  instantiate_camera({{"ExposureTimeMode", 1}});

  ASSERT_TRUE(param_client->is_set_parameter("ExposureTimeMode"));
  ASSERT_EQ(param_client->get_parameters({"ExposureTimeMode"}).front().as_int(), 1);

  // 'ExposureTime' has been restored
  ASSERT_TRUE(param_client->is_set_parameter("ExposureTime"));
}

TEST_F(ParamTest, override_default_restore_exposure)
{
  instantiate_camera({});

  ASSERT_TRUE(param_client->is_set_parameter("ExposureTimeMode"));
  ASSERT_EQ(param_client->get_parameters({"ExposureTimeMode"}).front().as_int(), 0);
  ASSERT_FALSE(param_client->is_set_parameter("ExposureTime"));

  spin_all();

  // expect: declare 'ExposureTimeMode' and 'ExposureTime'
  ASSERT_TRUE(log_client->regex_search(declare_ExposureTimeMode));
  ASSERT_TRUE(log_client->regex_search(declare_ExposureTime));

  // expect: only 'ExposureTimeMode' is set to override value
  ASSERT_TRUE(log_client->regex_search("setting integer parameter ExposureTimeMode to 0"));
  ASSERT_FALSE(log_client->regex_search("setting integer parameter ExposureTime to (.*)"));

  const std::vector<rcl_interfaces::msg::SetParametersResult> res_indiv =
    param_client->set_parameters({{"ExposureTimeMode", 1}});
  ASSERT_TRUE(res_indiv[0].successful);
  ASSERT_EQ(res_indiv[0].reason, std::string {});

  // 'ExposureTimeMode' applied
  ASSERT_TRUE(param_client->is_set_parameter("ExposureTimeMode"));
  ASSERT_EQ(param_client->get_parameters({"ExposureTimeMode"}).front().as_int(), 1);

  // 'ExposureTime' has been restored
  ASSERT_TRUE(param_client->is_set_parameter("ExposureTime"));
  ASSERT_TRUE(log_client->regex_search("setting integer parameter ExposureTime to (.*)"));
}

TEST_F(ParamTest, override_default_param_after_ae)
{
  instantiate_camera({});

  ASSERT_TRUE(param_client->is_set_parameter("ExposureTimeMode"));
  ASSERT_EQ(param_client->get_parameters({"ExposureTimeMode"}).front().as_int(), 0);
  ASSERT_FALSE(param_client->is_set_parameter("ExposureTime"));

  set_check_all_successful({{"ExposureTimeMode", 1}});
  ASSERT_TRUE(param_client->has_parameter("ExposureTime"));
  set_check_all_successful({{"Contrast", 2.0}});
  set_check_all_successful({{"ExposureTimeMode", 0}});
  // wait for request allback to reset old parameters
  wait_image();
  ASSERT_TRUE(param_client->has_parameter("ExposureTime"));
  set_check_all_successful({{"Contrast", 4.0}});
}

int
main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  const int rc = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return rc;
}
