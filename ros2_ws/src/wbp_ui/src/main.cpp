#include <rclcpp/rclcpp.hpp>
#include <QApplication>

#include "wbp_ui/main_window.hpp"
#include "wbp_ui/ui_node.hpp"

int main(int argc, char** argv)
{
    // Qt must see argc/argv
    QApplication app(argc, argv);

    // ROS init AFTER QApplication construction
    rclcpp::init(argc, argv);

    auto window = std::make_shared<wbp_ui::MainWindow>();
    auto node = std::make_shared<wbp_ui::UiNode>(window.get());


    window->show();

    std::thread ros_spin([&]() {
        rclcpp::spin(node);
    });

    int ret = app.exec();

    rclcpp::shutdown();
    ros_spin.join();

    return ret;
}
