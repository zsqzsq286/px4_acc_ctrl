#include "my_offboard_control/velocity_tracking.hpp"
#include <algorithm>
#include <rclcpp/qos.hpp>
#include <cmath>
#include <chrono>
#include <limits>

using namespace std::chrono_literals;
using namespace px4_msgs::msg;

namespace my_offboard_control {

VelocityTracking::VelocityTracking(
    const rclcpp::NodeOptions& options,
    std::shared_ptr<std::atomic<float>> rc_aux1,
    std::shared_ptr<std::atomic<bool>> rc_valid,
    std::shared_ptr<std::atomic<uint64_t>> rc_timestamp)
    : Node("unified_guidance_node", options),  // 节点名称与 Launch 文件中的 name 匹配，确保参数正确传递
      rc_aux1_(rc_aux1),
      rc_valid_(rc_valid),
      rc_timestamp_(rc_timestamp)
	{
		// 声明参数：是否使用PNG制导节点提供的速度
    this->declare_parameter<bool>("use_guidance_input", true);
    this->declare_parameter<double>("fixed_velocity_x", 1.0);
    this->declare_parameter<double>("fixed_velocity_y", 1.0);
    this->declare_parameter<double>("fixed_velocity_z", 0.0);
    this->declare_parameter<int>("px4_instance_id", 0);
    
    // 速度斜坡上升参数（避免初始时刻剧烈俯仰）
    this->declare_parameter<double>("velocity_ramp_duration", 1.0);
    this->declare_parameter<double>("control_rate_hz", 10.0);
    this->declare_parameter<bool>("auto_yaw_to_velocity", true);  // 默认启用 LOS 方向对准
		
    // 像素伺服偏航控制参数（已禁用，默认使用 LOS 模式）
    this->declare_parameter<bool>("pixel_yaw_control", false);  // 默认禁用像素伺服模式
    this->declare_parameter<double>("image_center_x", 320.0);
    this->declare_parameter<double>("image_center_y", 240.0);
    this->declare_parameter<double>("yaw_kp", 0.001);
    this->declare_parameter<double>("yaw_deadzone", 20.0);
    this->declare_parameter<double>("max_yaw_rate", 0.2);
    this->declare_parameter<double>("los_yaw_smoothing", 0.3);
    
    // EWMA异常检测参数（用于像素坐标）
    this->declare_parameter<bool>("enable_pixel_anomaly_detection", true);
    this->declare_parameter<double>("pixel_ewma_alpha", 0.8);
    this->declare_parameter<double>("pixel_anomaly_threshold", 3.0);
    this->declare_parameter<double>("pixel_initial_std", 10.0);
    this->declare_parameter<int>("pixel_max_reject_count", 5);
    
    // 边缘视场保护：动态调整偏航角平滑系数
    this->declare_parameter<bool>("enable_dynamic_yaw_smoothing", true);
    this->declare_parameter<double>("safe_zone_radius", 270.0);
    this->declare_parameter<double>("transition_width", 50.0);
    this->declare_parameter<double>("min_smoothing", 0.08);
    this->declare_parameter<double>("max_smoothing", 0.50);
    this->declare_parameter<bool>("enable_control_in_stopped", false);  // Stopped状态下是否发布控制（默认false，避免与其他节点冲突）

		use_guidance_input_ = this->get_parameter("use_guidance_input").as_bool();
		fixed_vel_x_ = this->get_parameter("fixed_velocity_x").as_double();
		fixed_vel_y_ = this->get_parameter("fixed_velocity_y").as_double();
		fixed_vel_z_ = this->get_parameter("fixed_velocity_z").as_double();
		px4_instance_id_ = this->get_parameter("px4_instance_id").as_int();
		auto_yaw_to_velocity_ = this->get_parameter("auto_yaw_to_velocity").as_bool();
    
    // 速度斜坡上升参数
    velocity_ramp_duration_ = this->get_parameter("velocity_ramp_duration").as_double();
    control_rate_hz_ = this->get_parameter("control_rate_hz").as_double();
    if (!std::isfinite(control_rate_hz_) || control_rate_hz_ <= 0.0) {
        RCLCPP_WARN(this->get_logger(),
            "Invalid control_rate_hz=%.3f, fallback to 10.0 Hz", control_rate_hz_);
        control_rate_hz_ = 10.0;
    }
    control_dt_ = 1.0 / control_rate_hz_;
    int ticks_1s = std::max(1, static_cast<int>(std::lround(control_rate_hz_)));
    log_interval_ticks_ = ticks_1s;
    offboard_switch_ticks_ = ticks_1s;
		
		// 像素伺服偏航控制参数
		pixel_yaw_control_ = this->get_parameter("pixel_yaw_control").as_bool();
		image_center_x_ = this->get_parameter("image_center_x").as_double();
		image_center_y_ = this->get_parameter("image_center_y").as_double();
		yaw_kp_ = this->get_parameter("yaw_kp").as_double();
		yaw_deadzone_ = this->get_parameter("yaw_deadzone").as_double();
    max_yaw_rate_ = this->get_parameter("max_yaw_rate").as_double();
    los_yaw_smoothing_ = this->get_parameter("los_yaw_smoothing").as_double();
    
    // 边缘视场保护：动态调整偏航角平滑系数
    enable_dynamic_yaw_smoothing_ = this->get_parameter("enable_dynamic_yaw_smoothing").as_bool();
    safe_zone_radius_ = this->get_parameter("safe_zone_radius").as_double();
    transition_width_ = this->get_parameter("transition_width").as_double();
    min_smoothing_ = this->get_parameter("min_smoothing").as_double();
    max_smoothing_ = this->get_parameter("max_smoothing").as_double();
    enable_control_in_stopped_ = this->get_parameter("enable_control_in_stopped").as_bool();
    
    // 获取EWMA异常检测参数
    enable_pixel_anomaly_detection_ = this->get_parameter("enable_pixel_anomaly_detection").as_bool();
    pixel_ewma_alpha_ = this->get_parameter("pixel_ewma_alpha").as_double();
    pixel_anomaly_threshold_ = this->get_parameter("pixel_anomaly_threshold").as_double();
    pixel_initial_std_ = this->get_parameter("pixel_initial_std").as_double();
    pixel_max_reject_count_ = this->get_parameter("pixel_max_reject_count").as_int();
    
    // 初始化EWMA模型状态
    pixel_x_model_initialized_ = false;
    pixel_y_model_initialized_ = false;
    filtered_pixel_x_ = 0.0;
    filtered_pixel_y_ = 0.0;
    pixel_x_variance_ = pixel_initial_std_ * pixel_initial_std_;
    pixel_y_variance_ = pixel_initial_std_ * pixel_initial_std_;
    pixel_x_reject_count_ = 0;
    pixel_y_reject_count_ = 0;
		
		// 初始化偏航角
		current_yaw_ = 0.0;
    los_yaw_initialized_ = false;
    yaw_initialized_from_odometry_ = false;
    
    // 初始化速度斜坡上升状态
    guidance_ramp_active_ = false;
    
    // 初始化odometry速度（用于其他用途）
    current_odometry_vel_x_.store(0.0);
    current_odometry_vel_y_.store(0.0);
    current_odometry_vel_z_.store(0.0);

    // 初始化 RC 控制状态
    rc_mode_ = RcMode::Stopped;
    offboard_ready_ = false;
    in_offboard_mode_.store(false);
    current_position_.valid = false;

    // 输出EWMA异常检测配置
    if (enable_pixel_anomaly_detection_) {
        RCLCPP_INFO(this->get_logger(), "Pixel anomaly detection ENABLED (EWMA): α=%.2f, threshold=%.1fσ, initial_σ=%.1f px, max_reject=%d",
            pixel_ewma_alpha_, pixel_anomaly_threshold_, pixel_initial_std_, pixel_max_reject_count_);
    } else {
        RCLCPP_INFO(this->get_logger(), "Pixel anomaly detection DISABLED");
    }

    // 输出动态偏航平滑参数验证
    RCLCPP_INFO(this->get_logger(), 
        "Dynamic yaw smoothing parameters: enabled=%s, min_smoothing=%.3f, max_smoothing=%.3f, safe_zone=%.1f px, transition=%.1f px, los_yaw_smoothing=%.3f (fallback)",
        enable_dynamic_yaw_smoothing_ ? "true" : "false",
        min_smoothing_, max_smoothing_, 
        safe_zone_radius_, transition_width_,
        los_yaw_smoothing_);
		
		// 确定偏航控制模式
		std::string yaw_mode;
		if (pixel_yaw_control_) {
			yaw_mode = "Pixel servo (target centering)";
		} else if (auto_yaw_to_velocity_) {
			yaw_mode = "Auto-align to LOS";
		} else {
			yaw_mode = "Free (no yaw control)";
		}
		RCLCPP_INFO(this->get_logger(), "Yaw control mode: %s", yaw_mode.c_str());

		// 配置QoS - PX4使用BEST_EFFORT可靠性和TRANSIENT_LOCAL持久性
		rclcpp::QoS qos_profile(10);
		qos_profile.reliability(rclcpp::ReliabilityPolicy::BestEffort);
		qos_profile.durability(rclcpp::DurabilityPolicy::TransientLocal);
		qos_profile.history(rclcpp::HistoryPolicy::KeepLast);

		// 根据实例ID构建话题名称前缀
		std::string topic_prefix;
		if (px4_instance_id_ == 0) {
			topic_prefix = "/fmu";
		} else {
			topic_prefix = "/px4_" + std::to_string(px4_instance_id_) + "/fmu";
		}

		// 创建发布者（使用与PX4兼容的QoS）
		offboard_control_mode_publisher_ = this->create_publisher<OffboardControlMode>(
			topic_prefix + "/in/offboard_control_mode", qos_profile);
		trajectory_setpoint_publisher_ = this->create_publisher<TrajectorySetpoint>(
			topic_prefix + "/in/trajectory_setpoint", qos_profile);
		vehicle_command_publisher_ = this->create_publisher<VehicleCommand>(
			topic_prefix + "/in/vehicle_command", qos_profile);

    // 订阅 vehicle_control_mode（确认 offboard 状态）
    control_mode_subscriber_ = this->create_subscription<VehicleControlMode>(
        topic_prefix + "/out/vehicle_control_mode", qos_profile,
        std::bind(&VelocityTracking::control_mode_callback, this, std::placeholders::_1));

    // 订阅 odometry（获取当前位置和偏航角，用于位置保持和偏航控制）
    odometry_subscriber_ = this->create_subscription<VehicleOdometry>(
        topic_prefix + "/out/vehicle_odometry", qos_profile,
        std::bind(&VelocityTracking::odometry_callback, this, std::placeholders::_1));

    // 如果使用制导输入，订阅期望速度话题（相对话题名，会自动加上namespace）
		if (use_guidance_input_) {
			velocity_setpoint_subscriber_ = this->create_subscription<geometry_msgs::msg::TwistStamped>(
				"guidance/velocity_setpoint", 10,
				std::bind(&VelocityTracking::velocity_setpoint_callback, this, std::placeholders::_1));
			velocity_setpoint_received_ = false;
			RCLCPP_INFO(this->get_logger(), "Velocity tracking node: Using PNG guidance input");
		}
		
		// 像素伺服偏航控制：订阅像素坐标
		// 使用绝对话题名 /target/pixel，确保在namespace下也能正确订阅根namespace的话题
		if (pixel_yaw_control_) {
			pixel_subscriber_ = this->create_subscription<geometry_msgs::msg::Point>(
				"/target/pixel", 10,
				std::bind(&VelocityTracking::pixel_callback, this, std::placeholders::_1));
			pixel_received_ = false;
			RCLCPP_INFO(this->get_logger(), "Pixel yaw control ENABLED. Image center: (%.1f, %.1f), Kp=%.4f", 
				image_center_x_, image_center_y_, yaw_kp_);
		} else if (auto_yaw_to_velocity_) {
			// LOS偏航控制：订阅视线向量
			// 使用绝对话题名 /target/los_vector，确保在namespace下也能正确订阅根namespace的话题
			los_vector_subscriber_ = this->create_subscription<geometry_msgs::msg::Vector3Stamped>(
				"/target/los_vector", 10,
				std::bind(&VelocityTracking::los_vector_callback, this, std::placeholders::_1));
			los_received_ = false;
			los_x_ = 1.0;  // 初始化：默认朝前
			los_y_ = 0.0;
			los_z_ = 0.0;
			RCLCPP_INFO(this->get_logger(), "Subscribing to /target/los_vector for LOS yaw control");
        
        // 如果启用动态平滑，也需要订阅像素数据用于边缘保护
        // 使用绝对话题名 /target/pixel，确保在namespace下也能正确订阅根namespace的话题
        if (enable_dynamic_yaw_smoothing_) {
            pixel_subscriber_ = this->create_subscription<geometry_msgs::msg::Point>(
                "/target/pixel", 10,
                std::bind(&VelocityTracking::pixel_callback, this, std::placeholders::_1));
            pixel_received_ = false;
            RCLCPP_INFO(this->get_logger(), "Pixel subscription enabled for dynamic yaw smoothing (edge protection)");
        }
		} else {
			RCLCPP_INFO(this->get_logger(), "Velocity tracking node: Using fixed velocity [%.2f, %.2f, %.2f] m/s",
				fixed_vel_x_, fixed_vel_y_, fixed_vel_z_);
		}

		RCLCPP_INFO(this->get_logger(), "Velocity tracking node: Controlling PX4 instance %d (topic prefix: %s)",
			px4_instance_id_, (px4_instance_id_ == 0 ? "/fmu" : ("/px4_" + std::to_string(px4_instance_id_) + "/fmu")).c_str());
	RCLCPP_INFO(this->get_logger(), "Auto yaw to velocity: %s", auto_yaw_to_velocity_ ? "ENABLED" : "DISABLED");

		offboard_setpoint_counter_ = 0;

		auto timer_callback = [this]() -> void {
        // 更新 RC 状态机（由遥控开关控制 offboard 切换）
        update_rc_state_machine();

        // 发布控制指令
			publish_offboard_control_mode();
			publish_trajectory_setpoint();
		};
        auto period_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::duration<double>(control_dt_));
		timer_ = this->create_wall_timer(period_ns, timer_callback);
        RCLCPP_INFO(this->get_logger(),
            "Control loop: %.1f Hz (dt=%.4f s), offboard_switch=%d ticks, log_interval=%d ticks",
            control_rate_hz_, control_dt_, offboard_switch_ticks_, log_interval_ticks_);
	}

void VelocityTracking::velocity_setpoint_callback(const geometry_msgs::msg::TwistStamped::SharedPtr msg)
{
	// 使用atomic变量的store方法保证线程安全
    double received_x = msg->twist.linear.x;
    double received_y = msg->twist.linear.y;
    double received_z = msg->twist.linear.z;
    
    current_vel_x_.store(received_x);
    current_vel_y_.store(received_y);
    current_vel_z_.store(received_z);
	
	if (!velocity_setpoint_received_) {
        double magnitude = std::sqrt(received_x*received_x + received_y*received_y + received_z*received_z);
        RCLCPP_INFO(this->get_logger(), 
            "[VELOCITY_SETPOINT] Received FIRST velocity setpoint: [%.2f, %.2f, %.2f] m/s | magnitude=%.2f m/s",
            received_x, received_y, received_z, magnitude);
		velocity_setpoint_received_ = true;
	} else {
        // 定期输出接收到的速度指令（用于调试）
        static int log_counter = 0;
        if (log_counter % 50 == 0) {  // 每秒1次（假设50Hz）
            double magnitude = std::sqrt(received_x*received_x + received_y*received_y + received_z*received_z);
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                "[VELOCITY_SETPOINT] Received velocity: [%.2f, %.2f, %.2f] m/s | magnitude=%.2f m/s | RC_mode=%d",
                received_x, received_y, received_z, magnitude, static_cast<int>(rc_mode_));
        }
        log_counter++;
    }
    
}

void VelocityTracking::los_vector_callback(const geometry_msgs::msg::Vector3Stamped::SharedPtr msg)
{
	// 接收并存储LOS单位向量
	los_x_.store(msg->vector.x);
	los_y_.store(msg->vector.y);
	los_z_.store(msg->vector.z);
	
	if (!los_received_) {
		RCLCPP_INFO(this->get_logger(), "Received first LOS vector from /target/los_vector");
		los_received_ = true;
	}
}

void VelocityTracking::pixel_callback(const geometry_msgs::msg::Point::SharedPtr msg)
{
    // 读取原始像素坐标
    double x_raw = msg->x;
    double y_raw = msg->y;
    
    // EWMA异常检测：动态高斯建模和假设检验
    double x = x_raw;
    double y = y_raw;
    
    if (enable_pixel_anomaly_detection_) {
        // X坐标异常检测
        if (!pixel_x_model_initialized_) {
            // 首次接收：直接初始化模型
            filtered_pixel_x_ = x;
            pixel_x_variance_ = pixel_initial_std_ * pixel_initial_std_;
            pixel_x_model_initialized_ = true;
            RCLCPP_DEBUG(this->get_logger(), "Pixel X model initialized: μ=%.1f, σ=%.1f", 
                filtered_pixel_x_, std::sqrt(pixel_x_variance_));
        } else {
            // 假设检验：检查是否在3σ范围内
            double x_std = std::sqrt(pixel_x_variance_);
            double x_deviation = std::abs(x - filtered_pixel_x_);
            double x_threshold = pixel_anomaly_threshold_ * x_std;
            
            if (x_deviation <= x_threshold) {
                // 正常点：接受并更新模型
                filtered_pixel_x_ = pixel_ewma_alpha_ * filtered_pixel_x_ + (1.0 - pixel_ewma_alpha_) * x;
                double residual = x - filtered_pixel_x_;
                pixel_x_variance_ = pixel_ewma_alpha_ * pixel_x_variance_ + 
                                    (1.0 - pixel_ewma_alpha_) * residual * residual;
                
                double min_variance = 1.0;
                if (pixel_x_variance_ < min_variance) {
                    pixel_x_variance_ = min_variance;
                }
                pixel_x_reject_count_ = 0;
            } else {
                // 异常点：增加拒绝计数
                pixel_x_reject_count_++;
                
                // 检查是否达到最大拒绝次数
                if (pixel_x_reject_count_ >= pixel_max_reject_count_) {
                    // 强制更新模型（承认现实，避免死锁）
                    filtered_pixel_x_ = pixel_ewma_alpha_ * filtered_pixel_x_ + (1.0 - pixel_ewma_alpha_) * x;
                    double residual = x - filtered_pixel_x_;
                    pixel_x_variance_ = pixel_ewma_alpha_ * pixel_x_variance_ + 
                                        (1.0 - pixel_ewma_alpha_) * residual * residual;
                    double min_variance = 1.0;
                    if (pixel_x_variance_ < min_variance) {
                        pixel_x_variance_ = min_variance;
                    }
                    pixel_x_reject_count_ = 0;
                    
                    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                        "Pixel X: %d consecutive rejects, forcing model update (raw=%.1f -> model=%.1f)",
                        pixel_max_reject_count_, x_raw, filtered_pixel_x_);
                } else {
                    // 拒绝，使用模型预测值
                    x = filtered_pixel_x_;
                    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                        "Pixel X anomaly detected: raw=%.1f, model=%.1f±%.1f (dev=%.1f > %.1f), reject_count=%d/%d, using model value",
                        x_raw, filtered_pixel_x_, x_std, x_deviation, x_threshold, 
                        pixel_x_reject_count_, pixel_max_reject_count_);
                }
            }
        }
        
        // Y坐标异常检测（同样的逻辑）
        if (!pixel_y_model_initialized_) {
            filtered_pixel_y_ = y;
            pixel_y_variance_ = pixel_initial_std_ * pixel_initial_std_;
            pixel_y_model_initialized_ = true;
            RCLCPP_DEBUG(this->get_logger(), "Pixel Y model initialized: μ=%.1f, σ=%.1f", 
                filtered_pixel_y_, std::sqrt(pixel_y_variance_));
        } else {
            double y_std = std::sqrt(pixel_y_variance_);
            double y_deviation = std::abs(y - filtered_pixel_y_);
            double y_threshold = pixel_anomaly_threshold_ * y_std;
            
            if (y_deviation <= y_threshold) {
                // 正常点：接受并更新模型
                filtered_pixel_y_ = pixel_ewma_alpha_ * filtered_pixel_y_ + (1.0 - pixel_ewma_alpha_) * y;
                double residual = y - filtered_pixel_y_;
                pixel_y_variance_ = pixel_ewma_alpha_ * pixel_y_variance_ + 
                                    (1.0 - pixel_ewma_alpha_) * residual * residual;
                
                double min_variance = 1.0;
                if (pixel_y_variance_ < min_variance) {
                    pixel_y_variance_ = min_variance;
                }
                pixel_y_reject_count_ = 0;
            } else {
                // 异常点：增加拒绝计数
                pixel_y_reject_count_++;
                
                // 检查是否达到最大拒绝次数
                if (pixel_y_reject_count_ >= pixel_max_reject_count_) {
                    // 强制更新模型（承认现实，避免死锁）
                    filtered_pixel_y_ = pixel_ewma_alpha_ * filtered_pixel_y_ + (1.0 - pixel_ewma_alpha_) * y;
                    double residual = y - filtered_pixel_y_;
                    pixel_y_variance_ = pixel_ewma_alpha_ * pixel_y_variance_ + 
                                        (1.0 - pixel_ewma_alpha_) * residual * residual;
                    double min_variance = 1.0;
                    if (pixel_y_variance_ < min_variance) {
                        pixel_y_variance_ = min_variance;
                    }
                    pixel_y_reject_count_ = 0;
                    
                    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                        "Pixel Y: %d consecutive rejects, forcing model update (raw=%.1f -> model=%.1f)",
                        pixel_max_reject_count_, y_raw, filtered_pixel_y_);
                } else {
                    // 拒绝，使用模型预测值
                    y = filtered_pixel_y_;
                    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                        "Pixel Y anomaly detected: raw=%.1f, model=%.1f±%.1f (dev=%.1f > %.1f), reject_count=%d/%d, using model value",
                        y_raw, filtered_pixel_y_, y_std, y_deviation, y_threshold, 
                        pixel_y_reject_count_, pixel_max_reject_count_);
                }
            }
        }
    }
    
    // 存储处理后的像素坐标（正常点或模型预测值）
    pixel_x_.store(x);
    pixel_y_.store(y);
	
	if (!pixel_received_) {
		RCLCPP_INFO(this->get_logger(), "Received first pixel from /target/pixel");
		pixel_received_ = true;
	}
}

void VelocityTracking::odometry_callback(const VehicleOdometry::SharedPtr msg)
{
    // 缓存当前位置（用于位置保持）- 加锁保护
    {
        std::lock_guard<std::mutex> lock(position_mutex_);
        current_position_.x = msg->position[0];
        current_position_.y = msg->position[1];
        current_position_.z = msg->position[2];
        current_position_.valid = true;
    }
    
    // 更新当前odometry速度（用于初始化速度滤波器，避免从0跳变）
    current_odometry_vel_x_.store(msg->velocity[0]);
    current_odometry_vel_y_.store(msg->velocity[1]);
    current_odometry_vel_z_.store(msg->velocity[2]);

    // 从四元数提取偏航角进行初始化（适用于所有偏航控制模式）
    double qw = msg->q[0];
    double qx = msg->q[1];
    double qy = msg->q[2];
    double qz = msg->q[3];
    if (!yaw_initialized_from_odometry_) {
        // 从四元数提取偏航角
        double siny_cosp = 2.0 * (qw * qz + qx * qy);
        double cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz);
        double yaw = std::atan2(siny_cosp, cosy_cosp);

        // 初始化当前偏航角为实际偏航角
        current_yaw_ = yaw;
        yaw_initialized_from_odometry_ = true;
        
        // 根据偏航控制模式输出不同的日志
        if (pixel_yaw_control_) {
            RCLCPP_INFO(this->get_logger(), "Yaw initialized from odometry (Pixel servo mode): %.3f rad (%.1f°)",
                current_yaw_, current_yaw_ * 180.0 / M_PI);
        } else if (auto_yaw_to_velocity_) {
            RCLCPP_INFO(this->get_logger(), "Yaw initialized from odometry (LOS align mode): %.3f rad (%.1f°)",
                current_yaw_, current_yaw_ * 180.0 / M_PI);
        }
    }
}

void VelocityTracking::control_mode_callback(const VehicleControlMode::SharedPtr msg)
{
    bool was_offboard = in_offboard_mode_.load();
    in_offboard_mode_.store(msg->flag_control_offboard_enabled);

    // 状态变化时打印日志
    if (!was_offboard && msg->flag_control_offboard_enabled) {
        RCLCPP_INFO(this->get_logger(), "[RC_SWITCH] ✓ OFFBOARD mode confirmed");
    } else if (was_offboard && !msg->flag_control_offboard_enabled) {
        RCLCPP_WARN(this->get_logger(), "[RC_SWITCH] ✗ Exited OFFBOARD mode");
    }
}

void VelocityTracking::update_rc_state_machine()
{
    // 1. 读取 RC 信号（原子操作）
    float aux1 = rc_aux1_->load();

    // 2. 获取当前时间（用于超时检测）
    rclcpp::Time current_time = this->get_clock()->now();

    // 3. 状态机逻辑
    switch (rc_mode_) {
        case RcMode::Stopped:
            // 停止状态 → 启动条件：aux1 > 0.5
            if (aux1 > 0.5) {
                RCLCPP_INFO(this->get_logger(),
                    "[RC_SWITCH] aux1=%.3f > 0.5 → Start switching to OFFBOARD", aux1);

                // 检查是否已经在 offboard 模式
                if (in_offboard_mode_.load()) {
                    RCLCPP_INFO(this->get_logger(),
                        "[RC_SWITCH] Already in OFFBOARD mode → Direct to GUIDANCE");
                    rc_mode_ = RcMode::Guidance;
                    offboard_ready_ = true;
                } else {
                    rc_mode_ = RcMode::SwitchingOffboard;
                    offboard_switch_start_time_ = current_time;
                    offboard_ready_ = false;
                    RCLCPP_INFO(this->get_logger(),
                        "[RC_SWITCH] State: STOPPED → SWITCHING_OFFBOARD (timeout=5s)");
                }
            }
            break;

        case RcMode::SwitchingOffboard:
            // 检查超时（5秒）
            if ((current_time - offboard_switch_start_time_).seconds() > 5.0) {
                RCLCPP_ERROR(this->get_logger(),
                    "[RC_SWITCH] OFFBOARD switch timeout (5s) → Abort, return to STOP");
                rc_mode_ = RcMode::Stopped;
                break;
            }

             // 检查是否成功切换到 offboard
			 if (in_offboard_mode_.load()) {
                RCLCPP_INFO(this->get_logger(),
                    "[RC_SWITCH] OFFBOARD confirmed → Start GUIDANCE");
                rc_mode_ = RcMode::Guidance;
                offboard_ready_ = true;
            }

            // 强制停止：aux1 < -0.5
            if (aux1 < -0.5) {
                RCLCPP_WARN(this->get_logger(),
                    "[RC_SWITCH] aux1=%.3f < -0.5 → Force STOP during switching", aux1);
                rc_mode_ = RcMode::Stopped;
                offboard_ready_ = false;
            }
            break;

        case RcMode::Guidance:
            // 制导模式 → 停止条件：aux1 < -0.5
            if (aux1 < -0.5) {
                RCLCPP_INFO(this->get_logger(),
                    "[RC_SWITCH] aux1=%.3f < -0.5 → STOP (position hold)", aux1);
                rc_mode_ = RcMode::Stopped;
                offboard_ready_ = false;
                // 退出Guidance模式时，重置速度斜坡状态
                guidance_ramp_active_ = false;
            }
            break;
    }
}

void VelocityTracking::switch_to_offboard_with_position_hold()
{
    static int position_hold_counter = 0;

    if (rc_mode_ != RcMode::SwitchingOffboard) {
        position_hold_counter = 0;  // 重置计数器
        return;
    }

    // 每个 tick 计数
    position_hold_counter++;

    // 发送 offboard_switch_ticks_ 次位置保持 setpoint 后（约1秒），发送切换命令
    if (position_hold_counter >= offboard_switch_ticks_) {
        // 发送 OFFBOARD 模式切换命令
        publish_vehicle_command(VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6);
        RCLCPP_INFO(this->get_logger(),
            "[RC_SWITCH] Sending OFFBOARD mode command (after %d position setpoints)",
            offboard_switch_ticks_);
    }

    // 持续发送位置保持 setpoint（在 publish_trajectory_setpoint 中处理）
}

void VelocityTracking::arm()
{
	publish_vehicle_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0);
	RCLCPP_INFO(this->get_logger(), "Arm command send");
}

void VelocityTracking::disarm()
{
	publish_vehicle_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0);
	RCLCPP_INFO(this->get_logger(), "Disarm command send");
}

void VelocityTracking::publish_offboard_control_mode()
{
    // 如果处于Stopped状态且不允许发布控制，则不发布
    if (rc_mode_ == RcMode::Stopped && !enable_control_in_stopped_) {
        return;
    }
    
	OffboardControlMode msg{};

    // 根据状态决定控制模式
    if (rc_mode_ == RcMode::SwitchingOffboard || rc_mode_ == RcMode::Stopped) {
        // 切换阶段或停止状态：位置控制（位置保持）
	msg.position = true;
        msg.velocity = false;
    } else {  // RcMode::Guidance
        // 制导模式：速度控制
        msg.position = false;
        msg.velocity = true;
    }

	msg.acceleration = false;
	msg.attitude = false;
	msg.body_rate = false;
	msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
	offboard_control_mode_publisher_->publish(msg);
}

void VelocityTracking::publish_trajectory_setpoint()
{
    // 调用 offboard 切换函数（仅在 SwitchingOffboard 状态生效）
    switch_to_offboard_with_position_hold();

	TrajectorySetpoint msg{};
	
    // 根据 RC 状态决定输出
    switch (rc_mode_) {
        case RcMode::Stopped:
            {
                // 如果 enable_control_in_stopped 为 false，不发布控制（让其他节点控制）
                if (!enable_control_in_stopped_) {
                    // 不发布控制命令，直接返回
                    return;
                }
                
                // 停止状态：位置保持在当前位置 - 加锁读取位置
                double pos_x, pos_y, pos_z;
                bool pos_valid;
                {
                    std::lock_guard<std::mutex> lock(position_mutex_);
                    pos_x = current_position_.x;
                    pos_y = current_position_.y;
                    pos_z = current_position_.z;
                    pos_valid = current_position_.valid;
                }

                if (pos_valid) {
                    msg.position = {
                        static_cast<float>(pos_x),
                        static_cast<float>(pos_y),
                        static_cast<float>(pos_z)
                    };
                    msg.velocity = {0.0f, 0.0f, 0.0f};  // 同时设置速度为0（更稳定）

                    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                        "[RC_SWITCH] STOPPED - Position hold at [%.2f, %.2f, %.2f]",
                        pos_x, pos_y, pos_z);
                } else {
                    // 位置无效，仅发送速度=0
                    msg.position = {
                        std::numeric_limits<float>::quiet_NaN(),
                        std::numeric_limits<float>::quiet_NaN(),
                        std::numeric_limits<float>::quiet_NaN()
                    };
                    msg.velocity = {0.0f, 0.0f, 0.0f};

                    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                        "[RC_SWITCH] STOPPED - Position invalid, sending velocity=0");
                }
            }
            break;

        case RcMode::SwitchingOffboard:
            {
                // 切换阶段：发送当前位置（位置保持，满足 PX4 offboard 切换要求）- 加锁读取
                double pos_x, pos_y, pos_z;
                bool pos_valid;
                {
                    std::lock_guard<std::mutex> lock(position_mutex_);
                    pos_x = current_position_.x;
                    pos_y = current_position_.y;
                    pos_z = current_position_.z;
                    pos_valid = current_position_.valid;
                }

                if (pos_valid) {
                    msg.position = {
                        static_cast<float>(pos_x),
                        static_cast<float>(pos_y),
                        static_cast<float>(pos_z)
                    };
                    msg.velocity = {
                        std::numeric_limits<float>::quiet_NaN(),
                        std::numeric_limits<float>::quiet_NaN(),
                        std::numeric_limits<float>::quiet_NaN()
                    };

                    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                        "[RC_SWITCH] SWITCHING - Sending position hold at [%.2f, %.2f, %.2f]",
                        pos_x, pos_y, pos_z);
                } else {
                    RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                        "[RC_SWITCH] SWITCHING - Position invalid! Cannot switch to OFFBOARD");
                    // 放弃切换
                    rc_mode_ = RcMode::Stopped;
                }
            }
            break;

        case RcMode::Guidance:
            // 制导模式：制导速度（原有逻辑）
            msg.position = {
                std::numeric_limits<float>::quiet_NaN(),
	                std::numeric_limits<float>::quiet_NaN(), 
                std::numeric_limits<float>::quiet_NaN()
            };
	
            // 检测进入Guidance模式：如果之前不在Guidance模式，启动速度斜坡上升
            rclcpp::Time current_time = this->get_clock()->now();
            static RcMode last_rc_mode = RcMode::Stopped;
            if (last_rc_mode != RcMode::Guidance) {
                // 刚进入Guidance模式，启动速度斜坡上升
                guidance_start_time_ = current_time;
                guidance_ramp_active_ = true;
                RCLCPP_INFO(this->get_logger(), 
                    "[VELOCITY_RAMP] Starting velocity ramp-up, duration: %.2f s", 
                    velocity_ramp_duration_);
            }
            last_rc_mode = RcMode::Guidance;

            // 获取指令速度（制导节点已经进行了低通滤波）
            double desired_vel_x, desired_vel_y, desired_vel_z;
            if (use_guidance_input_ && velocity_setpoint_received_) {
                desired_vel_x = current_vel_x_.load();
                desired_vel_y = current_vel_y_.load();
                desired_vel_z = current_vel_z_.load();
                
                // 安全检查：确保读取到的速度值是有效的（非NaN且有限）
                if (!std::isfinite(desired_vel_x) || !std::isfinite(desired_vel_y) || !std::isfinite(desired_vel_z)) {
                    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                        "[VELOCITY] Invalid velocity received (NaN/Inf), using zero velocity");
                    desired_vel_x = 0.0;
                    desired_vel_y = 0.0;
                    desired_vel_z = 0.0;
                }
            } else if (!use_guidance_input_) {
                desired_vel_x = fixed_vel_x_;
                desired_vel_y = fixed_vel_y_;
                desired_vel_z = fixed_vel_z_;
            } else {
                // 没有收到制导速度，使用零速度（等待数据准备）
                desired_vel_x = 0.0;
                desired_vel_y = 0.0;
                desired_vel_z = 0.0;
                // 调试：定期输出等待状态
                static int wait_counter = 0;
                if (wait_counter % 100 == 0) {  // 每2秒一次
                    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                        "[VELOCITY] Waiting for velocity setpoint from guidance/velocity_setpoint (use_guidance_input=true, received=false)");
                }
                wait_counter++;
            }

            // 速度斜坡上升：从实际速度平滑过渡到指令速度
	double vel_x, vel_y, vel_z;
            if (guidance_ramp_active_) {
                double elapsed = (current_time - guidance_start_time_).seconds();
                
                if (elapsed < velocity_ramp_duration_) {
                    // 斜坡期间：线性插值从实际速度到指令速度
                    double alpha = elapsed / velocity_ramp_duration_;  // 0 -> 1
                    
                    // 获取当前实际速度（从odometry）
                    double actual_vel_x = current_odometry_vel_x_.load();
                    double actual_vel_y = current_odometry_vel_y_.load();
                    double actual_vel_z = current_odometry_vel_z_.load();
                    
                    // 线性插值
                    vel_x = actual_vel_x * (1.0 - alpha) + desired_vel_x * alpha;
                    vel_y = actual_vel_y * (1.0 - alpha) + desired_vel_y * alpha;
                    vel_z = actual_vel_z * (1.0 - alpha) + desired_vel_z * alpha;
                    
                    RCLCPP_DEBUG_THROTTLE(this->get_logger(), *this->get_clock(), 500,
                        "[VELOCITY_RAMP] Ramping: elapsed=%.2f s, alpha=%.2f, "
                        "actual=[%.2f, %.2f, %.2f] -> desired=[%.2f, %.2f, %.2f] -> output=[%.2f, %.2f, %.2f]",
                        elapsed, alpha,
                        actual_vel_x, actual_vel_y, actual_vel_z,
                        desired_vel_x, desired_vel_y, desired_vel_z,
                        vel_x, vel_y, vel_z);
		} else {
                    // 斜坡结束，直接使用指令速度
                    guidance_ramp_active_ = false;
                    vel_x = desired_vel_x;
                    vel_y = desired_vel_y;
                    vel_z = desired_vel_z;
                    RCLCPP_INFO(this->get_logger(), 
                        "[VELOCITY_RAMP] Ramp-up completed, using desired velocity: [%.2f, %.2f, %.2f] m/s",
                        vel_x, vel_y, vel_z);
		}
	} else {
                // 斜坡已结束，直接使用指令速度
                vel_x = desired_vel_x;
                vel_y = desired_vel_y;
                vel_z = desired_vel_z;
	}
	
            msg.velocity = {
                static_cast<float>(vel_x),
	                static_cast<float>(vel_y),
                static_cast<float>(vel_z)
            };

            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                "[RC_SWITCH] GUIDANCE - Velocity: [%.2f, %.2f, %.2f] m/s",
                vel_x, vel_y, vel_z);
            break;
    }

    msg.acceleration = {
        std::numeric_limits<float>::quiet_NaN(),
	                    std::numeric_limits<float>::quiet_NaN(), 
        std::numeric_limits<float>::quiet_NaN()
    };
	
    // 偏航角控制：仅在 Guidance 模式下生效
    // 在STOPPED或SwitchingOffboard状态下，不控制偏航角（发送NaN）
    if (rc_mode_ == RcMode::Guidance) {
        // 仅在制导模式下控制偏航角
        // 默认使用 LOS 方向对准模式（模式2）
        if (auto_yaw_to_velocity_ && los_received_) {
            // 模式2：LOS方向对准（带平滑处理，避免左右摇摆）
            double los_x = los_x_.load();
            double los_y = los_y_.load();
            double desired_yaw = std::atan2(los_y, los_x);
            
            // 等待从odometry初始化偏航角（避免初始跳变）
            if (!yaw_initialized_from_odometry_) {
                // 还未从odometry初始化，发送NaN让PX4自行控制
                msg.yaw = std::numeric_limits<float>::quiet_NaN();
            } else {
                // 已从odometry初始化，开始平滑过渡到LOS方向
                if (!los_yaw_initialized_) {
                    // 首次：计算角度差，但限制初始跳变
                    double yaw_diff = desired_yaw - current_yaw_;
                    // 归一化到[-π, π]
                    while (yaw_diff > M_PI) yaw_diff -= 2.0 * M_PI;
                    while (yaw_diff < -M_PI) yaw_diff += 2.0 * M_PI;
                    
                    // 限制初始跳变（更严格的限制，避免大的初始变化）
                    double max_initial_delta = max_yaw_rate_ * control_dt_;
                    if (yaw_diff > max_initial_delta) {
                        yaw_diff = max_initial_delta;
                    } else if (yaw_diff < -max_initial_delta) {
                        yaw_diff = -max_initial_delta;
                    }
                    
                    // 计算动态平滑系数（边缘视场保护）
                    double dynamic_smoothing = los_yaw_smoothing_;
                    if (enable_dynamic_yaw_smoothing_ && pixel_received_) {
                        double pixel_x = pixel_x_.load();
                        double pixel_error = std::abs(pixel_x - image_center_x_);
                        
                        if (pixel_error <= safe_zone_radius_) {
                            // 情况1：在安全区内，保持最大平滑
                            dynamic_smoothing = min_smoothing_;
                        } else if (pixel_error >= (safe_zone_radius_ + transition_width_)) {
                            // 情况2：超出过渡区，保持最高灵敏度
                            dynamic_smoothing = max_smoothing_;
                        } else {
                            // 情况3：在过渡区内，进行余弦平滑插值
                            // 计算归一化进度 t (0.0 -> 1.0)
                            double t = (pixel_error - safe_zone_radius_) / transition_width_;
                            
                            // 余弦因子计算：从 0 变化到 1 的 S 型曲线
                            double cos_factor = (1.0 - std::cos(t * M_PI)) / 2.0;
                            
                            dynamic_smoothing = min_smoothing_ + (max_smoothing_ - min_smoothing_) * cos_factor;
                        }
                    }
                    
                    // 使用动态平滑系数进行初始过渡
                    current_yaw_ += dynamic_smoothing * yaw_diff;
                    los_yaw_initialized_ = true;
                } else {
                    // 后续：正常平滑处理
                    double yaw_diff = desired_yaw - current_yaw_;
                    // 归一化到[-π, π]
                    while (yaw_diff > M_PI) yaw_diff -= 2.0 * M_PI;
                    while (yaw_diff < -M_PI) yaw_diff += 2.0 * M_PI;
                    
                    // 计算动态平滑系数（边缘视场保护）
                    double dynamic_smoothing = los_yaw_smoothing_;
                    if (enable_dynamic_yaw_smoothing_ && pixel_received_) {
		double pixel_x = pixel_x_.load();
                        double pixel_error = std::abs(pixel_x - image_center_x_);
                        
                        if (pixel_error <= safe_zone_radius_) {
                            // 情况1：在安全区内，保持最大平滑
                            dynamic_smoothing = min_smoothing_;
                        } else if (pixel_error >= (safe_zone_radius_ + transition_width_)) {
                            // 情况2：超出过渡区，保持最高灵敏度
                            dynamic_smoothing = max_smoothing_;
                        } else {
                            // 情况3：在过渡区内，进行余弦平滑插值
                            // 计算归一化进度 t (0.0 -> 1.0)
                            double t = (pixel_error - safe_zone_radius_) / transition_width_;
                            
                            // 余弦因子计算：从 0 变化到 1 的 S 型曲线
                            double cos_factor = (1.0 - std::cos(t * M_PI)) / 2.0;
                            
                            dynamic_smoothing = min_smoothing_ + (max_smoothing_ - min_smoothing_) * cos_factor;
                        }
                    }
                    
                    // 限制偏航角变化速率（防止快速摆动）
                    double max_delta_yaw = max_yaw_rate_ * control_dt_;
                    if (yaw_diff > max_delta_yaw) {
                        yaw_diff = max_delta_yaw;
                    } else if (yaw_diff < -max_delta_yaw) {
                        yaw_diff = -max_delta_yaw;
		}
		
                    // 低通滤波平滑偏航角（使用动态平滑系数）
                    current_yaw_ += dynamic_smoothing * yaw_diff;
                }
		
		// 限制偏航角范围 [-π, π]
		current_yaw_ = std::atan2(std::sin(current_yaw_), std::cos(current_yaw_));
		
		msg.yaw = static_cast<float>(current_yaw_);
            }
		
	} else {
		// 模式3：不控制偏航
            msg.yaw = std::numeric_limits<float>::quiet_NaN();
        }
    } else {
        // 非制导模式（Stopped或SwitchingOffboard）：不控制偏航角，发送NaN
		msg.yaw = std::numeric_limits<float>::quiet_NaN();
	}
	
	msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
	
	// 添加调试信息 - 每秒输出一次（与控制频率解耦）
	static int debug_counter = 0;
	if (debug_counter % log_interval_ticks_ == 0) {  // 每1秒打印一次
        // 从msg.velocity读取实际发送的速度值（而不是使用局部变量）
        double logged_vel_x = std::isfinite(msg.velocity[0]) ? static_cast<double>(msg.velocity[0]) : 0.0;
        double logged_vel_y = std::isfinite(msg.velocity[1]) ? static_cast<double>(msg.velocity[1]) : 0.0;
        double logged_vel_z = std::isfinite(msg.velocity[2]) ? static_cast<double>(msg.velocity[2]) : 0.0;
        
		if (pixel_yaw_control_ && !std::isnan(msg.yaw)) {
			// 像素伺服模式
			double pixel_x = pixel_x_.load();
			double pixel_error_x = pixel_x - image_center_x_;
			RCLCPP_INFO(this->get_logger(), 
				"Vel: [%.2f, %.2f, %.2f] m/s | Pixel: %.1f (error: %.1f px) | Yaw: %.2f rad (%.1f°) | PIXEL-SERVO", 
                logged_vel_x, logged_vel_y, logged_vel_z, pixel_x, pixel_error_x, msg.yaw, msg.yaw * 180.0 / M_PI);
		} else if (auto_yaw_to_velocity_ && !std::isnan(msg.yaw)) {
			// LOS对准模式
			double los_x = los_x_.load();
			double los_y = los_y_.load();
			double los_z = los_z_.load();
			RCLCPP_INFO(this->get_logger(), "Vel: [%.2f, %.2f, %.2f] m/s | LOS: [%.3f, %.3f, %.3f] | Yaw: %.2f rad (%.1f°) | LOS-ALIGN", 
                logged_vel_x, logged_vel_y, logged_vel_z, los_x, los_y, los_z, msg.yaw, msg.yaw * 180.0 / M_PI);
		} else {
			// 自由模式
			RCLCPP_INFO(this->get_logger(), "Vel: [%.2f, %.2f, %.2f] m/s | Yaw: FREE", 
                logged_vel_x, logged_vel_y, logged_vel_z);
		}
	}
	debug_counter++;
	
	trajectory_setpoint_publisher_->publish(msg);
}

void VelocityTracking::publish_vehicle_command(uint16_t command, float param1, float param2)
{
	VehicleCommand msg{};
	msg.param1 = param1;
	msg.param2 = param2;
	msg.command = command;
	msg.target_system = 1;
	msg.target_component = 1;
	msg.source_system = 1;
	msg.source_component = 1;
	msg.from_external = true;
	msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
	vehicle_command_publisher_->publish(msg);
}

}  // namespace my_offboard_control
