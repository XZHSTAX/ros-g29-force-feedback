#include <rclcpp/rclcpp.hpp>

#include <linux/input.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include <fcntl.h>
#include <math.h>
#include <fstream>
#include <chrono>
#include "ros_g29_force_feedback/msg/force_feedback.hpp"

class G29ForceFeedback : public rclcpp::Node {

private:
    rclcpp::Subscription<ros_g29_force_feedback::msg::ForceFeedback>::SharedPtr sub_target;
    rclcpp::TimerBase::SharedPtr timer;
    // device info
    int m_device_handle;
    int m_axis_code = ABS_X;
    int m_axis_min;
    int m_axis_max;

    // rosparam
    std::string m_device_name;
    double m_loop_rate;
    double m_max_torque;
    double m_min_torque;
    double m_brake_position;
    double m_brake_torque;
    double m_auto_centering_max_torque;
    double m_auto_centering_max_position;
    double m_eps;
    bool m_auto_centering;

    // variables
    ros_g29_force_feedback::msg::ForceFeedback m_target; // 目标值，由topic的发布者给出，由targetCallback维护
    bool m_is_target_updated = false;
    bool m_is_brake_range = false;
    struct ff_effect m_effect;
    double m_position;       // 当前方向盘位置
    double m_torque;         // 当前方向盘力矩
    double m_attack_length;  // 当前方向盘攻角长度
    std::vector<double> positions_record; // 用于存储m_position的值
    std::vector<double> duration_record; // 用于存储m_position的值
    std::vector<double> ex_torque_record;
    std::vector<double> m_torque_record;
    std::chrono::steady_clock::time_point previous_time = std::chrono::steady_clock::now();// 用于计时
    std::chrono::steady_clock::time_point start_time = std::chrono::steady_clock::now();   // 用于计算总时长
    const double Moment_inertia = 0.00767;              // 转动惯量
    const double damping =  Moment_inertia * 24.30; // 阻尼系数
    const double max_torque_Nm = 2.1;                 // 最大力矩 (N·m)
    const int record_buff_length = 4;               // 记录n次数据后才能计算角速度和角加速度
    bool flag_position_updata = false;
public:
    G29ForceFeedback();
    ~G29ForceFeedback();

private:
    void targetCallback(const ros_g29_force_feedback::msg::ForceFeedback::SharedPtr in_target);
    void loop();
    int testBit(int bit, unsigned char *array);
    void initDevice();
    void calcRotateForce(double &torque, double &attack_length, const ros_g29_force_feedback::msg::ForceFeedback &target, const double &current_position);
    void calcCenteringForce(double &torque, const ros_g29_force_feedback::msg::ForceFeedback &target, const double &current_position);
    void uploadForce(const double &position, const double &force, const double &attack_length);
    double caculate_external_torque(double torque_machine);
};


G29ForceFeedback::G29ForceFeedback() 
    : Node("g29_force_feedback"){
    // 创建一个订阅者，订阅topic /ff_target，消息类型ros_g29_force_feedback::msg::ForceFeedback
    // 当收到来自/ff_target的消息时，函数targetCallback会被调用
    sub_target = this->create_subscription<ros_g29_force_feedback::msg::ForceFeedback>(
        "/ff_target", 
        rclcpp::SystemDefaultsQoS(), 
        std::bind(&G29ForceFeedback::targetCallback, this, std::placeholders::_1));
    positions_record.clear(); // 清空vector
    duration_record.clear(); // 清空vector
    ex_torque_record.clear(); // 清空vector
    m_torque_record.clear(); // 清空vector
    declare_parameter("device_name", m_device_name);
    declare_parameter("loop_rate", m_loop_rate);
    declare_parameter("max_torque", m_max_torque);
    declare_parameter("min_torque", m_min_torque);
    declare_parameter("brake_position", m_brake_position);
    declare_parameter("brake_torque", m_brake_torque);
    declare_parameter("auto_centering_max_torque", m_auto_centering_max_torque);
    declare_parameter("auto_centering_max_position", m_auto_centering_max_position);
    declare_parameter("eps", m_eps);
    declare_parameter("auto_centering", m_auto_centering);

    get_parameter("device_name", m_device_name);
    get_parameter("loop_rate", m_loop_rate);
    get_parameter("max_torque", m_max_torque);
    get_parameter("min_torque", m_min_torque);
    get_parameter("brake_position", m_brake_position);
    get_parameter("brake_torque", m_brake_torque);
    get_parameter("auto_centering_max_torque", m_auto_centering_max_torque);
    get_parameter("auto_centering_max_position", m_auto_centering_max_position);
    get_parameter("eps", m_eps);
    get_parameter("auto_centering", m_auto_centering);

    initDevice();

    rclcpp::sleep_for(std::chrono::seconds(1));
    // 创建一个定时器，每隔m_loop_rate*1s 调用一次函数loop
    timer = this->create_wall_timer(std::chrono::milliseconds((int)(m_loop_rate*1000)), 
            std::bind(&G29ForceFeedback::loop,this));
}

G29ForceFeedback::~G29ForceFeedback() {

    m_effect.type = FF_CONSTANT;
    m_effect.id = -1;
    m_effect.u.constant.level = 0;
    m_effect.direction = 0;

    std::chrono::steady_clock::time_point current_time = std::chrono::steady_clock::now();
    std::chrono::duration<double> duration = current_time - start_time;
    
    double time_elapsed = duration.count();
    std::cout << "time elapsed: " << time_elapsed << std::endl;
    // upload m_effect
    if (ioctl(m_device_handle, EVIOCSFF, &m_effect) < 0) {
        std::cout << "failed to upload m_effect" << std::endl;
    }
    std::ofstream output_file("/home/xzh/ROS2/ros2_g29/src/ros-g29-force-feedback/output.txt"); // 打开文件用于写入
    if (output_file.is_open()) {
        // for (double i: positions_record) {
        //     output_file << i << std::endl; // 写入每个m_position的值到文件
        // }
        std::cout << "length of positions_record: " << positions_record.size() << std::endl;
        std::cout << "length of duration_record: " << duration_record.size() << std::endl;
        std::cout << "length of ex_torque_record: " << ex_torque_record.size() << std::endl;
        if(positions_record.size() > duration_record.size() || positions_record.size() > ex_torque_record.size()){
            std::cout << "Record size not match!Fail to record!" << std::endl;
            return;
        }
        for(int i =0;i < duration_record.size();++i)
        {
            output_file << duration_record[i] << "," <<positions_record[i] << "," <<m_torque_record[i]<< ","<<ex_torque_record[i] << std::endl;
        }
        output_file.close(); // 关闭文件
        std::cout << "finish record!" << std::endl;
    }
    else{
        std::cout << "failed to open file" << std::endl;
    }

}


// update input event with timer callback
void G29ForceFeedback::loop() {

    struct input_event event;
    double last_position = m_position;
    // get current state
    while (read(m_device_handle, &event, sizeof(event)) == sizeof(event)) {
        if (event.type == EV_ABS && event.code == m_axis_code) {
            // 把转角信息event.value映射到[-1,1]
            m_position = (event.value - (m_axis_max + m_axis_min) * 0.5) * 2 / (m_axis_max - m_axis_min);
            positions_record.push_back(m_position); // 将m_position的值添加到vector中
            std::chrono::steady_clock::time_point current_time = std::chrono::steady_clock::now();
            std::chrono::duration<double> duration = current_time - previous_time;
            
            double time_elapsed = duration.count();
            duration_record.push_back(time_elapsed); // 将time_elapsed的值添加到vector中       
            previous_time = current_time; // 将当前时间点赋值给previous_time
            
            if(positions_record.size() <= record_buff_length){
                std::cout << "num_loop: " << positions_record.size() << std::endl;
                ex_torque_record.push_back(-1);
            }
            else{
                double external_torque = caculate_external_torque(m_torque);
                ex_torque_record.push_back(external_torque); // 将external_torque的值添加到vector中  
            }
            m_torque_record.push_back(m_torque); // 将m_torque的值添加到vector中
            flag_position_updata = true;
            break;
        }
        else{
            flag_position_updata = false;
            break;
        }
    }
    if(!flag_position_updata && positions_record.size() > record_buff_length){
        positions_record.push_back(m_position); // 将m_position的值添加到vector中
        std::chrono::steady_clock::time_point current_time = std::chrono::steady_clock::now();
        std::chrono::duration<double> duration = current_time - previous_time;
        
        double time_elapsed = duration.count();
        duration_record.push_back(time_elapsed); // 将time_elapsed的值添加到vector中       
        previous_time = current_time; // 将当前时间点赋值给previous_time
        
        if(positions_record.size() <= record_buff_length){
            std::cout << "num_loop: " << positions_record.size() << std::endl;
            ex_torque_record.push_back(-1);
        }
        else{
            double external_torque = caculate_external_torque(m_torque);
            ex_torque_record.push_back(external_torque); // 将external_torque的值添加到vector中  
        }
        m_torque_record.push_back(m_torque); // 将m_torque的值添加到vector中  
    }
    // 如果在最小制动范围，或使用模式为自动对齐，则使用calcCenteringForce来设置力和位置
    if (m_is_brake_range || m_auto_centering) {
        calcCenteringForce(m_torque, m_target, m_position);
        m_attack_length = 0.0;

    } else {
        calcRotateForce(m_torque, m_attack_length, m_target, m_position);
        m_is_target_updated = false;
    }

    uploadForce(m_target.position, m_torque, m_attack_length);
}


void G29ForceFeedback::calcRotateForce(double &torque,
                                       double &attack_length,
                                       const ros_g29_force_feedback::msg::ForceFeedback &target,
                                       const double &current_position) {

    double diff = target.position - current_position;
    double direction = (diff > 0.0) ? 1.0 : -1.0;
    // 如果误差小于eps
    if (fabs(diff) < m_eps) {
        torque = 0.0;
        attack_length = 0.0;
    
    // 如果误差小于最小制动位置，则开始制动，力矩缓慢减小，公式为 (目标力矩*m_brake_torque*direction*(-1))
    } else if (fabs(diff) < m_brake_position) {
        m_is_brake_range = true; // m_is_brake_range唯一可以变为true的地方
        torque = target.torque * m_brake_torque * -direction;
        attack_length = m_loop_rate;

    } else {
        torque = target.torque * direction;
        attack_length = m_loop_rate;
    }
}


void G29ForceFeedback::calcCenteringForce(double &torque,
                                          const ros_g29_force_feedback::msg::ForceFeedback &target,
                                          const double &current_position) {

    double diff = target.position - current_position;
    double direction = (diff > 0.0) ? 1.0 : -1.0;

    if (fabs(diff) < m_eps)
        torque = 0.0;

    else {
        double torque_range = m_auto_centering_max_torque - m_min_torque;
        double power = (fabs(diff) - m_eps) / (m_auto_centering_max_position - m_eps);
        double buf_torque = power * torque_range + m_min_torque;
        torque = std::min(buf_torque, m_auto_centering_max_torque) * direction;
    }
}


// update input event with writing information to the event file
void G29ForceFeedback::uploadForce(const double &position,
                                   const double &torque,
                                   const double &attack_length) {

    // std::cout << torque << std::endl;
    // set effect
    m_effect.u.constant.level = 0x7fff * std::min(torque, m_max_torque);
    m_effect.direction = 0xC000;
    m_effect.u.constant.envelope.attack_level = 0; /* 0x7fff * force / 2 */
    m_effect.u.constant.envelope.attack_length = attack_length;
    m_effect.u.constant.envelope.fade_level = 0;
    m_effect.u.constant.envelope.fade_length = attack_length;

    // upload effect
    if (ioctl(m_device_handle, EVIOCSFF, &m_effect) < 0) {
        std::cout << "failed to upload effect" << std::endl;
    }
}


// get target information of wheel control from ros message
void G29ForceFeedback::targetCallback(const ros_g29_force_feedback::msg::ForceFeedback::SharedPtr in_msg) {

    // 如果上次的目标位置和本次的目标位置一致，并且两次的目标力矩一致
    if (m_target.position == in_msg->position && m_target.torque == fabs(in_msg->torque)) {
        m_is_target_updated = false;

    // 如果目标有变
    } else {
        m_target = *in_msg; // 更新目标
        m_target.torque = fabs(m_target.torque);
        m_is_target_updated = true;
        m_is_brake_range = false;
    }
}


// initialize force feedback device
void G29ForceFeedback::initDevice() {
    // setup device
    unsigned char key_bits[1+KEY_MAX/8/sizeof(unsigned char)];
    unsigned char abs_bits[1+ABS_MAX/8/sizeof(unsigned char)];
    unsigned char ff_bits[1+FF_MAX/8/sizeof(unsigned char)];
    struct input_event event;
    struct input_absinfo abs_info;

    m_device_handle = open(m_device_name.c_str(), O_RDWR|O_NONBLOCK);
    if (m_device_handle < 0) {
        std::cout << "ERROR: cannot open device : "<< m_device_name << std::endl;
        exit(1);

    } else {std::cout << "device opened" << std::endl;}

    // which axes has the device?
    memset(abs_bits, 0, sizeof(abs_bits));
    if (ioctl(m_device_handle, EVIOCGBIT(EV_ABS, sizeof(abs_bits)), abs_bits) < 0) {
        std::cout << "ERROR: cannot get abs bits" << std::endl;
        exit(1);
    }

    // get some information about force feedback
    memset(ff_bits, 0, sizeof(ff_bits));
    if (ioctl(m_device_handle, EVIOCGBIT(EV_FF, sizeof(ff_bits)), ff_bits) < 0) {
        std::cout << "ERROR: cannot get ff bits" << std::endl;
        exit(1);
    }

    // get axis value range
    if (ioctl(m_device_handle, EVIOCGABS(m_axis_code), &abs_info) < 0) {
        std::cout << "ERROR: cannot get axis range" << std::endl;
        exit(1);
    }
    m_axis_max = abs_info.maximum;
    m_axis_min = abs_info.minimum;
    if (m_axis_min >= m_axis_max) {
        std::cout << "ERROR: axis range has bad value" << std::endl;
        exit(1);
    }

    // check force feedback is supported?
    if(!testBit(FF_CONSTANT, ff_bits)) {
        std::cout << "ERROR: force feedback is not supported" << std::endl;
        exit(1);

    } else { std::cout << "force feedback supported" << std::endl; }

    // auto centering off
    memset(&event, 0, sizeof(event));
    event.type = EV_FF;
    event.code = FF_AUTOCENTER;
    event.value = 0;
    if (write(m_device_handle, &event, sizeof(event)) != sizeof(event)) {
        std::cout << "failed to disable auto centering" << std::endl;
        exit(1);
    }

    // init effect and get effect id
    memset(&m_effect, 0, sizeof(m_effect));
    m_effect.type = FF_CONSTANT;
    m_effect.id = -1; // initial value
    m_effect.trigger.button = 0;
    m_effect.trigger.interval = 0;
    m_effect.replay.length = 0xffff;  // longest value
    m_effect.replay.delay = 0; // delay from write(...)
    m_effect.u.constant.level = 0;
    m_effect.direction = 0xC000;
    m_effect.u.constant.envelope.attack_length = 0;
    m_effect.u.constant.envelope.attack_level = 0;
    m_effect.u.constant.envelope.fade_length = 0;
    m_effect.u.constant.envelope.fade_level = 0;

    if (ioctl(m_device_handle, EVIOCSFF, &m_effect) < 0) {
        std::cout << "failed to upload m_effect" << std::endl;
        exit(1);
    }

    // start m_effect
    memset(&event, 0, sizeof(event));
    event.type = EV_FF;
    event.code = m_effect.id;
    event.value = 1;
    if (write(m_device_handle, &event, sizeof(event)) != sizeof(event)) {
        std::cout << "failed to start event" << std::endl;
        exit(1);
    }
}


// util for initDevice()
int G29ForceFeedback::testBit(int bit, unsigned char *array) {

    return ((array[bit / (sizeof(unsigned char) * 8)] >> (bit % (sizeof(unsigned char) * 8))) & 1);
}

double G29ForceFeedback::caculate_external_torque(double torque_machine) {
    /*
    此处，torque_machine是带有方向的
    */
    double position_k   = positions_record[positions_record.size() - 1]*450*3.1415926/180; // 从[-1,1]映射回弧度
    double position_k_1 = positions_record[positions_record.size() - 2]*450*3.1415926/180;
    double position_k_2 = positions_record[positions_record.size() - 3]*450*3.1415926/180;

    double time_elapsed_k   = duration_record[duration_record.size() - 1];
    double time_elapsed_k_1 = duration_record[duration_record.size() - 2];
    
    double omega_k = (position_k - position_k_1) / time_elapsed_k;
    double omega_k_1 = (position_k_1 - position_k_2) / time_elapsed_k_1;

    double alpha_k = (omega_k - omega_k_1) / time_elapsed_k;
        
    double External_Torque = Moment_inertia * alpha_k - torque_machine*max_torque_Nm + damping * omega_k;
    return External_Torque;
}


int main(int argc, char * argv[]){
    rclcpp::init(argc, argv);

    auto g29_ff = std::make_shared<G29ForceFeedback>();
    rclcpp::spin(g29_ff);

    rclcpp::shutdown();
    return 0;
}
