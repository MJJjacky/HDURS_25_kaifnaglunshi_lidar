#include <ros/ros.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Float32.h>
#include <serial/serial.h>
#include <sensor_msgs/Range.h>
#include <datmo/ThetaPost.h>

class WheelPwmSender
{
public:
    WheelPwmSender() : laser1_data_(0.0), laser2_data_(0.0)
    {
        // ���Ļ���
        pwm_sub_ = nh_.subscribe("/wheel_pwm", 10, &WheelPwmSender::pwmCallback, this);
        laser1_sub_ = nh_.subscribe("/laser", 10, &WheelPwmSender::laser1Callback, this);
        laser2_sub_ = nh_.subscribe("/laser2", 10, &WheelPwmSender::laser2Callback, this);
        tag_sub_ = nh_.subscribe("/yes",10,&WheelPwmSender::tagCallback, this);
        lidar_sub_ = nh_.subscribe("datmo/theta",10,&WheelPwmSender::lidarCallback,this);
        theta_pub_=nh_.advertise<std_msgs::Float32> ("theta",10);   
        // ��ʼ������
        try {
            serial_port_.setPort("/dev/base");
            serial_port_.setBaudrate(115200);
            serial::Timeout timeout = serial::Timeout::simpleTimeout(1000);
            serial_port_.setTimeout(timeout);
            serial_port_.open();
        }
        catch (serial::IOException& e) {
            ROS_ERROR_STREAM("�޷��򿪴���: " << e.what());
        }
        
        if (serial_port_.isOpen()) {
            ROS_INFO_STREAM("���ڳ�ʼ���ɹ�");
        } else {
            ROS_ERROR_STREAM("����δ��");
        }
    }

    ~WheelPwmSender()
    {
        if (serial_port_.isOpen())
            serial_port_.close();
    }

private:
    void pwmCallback(const std_msgs::Float32MultiArray::ConstPtr& msg)
    {
        if (msg->data.size() < 2) {
            ROS_WARN("�յ���PWM���ݲ���");
            return;
        }

        // ת��PWMֵ
        int16_t left_pwm  = static_cast<int16_t>(msg->data[0]);
        int16_t right_pwm = static_cast<int16_t>(msg->data[1]);

        // ת���������ݣ���ת���ף�
        uint16_t laser1_cm = static_cast<uint16_t>(laser1_data_ * 1000);
        uint16_t laser2_cm = static_cast<uint16_t>(laser2_data_ * 1000);
          
          
        //uint16_t tag_r = static_cast<uint16_t>(tag_)
        // �������ݰ���11�ֽڣ�
        uint8_t packet[12];
        packet[0] = 0x7B;  // ֡ͷ

        // ����PWM
        packet[1] = static_cast<uint8_t>((left_pwm >> 8) & 0xFF);
        packet[2] = static_cast<uint8_t>(left_pwm & 0xFF);

        // ����PWM
        packet[3] = static_cast<uint8_t>((right_pwm >> 8) & 0xFF);
        packet[4] = static_cast<uint8_t>(right_pwm & 0xFF);

        // ����1����
        packet[5] = static_cast<uint8_t>((laser1_cm >> 8) & 0xFF);
        packet[6] = static_cast<uint8_t>(laser1_cm & 0xFF);

        // ����2����
        packet[7] = static_cast<uint8_t>((laser2_cm >> 8) & 0xFF);
        packet[8] = static_cast<uint8_t>(laser2_cm & 0xFF);

        // ����У��λ�����ǰ9�ֽڣ�
        uint8_t checksum = 0;
        for (int i = 0; i < 9; ++i) {
            checksum ^= packet[i];
        }
        packet[9] = checksum;
        
        packet[10] = tag_;

        packet[11] = 0x7D;  // ֡β

        // �������
        std::stringstream ss;
        for (int i = 0; i < 12; i++) {
            ss << std::hex << "0x" << static_cast<int>(packet[i]) << " ";
        }
        ROS_INFO_STREAM("���ݰ�: " << ss.str());

        // ��������
        if (serial_port_.isOpen()) {
            serial_port_.write(packet, 12);
        } else {
            ROS_ERROR("����δ��");
        }
    }
void laser1Callback(const sensor_msgs::Range::ConstPtr& msg)
    {
        // �����Ч�Լ�飨���ݴ�����ʵ�ʲ�����
        if(msg->range >= msg->min_range && msg->range <= msg->max_range) {
            laser1_data_ = msg->range;
        } else {
            laser1_data_ = 0;
            // ��Ч����ʱ��Ϊ0
        }
    }

void laser2Callback(const sensor_msgs::Range::ConstPtr& msg)
    {
        if(msg->range >= msg->min_range && msg->range <= msg->max_range) {
            laser2_data_ = msg->range;
        } else {
            laser2_data_ = 0;
        }
    }

void tagCallback(const std_msgs::Float32MultiArray::ConstPtr& msg)
    {
           tag_ = msg->data[0];
    }
void lidarCallback(const datmo::ThetaPost::ConstPtr& msg)
    {
           for (size_t i = 0; i < msg->distance.size(); ++i) {
              lidar_distance=msg->distance[i];
              if (lidar_distance <= min_distance){
                min_distance = lidar_distance;
                lidar_theta = msg->theta[i];
                if (lidar_theta<0){
                    lidar_theta = -1*(3.14159265859+lidar_theta);
                }
                else if(lidar_theta>0){
                    lidar_theta = (3.14159265859-lidar_theta);
                }
              else{
              lidar_theta = 0;
              }
              std_msgs::Float32 msg;
              msg.data=lidar_theta;
              theta_pub_.publish(msg);
            }
    }
}
    ros::NodeHandle nh_;
    ros::Subscriber pwm_sub_, laser1_sub_, laser2_sub_ , tag_sub_ , lidar_sub_;
    ros::Publisher theta_pub_;
    serial::Serial serial_port_;
    float laser1_data_, laser2_data_ , tag_ , lidar_theta , lidar_distance;
    float min_distance = 100;
};



int main(int argc, char** argv)
{
    ros::init(argc, argv, "serial_node");
    WheelPwmSender sender;
    ros::spin();
    return 0;
}