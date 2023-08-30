#include <ros/ros.h>
#include <robot_msgs/can.h> 
#include <cstring>
#include <unistd.h>
#include <string.h>
#include <fcntl.h>
#include <poll.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <sys/select.h>
#include <sys/time.h>
#include <net/if.h>
#include <linux/can.h>
#include <linux/can/raw.h>

int s = socket(PF_CAN, SOCK_RAW, CAN_RAW);
struct sockaddr_can addr;
struct can_frame frame;
struct ifreq ifr;


bool pollIn ( int fd ) 
{
    struct pollfd fds[1];
    fds[0].fd = fd;
    fds[0].events = POLLIN;
    int pollReturn = poll(fds, 1, 500);;

    if (pollReturn > 0)
    {
        if (fds[0].revents & POLLIN)
        {
            fds[0].revents = 0;
            return true;
        }
    }
    return false;
}

union FloatToBytes {
  float f;
  uint8_t bytes[sizeof(float)];//задел на будущее
};


void Send_CAN_Float(u_int32_t ID, u_int8_t DLC, float curr, int socket) 
{
    can_frame frame;
    std::stringstream ss;
    ss.clear(); // очищаем содержимое потока
    ss << static_cast<int>(ID);//переводим десятичный формат приедшего числа к 16 и сохраняем в строку
    frame.can_id = std::stoul(ss.str(), nullptr, 16);
    frame.can_dlc = DLC;
    ROS_INFO("ID: %u  DATA: %f  DLC: %u", frame.can_id, curr, frame.can_dlc);
    memcpy(frame.data, &curr, sizeof(float));
    write(socket, &frame, sizeof(struct can_frame));
}



void Send_CAN(u_int32_t ID, u_int8_t DLC, u_int8_t Data, int socket)
 {
    can_frame frame;
    ROS_INFO("ID: %u  DATA: %u  DLC: %u", ID, Data, DLC);
    std::stringstream ss;
    ss.clear(); // очищаем содержимое потока
    ss << static_cast<int>(ID);
    frame.can_id = std::stoul(ss.str(), nullptr, 16);
    frame.can_dlc = DLC;
    frame.data[0] = Data;
    write(socket, &frame, sizeof(struct can_frame));
}

void can_send( const boost::shared_ptr<const robot_msgs::can>&  can_frame)
    {

    Send_CAN(can_frame->Id, 1 ,static_cast<uint8_t>(can_frame->data), s);

    }

int main(int argc, char **argv) {
    ros::init(argc, argv, "can_node");
    ros::NodeHandle can;
    ros::Publisher can_receive_pub = can.advertise<robot_msgs::can>("can_receive", 10);
    ros::Subscriber sub = can.subscribe<robot_msgs::can>("can_send", 10, can_send);
    robot_msgs::can can_frame;


    const char *ifname = "vcan0";//при необходимости менять на vcan0, если нужны тесты на виртуальном
    std::system("sudo ip link add dev vcan0 type vcan && sudo ip link set up vcan0");// для тестов на виртуальном
    //std::system("sudo ip link set can0 type can bitrate 1000000 && sudo ip link set up can0");
    if (s < 0) {
        perror("Error while opening socket");
        return -1;
    }

    strcpy(ifr.ifr_name, ifname);
    ioctl(s, SIOCGIFINDEX, &ifr);

    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;

    if (bind(s, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
        perror("Error in socket bind");
        return -2;
    }

    // Установка неблокирующего режима чтения и записи
    int flags = fcntl(s, F_GETFL, 0);
    fcntl(s, F_SETFL, flags | O_NONBLOCK);
    

    while (ros::ok()) {
        
        if(pollIn(s))
         {
        std::stringstream ss;

        int nbytes = read(s, &frame, sizeof(struct can_frame));
        ss.clear(); // очищаем содержимое потока
        ss << std::hex << static_cast<int>(frame.can_id);//переводим десятичный формат приедшего числа к 16 и сохраняем в строку
        can_frame.Id = std::stoul(ss.str(), nullptr, 10);//конфертация
        ROS_INFO("%u\n", can_frame.Id);

        if (nbytes < 0) {
            perror("Error while reading from socket");
            return -3;
        }
        else {
            if (frame.can_dlc == 1) {
                can_frame.data = static_cast<float>(frame.data[0]);
                ROS_INFO("%f", can_frame.data);
                can_receive_pub.publish(can_frame);
            }
            else {
                memcpy(&can_frame.data, frame.data, sizeof(float)); // преобразование типа
                ROS_INFO("%f\n", can_frame.data);
                can_receive_pub.publish(can_frame); 
            }
        }
                 }
        ros::spinOnce();
    }
       
    close(s);
    return 0;
    }
