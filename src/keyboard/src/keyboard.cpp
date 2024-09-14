#include <termios.h>
#include <signal.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/poll.h>
#include <std_msgs/String.h>
#include <boost/thread/thread.hpp>
#include <ros/ros.h>

#define KEYCODE_w 0x77
#define KEYCODE_a 0x61
#define KEYCODE_s 0x73
#define KEYCODE_d 0x64
#define KEYCODE_R 0x43 
#define KEYCODE_L 0x44
#define KEYCODE_U 0x41
#define KEYCODE_D 0x42
#define KEYCODE_q 0x71
#define KEYCODE_r 0x72
#define KEYCODE_z 0x7A
#define KEYCODE_x 0x78
#define KEYCODE_e 0x65

class SmartCarKeyboardTeleopNode
{
    private:   
        ros::NodeHandle n_;
        ros::Publisher pub_;

    public:
        SmartCarKeyboardTeleopNode()
        {
            pub_ = n_.advertise<std_msgs::String>("/humanrobo/keyboard", 10);
        } 
        ~SmartCarKeyboardTeleopNode() { }
        void keyboardLoop();
};

SmartCarKeyboardTeleopNode* tbk;
int kfd = 0;
struct termios cooked, raw;
bool done;

int main(int argc, char** argv)
{
    ros::init(argc,argv,"tbk", ros::init_options::AnonymousName | ros::init_options::NoSigintHandler);
    SmartCarKeyboardTeleopNode tbk;
    boost::thread t = boost::thread(boost::bind(&SmartCarKeyboardTeleopNode::keyboardLoop, &tbk));
    ros::spin();
    
    t.interrupt();
    t.join();
    tcsetattr(kfd, TCSANOW, &cooked);
    return(0);
}

void SmartCarKeyboardTeleopNode::keyboardLoop()
{
    char c;
    bool dirty = false;
    
    // get the console in raw mode
    tcgetattr(kfd, &cooked);
    memcpy(&raw, &cooked, sizeof(struct termios));
    raw.c_lflag &=~ (ICANON | ECHO);
    raw.c_cc[VEOL] = 1;
    raw.c_cc[VEOF] = 2;
    tcsetattr(kfd, TCSANOW, &raw);
    
    puts("Reading from keyboard");
    
    struct pollfd ufd;
    ufd.fd = kfd;
    ufd.events = POLLIN;
    
    for(;;)
    {
        boost::this_thread::interruption_point();
        
        // get the next event from the keyboard
        int num;
        if ((num = poll(&ufd, 1, 250)) < 0)
        {
            perror("poll():");
            return;
        }
        else if(num > 0)
        {
            if(read(kfd, &c, 1) < 0)
            {
                perror("read():");
                return;
            }
        }
        else
        {
            if (dirty == true)
            {
                dirty = false;
            }
            continue;
        }
        std_msgs::String msg;
        switch(c)
        {
            case KEYCODE_w:
                msg.data = "W";
                dirty = true;
                break;
            case KEYCODE_s:
                msg.data = "S";
                dirty = true;
                break;
            case KEYCODE_a:
                msg.data = "A";
                dirty = true;
                break;
            case KEYCODE_d:
                msg.data = "D";
                dirty = true;
                break;
            case KEYCODE_q:
                msg.data = "Q";
                dirty = true;
                break; 
            case KEYCODE_r:
                msg.data = "R";
                dirty = true;
                break;
            case KEYCODE_z:
                msg.data = "Z";
                dirty = true;
                break;
            case KEYCODE_x:
                msg.data = "X";
                dirty = true;
                break;
            case KEYCODE_U:
                msg.data = "UP";
                dirty = true;
                break;
            case KEYCODE_D:
                msg.data = "DOWN";
                dirty = true;
                break;
            case KEYCODE_R:
                msg.data = "RIGHT";
                dirty = true;
                break;
            case KEYCODE_L:
                msg.data = "LEFT";
                dirty = true;
                break;     
            case KEYCODE_e:
                msg.data = "E";
                dirty = true;
                break;  
            default:
                msg.data = "null";
                dirty = false;
            
        }

        pub_.publish(msg);
    }
}