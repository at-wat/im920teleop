#include <math.h>
#include <time.h>
#include <stdio.h>
#include <stdlib.h>
#include <strings.h>

#include <sys/stat.h>
#include <sys/time.h>
#include <sys/types.h>

#include <fcntl.h>
#include <termios.h>
#include <signal.h>
#include <unistd.h>

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

int main(int argc, char** argv)
{
	ros::init(argc, argv, "im920teleop");
	ros::NodeHandle nh;

	int fd;
	struct termios oldtio, newtio;

	std::string port_name;
	nh.param("port", port_name, std::string("/dev/ttyUSB0"));
	fd = open(port_name.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
	if(fd < 0)
	{
		ROS_INFO("Cannot find device at %s. ", port_name.c_str());
		return 0; 
	}

	ROS_INFO("Device opened.");
	tcgetattr(fd, &oldtio);
	bzero(&newtio, sizeof(newtio));

	newtio.c_cflag = CS8 | CLOCAL | CREAD | CRTSCTS;
	newtio.c_iflag = IGNPAR;
	newtio.c_oflag = 0;
	newtio.c_lflag = ICANON;
	newtio.c_cc[VMIN] = 0;
	newtio.c_cc[VTIME] = 0;

	cfsetispeed(&newtio, B19200);
	cfsetospeed(&newtio, B19200); 

	tcflush(fd, TCIFLUSH);
	tcsetattr(fd, TCSANOW, &newtio);

	const boost::function<void(const geometry_msgs::Twist::ConstPtr&)> cb = 
		[&](const geometry_msgs::Twist::ConstPtr &msg)->void{
				char buf[256];
				short vel = msg->linear.x * 1000;
				short avel = msg->angular.z * 1000;
				sprintf(buf, "TXDA %04X%04X\r\n", 
						(unsigned int)vel, (unsigned int)avel);
				write(fd, buf, strlen(buf));
			};

	ros::Publisher pub = nh.advertise<geometry_msgs::Twist>("cmd_vel_out", 1);
	ros::Subscriber sub = nh.subscribe("cmd_vel_in", 1, cb);
	
	ros::Rate wait(4);
	while(ros::ok())
	{
		ros::spinOnce();
		wait.sleep();

		char buf[256];
		int n_read = read(fd, buf, 255);
		if(n_read < 1)
		{
			continue;
		}

		geometry_msgs::Twist msg;
		unsigned int val[4], dummy;
		sscanf(buf, "%02X,%04X,%02X:%02X,%02X,%02X,%02X",
				&dummy, &dummy, &dummy,
				&val[0], &val[1], &val[2], &val[3]);
		short vel, avel;
		vel = (val[0] << 8) | val[1];
		avel = (val[2] << 8) | val[3];
		msg.linear.x = vel * 0.001;
		msg.angular.z = avel * 0.001;
		pub.publish(msg);
	}

	tcsetattr(fd, TCSANOW, &oldtio);
	return 0;
}

