#include <math.h>
#include <time.h>
#include <stdio.h>
#include <stdlib.h>
#include <strings.h>

#include <sys/stat.h>
#include <sys/time.h>
#include <sys/types.h>
#include <vector>
#include <string>

#include <fcntl.h>
#include <termios.h>
#include <signal.h>
#include <unistd.h>

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/String.h>
#include <ypspur_ros/ControlMode.h>

int main(int argc, char** argv)
{
	ros::init(argc, argv, "im920teleop");
	ros::NodeHandle nh("~");

	int fd;
	struct termios oldtio, newtio;

	std::string port_name;
	nh.param("port", port_name, std::string("/dev/ttyUSB0"));
	fd = open(port_name.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
	if(fd < 0)
	{
		ROS_ERROR("Cannot find device at %s. ", port_name.c_str());
		ros::Duration(2.0).sleep();
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

	ros::Time last_interrupt = ros::Time::now();

	ros::Publisher pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel_out", 1);
	ros::Publisher pub_mode = nh.advertise<ypspur_ros::ControlMode>("/ypspur_ros/control_mode", 2);
	ros::Publisher pub_str = nh.advertise<std_msgs::String>("/string_out", 2);

	const boost::function<void(const geometry_msgs::Twist::ConstPtr&)> cb = 
		[&](const geometry_msgs::Twist::ConstPtr &msg)->void{
				char buf[256];
				short vel = msg->linear.x * 1000;
				short avel = msg->angular.z * 1000;
				sprintf(buf, "TXDA 01%04X%04X\r\n", 
						(unsigned short)vel, (unsigned short)avel);
				write(fd, buf, strlen(buf));
			};
	ypspur_ros::ControlMode mode;
	const boost::function<void(const ypspur_ros::ControlMode::ConstPtr&)> cb_cm = 
		[&](const ypspur_ros::ControlMode::ConstPtr &msg)->void{
				mode = *msg;
			};
	const boost::function<void(const geometry_msgs::Twist::ConstPtr&)> cb_over = 
		[&](const geometry_msgs::Twist::ConstPtr &msg)->void{
				ypspur_ros::ControlMode mode;
				if(msg->linear.x == 0.0 && msg->angular.z == 0.0)
				{
					mode.vehicle_control_mode = ypspur_ros::ControlMode::OPEN;
				}
				else
				{
					mode.vehicle_control_mode = ypspur_ros::ControlMode::VELOCITY;
				}
				pub_mode.publish(mode);
				pub.publish(*msg);

				last_interrupt = ros::Time::now();
			};
	const boost::function<void(const geometry_msgs::Twist::ConstPtr&)> cb_ow = 
		[&](const geometry_msgs::Twist::ConstPtr &msg)->void{
				if(ros::Time::now() - last_interrupt > ros::Duration(1.0))
				{
					pub.publish(*msg);
					pub_mode.publish(mode);
				}
			};
	const boost::function<void(const std_msgs::String::ConstPtr&)> cb_str = 
		[&](const std_msgs::String::ConstPtr &msg)->void{
				char buf[256];
				strcpy(buf, "TXDA 02");
				for(auto c : msg->data)
				{
					char byte[3];
					sprintf(byte, "%02x", c);
					strcat(buf, byte); 
				}
				strcat(buf, "\r\n"); 
				write(fd, buf, strlen(buf));
			};

	ros::Subscriber sub = nh.subscribe("/cmd_vel_in", 1, cb);
	ros::Subscriber sub_over = nh.subscribe("/cmd_vel_over", 1, cb_over);
	ros::Subscriber sub_ow = nh.subscribe("/cmd_vel_overwritten", 1, cb_ow);
	ros::Subscriber sub_cm = nh.subscribe("/control_mode_in", 1, cb_cm);
	ros::Subscriber sub_str = nh.subscribe("/string", 1, cb_str);

	{
		const char *setting = "STCH 01\r\n";
		write(fd, setting, strlen(setting));
	}
	{
		const char *setting = "STRT 2\r\n";
		write(fd, setting, strlen(setting));
	}
	{
		const char *setting = "DCIO\r\n";
		write(fd, setting, strlen(setting));
	}

	ros::Rate wait(4);
	while(ros::ok())
	{
		ros::spinOnce();
		wait.sleep();

		char buf[256];
		int n_read = read(fd, buf, 255);
		if(n_read < 1)
		{
			if(n_read == 0)
			{
				ros::shutdown();
				continue;
			}
			if(errno == EAGAIN || errno == EWOULDBLOCK)
			{
				continue;
			}
			ros::shutdown();
			continue;
		}
		const std::string str(buf, 0, n_read);

		std::vector<unsigned int> val;

		if(n_read < 11)
		{
			continue;
		}
		// 01234567890123456789..
		// 00,0000,00:00,00,00,...
		for(int i = 11; i < n_read - 1; i += 3)
		{
			const std::string val_str(str, i, 2);
			unsigned int v = std::stoi("0x" + val_str, nullptr, 16);
			val.push_back(v);
		}

		if(val[0] == 1)
		{
			geometry_msgs::Twist msg;
			short vel, avel;
			vel = (val[1] << 8) | val[2];
			avel = (val[3] << 8) | val[4];
			msg.linear.x = vel * 0.001;
			msg.angular.z = avel * 0.001;
			pub.publish(msg);

			ypspur_ros::ControlMode mode;
			if(vel == 0 && avel == 0)
			{
				mode.vehicle_control_mode = ypspur_ros::ControlMode::OPEN;
			}
			else
			{
				mode.vehicle_control_mode = ypspur_ros::ControlMode::VELOCITY;
			}
			pub_mode.publish(mode);

			last_interrupt = ros::Time::now();
		}
		else if(val[0] == 2)
		{
			std_msgs::String msg;

			for(auto &c: val)
			{
				char ch[2];
				ch[0] = c;
				ch[1] = 0;
				msg.data += std::string(ch);
			}
			pub_str.publish(msg);
		}
	}

	tcsetattr(fd, TCSANOW, &oldtio);
	ros::Duration(1.5).sleep();
	return 0;
}

