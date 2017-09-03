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

	const boost::function<void(const geometry_msgs::Twist::ConstPtr&)> cb = 
		[&](const geometry_msgs::Twist::ConstPtr &msg)->void{
				char buf[256];
				short vel = msg->linear.x * 1000;
				short avel = msg->angular.z * 1000;
				sprintf(buf, "TXDA %04X%04X\r\n", 
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

	ros::Subscriber sub = nh.subscribe("/cmd_vel_in", 1, cb);
	ros::Subscriber sub_over = nh.subscribe("/cmd_vel_over", 1, cb_over);
	ros::Subscriber sub_ow = nh.subscribe("/cmd_vel_overwritten", 1, cb_ow);
	ros::Subscriber sub_cm = nh.subscribe("/control_mode_in", 1, cb_cm);

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
		if(n_read != 24)
		{
			if(n_read > 2 && buf[0] == 'O' && buf[1] == 'K') continue;
			buf[n_read] = 0;
			ROS_ERROR("[%d] %s", n_read, buf);
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

	tcsetattr(fd, TCSANOW, &oldtio);
	ros::Duration(1.5).sleep();
	return 0;
}

