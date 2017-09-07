#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <std_srvs/Empty.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <math.h>


#include "pid.hpp"

double get(
    const ros::NodeHandle& n,
    const std::string& name) {
    double value;
    n.getParam(name, value);
    return value;
}

class Controller
{
public:

    Controller(
	const std::string& target,
        const std::string& pose,
        const ros::NodeHandle& n)
        : m_target(target)
	, m_pose(pose)
        , m_pub()
        , m_pidX(
            get(n, "PIDs/X/kp"),
            get(n, "PIDs/X/kd"),
            get(n, "PIDs/X/ki"),
            get(n, "PIDs/X/minOutput"),
            get(n, "PIDs/X/maxOutput"),
            get(n, "PIDs/X/integratorMin"),
            get(n, "PIDs/X/integratorMax"),
            "x")
        , m_pidY(
            get(n, "PIDs/Y/kp"),
            get(n, "PIDs/Y/kd"),
            get(n, "PIDs/Y/ki"),
            get(n, "PIDs/Y/minOutput"),
            get(n, "PIDs/Y/maxOutput"),
            get(n, "PIDs/Y/integratorMin"),
            get(n, "PIDs/Y/integratorMax"),
            "y")
        , m_pidZ(
            get(n, "PIDs/Z/kp"),
            get(n, "PIDs/Z/kd"),
            get(n, "PIDs/Z/ki"),
            get(n, "PIDs/Z/minOutput"),
            get(n, "PIDs/Z/maxOutput"),
            get(n, "PIDs/Z/integratorMin"),
            get(n, "PIDs/Z/integratorMax"),
            "z")
        , m_pidYaw(
            get(n, "PIDs/Yaw/kp"),
            get(n, "PIDs/Yaw/kd"),
            get(n, "PIDs/Yaw/ki"),
            get(n, "PIDs/Yaw/minOutput"),
            get(n, "PIDs/Yaw/maxOutput"),
            get(n, "PIDs/Yaw/integratorMin"),
            get(n, "PIDs/Yaw/integratorMax"),
            "yaw")
        , m_state(Idle)
        , m_goal()
        , m_current()
        , m_subscribeGoal()
        , m_subscribeCurrent()
        , m_serviceTakeoff()
        , m_serviceLand()
        , m_serviceEmergency()
        , m_thrust(0)
        , m_startZ(0)
        , m_endZ(0)
    {
        ros::NodeHandle n1;
        m_pub = n1.advertise<geometry_msgs::Twist>("/crazyflie/cmd_vel", 1);
        m_subscribeGoal = n1.subscribe(m_target, 1, &Controller::update_goal, this);
	m_subscribeCurrent = n1.subscribe(m_pose,1, &Controller::update_current, this);

        m_serviceTakeoff = n1.advertiseService("takeoff", &Controller::takeoff, this);
        m_serviceLand = n1.advertiseService("land", &Controller::land, this);
        m_serviceEmergency = n1.advertiseService("emergency", &Controller::emergency, this);      

    }

    void run(double frequency)
    {
        ros::NodeHandle node;
        ros::Timer timer = node.createTimer(ros::Duration(1.0/frequency), &Controller::iteration, this);
        ros::spin();
    }

private:
    void update_goal(const geometry_msgs::PoseStamped::ConstPtr& msg)
    {
        m_goal = *msg;
    }

    void update_current(const geometry_msgs::PoseStamped::ConstPtr& msg)
    {
        m_current = *msg;
    }

    bool takeoff(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res)
    {
        ROS_INFO("Takeoff requested!");
        m_state = TakingOff;
        m_startZ = m_current.pose.position.z;
        return true;
    }

    bool land(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res)
    {
        ROS_INFO("Landing requested!");
        m_state = Landing;
        m_endZ = m_current.pose.position.z;
        return true;
    }

    bool emergency(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res)
    {
        ROS_INFO("Emergency requested!");
        m_state = Emergency;
        return true;
    }

    void pidReset()
    {
        m_pidX.reset();
        m_pidY.reset();
        m_pidZ.reset();
        m_pidYaw.reset();
    }

    float transform_world_to_body(
	const float* source,
	float* target, 
	const float phi,
    	const float theta,
	const float psi)
    {
	float sp    = sinf(phi);
	float cp    = cosf(phi);
	float st    = sinf(theta);
	float ct    = cosf(theta);
    	float ss    = sinf(psi);
    	float cs    = cosf(psi);
	target[0] = ct*cs*source[0] + ct*ss*source[1] -st*source[2];
    	target[1] = (sp*st*cs-cp*ss)*source[0] + (sp*st*ss+cp*cs)*source[1] + sp*ct*source[2];
    	target[2] = (cp*st*cs+sp*ss)*source[0] + (cp*st*ss-sp*cs)*source[1]+cp*ct*source[2];
    }

    void iteration(const ros::TimerEvent& e)
    {
        float dt = e.current_real.toSec() - e.last_real.toSec();
        switch(m_state)
        {
        case TakingOff:
            {

            pidReset();
            m_state=Automatic;
            m_pidZ.setIntegral(45000 / m_pidZ.ki());

        /*
                if (m_current.pose.position.z > m_startZ + 0.05 || m_thrust > 40000)
                {
                    pidReset();
                    m_pidZ.setIntegral(m_thrust / m_pidZ.ki());
                    m_state = Automatic;
                    m_thrust = 0;
                }
                else
                {
                    m_thrust += 10000 * dt;
                    geometry_msgs::Twist msg;
                    msg.linear.z = m_thrust;
                    m_pub.publish(msg);
                }
*/
            }
            break;
        case Landing:
            {

                m_endZ = m_endZ - 0.15 * dt;
                //m_endZ = m_endZ>0?m_endZ:0.0f;
                m_goal.pose.position.z = m_endZ;
                //ROS_INFO("m_endZ: %f",m_endZ);

                if (m_current.pose.position.z <= 0.05) {
                    m_endZ = 0;
                    m_state = Idle;
                    geometry_msgs::Twist msg;
                    m_pub.publish(msg);
                    break;
                }
            }
        case Automatic:
            {
                tfScalar roll_goal, pitch_goal, yaw_goal;
                tf::Matrix3x3(
                    tf::Quaternion(
                        m_goal.pose.orientation.x,
                        m_goal.pose.orientation.y,
                        m_goal.pose.orientation.z,
                        m_goal.pose.orientation.w
                    )).getRPY(roll_goal, pitch_goal, yaw_goal);

                tfScalar roll, pitch, yaw;
                tf::Matrix3x3(
                    tf::Quaternion(
                        m_current.pose.orientation.x,
                        m_current.pose.orientation.y,
                        m_current.pose.orientation.z,
                        m_current.pose.orientation.w
                    )).getRPY(roll, pitch, yaw);

                geometry_msgs::Twist msg;
		// transform from world to body frame
		float world_goal_position[3] = {
            		m_goal.pose.position.x-m_current.pose.position.x,
            		m_goal.pose.position.y-m_current.pose.position.y,
            		m_goal.pose.position.z-m_current.pose.position.z};


        	float body_goal_position[3]={0,0,0};
		transform_world_to_body(
			world_goal_position, 
			body_goal_position, 
            		roll, pitch, yaw);
            //ROS_INFO("MY: %.4f, %.4f, %.4f %.4f",body_goal_position[0],body_goal_position[1],body_goal_position[2],yaw);
            msg.linear.x = m_pidX.update(0.0, body_goal_position[0]);
            msg.linear.y = m_pidY.update(0.0, body_goal_position[1]);
            msg.linear.z = m_pidZ.update(0.0, body_goal_position[2]);
            //msg.linear.z = std::max(std::min(msg.linear.z, 60000.0), 0.0);
            msg.angular.z = m_pidYaw.update(yaw, yaw_goal);
            m_pub.publish(msg);
            }
            break;
        case Idle:
            {
                geometry_msgs::Twist msg;
                m_pub.publish(msg);

            }
            break;
        case Emergency:
            {
                m_state = Idle;
                geometry_msgs::Twist msg;
                m_pub.publish(msg);
            }
        }
    }

private:

    enum State
    {
        Idle = 0,
        Automatic = 1,
        TakingOff = 2,
        Landing = 3,
        Emergency = 4,
    };

private:
    std::string m_target;
    std::string m_pose;
    ros::Publisher m_pub;
    tf::TransformListener m_listener;
    PID m_pidX;
    PID m_pidY;
    PID m_pidZ;
    PID m_pidYaw;
    State m_state;
    geometry_msgs::PoseStamped m_goal;
    geometry_msgs::PoseStamped m_current;
    ros::Subscriber m_subscribeGoal;
    ros::Subscriber m_subscribeCurrent;
    ros::ServiceServer m_serviceTakeoff;
    ros::ServiceServer m_serviceLand;
    ros::ServiceServer m_serviceEmergency;
	
    float m_thrust;
    float m_startZ;
    float m_endZ;
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "pid_controller");

  // Read parameters
  ros::NodeHandle n("~");
  std::string target;
  n.param<std::string>("target", target, "/goal");
  std::string pose;
  n.getParam("pose", pose);
  double frequency;
  n.param("frequency", frequency, 50.0);

  Controller controller(target, pose, n);
  controller.run(frequency);

  return 0;
}
