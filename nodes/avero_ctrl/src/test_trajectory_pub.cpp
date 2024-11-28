#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <cmath>
#include <nav_msgs/Odometry.h>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>


class DroneController {
public:
    DroneController() {
        // Publisher für die Pfadnachricht erstellen
        path_pub_ = nh_.advertise<nav_msgs::Path>("desired_drone_path", 1, true);

        // Publisher für die Geschwindigkeitsnachricht erstellen
        

        //odometry publisher
        odometry_pub_ = nh_.advertise<nav_msgs::Odometry>("desired_drone_odometry", 1, true);

        // Startpunkt der Drohne auf (0,0,0) setzen
        geometry_msgs::Point start_point;
        start_point.x = 2.0;
        start_point.y = 0.0;
        start_point.z = 3.0;

        // Abfolge von Punkten für den Kreis erstellen
        generateCircularPath(start_point);

        // ROS-Spin starten
        ros::spin();
    }

private:
    void generateCircularPath(geometry_msgs::Point start_point) {
        // Anzahl der Punkte im Kreis
        int num_points = 10000;

        // Radius des Kreises
        double radius = 2.0;

        // Zeit zwischen den Punkten
        double time_interval = 0.00075*2;
        double time_interval_start = 5;

        // Pfadnachricht erstellen
        nav_msgs::Path path_msg;
        path_msg.header.frame_id = "map";

        // Geschwindigkeitsnachricht erstellen
        geometry_msgs::Twist velocity_msg;

        geometry_msgs::PoseStamped start_pose;
        start_pose.header.stamp = ros::Time::now();
        start_pose.header.frame_id = "map";
        start_pose.pose.position = start_point;

        path_msg.poses.push_back(start_pose);

        // Geschwindigkeitsnachricht für den Startpunkt hinzufügen
        velocity_msg.linear.x = radius / time_interval_start;
        velocity_msg.linear.y = 0.0;
        velocity_msg.linear.z = 3.0 / time_interval_start;

    

        // Odometry-Nachricht erstellen
        nav_msgs::Odometry odometry_msg;
        odometry_msg.header.frame_id = "map";
        odometry_msg.child_frame_id = "base_link";
        odometry_msg.header.stamp = ros::Time::now();

        //zugehörige velocity und pose
        odometry_msg.pose.pose = start_pose.pose;
        odometry_msg.twist.twist = velocity_msg;
        odometry_pub_.publish(odometry_msg);


        // Warten, um die Geschwindigkeitsnachrichten zu synchronisieren
        ros::Duration(time_interval_start).sleep();
        geometry_msgs::Point point = start_point;
        geometry_msgs::PoseStamped pose_stamped;
    

        double x_before;
        double y_before;
        double z_before;
        double angle;
        Eigen::Quaterniond des_rot;
        des_rot.setIdentity();


        for (int i = 0; i < num_points; ++i) {
            angle = 2.0 * M_PI * i / num_points;

            //position von zuvor für die geschwindigkeitsberechnung
            x_before = point.x;
            y_before = point.y;
            z_before = point.z;

            // Punkte erstellen
            point.x = radius * cos(angle);
            point.y = radius * sin(angle);
            point.z = 3.0; // Konstante Höhe über der z-Achse

            // Rotation erstellen
            des_rot = (Eigen::AngleAxisd(2*M_PI*i/num_points, Eigen::Vector3d::UnitX()));

            // Geschwindigkeit setzen (z.B., konstante Geschwindigkeit von 1 m/s)
            velocity_msg.linear.x = (point.x - x_before) / time_interval;
            velocity_msg.linear.y = (point.y - y_before) / time_interval;
            velocity_msg.linear.z = (point.z - z_before) / time_interval;

            // Zeitstempel für die Punkte festlegen
            ros::Time point_time = ros::Time::now() + ros::Duration(i * time_interval);

            // PoseStamped-Nachricht erstellen
            pose_stamped.header.stamp = point_time;
            pose_stamped.header.frame_id = "map";
            

            pose_stamped.pose.position = point;

            // PoseStamped-Nachricht zum Pfad hinzufügen
            path_msg.poses.push_back(pose_stamped);
            path_msg.header.stamp = point_time;

            // Odometry-Nachricht erstellen
            
            odometry_msg.header.stamp = point_time;
            odometry_msg.header.frame_id = "map";

            // Gewünschte Pose setzen
            odometry_msg.pose.pose = pose_stamped.pose;
            odometry_msg.twist.twist = velocity_msg;
            odometry_msg.pose.pose.orientation.x = des_rot.x();
            odometry_msg.pose.pose.orientation.y = des_rot.y();
            odometry_msg.pose.pose.orientation.z = des_rot.z();
            odometry_msg.pose.pose.orientation.w = des_rot.w();
            // eine desired rot vel auszurechnen weiss ich ncht direkt wie das geht

            // Odometry-Nachricht veröffentlichen
            odometry_pub_.publish(odometry_msg);
            ROS_INFO("Published Odometry message for point x: %f, y: %f, z: %f", point.x, point.y, point.z);
            ROS_INFO("Published velocity x: %f, y: %f, z: %f", velocity_msg.linear.x, velocity_msg.linear.y, velocity_msg.linear.z);

            // Warten, um die Geschwindigkeitsnachrichten zu synchronisieren
            ros::Duration(time_interval).sleep();
        }

        // Pfadnachricht veröffentlichen
        path_pub_.publish(path_msg);
    }

    ros::NodeHandle nh_;
    ros::Publisher path_pub_;
    ros::Publisher odometry_pub_;
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "drone_controller");
    DroneController droneController;
    return 0;
}