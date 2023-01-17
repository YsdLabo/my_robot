#include<ros/ros.h>
#include<tf/transform_listener.h>
#include<geometry_msgs/PoseWithCovarianceStamped.h>
#include<geometry_msgs/PoseStamped.h>

int main(int argc, char** argv)
{
	ros::init(argc, argv, "my_navi");
	ros::NodeHandle nh;
	tf::TransformListener tf_listener;
	ros::Rate  loop_rate(1);

	ros::Publisher pub_start = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("/initialpose", 1, true);
	ros::Publisher pub_goal = nh.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 1, true);

	int next_num = 0;

	// ウェイポイントを (x, y, θ)の順で入力しましょう
	double way_point[][3] = {
		{-2.0, -0.5, 0},
		{1.8, -0.5, 0},
		{1.8, 1.0, 90.0}
	};

	// 初期位置の設定
	geometry_msgs::PoseWithCovarianceStamped start_point;
	start_point.header.stamp = ros::Time::now();
	start_point.header.frame_id = "map";
	start_point.pose.pose.position.x = way_point[0][0];
	start_point.pose.pose.position.y = way_point[0][1];
	start_point.pose.pose.position.z = 0.0;
	start_point.pose.pose.orientation = tf::createQuaternionMsgFromYaw(way_point[0][2] * M_PI / 180.0);
	pub_start.publish(start_point);
	ros::Duration(1.0).sleep();

	while(ros::ok())
	{
		// map座標系から見たロボット（base_link）の位置をtfから取得
		tf::StampedTransform trans;
		try{
			tf_listener.waitForTransform("map", "base_link", ros::Time(0), ros::Duration(3.0));
			tf_listener.lookupTransform("map", "base_link", ros::Time(0), trans);
		}
		catch(tf::TransformException &ex) {
			ROS_ERROR("%s", ex.what());
			ros::Duration(1.0).sleep();
			continue;
		}

		// 以下のx_now、y_now、yaw_nowが現在位置と姿勢
		double x_now = trans.getOrigin().x();
		double y_now = trans.getOrigin().y();
		double yaw_now = tf::getYaw(trans.getRotation());

		// 距離の計算
		double dx = x_now - way_point[next_num][0];
		double dy = y_now - way_point[next_num][1];
		double dist = sqrt(dx*dx + dy*dy);

		// ウェイポイントへの到達判定
		// ウェイポイントを中心に半径0.5mの範囲内に入ったら目的地を更新する
		if(dist < 0.5)
		{
			switch(next_num) {
			case 0:
				next_num = 1;
				break;
			case 1:
				next_num = 2;
				break;
			case 2:
				next_num = 2;
				break;
			}
			// 次のウェイポイントを配信する
			geometry_msgs::PoseStamped goal_point;
			goal_point.header.stamp = ros::Time::now();
			goal_point.header.frame_id = "map";
			goal_point.pose.position.x = way_point[next_num][0];
			goal_point.pose.position.y = way_point[next_num][1];
			goal_point.pose.position.z = 0.0;
			goal_point.pose.orientation = tf::createQuaternionMsgFromYaw(way_point[next_num][2] * M_PI / 180.0);
			pub_goal.publish(goal_point);
		}
		// 現在位置の表示
		ROS_INFO("[%lf : %lf : %lf]", x_now, y_now, yaw_now/M_PI*180.0);
		loop_rate.sleep();
	}
}
