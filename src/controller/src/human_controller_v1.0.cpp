#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float64MultiArray.h>
#include <sensor_msgs/Imu.h>
#include<cmath>
#include<tf/tf.h>
#include<std_msgs/String.h>
#include<algorithm>
class SubandPub
{
  	public:
		//构造函数
		SubandPub()
		{
			//命令消息publisher
			command_publisher = nh.advertise<std_msgs::Float64MultiArray>("/humanrobo/human_controller/command", 10);
			//imu数据订阅
			imu_subscriber = nh.subscribe("/humanrobo/imu", 10, &SubandPub::sensorCallBack, this);
			//键盘消息订阅
			keyboard_subscriber = nh.subscribe("/humanrobo/keyboard", 10, &SubandPub::keyboardCallBack, this);
			//定时器用于控制 控制周期
			timer = nh.createTimer(ros::Duration(D_T),&SubandPub::timerCallback, this);//每0.02s回调一次

			//设置数组大小
			EX_POS.data.resize(n_joints, 0.0);
			AC_POS.data.resize(n_joints, 0.0);

			//设置角度限制
			POS_LIM.resize(n_joints,std::vector<float>(2,0));
			//head
			POS_LIM[0][0]  = -M_PI; POS_LIM[0][1]  = M_PI;
			POS_LIM[1][0]  = -M_PI; POS_LIM[1][1]  = M_PI;
			POS_LIM[2][0]  = -M_PI; POS_LIM[2][1]  = M_PI;
			//left arm
			POS_LIM[3][0]  = -M_PI; POS_LIM[3][1]  = M_PI;
			POS_LIM[4][0]  = -M_PI; POS_LIM[4][1]  = M_PI;
			POS_LIM[5][0]  = -M_PI; POS_LIM[5][1]  = M_PI;
			POS_LIM[6][0]  = -M_PI; POS_LIM[6][1]  = M_PI;
			POS_LIM[7][0]  = -M_PI; POS_LIM[7][1]  = M_PI;
			//right arm
			POS_LIM[8][0]  = -M_PI; POS_LIM[8][1]  = M_PI;
			POS_LIM[9][0]  = -M_PI; POS_LIM[9][1]  = M_PI;
			POS_LIM[10][0] = -M_PI; POS_LIM[10][1] = M_PI;
			POS_LIM[11][0] = -M_PI; POS_LIM[11][1] = M_PI;
			// POS_LIM[12][0] = -M_PI; POS_LIM[12][1] = M_PI;
			// //waist
			// POS_LIM[13][0] = -M_PI; POS_LIM[13][1] = M_PI;
			// POS_LIM[14][0] = -M_PI; POS_LIM[14][1] = M_PI;
			// //left leg
			// POS_LIM[15][0] = -M_PI; POS_LIM[15][1] = M_PI;
			// POS_LIM[16][0] = -M_PI; POS_LIM[16][1] = M_PI;
			// POS_LIM[17][0] = -M_PI; POS_LIM[17][1] = M_PI;
			// POS_LIM[18][0] = -M_PI; POS_LIM[18][1] = M_PI;
			// POS_LIM[19][0] = -M_PI; POS_LIM[19][1] = M_PI;
			// POS_LIM[20][0] = -M_PI; POS_LIM[20][1] = M_PI;
			// //right leg
			// POS_LIM[21][0] = -M_PI; POS_LIM[21][1] = M_PI;
			// POS_LIM[22][0] = -M_PI; POS_LIM[22][1] = M_PI;
			// POS_LIM[23][0] = -M_PI; POS_LIM[23][1] = M_PI;
			// POS_LIM[24][0] = -M_PI; POS_LIM[24][1] = M_PI;
			// POS_LIM[25][0] = -M_PI; POS_LIM[25][1] = M_PI;
			// POS_LIM[26][0] = -M_PI; POS_LIM[26][1] = M_PI;

		}

		//期望角度和实际角度
		std_msgs::Float64MultiArray EX_POS, AC_POS;

		//定义每个关节极限角度，0为下限，1为上限
		std::vector<std::vector<float>> POS_LIM;
		
		//定义默认状态下的腿长
		float autoH = 186.0;

		//标记支撑脚和摆动脚：S:支撑，B：摆动
		int S_leg = 0;//支撑脚，0:left. 1:right
		int B_leg = 1;//摆动脚，
		
		//标记是否开始循环周期
		int iscal = 0;
		//迈步周期计数器
		int cal = 0;

		//控制周期：s
		float D_T = 0.02;
		//迈步周期：s
		float C_T = 1.5;
		//迈步周期所包含的控制周期数
		int walkcal = C_T / D_T;

		// 一个迈步周期各个阶段所占的比例，需要和为1
		// float kickrate = 0.3;
		float riserate = 0.3;
		float moverate = 0.5;
		float landrate = 0.2;

		//控制步态的参数
		float StepHeight = 10;
		float StepMaxDis = 30;
		float LegMaxDY = 20;
		float riseAng = 0.3;
		float landAng = -0.1;

		//脚的位置
		float DX_B = 0;
		float DX_S = 0;
		float DY_B = 0;
		float DY_S = 0;
		
		//保存上个周期结束时的脚位置
		float lastDX_S = 0;
		float lastDX_B = 0;
		float lastDY_S = 0;
		float lastDY_B = 0;

		//抬腿高度
		float footH = 0;
		//脚底角度，脚尖点地为正
		float f_Ang = 0;
		
		// 关节尺寸
		float H_foot = 16.7;	//foot关节到脚底距离
		float Y_hip = 26.55;	//foot关节与hip_IO关节的横向距离
		float H_hip = 48.3;		//hip_IO到thigh_BF距离
		float L_thigh = 53.0;	//大腿长度
		float L_shank = 53.29;	//小腿长度
		float H_ankle = 28.0;	//脚踝高度
		float L_footF = 43.9;	//脚底前面长度，到ankleBF
		float L_footB = 13.7;	//脚底后面长度，到ankleBF

		//垫脚尖需要的参数
		float R_F_2 = pow(H_ankle + H_foot, 2) + L_footF * L_footF;	//脚尖着地半径
		float R_B_2 = pow(H_ankle + H_foot, 2) + L_footB * L_footB;	//脚跟着地半径
		//脚底控制函数
		void FootPos2JointAng(float x, float y, float H, int lr, float f){
			float X_makeup, H_makeup;
			//可以不用if，但大多数情况f都是0，用来减少运算量。
			//f为脚底pitch转角，踮脚尖为正
			if(f > 0){
				float ang_f = atanf((H_ankle + H_foot) / L_footF);
				float H1 = sinf(ang_f + f) * sqrt(R_F_2);
				H_makeup = - (H1 - (H_ankle + H_foot));
				float X1 = cosf(ang_f + f) * sqrt(R_F_2);
				X_makeup = L_footF - X1;
			}
			else if(f < 0){
				float ang_b = atanf((H_ankle + H_foot) / L_footB);
				float H1 = sinf(ang_b - f) * sqrt(R_B_2);
				H_makeup = - (H1 - (H_ankle + H_foot));
				float X1 = cosf(ang_b - f) * sqrt(R_B_2);
				X_makeup = - (L_footB - X1);
			}
			else{
				X_makeup = 0;
				H_makeup = 0;
			}

			H += H_makeup;
			x += X_makeup;

			y -= Y_hip;

			// 计算基础几何
			float h = H - H_foot;
			float D = sqrt(h*h + y*y);
			float L = sqrt(D*D - Y_hip*Y_hip);
			float L1 = L - H_hip - H_ankle;
			float L2 = sqrt(L1*L1 + x*x);

			//避免超长
			if(L2 > L_shank + L_thigh){
				L2 = L_shank + L_thigh;
				ROS_INFO("too long");
			} 
			float gamma = acosf((L_shank*L_shank + L_thigh*L_thigh - L2*L2) / (2 * L_shank * L_thigh));
			float alpha = acosf((L2*L2 + L_thigh*L_thigh - L_shank*L_shank) / (2 * L2 * L_thigh));
			float beta = M_PI - gamma - alpha;

			//确定每个关节以标准坐标方向的角度
			float hip_IO;
			if(lr == 0){
				hip_IO = atanf(y/h) + asinf(Y_hip/D);
			}
			else{
				hip_IO = - atanf(y/h) - asinf(Y_hip/D);
			}
			float hip_SP = 0;
			float thigh_BF = - (atanf(x/L1) + alpha);
			float knee_BF = M_PI - gamma;
			float ankle_BF = atanf(x/L1) - beta;
			float ankle_IO = - hip_IO;


			// 根据lr选择控制左右腿
			if(lr == 0){
				EX_POS.data[0] = hip_IO;
				EX_POS.data[1] = hip_SP;
				EX_POS.data[2] = thigh_BF;
				EX_POS.data[3] = knee_BF;
				EX_POS.data[4] = ankle_BF + f;
				EX_POS.data[5] = ankle_IO;
			}
			else{
				EX_POS.data[6] = hip_IO;
				EX_POS.data[7] = hip_SP;
				EX_POS.data[8] = thigh_BF;
				EX_POS.data[9] = knee_BF;
				EX_POS.data[10] = - (ankle_BF + f);//urdf关节坐标有问题，所以此处补偿一个负号
				EX_POS.data[11] = ankle_IO;
			}

		}
		void Actuator(void){
		float maxv, deltaT;
		maxv = 3.14;
		deltaT = 0.02;
		//中间层向执行层转换
		for(unsigned int i = 0; i < n_joints; i++){
			//限制极限角度
			if(EX_POS.data[i] > POS_LIM[i][1]){
				EX_POS.data[i] = POS_LIM[i][1];
			}
			else if(EX_POS.data[i] < POS_LIM[i][0]){
				EX_POS.data[i] = POS_LIM[i][0];
			}
			else if(EX_POS.data[i] < POS_LIM[i][1] && EX_POS.data[i] > POS_LIM[i][0]){

			}
			else{
				ROS_INFO("EX_POS.positions[%u] invalid: %f",i, EX_POS.data[i]);
			}

			//限制转速
			if(EX_POS.data[i]-AC_POS.data[i] > maxv * deltaT){
				AC_POS.data[i] = AC_POS.data[i] + maxv * deltaT;
			}
			else if(EX_POS.data[i]-AC_POS.data[i] < - maxv * deltaT){
				AC_POS.data[i] = AC_POS.data[i] - maxv * deltaT;
			}
			else{
				AC_POS.data[i] = EX_POS.data[i];
			}
      	}


        // 发布消息
        command_publisher.publish(AC_POS);
    }
  	private:
		ros::NodeHandle nh;
		ros::Publisher command_publisher;
		ros::Subscriber imu_subscriber;
		ros::Subscriber keyboard_subscriber;
		ros::Timer timer;
		int n_joints = 12;//暂时先只动腿一共12个关节
		


		//传感器回调函数
		void sensorCallBack(const sensor_msgs::Imu msg){
		tf::Quaternion quaternion(
			msg.orientation.x,
			msg.orientation.y,
			msg.orientation.z,
			msg.orientation.w
		);
		double roll,pitch,yaw;
		tf::Matrix3x3(quaternion).getRPY(roll, pitch, yaw);
		// 弧度换算成角度
		//   ROS_INFO("receiving: roll= %.0f pitch= %.0f yaw= %.0f", roll, pitch, yaw);
		}
		//键盘事件监听
		void keyboardCallBack(const std_msgs::String::ConstPtr& msg){
		std::string keyin = msg->data;
		ROS_INFO("keyin: %s", msg->data.c_str());
			if(keyin == "W"){
				autoH++;
			}else if(keyin == "A"){
				footH++;
			}else if(keyin == "S"){
				autoH--;
			}else if(keyin == "D"){
				footH--;
			}else if(keyin == "UP"){
			}else if(keyin == "DOWN"){
			}else if(keyin == "LEFT"){
			}else if(keyin == "RIGHT"){
			}else if(keyin == "Q"){
			}else if(keyin == "R"){//开始执行走路动作
				autoH = 186;
				cal = 0;
				iscal = 1;
			}else if(keyin == "Z"){
				f_Ang+=0.02;
			}else if(keyin == "X"){
				f_Ang-=0.02;
			}else if(keyin == "E"){//停止走路动作并恢复站立状态
				iscal = 0;
				autoH = 186;
			}else{
			}
		}

		//定时器定时发送命令的回调函数
		void timerCallback(const ros::TimerEvent& e){
			//根据iscal情况执行腿部动作
			if(iscal){
				//cal不超过walkcal	
				cal = cal % (walkcal + 1);
				float phaseRate; 
				float circleRate = 1.0 * cal / walkcal;
				
				// 抬腿阶段
				if(cal <= riserate * walkcal){
					//在本阶段的0-1
					phaseRate = cal / (riserate * walkcal);

					footH = StepHeight * sinf(M_PI * circleRate);
					f_Ang = (0.5 * (1 - cosf(M_PI * phaseRate))) * riseAng;
					
					DX_B = lastDX_S + (0.5 * (1 - cosf(M_PI * circleRate))) * (  StepMaxDis - lastDX_S);
					DX_S = lastDX_B + (0.5 * (1 - cosf(M_PI * circleRate))) * (- StepMaxDis - lastDX_B);

					DY_B = -10 + 30 * sinf(M_PI * circleRate);
					DY_S = -10 - 10 * sinf(M_PI * circleRate);
				}

				// 转移阶段
				else if(cal <= (riserate + moverate) * walkcal){
					//在本阶段的0-1
					phaseRate = (cal - riserate * walkcal) / (moverate * walkcal);

					footH = StepHeight * sinf(M_PI * circleRate);
					f_Ang = riseAng + (0.5 * (1 - cosf(M_PI * phaseRate))) * (landAng - riseAng);
					
					DX_B = lastDX_S + (0.5 * (1 - cosf(M_PI * circleRate))) * (  StepMaxDis - lastDX_S);
					DX_S = lastDX_B + (0.5 * (1 - cosf(M_PI * circleRate))) * (- StepMaxDis - lastDX_B);

					DY_B = -10 + 30 * sinf(M_PI * circleRate);
					DY_S = -10 - 10 * sinf(M_PI * circleRate);
				}

				// 落地阶段
				else if(cal <= (riserate + moverate + landrate) * walkcal){
					//在本阶段的0-1
					phaseRate = (cal - (riserate + moverate) * walkcal) / (landrate * walkcal);

					f_Ang = landAng - (0.5 * (1 - cosf(M_PI * phaseRate))) * landAng;
					footH = StepHeight * sinf(M_PI * circleRate);

					DX_B = lastDX_S + (0.5 * (1 - cosf(M_PI * circleRate))) * (  StepMaxDis - lastDX_S);
					DX_S = lastDX_B + (0.5 * (1 - cosf(M_PI * circleRate))) * (- StepMaxDis - lastDX_B);

					DY_B = -10 + 30 * sinf(M_PI * circleRate);
					DY_S = -10 - 10 * sinf(M_PI * circleRate);

					//交换支撑腿和摆动腿
					if(cal > (riserate + moverate + landrate) * walkcal - 1){
						ROS_INFO("swaped");
						lastDX_B = DX_B;
						lastDX_S = DX_S;
						lastDY_B = DY_B;
						lastDY_S = DY_S;

						int temp;
						temp = S_leg;
						S_leg = B_leg;
						B_leg = temp;

						float tempY;
						tempY = DY_B;
						DY_B = DY_S;
						DY_S = tempY;

						float tempX;
						tempX = DX_B;
						DX_B = DX_S;
						DX_S = tempX;
					}
				}

				cal++;
				FootPos2JointAng(DX_S, DY_S, autoH, S_leg, 0.0);//最后一个0表示左腿；
				FootPos2JointAng(DX_B, DY_B, autoH-footH, B_leg, f_Ang);//最后一个1表示右腿；

			}
			else{
				FootPos2JointAng(0, 20, autoH - footH, 0, f_Ang);//最后一个0表示左腿；
				FootPos2JointAng(0, -20, autoH, 1, 0.0);//最后一个1表示右腿； 
			}
			Actuator();  
		}
};



//主函数
int main(int argc, char** argv)
{
  // 初始化 ROS 节点
  ros::init(argc, argv, "human_controller");
  SubandPub human_controll;
  ros::spin();
  return 0;
}