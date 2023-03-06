 /*
 * ///////////////////////////////
 * //  功能 : 接收 久凌电子的 UWB 模块的数据
 *           并发布话题
 *           后期可以修改话题发布的消息类型,目前的消息类型是 geometry_msgs_point 只有x y z 
 *           然后以 odom 数据的格式
 *             
 * //   话题名称:  uwb_position
 * //   话题消息类型:  geometry_msgs::Point
 *      test_uwbrecord_node_robotpose
 *      UWB发布 odom数据
 *       型,用于记录数据对比.
 * ///////////////////////////////
 */

#include <ros/ros.h>
#include <serial/serial.h>
#include <iostream>
#include <fstream> 
#include <std_msgs/String.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Float32.h>

#include <geometry_msgs/PoseWithCovarianceStamped.h> //amcl_pose
#include <tf/transform_datatypes.h> //转换函数头文件
#include <tf/tf.h>
#include <cmath>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Point.h> 
#include <geometry_msgs/PointStamped.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>

/**
 * 以下是矩阵运算相关头文件
 * 
 */
#include <Eigen/Dense>
#include<stdio.h>
#include<Eigen/Core> 
#include<Eigen/SVD>  

#include <Eigen/SVD>   
#include <vector>
#include <iomanip>
#include <ctime> 

/***********************************/
using namespace std;
/*******************************/
//publisher发布的消息类型
geometry_msgs::PointStamped uwb_position;
geometry_msgs::PointStamped uwb_position1;
geometry_msgs::PointStamped uwb_position_last;      // 用于u滤除野值点
geometry_msgs::PointStamped uwb_position1_last;
int flag = 0; //        用于u滤除野值点
double delta = 0;
double delta1 = 0;
int iteration_number = 0;
 
sensor_msgs::Imu     uwb_data_;
sensor_msgs::Imu     uwb_data_1;//orientation_covariance=raw angular_velocity_covariance ==kalman
nav_msgs::Odometry   odom;
geometry_msgs::Point uwb_position_center;
nav_msgs::Path       path;
geometry_msgs::PoseStamped                pose_stamped;

//订阅消息
geometry_msgs::QuaternionStamped attitude_data_;
sensor_msgs::Imu imu_data_;
std_msgs::Float32 height_above_takeoff_;

std_msgs::Float32 take_off_height_;
/*********************************/
//函数申明
vector<double>  TOA_LS(vector<vector<double> > Panchor,vector<double> d);
vector<double> distance_error(vector<double> px , vector<vector<double> > py , vector<double> d );
double cost( vector<vector<double> > Ptag, vector<vector<double> > Panchor ,vector<vector<double> > r , double d);
vector<double> grad1( vector<vector<double> > Ptag, vector<vector<double> > Panchor, vector<vector<double> > r , double d );
vector<double> grad2( vector<vector<double> > Ptag, vector<vector<double> > Panchor, vector<vector<double> > r , double d );
vector<vector<double> >  gd_test( vector<vector<double> > Ptag, vector<vector<double> > Panchor, vector<vector<double> > r , double d);
/*********************************/
// 回调函数
void heightSubCallback(const std_msgs::Float32::ConstPtr& heightAboveTakeoff)
{
  height_above_takeoff_ = *heightAboveTakeoff;
}
/*************************************/
/**
 * 求逆函数
 * 用法: std::cout<<pseudoInverse(B)<<std::endl;  
 * **/
template<typename _Matrix_Type_> 
_Matrix_Type_ pseudoInverse(const _Matrix_Type_ &a, double epsilon = 
    std::numeric_limits<double>::epsilon()) 
{  
    Eigen::JacobiSVD< _Matrix_Type_ > svd(a ,Eigen::ComputeThinU | Eigen::ComputeThinV);  
    double tolerance = epsilon * std::max(a.cols(), a.rows()) *svd.singularValues().array().abs()(0);  
    return svd.matrixV() *  (svd.singularValues().array().abs() > tolerance).select(svd.singularValues().array().inverse(), 0).matrix().asDiagonal() * svd.matrixU().adjoint(); 
}
/*********************************/
#pragma pack(pop)
#pragma pack(push)
#pragma pack(1)//单字节对齐
typedef struct 
{
    uint8_t   Head[6];//占6个字节
    uint8_t   len;//占1个字节  time-Pos_TagZ的长度
    uint32_t  Timer;//占4个字节 标签测据时间
    uint16_t  Tagid;//占2个字节 标签地址
    uint16_t  Ancid;//占2个字节
    uint8_t   Seq;//占1个字节
    uint8_t   Mask;//占1个字节 17 
    int	    rawrange[8];
    bool    kalman_enable; 
    int     kalmanrange[8];
    bool    pos_enable;
    uint8_t   dimen;
    uint8_t   ancmask;
    bool    Coord_valid;
    float   x;
    float   y;
    float   z;
    uint8_t   Check;
    uint8_t   Foot[2];
}ST_protocol;//一共占了101个字节
//用于传输的联合体变量
typedef union
{ 
    uint8_t     buf[101];
    ST_protocol data;
	//uint32 num;
	//float f;
		
}UnData;//用于传输的联合体变量

#pragma pack(pop)
 
int main(int argc, char** argv)
{
    /*********************************/
    ros::init(argc, argv, "UWB_serial_port");
    //创建句柄
    ros::NodeHandle n;
    //创建 发布者
    ros::Publisher uwb_position_pub  = n.advertise<geometry_msgs::PointStamped>("uwb_position",1);
    //创建一个serial类
    serial::Serial sp;
    serial::Serial sp1;
    //创建timeout
    try
    {
        //设置要打开的串口名称
        sp.setPort("/dev/ttyUSB0");
        sp1.setPort("/dev/ttyUSB1");
        //设置串口通信的波特率
        sp.setBaudrate(115200);//见uwb说明书 UWB波特率 115200
        sp1.setBaudrate(115200);//见uwb说明书 UWB波特率 115200
        // sp.setBaudrate(9600);
        //串口设置timeout
        serial::Timeout to = serial::Timeout::simpleTimeout(50);
        sp.setTimeout(to);
        sp1.setTimeout(to);
        //打开串口
        sp.open();
        sp1.open();
    }
    catch(serial::IOException& e)
    {
        ROS_ERROR_STREAM("Unable to open port.");
        return -1;
    }
    //判断串口是否打开成功
    if(sp.isOpen() && sp1.isOpen()  )
    {
        ROS_INFO_STREAM("/dev/ttyUSB0 is opened.");
    }
    else
    {
        ROS_INFO_STREAM("open port failed.");
        return -1;
    }
    ros::Rate loop_rate(20); //设置频率
    UnData revData;//用于装载 UWB传过来的数据
    UnData revData1;//用于装载 UWB1传过来的数据
    while(ros::ok())
    {
        //获取缓冲区内的字节数
        size_t n = sp.available();
        size_t n1 = sp1.available();
        if(n!=0&&n1!=0)
        {
            uint8_t buffer[1024];
            uint8_t buffer1[1024];
            //读出数据
            n = sp.read(buffer, n);
            n1 = sp1.read(buffer1, n1);
            std::cout<<"0单个长度:  "<< n <<std::endl;
            std::cout<<"1单个长度:  "<< n1 <<std::endl;
            /**----------------*/
            for(int i = 0;i< n;i++ )
            {
                if( buffer[i] == 0x43 )   // 寻找帧头  Cmd...
                {
                    if( buffer[i+1] == 0x6d )
                        if( buffer[i+2] == 0x64 )
                            if( buffer[i+3] == 0x4d )
                                if( buffer[i+4] == 0x3a )
                                    if( buffer[i+5] == 0x34 )
                                    {
                                        for( int j = 0; j<101 ; j++ )
                                            revData.buf[j] = buffer[i+j]; 
                                        break;
                                    }
                }
            }
            for(int i = 0;i< n1;i++ )
            {
                if( buffer1[i] == 0x43 )    // 寻找帧头  Cmd...
                {
                    if( buffer1[i+1] == 0x6d )
                        if( buffer1[i+2] == 0x64 )
                            if( buffer1[i+3] == 0x4d )
                                if( buffer1[i+4] == 0x3a )
                                    if( buffer1[i+5] == 0x34 )
                                    {
                                        for( int j = 0; j<101 ; j++ )
                                            revData1.buf[j] = buffer1[i+j]; 
                                        break;
                                    }
                }
            }
            std::cout<<"  得到一帧UWB数据 "<<std::endl;
            /* 变量定义 */
            vector<vector<double> > Ptag( 2,vector<double>(3) );    // 标签坐标变量定义
            vector<vector<double> > Ptag_ls( 2,vector<double>(3) );
            vector<vector<double> > Panchor = { {0,0,0},{0,1.4,0},{-0.8,1.4,0},{-0.8,0,0} }; // 基站坐标
            vector<vector<double> > r( 2,vector<double>(4) );   // 测量距离
            double d = 0.39;    //两个UWB标签之间的间距
            /*********
             *  串口数据 赋值
             * ********/
            if(  revData.data.kalmanrange[0] !=0 && revData.data.kalmanrange[1] !=0 &&
                 revData.data.kalmanrange[2] !=0 && revData.data.kalmanrange[3] !=0 &&
                 revData1.data.kalmanrange[0] !=0 && revData1.data.kalmanrange[1] !=0 &&
                 revData1.data.kalmanrange[2] !=0 && revData1.data.kalmanrange[2] !=0 
                )
            {
                for(int j = 0;j<4;j++)   //  单位转换 m米
                    r[0][j] = revData.data.kalmanrange[j]*0.001;
                for(int j = 0;j<4;j++)
                    r[1][j] = revData1.data.kalmanrange[j]*0.001;
                Ptag_ls[0] = TOA_LS(Panchor,r[0]);  // 最小二乘法求初值
                Ptag_ls[1] = TOA_LS(Panchor,r[1]);
                Ptag[0]    = Ptag_ls[0];
                Ptag[1]    = Ptag_ls[1];
                Ptag[0][2] = height_above_takeoff_.data;    //高度方向上赋值                
                Ptag[1][2] = height_above_takeoff_.data;
                Ptag = gd_test( Ptag, Panchor,  r , d);     // 本文算法 求解

                if( abs( Ptag[0][0] ) >100 |  abs( Ptag[0][1] ) >100 |
                    abs( Ptag[1][0] ) >100 |  abs( Ptag[1][1] ) >100 |
                    abs( Ptag[0][2] ) >100 |  abs( Ptag[1][2] ) >100 )   
                {
                    // std::cout<<"  计算异常 0 "<<std::endl;
                    Ptag[0]    = Ptag_ls[0];
                    Ptag[1]    = Ptag_ls[1];
                    Ptag[0][2] = height_above_takeoff_.data; 
                    Ptag[1][2] = height_above_takeoff_.data;
                    if( abs( Ptag[0][0] ) >100 |  abs( Ptag[0][1] ) >100 |
                        abs( Ptag[1][0] ) >100 |  abs( Ptag[1][1] ) >100 |
                        abs( Ptag[0][2] ) >100 |  abs( Ptag[1][2] ) >100 )   
                    {
                        // std::cout<<"  计算异常 1 "<<std::endl;
                        continue; // 计算结果异常
                    }
                }                
               
            }
        }
        // std::cout<<" 测试点 6 "<<std::endl;
        ros::spinOnce();
        // std::cout<<" 测试点 7"<<std::endl;
        loop_rate.sleep();
        // std::cout<<" 测试点 8 "<<std::endl;
    }
    //关闭串口
    sp.close();
    sp1.close();
    return 0;
}

/**
 * @brief 最小二乘法求坐标函数
 * 
 * @param Panchor  vector<vector<double> >
 * @param d     vector<double> 
 * @return ** vector<double> 
 */
vector<double>  TOA_LS(vector<vector<double> > Panchor,vector<double> d)
{
    /* 变量定义 */
    const int m = 4; // 四个基站
    vector<double> Ptag(3,0);   // 求解得到的坐标
    Eigen::Matrix<double, 3, 3> H; // 系数矩阵
    Eigen::Matrix<double, 3, 1> b; 
    Eigen::Matrix<double, 3, 1> E; // 存放结果
    Eigen::MatrixXd A(3,3);  //存放中间步骤计算结果
    b<<0,0,0; 
    for(int k =1;k<m;k++) // 对各个变量进行赋值
    {
        for(int i =1;i<=3;i++)
        {	
            H(k-1,i-1) = Panchor[k-1][i-1] - Panchor[m-1][i-1];
            b[k-1] = b[k-1] +
                     ( pow( Panchor[k-1][i-1] , 2 ) - pow( Panchor[m-1][i-1] ,2 ) );
	    }
        b[k-1] =0.5*( b[k-1] + pow( d[m-1] , 2 ) -pow( d[ k-1 ] , 2 ));
    }
	//std::cout<<H<<std::endl;
	A = H.transpose() * H;
	//std::cout<<A<<std::endl;
	A = pseudoInverse( A );
	//std::cout<<A<<std::endl;
	E =  H.transpose()*b;
	//std::cout<<"b :"<<b<<std::endl;
	//std::cout<<"E :"<<E<<std::endl;
	E = A*E;
	//std::cout<<"1:"<<E<<std::endl;

	Ptag[0] = E(0);     //将计算结果赋给Ptag 并返回
	Ptag[1] = E(1);
	Ptag[2] = E(2);
    return Ptag;
}
/*********************************************/
/**
 * 迭代求解tag 位置的函数
 *  input: 二维vector Ptag 2*3  两个标签的位置;
 *         二维vector Panchor 4*3 四个基站的位置
 *         二维vector r 2*4 两个标签到各个基站距离
 *         double d 两个标签之间的距离
 *  output: 标签坐标
 * **/
vector<vector<double> > gd_test( vector<vector<double> > Ptag, vector<vector<double> > Panchor, vector<vector<double> > r , double d)
{
	// int iteration_number = 0;
	double J =0;  // 定义代价函数变量
	double J_last = 0;

	vector<double> g1(3); // 定义偏导变量
	vector<double> g2(3);	
	J = cost(  Ptag,  Panchor , r ,d);
    
    // std::cout<< " 测试点 gd " <<std::endl;
	while(fabs(J - J_last) >0.001  && iteration_number < 100)// 迭代次数限定
	{
		J_last = J;
		
		g1 = grad1(  Ptag,  Panchor, r ,  d );  // 求偏导
		g2 = grad2(  Ptag,  Panchor, r ,  d );

		for(int i =0;i<3;i++)
		{
			Ptag[0][i] = Ptag[0][i]-0.001*g1[i];  //  步长 0.001
			Ptag[1][i] = Ptag[1][i]-0.001*g2[i];
		}
		J = cost(  Ptag,  Panchor , r ,d);   // 计算当前位置的代价函数值
		iteration_number++;
	}
	cout<<" iteration_number: "<< iteration_number <<endl;
	return Ptag;
}
/**
*   vector<double> distance_error函数用于求 || px-py ||2 - d2
*   input: 当前ptag、基站坐标，测距值、标签间距
**/
vector<double> distance_error(vector<double> px , vector<vector<double> > py , vector<double> d )
{	
	const int chicun = py.size();
	vector<double> f(chicun);
	Eigen::Matrix<double, 1, 3> px_mat;

	Eigen::MatrixXd py_mat(1,1);
	py_mat.resize(chicun,3);

	const int d_size = d.size();
	Eigen::MatrixXd d_mat(1,1);
	d_mat.resize(1,d_size);
	for(int i =0;i<3;i++)//遍历赋值矩阵
	{
		px_mat(0,i) = px[i];
		for(int j = 0 ; j<4;j++)
		{
			if( chicun ==4 ) //如果输入的 tag 和 tag 矩阵大小就会变化
			{
				if(i == 0)  d_mat(0,j) = d[j];	
				py_mat(j,i) = py[j][i];		
			}				
			else break;
		}
		if(chicun !=4)  
		{
			py_mat(0,i) = py[0][i];
			d_mat(0) = d[0];		
		}
	}
	if(chicun ==4)
		for(int k = 0;k<4;k++)
			f[k] = pow( (px_mat - py_mat.block(k,0,1,3)).norm(),2) - pow(d_mat(k) ,2);
	else f[0] = pow( (px_mat - py_mat).norm(),2) - pow(d_mat(0),2);
	return f;
}
/***********************************************/
/**
*   double cost函数用于求代价函数的值
*   input: 当前ptag、基站坐标，测距值、标签间距
**/
double cost( vector<vector<double> > Ptag, vector<vector<double> > Panchor ,vector<vector<double> > r , double d)
{	// vector  Ptag 2*3  Panchor4*3 r2*4  d 1*1  向量维度
	// const int n =2;
	// const int m =4;
	double j = 0;
	vector<double> d_vector(1);
	d_vector[0] = d;
	
	vector<double> v = distance_error( Ptag[0], Panchor,r[0]);   // 求 pow(|| px-py || ,2) - pow( d,2 )
	//cout << "进入cost函数:  "<< endl;  
	for( auto &i : v )  // 每个元素进行平方
	{
	    i *= i;  
	}
	//cout << "进入cost函数:1  "<< endl;
	vector<double> v1 = distance_error( Ptag[1], Panchor,r[1]);  
	for( auto &i : v1 )  // 每个元素进行平方
	{  
	    i *= i;  
	}
	vector<vector<double> > pp ={Ptag[1]};   //  代表 第二个标签的坐标
	//j = accumulate(v.begin(),v.end(),0) + accumulate(v1.begin(),v1.end(),0) +200* pow(distance_error( Ptag[0], pp,d_vector)[0],2);
    	//j = accumulate(v.begin(),v.end(),0);
	j = v[0] + v[1]+v[2]+v[3];
	//cout << " j = "<< j<<endl;  
	//j =j + accumulate(v1.begin(),v1.end(),0);
	j = j + v1[0] + v1[1]+v1[2]+v1[3];  //  到这里 只求了 J 多项式函数的第二项
	//cout << " j = "<< j<<endl; 
	j = j+200* pow( distance_error( Ptag[0], pp,d_vector )[0],2 );  // 求总的 J 值
	//cout << " j = "<< j<<endl; 
    return j;
}

/***********************************************/
/**
*   vector<double> grad1  对0号标签进行求偏导
*
**/
vector<double> grad1( vector<vector<double> > Ptag, vector<vector<double> > Panchor, vector<vector<double> > r , double d )
{
    // 变量定义
	Eigen::Matrix<double, 2, 3> Ptag_mat;   // 用来装UWB标签坐标
	Eigen::Matrix<double, 4, 3> Panchor_mat;    //用来装基站坐标
	Eigen::Matrix<double, 1, 3> g1_mat;
	Eigen::Matrix<double, 1, 4> temp_mat;
	Eigen::Matrix<double, 4, 3> temp_mat1;

	vector<double> d_vector(1);
	d_vector[0] = d;

	for(int i = 0;i<4;i++)
		for(int j=0;j<3;j++)
		{
			if(i<2) Ptag_mat(i,j) = Ptag[i][j];
			//cout << "运行到这儿: grad1_1 "<< endl;
			Panchor_mat(i,j) = Panchor[i][j];
			//cout << "运行到这儿: grad1_5 "<< endl;
		}
	//cout << "运行到这儿: grad1_4 "<< endl;
	vector<vector<double> > pp ={Ptag[1]};
	for(int i =0; i<4;i++)
	{
		//cout << "运行到这儿: grad1_2 "<< endl;
		temp_mat(i) = 4*distance_error( Ptag[0], Panchor,r[0])[i];
		//cout << "运行到这儿: grad1_3 "<< endl;
		//cout << "运行到这儿:  for循环为: "<< i << endl;
		temp_mat1.row(i) = Ptag_mat.row(0)-Panchor_mat.row(i);
		//cout << "运行到这儿: grad1_4 "<< endl;
	}
	g1_mat = 800*distance_error( Ptag[0], pp,d_vector)[0]*( Ptag_mat.row(0) - Ptag_mat.row(1)) +  temp_mat*temp_mat1 ;

	vector<double> g1= { g1_mat(0),g1_mat(1),g1_mat(2) };	
	return g1;
}

/***********************************************/
/**
*       vector<double> grad2  对1号标签进行求偏导
*       与前面的 grad1 函数 类似
**/
vector<double> grad2( vector<vector<double> > Ptag, vector<vector<double> > Panchor, vector<vector<double> > r , double d )
{
	Eigen::Matrix<double, 2, 3> Ptag_mat;
	Eigen::Matrix<double, 4, 3> Panchor_mat;
	Eigen::Matrix<double, 1, 3> g2_mat;
	Eigen::Matrix<double, 1, 4> temp_mat;
	Eigen::Matrix<double, 4, 3> temp_mat1;

	vector<double> d_vector(1);
	d_vector[0] = d;

	for(int i = 0;i<4;i++)
		for(int j=0;j<3;j++)
		{
			if(i<2) Ptag_mat(i,j) = Ptag[i][j];
			Panchor_mat(i,j) = Panchor[i][j];
		}

	vector<vector<double> > pp ={Ptag[1]};
	
	for(int i =0; i<4;i++)
	{
		temp_mat(i) = 4*distance_error( Ptag[1], Panchor,r[1])[i];
		temp_mat1.row(i) = Ptag_mat.row(1)-Panchor_mat.row(i);
	}
	
	g2_mat = -800*distance_error( Ptag[0], pp,d_vector)[0]*( Ptag_mat.row(0) - Ptag_mat.row(1)) +  temp_mat*temp_mat1;

	vector<double> g2= { g2_mat(0),g2_mat(1),g2_mat(2) };	
	return g2;
}


/*

伪代码：UWB标签定位算法
输入: 标签0与标签1的一帧数据，两个标签的安装距离，无人机高度（  101字节*2 + 8字节 + 4字节  ）

从输入的一帧数据中提取两个标签的测距信息     32字节

对测距信息进行一维卡尔曼滤波            264字节

将一维卡尔曼滤波的测距信息代入线性最小二乘法，计算得到标签的xy坐标，无人机高度作为标签的z坐标   228字节


caculate:计算代价函数J的值

if( 迭代次数>100 )
{
	结束算法，输出此时计算得到的标签坐标
}
else
{
    if( 判断迭代收敛条件 |Jt - J(t-1)|<δ )
    {
	    结束算法，输出此时计算得到的标签坐标
    }
    else
    {
        修改当前标签坐标Pm= Pm - Δ * ∂J/∂Pm
        goto caculate;
    }
}








*/

















