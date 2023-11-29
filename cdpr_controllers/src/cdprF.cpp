
#include <ros/ros.h>
#include <cdpr/cdpr.h>
#include <log2plot/logger.h>
#include <chrono>
#include <cdpr_controllers/butterworth.h>
#include <cdpr_controllers/tda.h>
#include <iostream>

#define max_(a,b) (((a) > (b)) ? (a) : (b))
#define min_(a,b) (((a) < (b)) ? (a) : (b))

using namespace std;

/*
 * Basic PID controller to show input/output of the CDPR class
 *
 * Does not consider dynamics except for gravity
 * Actual TDA depends on "control" parameter
 *
 */


void Param(ros::NodeHandle &nh, const string &key, double &val)
{
    if(nh.hasParam(key))
        nh.getParam(key, val);
    else
        nh.setParam(key, val);
}

int main(int argc, char ** argv)
{

    cout.precision(3);
    // init ROS node
    ros::init(argc, argv, "cdpr_control");
    ros::NodeHandle nh, nh_priv("~");

    // init CDPR class from parameter server
    CDPR robot(nh);
    const unsigned int n = robot.n_cables();
    robot.setDesiredPose(0,0,1,0,0,0);
    
    ifstream newfile("/home/yuelinzou/ubuntu/matlab/data.txt");

    // log path
    std::string path = "/home/yuelinzou/Results/cdpr/";

    double tauMin, tauMax;
    robot.tensionMinMax(tauMin, tauMax);


    // get control type    
    std::string control_type = "minW";
    double dTau_max = 0;
    bool warm_start = false;

    if(nh_priv.hasParam("control"))
    nh_priv.getParam("control", control_type);
    TDA::minType control = TDA::minT;
    if(control_type == "noMin")
        control = TDA::noMin;
    else if(control_type == "minT")
        control = TDA::minT;
    else if(control_type == "minW")
        control = TDA::minW;

    path += control_type;


    if(dTau_max)
        path += "_cont";
    if(!warm_start)
        path += "_nows";


    vpColVector tau(n);

    vpColVector g(6), err, err_i(6), err0(6), v(6), d_err(6), w(6),f;
    g[2] = - robot.mass() * 9.81;
    vpMatrix R_R(6,6), W(6,n);

    double dt = 0.079;
    ros::Rate loop(1/dt);
    vpHomogeneousMatrix M;
    vpRotationMatrix R;

    // gain
    double Kp = 300, Ki = 0.8, Kd = 2;  // tuned for Caroca
    Param(nh, "Kp", Kp);
    Param(nh, "Ki", Ki);
    Param(nh, "Kd", Kd);

    // variables to log
    log2plot::Logger logger(path + "_");
    double t;
    logger.setTime(t);
    vpPoseVector pose_err;
    //td::string name, const std::string legend, const std::string ylabel, const bool keep_file = true)
    logger.saveTimed(pose_err, "pose_err", "[x,y,z,\\theta_x,\\theta_y,\\theta_z]", "Pose error");
    logger.saveTimed(tau, "tau", "\\tau_", "Tensions");
    vpColVector residual(6);
    logger.saveTimed(residual, "res", "[f_x,f_y,f_z,m_x,m_y,m_z]", "Residuals");

    // chrono
    vpColVector comp_time(1);
    logger.saveTimed(comp_time, "dt", "[\\delta t]", "Comp. time");
    std::chrono::time_point<std::chrono::system_clock> start, end;
    std::chrono::duration<double> elapsed_seconds;

    // filter for d_error (dim. 6)
    Butterworth_nD filter(6, 1, dt);

    TDA tda(robot, nh, control);
    tda.ForceContinuity(dTau_max);

    cout << "CDPR control ready" << fixed << endl;

    //set the waypoints 
    XmlRpc::XmlRpcValue wp;
    nh_priv.getParam("/Tra/waypoints", wp);

    std::vector<std::vector<double>> waypoints(wp.size(),std::vector<double>(3,0));
    for(int i = 0; i < wp.size(); i++)
    {
        waypoints[i][0] = wp[i][0];
        waypoints[i][1] = wp[i][1];
        waypoints[i][2] = wp[i][2];
    }
    int indexpath = 0;

    double *velocity;
    velocity = robot.velocityPose;

    while(ros::ok())
    {
        nh.getParam("Kp", Kp);
        nh.getParam("Ki", Ki);
        nh.getParam("Kd", Kd);
        t = ros::Time::now().toSec();
        
        std::string line;
        getline(newfile, line);

        //if(!robot.ok())  // messages have been received
        {
            //cout<<tau<<"----------------1"<<endl;
            start = std::chrono::system_clock::now();
            // current position
            robot.getPose(M);
            M.extract(R);
            vpTranslationVector translation;
            M.extract(translation);

            if(abs(translation[0]+abs(translation[1])+abs(translation[2]))<1e-5){
                ros::spinOnce();
                continue;
            }

            //user define
            R.setIdentity();
            for(unsigned int i=0;i<3;++i)
                for(unsigned int j=0;j<3;++j)
                    R_R[i][j] = R_R[i+3][j+3] = R[i][j]; //R[i][j]
            //transform cartersion to joint space
            vpMatrix W(6, n),J(n,6);
            vpColVector v(6,0);

            for(int i = 0; i<3; i++){
                v[i] = waypoints[indexpath][i] - translation[i];
            }
            if(abs(v[2])+abs(v[1])+abs(v[0])<0.25) indexpath++;
            std::cout<<translation.t()<<std::endl;
            std::cout<<indexpath<<std::endl;
            std::cout<<v[0]<<" "<<v[1]<<" "<<v[2]<<std::endl;

            //obtain the velocity value
            double velpos = abs(velocity[0])+abs(velocity[1])+abs(velocity[2]);

            v[0] = 0.07*v[0]; //0.02/v[2]
            v[1] = 0.07*v[1];
            v[2] = 0.07*v[2];

            err = v;
            err = R_R * err;

            // I term to wrench in fixed frame
            for(unsigned int i=0;i<6;++i)
                if(w[i] < robot.mass()*9.81)
                    err_i[i] += err[i] * dt;

            w = Kp * (err + Ki*err_i);

            // D term?
            if(err0.infinityNorm())
            {
                // compute and filter error derivative
                d_err =(err - err0)/dt;
                filter.Filter(d_err);

                w += Kp * Kd * d_err;
            }

            err0 = err;

            // remove gravity + to platform frame
            w = R_R.t()*(w-g);

            // build W matrix depending on current attach points
            robot.computeW(W,R);

            // call cable tension distribution
            tau = tda.ComputeDistribution(W, w);

            // send tensions
            robot.sendTensions(tau);

            end = std::chrono::system_clock::now();
            elapsed_seconds = end-start;
        }

        ros::spinOnce();
        loop.sleep();
    }

}
