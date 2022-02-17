#include "mpc.h"
 
using namespace std;
 
void MPC::init(ros::NodeHandle &nh)
{
    nh.param("mpc/du_threshold", du_th, -1.0);
    nh.param("mpc/dt", dt, -1.0);
    nh.param("mpc/whell_base", wheel_base, -1.0);
    nh.param("mpc/max_iter", max_iter, -1);
    nh.param("mpc/predict_steps", T, -1);
    nh.param("mpc/max_steer", max_steer, -1.0);
    nh.param("mpc/max_dsteer", max_dsteer, -1.0);
    nh.param("mpc/max_speed", max_speed, -1.0);
    nh.param("mpc/min_speed", min_speed, -1.0);
    nh.param("mpc/max_accel", max_accel, -1.0);
    nh.param("mpc/in_test", in_test, false);
    nh.param("mpc/control_a", control_a, false);

    has_odom = false;
    receive_traj_ = false;
    max_csteer = max_dsteer * dt;
    max_cv = max_accel * dt;
    xref = Eigen::Matrix<double, 4, 50>::Zero();
    last_output = output = dref = Eigen::Matrix<double, 2, 50>::Zero();

    pos_cmd_pub_ = nh.advertise<ackermann_msgs::AckermannDriveStamped>("position_cmd", 10);
    vis_pub = nh.advertise<visualization_msgs::Marker>("/following_path", 10);
    fake_odom_pub = nh.advertise<nav_msgs::Odometry>("odometry", 1);
    cmd_timer_ = nh.createTimer(ros::Duration(dt), &MPC::cmdCallback, this);
    // cmd_timer_ = nh.createTimer(ros::Duration(0.02), &MPC::cmdCallback, this);
    odom_sub_ = nh.subscribe("odom_world", 1, &MPC::rcvOdomCallBack, this);

    if (in_test)
    {
        csp_path = csp.get_path();
        traj_duration_ = csp.get_duration();
        start_time_ = ros::Time::now();
        t_track = 0.0;
        receive_traj_ = true;
    }
}

void MPC::rcvOdomCallBack(nav_msgs::OdometryPtr msg)
{
    has_odom = true;
    now_state.x = msg->pose.pose.position.x;
    now_state.y = msg->pose.pose.position.y;
    Eigen::Quaterniond q(msg->pose.pose.orientation.w,
                msg->pose.pose.orientation.x,
                msg->pose.pose.orientation.y,
                msg->pose.pose.orientation.z);
    Eigen::Matrix3d R(q);
    Eigen::Vector2d lvel(msg->twist.twist.linear.x,msg->twist.twist.linear.y);
    now_state.theta = atan2(R.col(0)[1],R.col(0)[0]);
    now_state.v = lvel.norm();

    double direction = atan2(lvel(1), lvel(0));
    if ((direction-now_state.theta)>M_PI/2)
    {
        now_state.v = -now_state.v;
    }
}

void MPC::cmdCallback(const ros::TimerEvent &e)
{
    drawFollowPath();
    // return;

    if (!has_odom && !receive_traj_)
        return;
    
    ros::Time time_now = ros::Time::now();
    double t_cur = (time_now - start_time_).toSec();

    // for test
    t_cur = get_nearest();
    ROS_INFO("state: x=%f, y=%f, v=%f, theta=%f", now_state.x, now_state.y, now_state.v, now_state.theta);

    if (t_cur <= traj_duration_ && t_cur >= 0.0)
    {
        // for (int i=0; i<T; i++)
        // {
        //     Eigen::Vector3d temp;
        //     if (t_cur + (i+1)*dt < traj_duration_)
        //         temp = csp.get_state(t_cur + (i+1)*dt);
        //     else
        //         temp = csp.get_state(traj_duration_ - dt);

        //     xref(0, i) = temp[0];
        //     xref(1, i) = temp[1];
        //     xref(2, i) = 10 / 3.6;
        //     xref(3, i) = temp[2];
        //     dref(0, i) = 0;
        //     dref(1, i) = 0;
        // }

        double t_temp = t_cur;
        for (int i=0; i<T; i++)
        {
            Eigen::Vector3d temp;

            if (now_state.v > 0)
            {
                t_temp+=now_state.v*dt;
            }

            if (t_temp < traj_duration_)
            {
                temp = csp.get_state(t_temp);
                xref(2, i) = 10 / 3.6;
            }
            else
            {
                temp = csp.get_state(traj_duration_);
                xref(2, i) = 0;
            }

            xref(0, i) = temp[0];
            xref(1, i) = temp[1];
            xref(3, i) = temp[2];

            if (control_a)
                dref(0, i) = 0.0;
            else
                dref(0, i) = xref(2, i);

            // cout<<"xref: x="<<xref(0, i)<<"   y="<<xref(1, i)<<"   v="<<xref(2, i)<<"   theta="<<xref(3, i)<<endl;
        }
        smooth_yaw();
        getCmd();
    }
    else if (t_cur > traj_duration_)
    {
        cmd.drive.speed = 0.0;
        cmd.drive.steering_angle = 0.0;
    }
    else
    {
        cout << "[Traj server]: invalid time." << endl;
    }
    pos_cmd_pub_.publish(cmd);
    ROS_INFO("in MPC, the cmd is: a=%f, steer=%f", cmd.drive.acceleration, cmd.drive.steering_angle);
    pub_fake_odom();
}

void MPC::getLinearModel(const MPCState& s, double delta)
{
    if (control_a)
    {
        A = Eigen::Matrix4d::Identity();
        A(0, 2) = dt * cos(s.theta);
        A(1, 2) = dt * sin(s.theta);
        A(0, 3) = -s.v * A(1, 2);
        A(1, 3) = s.v * A(0, 2);
        A(3, 2) = dt * tan(delta) / wheel_base;

        B = Eigen::Matrix<double, 4, 2>::Zero();
        B(2, 0) = dt;
        B(3, 1) = dt * s.v / (wheel_base * pow(cos(delta), 2));

        C = Eigen::Vector4d::Zero();
        C(0) = -A(0, 3) * s.theta; 
        C(1) = -A(1, 3) * s.theta; 
        C(3) = -B(3, 1) * delta; 
    }
    else
    {
        B = Eigen::Matrix<double, 3, 2>::Zero();
        B(0, 0) = cos(s.theta) * dt;
        B(1, 0) = sin(s.theta) * dt;
        B(2, 0) = dt * tan(delta) / wheel_base;
        B(2, 1) = dt * s.v / (wheel_base * pow(cos(delta), 2));

        A = Eigen::Matrix3d::Identity();
        A(0, 2) = -B(1, 0) * s.v;
        A(1, 2) = B(0, 0) * s.v;

        C = Eigen::Vector3d::Zero();
        C(0) = -A(0, 2) * s.theta; 
        C(1) = -A(1, 2) * s.theta; 
        C(2) = -B(2, 1) * delta; 
    }
}

void MPC::stateTrans(MPCState& s, double a, double delta)
{
    if (control_a)
    {
        if (delta >= max_steer)
        {
            delta = max_steer;
        }else if (delta<= - max_steer)
        {
            delta = -max_steer;
        }

        s.x = s.x + s.v * cos(s.theta) * dt;
        s.y = s.y + s.v * sin(s.theta) * dt;
        s.theta = s.theta + s.v / wheel_base * tan(delta) * dt;
        s.v = s.v + a * dt;

        if (s.v >= max_speed)
        {
            s.v = max_speed;
        }else if (s.v<= min_speed)
        {
            s.v = min_speed;
        }
    }
    else
    {
        if (delta >= max_steer)
        {
            delta = max_steer;
        }else if (delta<= - max_steer)
        {
            delta = -max_steer;
        }
        if (s.v >= max_speed)
        {
            s.v = max_speed;
        }else if (s.v<= min_speed)
        {
            s.v = min_speed;
        }

        s.x = s.x + a * cos(s.theta) * dt;
        s.y = s.y + a * sin(s.theta) * dt;
        s.theta = s.theta + a / wheel_base * tan(delta) * dt;
    }
}

void MPC::predictMotion(void)
{
    xbar[0] = now_state;

    MPCState temp = now_state;
    for (int i=1; i<T+1; i++)
    {
        stateTrans(temp, output(0, i-1), output(1, i-1));
        xbar[i] = temp;
    }
}

void MPC::solveMPCA(void)
{
    const int dimx = 4 * T;
    const int dimu = 2 * T;
    const int nx = dimx + dimu;

    Eigen::SparseMatrix<double> hessian;
    Eigen::VectorXd gradient = Eigen::VectorXd::Zero(nx);
    Eigen::SparseMatrix<double> linearMatrix;
    Eigen::VectorXd lowerBound;
    Eigen::VectorXd upperBound;

    // first-order
    for (int i=0, j=0; i<dimx; i+=4, j++)
    {
        gradient[i] = -2 * Q[0] * xref(0, j);
        gradient[i+1] = -2 * Q[1] * xref(1, j);
        gradient[i+2] = -2 * Q[2] * xref(2, j);
        gradient[i+3] = -2 * Q[3] * xref(3, j);
    }

    // second-order
    const int nnzQ = nx + dimu - 2;
    int irowQ[nnzQ];
    int jcolQ[nnzQ];
    double dQ[nnzQ];
    for (int i=0; i<nx; i++)
    {
        irowQ[i] = jcolQ[i] = i;
    }
    for (int i=nx; i<nnzQ; i++)
    {
        irowQ[i] = i - dimu + 2;
        jcolQ[i] = i - dimu;
    }
    for (int i=0; i<dimx; i+=4)
    {
        dQ[i] = Q[0] * 2.0;
        dQ[i+1] = Q[1] * 2.0;
        dQ[i+2] = Q[2] * 2.0;
        dQ[i+3] = Q[3] * 2.0;
    }
    dQ[dimx] = dQ[nx-2] = (R[0] + Rd[0]) * 2.0;
    dQ[dimx + 1] = dQ[nx-1] = (R[1] + Rd[1]) * 2.0;
    for (int i=dimx+2; i<nx-2; i+=2)
    {
        dQ[i] = 2 * (R[0] + 2 * Rd[0]);
        dQ[i+1] = 2 * (R[1] + 2 * Rd[1]);
    }
    for (int i=nx; i<nnzQ; i+=2)
    {
        dQ[i] = -Rd[0] * 2.0;
        dQ[i+1] = -Rd[1] * 2.0;
    }
    hessian.resize(nx, nx);
    Eigen::MatrixXd QQ(nx, nx);
    for (int i=0; i<nx; i++)
    {
        hessian.insert(irowQ[i], jcolQ[i]) = dQ[i];
    }
    for (int i=nx; i<nnzQ; i++)
    {
        hessian.insert(irowQ[i], jcolQ[i]) = dQ[i];
        hessian.insert(jcolQ[i], irowQ[i]) = dQ[i];
    }

    // equality constraints
    MPCState temp = now_state;
    getLinearModel(temp, dref(1, 0));
    int my = dimx;
    double b[my];
    const int nnzA = 15 * T - 9;
    int irowA[nnzA];
    int jcolA[nnzA];
    double dA[nnzA];
    Eigen::Vector4d temp_vec(temp.x, temp.y, temp.v, temp.theta);
    Eigen::Vector4d temp_b = A*temp_vec + C;
    
    for (int i=0; i<dimx; i++)
    {
        irowA[i] = jcolA[i] = i;
        dA[i] = 1;
    }
    b[0] = temp_b[0];
    b[1] = temp_b[1];
    b[2] = temp_b[2];
    b[3] = temp_b[3];
    irowA[dimx] = 2;
    jcolA[dimx] = dimx;
    dA[dimx] = -B(2, 0);
    irowA[dimx+1] = 3;
    jcolA[dimx+1] = dimx+1;
    dA[dimx+1] = -B(3, 1);
    int ABidx = 11*T - 11;
    int ABbegin = dimx+2;
    for (int i=0, j=1; i<ABidx; i+=11, j++)
    {
        getLinearModel(xbar[j], dref(1, j));
        for (int k=0; k<4; k++)
        {
            b[4*j+k] = C[k];
            irowA[ABbegin + i + k] = 4*j + k;
            jcolA[ABbegin + i + k] = irowA[ABbegin + i + k] - 4;
            dA[ABbegin + i + k] = -A(k, k);
        }
        irowA[ABbegin + i + 4] = 4*j;
        jcolA[ABbegin + i + 4] = 4*j - 2;
        dA[ABbegin + i + 4] = -A(0, 2);
        
        irowA[ABbegin + i + 5] = 4*j;
        jcolA[ABbegin + i + 5] = 4*j - 1;
        dA[ABbegin + i + 5] = -A(0, 3);

        irowA[ABbegin + i + 6] = 4*j + 1;
        jcolA[ABbegin + i + 6] = 4*j - 2;
        dA[ABbegin + i + 6] = -A(1, 2);

        irowA[ABbegin + i + 7] = 4*j + 1;
        jcolA[ABbegin + i + 7] = 4*j - 1;
        dA[ABbegin + i + 7] = -A(1, 3);
        
        irowA[ABbegin + i + 8] = 4*j + 3;
        jcolA[ABbegin + i + 8] = 4*j - 2;
        dA[ABbegin + i + 8] = -A(2, 3);
        
        irowA[ABbegin + i + 9] = 4*j +2;
        jcolA[ABbegin + i + 9] = dimx + 2*j;
        dA[ABbegin + i + 9] = -B(2, 0);
        
        irowA[ABbegin + i + 10] = 4*j + 3;
        jcolA[ABbegin + i + 10] = dimx + 2*j + 1;
        dA[ABbegin + i + 10] = -B(3, 1);
    }

    // iequality constraints
    const int mz  = T - 1;
    const int nnzC = dimu - 2;
    int   irowC[nnzC];
    int   jcolC[nnzC];
    double   dC[nnzC];
    for (int i=0, j=0; i<mz; i++, j+=2)
    {
        irowC[j] = i;
        irowC[j+1] = i;
        jcolC[j] = dimx + 1 + j;
        jcolC[j+1] = jcolC[j] +2;
        dC[j] = -1.0;
        dC[j+1] = 1.0;
    }

    // xlimits and all
    int mx = 3*T;
    int nc = mx+my+mz;
    lowerBound.resize(nc);
    upperBound.resize(nc);
    linearMatrix.resize(nc, nx);
    for (int i=0, j=0, k=0; i<mx; i+=3, j+=4, k+=2)
    {
        lowerBound[i] = min_speed;
        lowerBound[i+1] = -max_accel;
        lowerBound[i+2] = -max_steer;
        upperBound[i] = max_speed;
        upperBound[i+1] = max_accel;
        upperBound[i+2] = max_steer;
        linearMatrix.insert(i, j+2) = 1;
        linearMatrix.insert(i+1, dimx+k) = 1;
        linearMatrix.insert(i+2, dimx+k+1) = 1;
    }
    for (int i=0; i<nnzA; i++)
    {
        linearMatrix.insert(irowA[i]+mx, jcolA[i]) = dA[i];
    }
    for (int i=0; i<my; i++)
    {
        lowerBound[mx+i] = upperBound[mx+i] = b[i];
    }
    for (int i=0; i<nnzC; i++)
    {
        linearMatrix.insert(irowC[i]+mx+my, jcolC[i]) = dC[i];
    }
    for (int i=0; i<mz; i++)
    {
        lowerBound[mx+my+i] = -max_csteer;
        upperBound[mx+my+i] = max_csteer;
    }

    // instantiate the solver
    OsqpEigen::Solver solver;

    // settings
    solver.settings()->setVerbosity(false);
    solver.settings()->setWarmStart(true);

    // set the initial data of the QP solver
    solver.data()->setNumberOfVariables(nx);
    solver.data()->setNumberOfConstraints(nc);
    if(!solver.data()->setHessianMatrix(hessian)) return;
    if(!solver.data()->setGradient(gradient)) return;
    if(!solver.data()->setLinearConstraintsMatrix(linearMatrix)) return;
    if(!solver.data()->setLowerBound(lowerBound)) return;
    if(!solver.data()->setUpperBound(upperBound)) return;

    // instantiate the solver
    if(!solver.initSolver()) return;

    // controller input and QPSolution vector
    Eigen::VectorXd QPSolution;

    // solve the QP problem
    if(solver.solveProblem() != OsqpEigen::ErrorExitFlag::NoError) return;

    // get the controller input
    QPSolution = solver.getSolution();
    ROS_INFO("Solution: a0=%f     delta0=%f", QPSolution[dimx], QPSolution[dimx+1]);
    for (int i=0; i<dimu; i+=2)
    {
        output(0, i) = QPSolution[dimx+i];
        output(1, i) = QPSolution[dimx+i+1];
    }
}

void MPC::solveMPCV(void)
{
    const int dimx = 3 * T;
    const int dimu = 2 * T;
    const int nx = dimx + dimu;

    Eigen::SparseMatrix<double> hessian;
    Eigen::VectorXd gradient = Eigen::VectorXd::Zero(nx);
    Eigen::SparseMatrix<double> linearMatrix;
    Eigen::VectorXd lowerBound;
    Eigen::VectorXd upperBound;

    // first-order
    for (int i=0, j=0; i<dimx; i+=3, j++)
    {
        gradient[i] = -2 * Q[0] * xref(0, j);
        gradient[i+1] = -2 * Q[1] * xref(1, j);
        gradient[i+2] = -2 * Q[3] * xref(3, j);
    }

    // second-order
    const int nnzQ = nx + dimu - 2;
    int irowQ[nnzQ];
    int jcolQ[nnzQ];
    double dQ[nnzQ];
    for (int i=0; i<nx; i++)
    {
        irowQ[i] = jcolQ[i] = i;
    }
    for (int i=nx; i<nnzQ; i++)
    {
        irowQ[i] = i - dimu + 2;
        jcolQ[i] = i - dimu;
    }
    for (int i=0; i<dimx; i+=3)
    {
        dQ[i] = Q[0] * 2.0;
        dQ[i+1] = Q[1] * 2.0;
        dQ[i+2] = Q[3] * 2.0;
    }
    dQ[dimx] = dQ[nx-2] = (R[0] + Rd[0]) * 2.0;
    dQ[dimx + 1] = dQ[nx-1] = (R[1] + Rd[1]) * 2.0;
    for (int i=dimx+2; i<nx-2; i+=2)
    {
        dQ[i] = 2 * (R[0] + 2 * Rd[0]);
        dQ[i+1] = 2 * (R[1] + 2 * Rd[1]);
    }
    for (int i=nx; i<nnzQ; i+=2)
    {
        dQ[i] = -Rd[0] * 2.0;
        dQ[i+1] = -Rd[1] * 2.0;
    }
    hessian.resize(nx, nx);
    Eigen::MatrixXd QQ(nx, nx);
    for (int i=0; i<nx; i++)
    {
        hessian.insert(irowQ[i], jcolQ[i]) = dQ[i];
    }
    for (int i=nx; i<nnzQ; i++)
    {
        hessian.insert(irowQ[i], jcolQ[i]) = dQ[i];
        hessian.insert(jcolQ[i], irowQ[i]) = dQ[i];
    }

    // equality constraints
    MPCState temp = now_state;
    getLinearModel(temp, dref(1, 0));
    int my = dimx;
    double b[my];
    const int nnzA = 12 * T - 5;
    int irowA[nnzA];
    int jcolA[nnzA];
    double dA[nnzA];
    Eigen::Vector3d temp_vec(temp.x, temp.y, temp.theta);
    Eigen::Vector3d temp_b = A*temp_vec + C;
    
    for (int i=0; i<dimx; i++)
    {
        irowA[i] = jcolA[i] = i;
        dA[i] = 1;
    }
    b[0] = temp_b[0];
    b[1] = temp_b[1];
    b[2] = temp_b[2];
    irowA[dimx] = 0;
    jcolA[dimx] = dimx;
    dA[dimx] = -B(0, 0);
    irowA[dimx+1] = 1;
    jcolA[dimx+1] = dimx;
    dA[dimx+1] = -B(1, 0);
    irowA[dimx+2] = 2;
    jcolA[dimx+2] = dimx;
    dA[dimx+2] = -B(2, 0);
    irowA[dimx+3] = 2;
    jcolA[dimx+3] = dimx+1;
    dA[dimx+3] = -B(2, 1);
    int ABidx = 9*T - 9;
    int ABbegin = dimx+4;
    for (int i=0, j=1; i<ABidx; i+=9, j++)
    {
        getLinearModel(xbar[j], dref(1, j));
        for (int k=0; k<3; k++)
        {
            b[3*j+k] = C[k];
            irowA[ABbegin + i + k] = 3*j + k;
            jcolA[ABbegin + i + k] = irowA[ABbegin + i + k] - 3;
            dA[ABbegin + i + k] = -A(k, k);
        }
        irowA[ABbegin + i + 3] = 3*j;
        jcolA[ABbegin + i + 3] = 3*j - 1;
        dA[ABbegin + i + 3] = -A(0, 2);

        irowA[ABbegin + i + 4] = 3*j + 1;
        jcolA[ABbegin + i + 4] = 3*j - 1;
        dA[ABbegin + i + 4] = -A(1, 2);
        
        irowA[ABbegin + i + 5] = 3*j;
        jcolA[ABbegin + i + 5] = dimx + 2*j;
        dA[ABbegin + i + 5] = -B(0, 0);
        
        irowA[ABbegin + i + 6] = 3*j + 1;
        jcolA[ABbegin + i + 6] = dimx + 2*j;
        dA[ABbegin + i + 6] = -B(1, 0);

        irowA[ABbegin + i + 7] = 3*j + 2;
        jcolA[ABbegin + i + 7] = dimx + 2*j;
        dA[ABbegin + i + 7] = -B(2, 0);
        
        irowA[ABbegin + i + 8] = 3*j + 2;
        jcolA[ABbegin + i + 8] = dimx + 2*j + 1;
        dA[ABbegin + i + 8] = -B(2, 1);
    }

    // iequality constraints
    const int mz  = 2 * T - 2;
    const int nnzC = 2 * dimu - 4;
    int   irowC[nnzC];
    int   jcolC[nnzC];
    double   dC[nnzC];
    for (int i=0, k=0; i<mz; i+=2, k+=4)
    {
        irowC[k] = i;
        jcolC[k] = dimx  + i;
        dC[k] = -1.0;

        irowC[k+1] = i;
        jcolC[k+1] = jcolC[k] +2;
        dC[k+1] = 1.0;

        irowC[k+2] = i + 1;
        jcolC[k+2] = dimx + 1 + i;
        dC[k+2] = -1.0;

        irowC[k+3] = i + 1;
        jcolC[k+3] = jcolC[k+2] +2;
        dC[k+3] = 1.0;
    }

    // xlimits and all
    int mx = dimu;
    int nc = mx+my+mz;
    lowerBound.resize(nc);
    upperBound.resize(nc);
    linearMatrix.resize(nc, nx);
    for (int i=0; i<mx; i+=2)
    {
        lowerBound[i] = min_speed;
        lowerBound[i+1] = -max_steer;
        upperBound[i] = max_speed;
        upperBound[i+1] = max_steer;
        linearMatrix.insert(i, dimx+i) = 1;
        linearMatrix.insert(i+1, dimx+i+1) = 1;
    }

    for (int i=0; i<nnzA; i++)
    {
        linearMatrix.insert(irowA[i]+mx, jcolA[i]) = dA[i];
    }

    for (int i=0; i<my; i++)
    {
        lowerBound[mx+i] = upperBound[mx+i] = b[i];
    }

    for (int i=0; i<nnzC; i++)
    {
        linearMatrix.insert(irowC[i]+mx+my, jcolC[i]) = dC[i];
    }

    for (int i=0; i<mz; i+=2)
    {
        lowerBound[mx+my+i] = -max_cv;
        upperBound[mx+my+i] = max_cv;
        lowerBound[mx+my+i+1] = -max_csteer;
        upperBound[mx+my+i+1] = max_csteer;
    }

    // instantiate the solver
    OsqpEigen::Solver solver;

    // settings
    solver.settings()->setVerbosity(false);
    solver.settings()->setWarmStart(true);

    // set the initial data of the QP solver
    solver.data()->setNumberOfVariables(nx);
    solver.data()->setNumberOfConstraints(nc);
    if(!solver.data()->setHessianMatrix(hessian)) return;
    if(!solver.data()->setGradient(gradient)) return;
    if(!solver.data()->setLinearConstraintsMatrix(linearMatrix)) return;
    if(!solver.data()->setLowerBound(lowerBound)) return;
    if(!solver.data()->setUpperBound(upperBound)) return;

    // instantiate the solver
    if(!solver.initSolver()) return;

    // controller input and QPSolution vector
    Eigen::VectorXd QPSolution;

    // solve the QP problem
    if(solver.solveProblem() != OsqpEigen::ErrorExitFlag::NoError) return;

    // get the controller input
    QPSolution = solver.getSolution();
    ROS_INFO("Solution: v0=%f     delta0=%f", QPSolution[dimx], QPSolution[dimx+1]);
    for (int i=0; i<dimu; i+=2)
    {
        output(0, i) = QPSolution[dimx+i];
        output(1, i) = QPSolution[dimx+i+1];
    }
}

void MPC::getCmd(void)
{
    int iter;

    for (iter=0; iter<max_iter; iter++)
    {
        ROS_INFO("predit_motion");
        predictMotion();
        last_output = output;
        ROS_INFO("begin_solve");
        // solveMPC();
        if (control_a)
            solveMPCA();
        else
            solveMPCV();
        double du = 0;
        for (int i=0; i<output.cols(); i++)
        {
            du = du + fabs(output(0, i) - last_output(0, i))+ fabs(output(1, i) - last_output(1, i));
        }
        if (du <= du_th)
        {
            break;
        }
    }
    if (iter == max_iter)
    {
        ROS_WARN("MPC Iterative is max iter");
    }

    if (control_a)
        cmd.drive.acceleration = output(0, 0);
    else
        cmd.drive.speed = output(0, 0);
    cmd.drive.steering_angle = output(1, 0);
}
 
int main( int argc, char * argv[] )
{ 
  ros::init(argc, argv, "mpc_node");
  ros::NodeHandle nh("~");
  ros::Duration(2.0).sleep();

  MPC mpc_tracker;

  mpc_tracker.init(nh);

  ros::spin();

  return 0;
}
