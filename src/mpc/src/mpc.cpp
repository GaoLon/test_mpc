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

    has_odom = false;
    receive_traj_ = false;
    max_csteer = max_dsteer * dt;
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
        has_odom = true;
    }
}

void MPC::rcvOdomCallBack(nav_msgs::OdometryPtr msg)
{
//   has_odom = true;
//   now_state.x = msg->pose.pose.position.x;
//   now_state.y = msg->pose.pose.position.y;
//   Eigen::Quaterniond q(msg->pose.pose.orientation.w,
// 			msg->pose.pose.orientation.x,
// 			msg->pose.pose.orientation.y,
// 			msg->pose.pose.orientation.z);
//   Eigen::Matrix3d R(q);
//   Eigen::Vector2d lvel(msg->twist.twist.linear.x,msg->twist.twist.linear.y);
//   now_state.theta = atan2(R.col(0)[1],R.col(0)[0]);
//   now_state.v = lvel.norm();
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

        for (int i=0; i<T; i++)
        {
            Eigen::Vector3d temp;
            double t_temp = t_cur;

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
            
            cout<<"xref: x="<<xref(0, i)<<"   y="<<xref(1, i)<<"   v="<<xref(2, i)<<"   theta="<<xref(3, i)<<endl;
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
    stateTrans(now_state, output(0, 0), output(1, 0));
    pub_fake_odom();
}

void MPC::getLinearModel(const MPCState& s, double delta)
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

void MPC::stateTrans(MPCState& s, double a, double delta)
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

void MPC::solveMPCOS(void)
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
    //solver.settings()->setVerbosity(false);
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

void MPC::solveMPC(void)
{
    cout<<"xref and dref:"<<endl;
    for(int i=0; i<T; i++)
    {
        cout<<xref(0, i)<<" "<<xref(1, i)<<" "<<xref(2, i)<<" "<<xref(3, i)<<endl;
        cout<<dref(0, i)<<" "<<dref(1, i)<<endl;
    }
    cout<<"now_state: "<<now_state.x<<" "<<now_state.y<<" "<<now_state.v<<" "<<now_state.theta<<endl;
    // one-order part
    const int dimx = 4 * T;
    const int dimu = 2 * T;
    const int nx = dimx + dimu;
    double c[nx] = {0};
    for (int i=0, j=0; i<dimx; i+=4, j++)
    {
        c[i] = -2 * Q[0] * xref(0, j);
        c[i+1] = -2 * Q[1] * xref(1, j);
        c[i+2] = -2 * Q[2] * xref(2, j);
        c[i+3] = -2 * Q[3] * xref(3, j);
    }
    cout<<"linear part:"<<endl;
    for(int i=0;i <nx; i++)
    {
        cout<<c[i]<<" ";
    }
    cout<<endl;

    // state and output limitation
    double xupp[nx] = {0};
    char ixupp[nx] = {0};
    double xlow[nx] = {0};
    char ixlow[nx] = {0};
    for (int i=0; i<dimx; i+=4)
    {
        xupp[i+2] = max_speed;
        xlow[i+2] = min_speed; 
        ixupp[i+2] = 1;
        ixlow[i+2] = 1;
    }
    for (int i=dimx; i< nx; i+=2)
    {
        xupp[i] = max_accel;
        xupp[i+1] = max_steer;
        xlow[i] = -max_accel;
        xlow[i+1] = -max_steer;
        ixupp[i] = ixlow[i] = ixupp[i+1] = ixlow[i+1]  = 1;
    }
    cout<<"xlimit: iup, up, ilow, low"<<endl;
    for(int i=0;i <nx; i++)
    {
        cout<<(int)ixupp[i]<<" "<<xupp[i]<<" "<<(int)ixlow[i]<<" "<<xlow[i]<<endl;
    }
    
    // second order part
    const int nnzQ = nx + 2*T - 2;
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
    Eigen::MatrixXd QQ(nx, nx);
    for (int i=0;i<nx;i++)
    {
        for (int j=0;j<nx;j++)
        {
            QQ(i, j) = 0;
        }
    }
    for (int i=0; i<nnzQ; i++)
    {
        QQ(irowQ[i], jcolQ[i]) = dQ[i];
    }
    cout<<"Q Matrix:"<<endl;
    cout<<QQ<<endl;

    // equality constraints: model
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
    Eigen::MatrixXd AA(my, nx);
    for (int i=0;i<my;i++)
    {
        for (int j=0;j<nx;j++)
        {
            AA(i, j) = 0;
        }
    }
    for (int i=0; i<nnzA; i++)
    {
        AA(irowA[i], jcolA[i]) = dA[i];
    }
    cout<<"A Matrix:"<<endl;
    cout<<AA<<endl;
    cout<<"b Vector:"<<endl;
    for (int i=0; i<my; i++)
    {
        cout<<b[i]<<" ";
    }
    cout<<endl;

    // differential inequality constraints: limit \delta steer
    const int mz  = T - 1;
    double clow[mz];
    char  iclow[mz];
    double cupp[mz];
    char  icupp[mz];
    fill(iclow, iclow+mz, 1);
    fill(clow, clow+mz, -max_csteer);
    fill(icupp, icupp+mz, 1);
    fill(cupp, cupp+mz, max_csteer);
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
    Eigen::MatrixXd CC(mz, nx);
    for (int i=0;i<mz;i++)
    {
        for (int j=0;j<nx;j++)
        {
            CC(i, j) = 0;
        }
    }
    for (int i=0; i<nnzC; i++)
    {
        CC(irowC[i], jcolC[i]) = dC[i];
    }
    cout<<"C Matrix:"<<endl;
    cout<<CC<<endl;
    cout<<"C Matrix limit: irow row iup up"<<endl;
    for (int i=0; i<T-1; i++)
    {
        cout<<(int)iclow[i]<<" "<<clow[i]<<" "<<(int)icupp[i]<<" "<<cupp[i]<<endl;
    }

    // solve
    QpGenSparseMa27 * qp 
        = new QpGenSparseMa27( nx, my, mz, nnzQ, nnzA, nnzC );
    
    QpGenData      * prob = (QpGenData * ) qp->copyDataFromSparseTriple(
            c,      irowQ,  nnzQ,   jcolQ,  dQ,
            xlow,   ixlow,  xupp,   ixupp,
            irowA,  nnzA,   jcolA,  dA,     b,
            irowC,  nnzC,   jcolC,  dC,
            clow,   iclow,  cupp,   icupp );
    
    QpGenVars      * vars 
        = (QpGenVars *) qp->makeVariables( prob );
    QpGenResiduals * resid 
        = (QpGenResiduals *) qp->makeResiduals( prob );
    
    GondzioSolver  * s     = new GondzioSolver( qp, prob );
    
    // if( !quiet ) s->monitorSelf();
    int ierr = s->solve(prob,vars, resid);
    
    if( ierr == 0 ) {
        double result[nx] = {0};
        ROS_INFO("Solution: a0=%f     delta0=%f", result[dimx], result[dimx+1]);
        // vars->x->writefToStream( cout, "x[%{index}] = %{value}" );
        vars->x->copyIntoArray(result);
        for (int i=0; i<dimu; i+=2)
        {
            output(0, i) = result[dimx+i];
            output(1, i) = result[dimx+i+1];
        }
    } else {
        ROS_WARN("Could not solve the problem!!!");
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
        solveMPCOS();
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

    cmd.drive.speed = now_state.v + output(0, 0) * dt;
    cmd.drive.steering_angle = output(1, 0);
}
 
int main( int argc, char * argv[] )
{ 
  ros::init(argc, argv, "mpc_node");
  ros::NodeHandle nh("~");

  MPC mpc_tracker;

  mpc_tracker.init(nh);

  ros::spin();

  return 0;
}
