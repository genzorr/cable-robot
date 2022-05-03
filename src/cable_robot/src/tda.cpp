#include "cable_robot/tda.h"
#include <visp/vpIoTools.h>

// this script is associated with TDAs 
// note that due to the dimension problem, if run "cvxgen_minT" method, must comment "cvxgen_slack"
// and "adaptive_gains" code block, as well as uncomment relative header files.
// these code based on CVXGEN can be separated into different files in order to avoid the impact among them

// ***************************************************
//* uncomment for the slack varibles algorithm
//****************************************************
/*#include "../cvxgen_slack/solver.h"
#include "../cvxgen_slack/solver.c"
#include "../cvxgen_slack/matrix_support.c"
#include "../cvxgen_slack/util.c"
#include "../cvxgen_slack/ldl.c"*/

//************************************************************
//* minimize the cable tensions
//************************************************************
//#include "../cvxgen_minT/solver.h"
//#include "../cvxgen_minT/solver.c"
//#include "../cvxgen_minT/matrix_support.c"
//#include "../cvxgen_minT/util.c"
//#include "../cvxgen_minT/ldl.c"

//************************************************************
//* adaptive gains 
//************************************************************
/*#include "../cvxgen_gains/solver.h"
#include "../cvxgen_gains/solver.c"
#include "../cvxgen_gains/matrix_support.c"
#include "../cvxgen_gains/util.c"
#include "../cvxgen_gains/ldl.c"*/

// declare the namespaces used in CVXGEN code
//Vars vars;
//Params params;
//Workspace work;
//Settings settings;


using std::cout;
using std::endl;
using std::vector;


TDA::TDA(double mass, int nCables, double fMin, double fMax, minType _control, bool warm_start)
: m(mass), n(nCables), tauMin(fMin), tauMax(fMax)
{
    // indicator of number of interaton which has infeasible  tension
    index = 0;

    control = _control;
    update_d = false;

    x.resize(n);

    reset_active = !warm_start;
//    cout << "reset_active" << reset_active << endl;
    active.clear();

    // prepare variables
    if(control == minT)
    {
        // min |tau|
        //  st W.tau = w        // assume the given wrench is feasible dependent on the gains
        //  st t- < tau < tau+

        // min tau
        Q.eye(n);
        r.resize(n);
        // equality constraint
        A.resize(6,n);
        b.resize(6);
        // min/max tension constraints
        C.resize(2*n,n);
        d.resize(2*n);
        for(int i=0;i<n;++i)
        {
            C[i][i] = 1;
            d[i] = tauMax;
            C[i+n][i] = -1;
            d[i+n] = -tauMin;
        }
    }
    else if(control == minW)
    {
        // min |W.tau - w|      // does not assume the given wrench is feasible
        //   st t- < tau < t+

        Q.resize(6,n);
        r.resize(6);
        // no equality constraints
        A.resize(0,n);
        b.resize(0);
        // min/max tension constraints
        C.resize(2*n,n);
        d.resize(2*n);
        for(int i=0;i<n;++i)
        {
            C[i][i] = 1;
            d[i] = tauMax;
            C[i+n][i] = -1;
            d[i+n] = -tauMin;
        }
    }
    else if ( control == closed_form)
    {
        // tau=f_m+f_v
        //  f_m=(tauMax+tauMin)/2
        //  f=f_m- (W^+)(w+W*f_m)

        f_m.resize(n);
        f_v.resize(n);
        // no equality constraints
        d.resize(2 * n);
        for (unsigned int i = 0; i < n; ++i)
        {
            f_m[i] = (tauMax + tauMin) / 2;
            d[i] = tauMax;
            d[i + n] = -tauMin;
        }
    }
//    tau.init(x, 0, n);
}

vpColVector TDA::ComputeDistribution(vpMatrix &W, vpColVector &w)
{
    if(reset_active)
        for(int i=0;i<active.size();++i)
            active[i] = false;

    if(update_d && control != noMin && control != closed_form)
    {
        for(unsigned int i=0;i<n;++i)
        {
            d[i] = std::min(tauMax, tau[i]+dTau_max);
            d[i+n] = -std::max(tauMin, tau[i]-dTau_max);
        }
    }

    if(control == noMin)
        x = W.pseudoInverse() * w;
//    else if(control == minT)
//        solve_qp::solveQP(Q, r, W, w, C, d, x, active);
//    else if(control == minW)
//        solve_qp::solveQPi(W, w, C, d, x, active);
    // closed form
    else if( control == closed_form)
    {   
        // declaration 
//        cout << "Using closed form" << endl;
        int num_r, index=0;
        vpColVector fm(n), tau_(n), w_(6);
        vpMatrix W_(6,n);
        double range_lim, norm_2;
        x= f_m + W.pseudoInverse() * (w - (W*f_m));
        f_v= x- f_m;
        fm=f_m;
        //compute the range limit of f_v
        norm_2 = sqrt(f_v.sumSquare());
        w_=w; W_=W ; num_r= n-6;
        //cout << "redundancy " << num_r<< endl;
        range_lim= sqrt(m)*(tauMax+tauMin)/4;
        //cout << "the maximal limit" << range_lim<<endl;
       for (int i = 0; i < n; ++i)
        {
            // judge the condition
            if ( norm_2 <= range_lim )
            {                 
                if (  (x.getMaxValue() > tauMax || x.getMinValue() < tauMin ) && num_r >0  )
                {
                    // search the relevant element which is unsatisfied
                    for (int j = 0; j < n; ++j)
                      {  
                        if(  x[j] == x.getMinValue() && x.getMinValue() < tauMin )
                                i = j;
                        else if  (x[j] == x.getMaxValue() && x.getMaxValue() > tauMax)
                                i=j;
                     }
//                    cout << "previous tensions" << "  "<<x.t()<<endl;
//                    cout << " i"<<"  "<< i <<endl;
                    // reduce the redundancy order
                    num_r--;
//                    cout << "number of redundancy"<<"  "<< num_r<<endl;
                    // re- calculate the external wrench with maximal element
                    if ( x.getMaxValue() > tauMax)
                    {
                        w_= -tauMax*W_.getCol(i)+w_;
                        tau_[i]=tauMax;
                    }
                    else 
                    {
                        w_= - tauMin * W_.getCol(i)+w_;
                        tau_[i]=tauMin;
                    }
//                    cout << "torque" << tau_.t() << endl;

                    fm[i]=0;

                    // drop relative column
                    W_[0][i]=W_[1][i]=W_[2][i]=W_[3][i]=W_[4][i]=W_[5][i]=0;

                    //compute the tensions again without unsatisfied component
                    x = fm + W_.pseudoInverse()*(w_- (W_*fm));

                    // construct the latest TD with particular components which equal to minimum and maximum
                    x=tau_+x;
//                    cout << "tensions" << x.t() << endl;

                    // compute the force limit
                    f_v = x - f_m;
                    norm_2 = sqrt(f_v.sumSquare());
                    // initialize the index in order to inspect from the first electment
                    i = 0;
                }
//                else if(num_r <0)
//                    cout << "no feasible redundancy existing" << endl;
            }
//            else
//                cout << "no feasible tension distribution" << endl;
        }
    }
    else
        cout << "No appropriate TDA " << endl;
//    cout << "check constraints :" << endl;
//    for(int i=0;i<n;++i)
//        cout << "   " << -d[i+n] << " < " << tau[i] << " < " << d[i] << std::endl;
    update_d = dTau_max;
    return x; //tau
}

vpColVector TDA::ComputeDistributionG(vpMatrix &W, vpColVector &ve, vpColVector &pe, vpColVector &w )
{   
    cout << " using variational gains algorithm based on quadratic problem" <<endl;

    //*********************************************************
    /*
    // save the matrices
    vpMatrix::saveMatrixYAML("/home/" + vpIoTools::getUserName() + "/Results/matrices/Q_matrix", Q);
    vpMatrix::saveMatrixYAML("/home/" + vpIoTools::getUserName() + "/Results/matrices/A_matrix", A);
    vpMatrix::saveMatrixYAML("/home/" + vpIoTools::getUserName() + "/Results/matrices/r_matrix", r);
    vpMatrix::saveMatrixYAML("/home/" + vpIoTools::getUserName() + "/Results/matrices/b_matrix", b);
    vpMatrix::saveMatrixYAML("/home/" + vpIoTools::getUserName() + "/Results/matrices/d_matrix", d);
    */
    //**********************************************
    /*        
    std_msgs::Float32MultiArray msg;
    msg.data.resize(54);
    for(int i=0; i<6 ;++i)
    {
        msg.data[9*i] = W[i][0];
        msg.data[9*i+1] = W[i][1];
        msg.data[9*i+2] = W[i][2];
        msg.data[9*i+3] = W[i][3];
        msg.data[9*i+4] = W[i][4];
        msg.data[9*i+5] = W[i][5];
        msg.data[9*i+6] = W[i][6];
        msg.data[9*i+7] = W[i][7];
        msg.data[9*i+8] = w[i];
    }
    bary_pub.publish(msg);
    */
    //*********************************************
    /*    
    int num_iters;
    int Kp =80, Kd = 20;
    for (int i = 0; i < 144; i++)
    {   
        if (  i < 100) //(78 for 10 variables)
        {   // D matrix for tau solution
            if ( i%13 == 0)
                params.Q[i] = 1/(tauMin*tauMin);
            else
                params.Q[i] = 0.0; 
        }
        else 
        {    // D matrix for slack variable
            if ( i%13 == 0)
                params.Q[i] =1/25;
            else
                params.Q[i] = 0.0; 
        }   
    }
    // declare the C vector
    for (int i = 0; i < 12; ++i)   
        params.c[i]=0;
    // declare the A array, the entries are defined as the column order
    int k=0;
    // A matrix A=[ W -x -xd ]  6x10
    for (int j = 0; j < 12; ++j)
    { 
        for (int i = 0; i <6; ++i)
        {   
            if (j < 8)
                params.A[k] = W[i][j];
            else if ( j == 8 && i < 3)
                params.A[k] = -pe[i];
            else if ( j == 9 && i < 3)
                params.A[k] = -ve[i];
            else if ( j == 10 && i > 2)
                params.A[k] = -pe[i];
            else if ( j == 11 && i > 2)
                params.A[k] = -ve[i];
            else
                params.A[k] = 0;
            k++;     
        }
    }

    // the right side of equality constraints 
        for (int i = 0; i < 3; ++i)
            params.b[i] = w[i] + Kp*pe[i] + Kd*ve[i];
    
        for (int i = 3; i < 6; ++i)
            params.b[i] = w[i] + Kp*pe[i] + Kd*ve[i];

    set_defaults();
    setup_indexing();
    // solve problem instance for the record. 
    settings.verbose = 1;
    num_iters = solve();
    for (int i = 0; i < 12; i++)
    {
        printf("  %9.4f\n", vars.x[i]);
        x[i]=vars.x[i];
    }
    for (int i = 0; i < 3; ++i)
    {
        w[i]+=(x[8]+Kp)*pe[i]+(x[9]+Kd)*ve[i];
        w[i+3]+=(x[10]+Kp)*pe[i+3]+(x[11]+Kd)*ve[i+3];
    }
    w_d = W*tau-w;
*/
    cout << "check constraints :" << endl;
    for(int i=0;i<n;++i)
        cout << "   " << -d[i+n] << " < " << tau[i] << " < " << d[i] << std::endl;
    //update_d = dTau_max;
    return tau;
}

