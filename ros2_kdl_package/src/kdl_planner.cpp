#include "kdl_planner.h"


KDLPlanner::KDLPlanner(){}

KDLPlanner::KDLPlanner(double _maxVel, double _maxAcc)
{
    velpref_ = new KDL::VelocityProfile_Trap(_maxVel,_maxAcc);
}

//2a. store the arguments of the new constructors in the corresponding class variable
//traiettoria lineare con profilo trapezoidale
KDLPlanner::KDLPlanner(double _trajDuration, double _accDuration, Eigen::Vector3d _trajInit, Eigen::Vector3d _trajEnd)
{
    trajDuration_ = _trajDuration;
    accDuration_ = _accDuration;
    trajInit_ = _trajInit;
    trajEnd_ = _trajEnd;
    time_law = TRAPEZOIDAL;
    path_type = LINEAR;

}

//traiettoria lineare con profilo cubico
KDLPlanner::KDLPlanner(double _trajDuration, Eigen::Vector3d _trajInit, Eigen::Vector3d _trajEnd)
{
    trajDuration_ = _trajDuration;
    trajInit_ = _trajInit;
    trajEnd_ = _trajEnd;
    time_law = CUBIC;
    path_type = LINEAR;
}

//traiettoria circolare con profilo trapezoidale
KDLPlanner::KDLPlanner(double _trajDuration, double _accDuration, Eigen::Vector3d _trajInit, double _trajRadius)
{
    trajDuration_ = _trajDuration;
    accDuration_ = _accDuration;
    trajInit_ = _trajInit;
    trajRadius_ = _trajRadius;
    time_law = TRAPEZOIDAL;
    path_type = CIRCULAR;

}


//traiettoria circolare con profilo cubico
KDLPlanner::KDLPlanner(double _trajDuration, Eigen::Vector3d _trajInit, double _trajRadius)
{
    trajDuration_ = _trajDuration;
    trajInit_ = _trajInit;
    trajRadius_ = _trajRadius;
    time_law = CUBIC;
    path_type = CIRCULAR;
}

void KDLPlanner::CreateTrajectoryFromFrames(std::vector<KDL::Frame> &_frames,
                                            double _radius, double _eqRadius
                                            )
{
    path_ = new KDL::Path_RoundedComposite(_radius,_eqRadius,new KDL::RotationalInterpolation_SingleAxis());

    for (unsigned int i = 0; i < _frames.size(); i++)
    {
        path_->Add(_frames[i]);
    }
    path_->Finish();

    velpref_->SetProfile(0,path_->PathLength());
    traject_ = new KDL::Trajectory_Segment(path_, velpref_);
}

void KDLPlanner::createCircPath(KDL::Frame &_F_start,
                                KDL::Vector &_V_centre,
                                KDL::Vector& _V_base_p,
                                KDL::Rotation& _R_base_end,
                                double alpha,
                                double eqradius
                                )
{
    KDL::RotationalInterpolation_SingleAxis* otraj;
    otraj = new KDL::RotationalInterpolation_SingleAxis();
    otraj->SetStartEnd(_F_start.M,_R_base_end);
    path_circle_ = new KDL::Path_Circle(_F_start,
                                        _V_centre,
                                        _V_base_p,
                                        _R_base_end,
                                        alpha,
                                        otraj,
                                        eqradius);
    velpref_->SetProfile(0,path_circle_->PathLength());
    traject_ = new KDL::Trajectory_Segment(path_circle_, velpref_);
}

/*1a.Define a new KDLPlanner::trapezoidal_vel function that takes the current time t and the acceleration time tc as double arguments 
and returns three double variables s, s_dot and s_ddot that represent the curvilinear abscissa of your trajectory.*/
void KDLPlanner::trapezoidal_vel(double t_, double tc_, double tf_, double &s, double &s_dot, double &s_ddot) {
  double sc_ddot;
  sc_ddot=2/(tf_*tc_-std::pow(tc_,2));

  if(0 <= t_ && t_<= tc_)
  {
    s = 0.5*sc_ddot*std::pow(t_,2);
    s_dot = sc_ddot*t_;
    s_ddot = sc_ddot;
  }
  else if(t_ <= tf_-tc_)
  {
    s =   sc_ddot*tc_*(t_-tc_/2);
    s_dot = 0.5*sc_ddot;
    s_ddot = 0;
  }
  else if(t_ <= tf_)
  {
    s =  1 - 0.5*sc_ddot*std::pow(tf_-t_,2);
    s_dot = 0;
    s_ddot = 0;
  }
  else {
    std::cout<<"Error: t is greater than tf"<<std::endl;
  }

}

// Cubic polynomial function
/*1b.Create a function named KDLPlanner::cubic_polinomial that creates the cubic polynomial curvilinear abscissa for your trajectory.
    The function takes as argument a double t representing time and returns three double s, s_dot and s_ddot that represent the curvilinear abscissa of your trajectory.*/
void KDLPlanner::cubic_polynomial(double t_, double tf_, double &s, double &s_dot, double &s_ddot) {
   
    double s0,sf,v0,vf,a0,a1,a2,a3;
    
    //Initialize s0,sf,v0,vf
    s0=0;
    sf=1;
    v0=0;
    vf=0;

    //Calculate coefficients a0-a3
    a0 = s0;
    a1 = v0;
    a2 = (3 * (sf - s0) / std::pow(tf_,2)) - ((2 * v0 + vf) / tf_);
    a3 = (-2 * (sf - s0) / std::pow(tf_,3)) + ((v0 + vf) / std::pow(tf_,2));

    // Calculate position s(t)
    s = a3 * std::pow(t_,3) + a2 * std::pow(t_,2) + a1 * t_ + a0;

    // Calculate velocity s_dot(t)
    s_dot = 3 * a3 * std::pow(t_,2) + 2 * a2 * t_ + a1;

    // Calculate acceleration s_ddot(t)
    s_ddot = 6 * a3 * t_ + 2 * a2;
}

KDL::Trajectory* KDLPlanner::getTrajectory()
{
	return traject_;
}

trajectory_point KDLPlanner::compute_trajectory(double time)
{
  /* trapezoidal velocity profile with accDuration_ acceleration time period and trajDuration_ total duration.
     time = current time
     trajDuration_  = final time
     accDuration_   = acceleration time
     trajInit_ = trajectory initial point
     trajEnd_  = trajectory final point */

  // trajectory_point traj;

  // Eigen::Vector3d ddot_traj_c = -1.0/(std::pow(accDuration_,2)-trajDuration_*accDuration_)*(trajEnd_-trajInit_);

  // if(time <= accDuration_)
  // {
  //   traj.pos = trajInit_ + 0.5*ddot_traj_c*std::pow(time,2);
  //   traj.vel = ddot_traj_c*time;
  //   traj.acc = ddot_traj_c;
  // }
  // else if(time <= trajDuration_-accDuration_)
  // {
  //   traj.pos = trajInit_ + ddot_traj_c*accDuration_*(time-accDuration_/2);
  //   traj.vel = ddot_traj_c*accDuration_;
  //   traj.acc = Eigen::Vector3d::Zero();
  // }
  // else
  // {
  //   traj.pos = trajEnd_ - 0.5*ddot_traj_c*std::pow(trajDuration_-time,2);
  //   traj.vel = ddot_traj_c*(trajDuration_-time);
  //   traj.acc = -ddot_traj_c;
  // }
    double s, s_dot, s_ddot;
     trajectory_point traj;

    // Selezione del tipo di legge temporale
    if (time_law == CUBIC) {
        cubic_polynomial(time, trajDuration_, s, s_dot, s_ddot);
    } else if (time_law == TRAPEZOIDAL) {

        trapezoidal_vel(time, accDuration_, trajDuration_, s, s_dot, s_ddot);
    }

    // Selezione del tipo di percorso
    if (path_type == CIRCULAR) {
        // Applica la posizione curvilinea al percorso circolare
        traj.pos(0)=trajInit_(0);
        traj.pos(1)=trajInit_(1)-trajRadius_*cos(2*M_PI*s);
        traj.pos(2)=trajInit_(2)-trajRadius_*sin(2*M_PI*s);

        traj.vel(0)=0;
        traj.vel(1)=trajRadius_*sin(2*M_PI*s)*2*M_PI*s_dot;
        traj.vel(2)=-trajRadius_*cos(2*M_PI*s)*2*M_PI*s_dot;

        traj.acc(0)=0;
        traj.acc(1)=4 * std::pow(M_PI,2) * trajRadius_ * cos(2 * M_PI * s) * std::pow(s_dot,2)
              + 2 * M_PI * trajRadius_ * sin(2 * M_PI * s) * s_ddot;
        traj.acc(2) = 2 * M_PI * trajRadius_ * (2 * M_PI * sin(2 * M_PI * s) * std::pow(s_dot,2) - cos(2 * M_PI * s) * s_ddot);

    }
    else if (path_type == LINEAR) {
      //Posizione lungo la traiettoria impostata come interpolazione lineare tra trajInit_ e trajEnd_ in funzione di s.
      traj.pos = trajInit_ + (trajEnd_ - trajInit_) * s;

      // Velocità costante lungo la traiettoria
      traj.vel = s_dot * (trajEnd_ - trajInit_).normalized(); // Normalizzo per ottenere direzione costante

      // Accelerazione nulla lungo la traiettoria lineare
      traj.acc = Eigen::Vector3d::Zero();

    }

  return traj;

}