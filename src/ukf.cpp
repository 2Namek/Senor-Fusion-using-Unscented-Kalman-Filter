#include "ukf.h"
#include "Eigen/Dense"
#include <iostream>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

/**
 * Initializes Unscented Kalman filter
 * This is scaffolding, do not modify
 */
UKF::UKF() {
	cout<< "Initialization started"<<endl;
   n_z = 3;
	 n_x = 5;
  n_aug = 7;
  n_sig = (2*7)+1;
  // if this is false, laser measurements will be ignored (except during init)
  use_laser_ = true;
  is_initialized_ = false;


  // if this is false, radar measurements will be ignored (except during init)
  use_radar_ = true;

  // initial state vector
  x_ = VectorXd(5);
  x_ << 0,0,0,0,0;
  x_aug = VectorXd(7);
 x_aug << 0,0,0,0,0,0,0;
  P_aug = MatrixXd(7, 7);
      
  Xsig_aug = MatrixXd(n_aug,n_sig);
  Xsig_aug.fill(0.0);
  Xsig_pred = MatrixXd(n_x,n_sig);
  Xsig_pred.fill(0.0);
  weights = VectorXd(n_sig);
  
  z_polar = MatrixXd(n_z, n_sig);
  z_polar.fill(0.0);
  z_ztransp = VectorXd(n_z);
  z_ztransp << 0,0,0;
  
  x_xtransp = VectorXd(n_x);
  x_xtransp << 0,0,0,0,0;
  S = MatrixXd(n_z,n_z);
   S.fill(0.0);
  
  x_delta = VectorXd(n_x);
  x_delta.fill(0.0);
  K = MatrixXd(n_x, n_z);
  
  K.fill(0.0);
  // initial covariance matrix
  P_ = MatrixXd(5, 5);
P_.fill(0.0);
 v=0;
 y=0;
 y_dot=0;
 err_acc=0;
 err_ang=0;
 a=0;
 b=0;
 c=0;
 d=0;
 e=0;
 lambda = 3 - n_aug;
     float w_i;
  for(int i=0;i<n_sig;i++)
  {
        int null = 0;
    if (i == null)
    {
        w_i =lambda/(lambda+n_aug);
    }
    else
    {
        w_i=1/(2*(lambda+n_aug));
    }
    weights(i) = w_i;
  }
 //std::cout<<"weights =  "<<weights<<std::endl;

  
  

  // Process noise standard deviation longitudinal acceleration in m/s^2
  std_a = 2.5;

  // Process noise standard deviation yaw acceleration in rad/s^2
  std_yawdd = 1;
  

  //DO NOT MODIFY measurement noise values below these are provided by the sensor manufacturer.
  // Laser measurement noise standard deviation position1 in m
  std_laspx_ = 0.15;

  // Laser measurement noise standard deviation position2 in m
  std_laspy_ = 0.15;

  // Radar measurement noise standard deviation radius in m
  std_radr_ = 0.3;

  // Radar measurement noise standard deviation angle in rad
  std_radphi_ = 0.03;

  // Radar measurement noise standard deviation radius change in m/s
  std_radrd_ = 0.3;
  //DO NOT MODIFY measurement noise values above these are provided by the sensor manufacturer.
    H_ = MatrixXd(2, 5);
   H_ << 1, 0, 0, 0,0,
		0, 1, 0, 0,0;
    R_ = MatrixXd(2, 2);				
	R_ <<   0.0225, 0,
    		0,0.0225;
	R = MatrixXd(n_z, n_z);
	R << std_radr_*std_radr_,0,0,
  0,std_radphi_*std_radphi_,0,
  0,0,std_radrd_*std_radrd_;
  /**
  TODO:

  Complete the initialization. See ukf.h for other member properties.

  Hint: one or more values initialized above might be wildly off...
  */
}

UKF::~UKF() {}

/**
 * @param {MeasurementPackage} meas_package The latest measurement data of
 * either radar or laser.
 */
void UKF::ProcessMeasurement(MeasurementPackage meas_package) {
  if (!is_initialized_) {
    //cout<< "inside not initialized"<<endl;
    x_ = VectorXd(5);
	x_ << 1, 1, 1, 1,1;
	

    if (meas_package.sensor_type_ == MeasurementPackage::RADAR) {
      
    }
    else if (meas_package.sensor_type_ == MeasurementPackage::LASER) {
      /**
      Initialize state.
      */
	 x_ << meas_package.raw_measurements_[0], meas_package.raw_measurements_[1],0, 0, 0;
	  cout<<x_<<endl;
    }
	
    // done initializing, no need to predict or update
	
	previous_timestamp_ = meas_package.timestamp_;
    is_initialized_ = true;
	cout<<is_initialized_<<endl;
    return;
  }

/**
 * Predicts sigma points, the state, and the state covariance matrix.
 * @param {double} delta_t the change in time (in seconds) between the last
 * measurement and this one.
 */
    count++;
   //cout<<count<<endl;
	double dt = (meas_package.timestamp_ - previous_timestamp_) / 1000000.0;	//dt - expressed in seconds
	previous_timestamp_ = meas_package.timestamp_;
	
	Prediction(dt);
	
	 if (meas_package.sensor_type_ == MeasurementPackage::RADAR){
	//cout<< "RADAR Skipped"<<endl;
	UpdateRadar(meas_package);
	 }
	 else{
		 
		UpdateLidar(meas_package);	 
	 }
  cout << "x_ = " << x_ << endl;
  cout << "P_ = " << P_ << endl;
  cout<<"------------------------------------"<<endl;
}

void UKF::Prediction(double delta_t) {
 
  /**
  TODO:

  Complete this function! Estimate the object's location. Modify the state
  
  vector, x_. Predict sigma points, the state, and the state covariance matrix.
  */
  //std::cout<<"Prediction Started............................................"<<std::endl;
  x_aug.head(5) << x_;
  x_aug(5)=0;
  x_aug(6)=0;
    
P_aug.block(0,0,5,5) = P_;
P_aug.block(0,5,5,2) <<  0,  0,
                         0,  0,
                         0,  0,
                         0,  0,
                         0,  0;
P_aug.block(5,0,2,5) <<         0,         0,         0,         0,         0,
                                0,         0,         0,         0,         0;
P_aug.block(5,5,2,2) <<   std_a*std_a,  0,
                          0,std_yawdd*std_yawdd;



 MatrixXd xx = MatrixXd(n_aug, n_aug);
//std::cout << "xxxxx"<<std::endl;
 for(int i=0;i<n_aug;i++)
 {
     xx.col(i) = x_aug;
 }

 MatrixXd A = P_aug.llt().matrixL();

  MatrixXd add = MatrixXd(7,7);
  MatrixXd sub = MatrixXd(7,7);
 
  
  float lam = sqrt(lambda+n_aug);
    add = xx + (lam*A);
   
    sub = xx - (lam*A);

    Xsig_aug.col(0)= x_aug;
    Xsig_aug.col(1)= add.col(0);
    Xsig_aug.col(2)= add.col(1);
    Xsig_aug.col(3)= add.col(2);
    Xsig_aug.col(4)= add.col(3);
    Xsig_aug.col(5)= add.col(4);
    Xsig_aug.col(6)= add.col(5);
    Xsig_aug.col(7)= add.col(6);
    Xsig_aug.col(8)= sub.col(0);
    Xsig_aug.col(9)= sub.col(1);
    Xsig_aug.col(10)= sub.col(2);
    Xsig_aug.col(11)= sub.col(3);
    Xsig_aug.col(12)= sub.col(4);
    Xsig_aug.col(13)= sub.col(5);
    Xsig_aug.col(14)= sub.col(6);
	
  //cout << "xaug = " << Xsig_aug << endl;
  //.................................//
  
  
 
  
for(int i=0;i<2 * n_aug + 1;i++)
{


  v = Xsig_aug(2,i);
  y = Xsig_aug(3,i);
  y_dot = Xsig_aug(4,i);
  err_acc = Xsig_aug(5,i);
  err_ang = Xsig_aug(6,i);
  
int null = 0;
if (y_dot != null )
{

  //std::cout<<"y_dot is not 0"<<std::endl;
  a = (v/y_dot)*(sin(y+y_dot*delta_t)-sin(y)) + (0.5*delta_t*delta_t*cos(y)*err_acc);
  b = (v/y_dot)*(-cos(y+y_dot*delta_t)+cos(y))+ (0.5*delta_t*delta_t*sin(y)*err_acc);
  c = err_acc*delta_t;
  d = y_dot*delta_t+0.5*delta_t*delta_t*err_ang;
  e = delta_t*err_ang;
}
else
{
    	//std::cout<<"y_dot=0"<<std::endl;
  a = v*cos(y)*delta_t+ (0.5*delta_t*delta_t*cos(y)*err_acc);
  b = v*sin(y)*delta_t+ (0.5*delta_t*delta_t*sin(y)*err_acc);
  c = err_acc*delta_t;
  //c=0;
  d = 0.5*delta_t*delta_t*err_ang;
  //d=0;
  e = delta_t*err_ang;
  //0;
}

  x_delta << a , b, c, d, e;

 // std::cout<<Xsig_aug.block(0,i,5,1)<<std::endl;
Xsig_pred.col(i) =  Xsig_aug.block(0,i,5,1)+x_delta;

  
  
  
    
}
//cout << "xaugpred_x_ = " << Xsig_pred << endl;
//.................................//
 VectorXd x = VectorXd(n_x);
  MatrixXd P = MatrixXd(n_x, n_x);
  x << 0,0,0,0,0;
   
  P << 0,0,0,0,0,
  0,0,0,0,0,
  0,0,0,0,0,
  0,0,0,0,0,
  0,0,0,0,0;

//predict state mean
  for (int i=0;i<n_sig;i++)
  {

  x = x+(weights(i)*Xsig_pred.col(i));
  }



  //predict state covariance matrix
//predict state mean
  for (int i=0;i<n_sig;i++)
  {
  x_xtransp = (Xsig_pred.col(i)-x);
   //while (x_xtransp(3)> 3.14159265) x_xtransp(3)-=2.*3.14159265;
    //while (x_xtransp(3)<-3.14159265) x_xtransp(3)+=2.*3.14159265;
	while(x_xtransp(3)>3.14159265)
	{
		//cout << "stuck in while of x_xtransp " <<x_xtransp << endl;
		x_xtransp(3) = x_xtransp(3) - (2.*3.14159265);
	}
	while(x_xtransp(3)<-3.14159265)
	{
		//cout << "stuck in while x_xtransp" <<x_xtransp << endl;
		x_xtransp(3) = x_xtransp(3) + (2.*3.14159265);
	}
  P = P+(weights(i)*x_xtransp*x_xtransp.transpose());
  }
x_=x;
P_=P;
  
   cout << "pred_x_ = " << x_ << endl;
  cout << "pred_P_ = " << P_ << endl;
  //std::cout<<"Prediction ended............................................"<<std::endl;
}
/**
 * Updates the state and the state covariance matrix using a laser measurement.
 * @param {MeasurementPackage} meas_package
 */

void UKF::UpdateLidar(MeasurementPackage meas_package) {
  /**
  TODO:

  Complete this function! Use lidar data to update the belief about the object's
  position. Modify the state vector, x_, and covariance, P_.

  You'll also need to calculate the lidar NIS.
  */
//std::cout<<"Lidar Started............................................"<<std::endl;
  VectorXd z = VectorXd(2);
   z << 0,0;
  
  z.head(2) = meas_package.raw_measurements_;

   VectorXd z_pred = H_ * x_;

	VectorXd y = z - z_pred;
	
	MatrixXd Ht = H_.transpose();
	
	MatrixXd S = H_ * P_ * Ht + R_;
	
	MatrixXd Si = S.inverse();
	float lidar_nis = y.transpose() * Si * y;

	cout << "Lidar NIS = " << lidar_nis << endl;
	MatrixXd PHt = P_ * Ht;
	
	MatrixXd K = PHt * Si;
 

	//new estimate
	x_ = x_ + (K * y);
	long x_size = x_.size();
	
	MatrixXd I = MatrixXd::Identity(x_size, x_size);
	P_ = (I - K * H_) * P_;
	
//std::cout<<"Lidar ended............................................"<<std::endl;
}

/**
 * Updates the state and the state covariance matrix using a radar measurement.
 * @param {MeasurementPackage} meas_package
 */
 
void UKF::UpdateRadar(MeasurementPackage meas_package) {
  /**
  TODO:

  Complete this function! Use radar data to update the belief about the object's
  position. Modify the state vector, x_, and covariance, P_.

  You'll also need to calculate the radar NIS.
  */
  //std::cout<<"Rader Started............................................"<<std::endl;
  VectorXd z_pred = VectorXd(n_z);
  z_pred << 0,0,0;
 MatrixXd P = MatrixXd(n_z, n_z);
  P.fill(0.0);
  //cout << "xaugpred_x_ = " << Xsig_pred << endl;
    for(int i=0;i<n_sig;i++)
  {
  
  
  
   pxx = Xsig_pred.col(i)(0);
   pyy = Xsig_pred.col(i)(1);
   vel = Xsig_pred.col(i)(2);
   psi = Xsig_pred.col(i)(3);

   //std::cout<<pxx<<" "<<pyy<<" "<<v<<" "<<psi<<std::endl;

   phi = (sqrt(pow(pxx,2)+pow(pyy,2)));
   theta = atan2(pyy,pxx);
   phi_dot = (((pxx*vel*cos(psi))+(pyy*vel*sin(psi)))/(sqrt(pow(pxx,2)+pow(pyy,2))));
   //std::cout<<phi<<" "<<theta<<" "<<phi_dot<<std::endl;

   z_polar.col(i) <<  phi , theta, phi_dot;
  
   z_pred = z_pred + weights(i)*z_polar.col(i);
   

  }
   std::cout<<" z_polar"<<z_polar<<std::endl;
  
 for (int i=0;i<n_sig;i++)
  {
  z_ztransp = (z_polar.col(i)-z_pred);
 //while (z_ztransp(1)> M_PI) z_ztransp(3)-=2.*M_PI;
  //while (z_ztransp(1)<-M_PI) z_ztransp(3)+=2.*M_PI;
		while(z_ztransp(1)>3.14159265)
	{
		//cout << "stuck in while z_ztransp in prediction" <<z_ztransp << endl;
		z_ztransp(1) = z_ztransp(1) - (2.*3.14159265);
	}
	while(z_ztransp(1)<-3.14159265)
	{
		//cout << "stuck in while z_ztransp in prediction" <<z_ztransp << endl;
		z_ztransp(1) = z_ztransp(1) + (2.*3.14159265);
	}
  P = P+(weights(i)*z_ztransp*z_ztransp.transpose());
  }
S = P + R;
std::cout<<"P = "<<P<<" "<<std::endl;
std::cout<<"S = "<<S<<" "<<std::endl;
 //.................................//
 
  //calculate cross correlation matrix
   MatrixXd Tc = MatrixXd(n_x, n_z);
  Tc.fill(0.0);
  VectorXd z = VectorXd(n_z);
  z << 0,0,0;
  z = meas_package.raw_measurements_;
  
  for (int i=0;i<n_sig;i++)
  {
  x_xtransp = (Xsig_pred.col(i)-x_);
   z_ztransp = (z_polar.col(i)-z_pred);
   //while (x_xtransp(3)> M_PI) x_xtransp(3)-=2.*M_PI;
   //while (x_xtransp(3)<-M_PI) x_xtransp(3)+=2.*M_PI;
   //while (z_ztransp(1)> M_PI) z_ztransp(3)-=2.*M_PI;
   //while (z_ztransp(1)<-M_PI) z_ztransp(3)+=2.*M_PI;
   while(x_xtransp(3)>3.14159265)
	{
		//cout << "stuck in while z_ztransp in radar update" <<x_xtransp << endl;
		x_xtransp(3) = x_xtransp(3) - (2.*3.14159265);
	}
	while(x_xtransp(3)<-3.14159265)
	{
		//cout << "stuck in while z_ztransp in radar update" <<x_xtransp << endl;
		x_xtransp(3) = x_xtransp(3) + (2.*3.14159265);
	}
   	while(z_ztransp(1)>3.14159265)
	{
		//cout << "stuck in while z_ztransp in radar update" <<z_ztransp << endl;
		z_ztransp(1) = z_ztransp(1) - (2.*3.14159265);
	}
	while(z_ztransp(1)<-3.14159265)
	{
		//cout << "stuck in while z_ztransp in radar update" <<z_ztransp << endl;
		z_ztransp(1) = z_ztransp(1) + (2.*3.14159265);
	}
  Tc = Tc+(weights(i)*x_xtransp*z_ztransp.transpose());
  }
  //std::cout<<"Tc = "<<Tc<<" "<<std::endl;

  //calculate Kalman gain K;
  K = Tc*S.inverse();
  //std::cout<<"K = "<<K<<" "<<std::endl;
  //update state mean and covariance matrix
  //std::cout<<" z_pred"<<z_pred<<std::endl;
  //std::cout<<"z_raw = "<<z<<" "<<std::endl;
  
  VectorXd y=z-z_pred;
  float radar_nis = y.transpose() * S.inverse() * y;

  cout << "Radar NIS = " << radar_nis << endl;
   //while (y(1)> M_PI) y(1)-=2.*M_PI;
  //while (y(1)<-M_PI) y(1)+=2.*M_PI;
  	while(y(1)>3.14159265)
	{
		//cout << "stuck in while " <<y << endl;
		y(1) = y(1) - (2.*3.14159265);
	}
	while(y(1)<-3.14159265)
	{
		
		//cout << "stuck in while " <<y << endl;
		y(1) = y(1) + (2.*3.14159265);
	}
  x_ = x_ + K*y;
 
  P_ = P_ - K*S*K.transpose();
  //std::cout<<"Rader ended............................................"<<std::endl;


}
  

