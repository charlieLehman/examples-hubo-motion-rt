/*
Copyright (c) 2016, Charlie K. Lehman
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.
    * Neither the name of the author nor the names of its contributors may
      be used to endorse or promote products derived from this software
      without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT,
INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <Hubo_Control.h>
#include <iostream>
#include <vector>
#include <cmath>
    
int main( int argc, char **argv )
{
    //Create the hubo object
    Hubo_Control hubo;

    //Declare allocators for vectors to store angles for Left and Right Arm joints
    std::vector<ArmVector, Eigen::aligned_allocator<ArmVector> > Rangles(4), Langles(4);

    //Declare allocators for vectors to store Homogeneous Matrices for IK Solver 
    std::vector<Eigen::Isometry3d, Eigen::aligned_allocator<Eigen::Isometry3d> > Br(4), Bl(4);

    //Declare Left and Right arm joint states
    ArmVector qpL, qpR;

    //Adjust the speed and acceleration of motion
    ArmVector accel, speeds;
    accel.setOnes(); accel *= 0.1;
    speeds.setOnes(); speeds *= 0.2;
    hubo.setLeftArmNomAcc( accel );
    hubo.setRightArmNomAcc( accel );
    hubo.setLeftArmNomSpeeds( speeds );
    hubo.setRightArmNomSpeeds( speeds );

    //Rectangle Dimensions
    double h = .2;
    double l = .4;

    //DH paramaters
    double d = -.2;
    double a = .4;
    double theta = .250*M_PI;
    double alpha = .1*M_PI;

    //Pythagoras
    double ay = sqrt(pow(a,2)+pow(h,2));//Up
    double ayz = sqrt(pow(l,2)+pow(a,2)+pow(h,2));//Over
    double az = sqrt(pow(a,2)+pow(d,2));//Down

    //Create first Homogeneous Matrix
    hubo.DH2HG(Bl[0],theta,alpha,a,d);
    hubo.DH2HG(Br[0],-theta,-alpha,a,d);

    //Create second Homogeneous Matrix
    hubo.DH2HG(Bl[1],theta,alpha,ay,d+h);
    hubo.DH2HG(Br[1],-theta,-alpha,ay,d+h);

    //Create third Homogeneous Matrix
    hubo.DH2HG(Bl[2],theta+atan(h/a),alpha,ayz,d+h);
    hubo.DH2HG(Br[2],-theta-atan(h/a),-alpha,ayz,d+h);

    //Create fourth Homogeneous Matrix
    hubo.DH2HG(Bl[3],theta+atan(h/a),alpha,az,d);
    hubo.DH2HG(Br[3],-theta-atan(h/a),-alpha,az,d);

    //Get current states for both arms
    hubo.getLeftArmAngles(qpL);
    hubo.getRightArmAngles(qpR);
    
    //Solve IK for all positions
    hubo.huboArmIK(Langles[0], Bl[0], qpL,LEFT);
    hubo.huboArmIK(Rangles[0], Br[0], qpR,RIGHT);
    hubo.huboArmIK(Langles[1], Bl[1], Langles[0],LEFT);
    hubo.huboArmIK(Rangles[1], Br[1], Rangles[0],RIGHT);
    hubo.huboArmIK(Langles[2], Bl[2], Langles[1],LEFT);
    hubo.huboArmIK(Rangles[2], Br[2], Rangles[1],RIGHT);
    hubo.huboArmIK(Langles[3], Bl[3], Langles[2],LEFT);
    hubo.huboArmIK(Rangles[3], Br[3], Rangles[2],RIGHT);
    
    int traj = 0; //trajectory iterator
    double tol = .005;//tolerance
    
    //Continuously draw a rectangle with each hand
    while(true)
    { 
      hubo.update(true); //Get most updated state from hubo

      //Update arm joint states
      hubo.getLeftArmAngles(qpL);
      hubo.getRightArmAngles(qpR);

      //Move toward target joint angles and compare to current joint states
      //until within tolerance. Then move to next target joint angle.
        if( (qpR-Rangles[traj]).norm()<tol && (qpL-Langles[traj]).norm()<tol)
        {
          std::cout << "traj = " << traj << std::endl;
          traj++;
          if(traj>3)
             traj=0;
        }
      //Send hubo the joint angles
      hubo.setLeftArmAngles(Langles[traj],false);
      hubo.setRightArmAngles(Rangles[traj],false);
      hubo.sendControls();
    }
}
