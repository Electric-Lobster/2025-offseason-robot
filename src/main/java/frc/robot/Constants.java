// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.SubSystems.DriveTrain;


/** Add your docs here. */
public final class Constants  {

   
    //Drive train constant 
    public static class DrivetrainConstants{
    

    //Motor Id's
    public static final int FLMotorId = 2; 
    public static final int FRMotorId = 7;
    public static final int BLMotorId = 3;
    public static final int BRMotorId = 6;
    
    
    //Currents limits 
    public static final int stallCurrentLimit = 55;
    
    public static final int freeCurrentLimit = 40;
    

    //PIDControllId's
    public static final double[] leftPidControllerId = {0.1,0,0};
    public static final double[] rightPidControllerId = {0.1,0,0};
    
    
    //Encoders
    public static final int countsPerRevolutionId = 0;
    }


} 

