// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.SubSystems.DriveTrain;

/** Add your docs here. */
public final class Constants {

public static class DrivetrainConstants{
    
    public static final int FLMotor = 2; 

    public static final int FRMotor = 7;
    
    public static final int BLMotor = 3;
    
    public static final int BRMotor = 6;
    
    public static final int stallCurrentLimit = 55;
    
    public static final int freeCurrentLimit = 40;
    
    public static final double[] leftPidControllerId = {0.1,0,0};
    
    public static final double[] rightPidControllerId = {0.1,0,0};
    
    public static final double distancePerAPulse = -1.0/497.0;}

    
} 

