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
    /**Front Left Motor Id value is set to 2 
     *leftPidControllerId
     */
    public static final int FLMotorId = 2; 
    public static final int FRMotorId = 7;
    public static final int BLMotorId = 3;
    public static final int BRMotorId = 6;
    
    //Currents limits 
    /** The maxium current our motor can take with our permission  
     * = 55
     */
    public static final int stallCurrentLimit = 55;
    /** The amount our motor can take without blowing up 
     * = 40 
     */
    public static final int freeCurrentLimit = 40;
    
    //PIDControllId's
    /** store the turning left parameters 
     * 0.1,0,0
     */
    public static final double[] leftPidControllerId = {0.1,0,0};
    /** store the tuning right parameters
     * 0.1,0,0
     */
    public static final double[] rightPidControllerId = {0.1,0,0};
    
    //Distance per a pulse for the encoder 
    /** Gives the distance per a pulse for the encoder 
     *
     */
    public static final int countsPerRevolutionId = -1/497;}


} 

