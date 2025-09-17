// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.SubSystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkRelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.DrivetrainConstants.*;

public class DriveTrain extends SubsystemBase {
  //motors 
  SparkMax FLDrive = new SparkMax(FLMotorId, MotorType.kBrushless);
  SparkMax FRDrive = new SparkMax(FRMotorId, MotorType.kBrushless);
  SparkMax BLDrive = new SparkMax(BLMotorId, MotorType.kBrushless);
  SparkMax BRDrive = new SparkMax(BRMotorId, MotorType.kBrushless);

  // motor configs 
  SparkMaxConfig FLConfig = new SparkMaxConfig();
  SparkMaxConfig FRConfig = new SparkMaxConfig();
  SparkMaxConfig BLConfig = new SparkMaxConfig();
  SparkMaxConfig BRConfig = new SparkMaxConfig();

  //Encoders
  RelativeEncoder FLEncoder;
  RelativeEncoder FREncoder;
  RelativeEncoder BLEncoder;
  RelativeEncoder BREncoder;

  //PID
  SparkClosedLoopController leftPID;
  SparkClosedLoopController rightPID;

  /** Creates a new DriveTrain. */
  public DriveTrain() {
    FLConfig.inverted(false).idleMode(IdleMode.kBrake).smartCurrentLimit(stallCurrentLimit,freeCurrentLimit);
    FLConfig.closedLoop.pid(leftPidControllerId[0],leftPidControllerId[1],leftPidControllerId[2]);
    FLConfig.encoder.countsPerRevolution(countsPerRevolutionId);
    FLDrive.configure(FLConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    
    FRConfig.inverted(true).idleMode(IdleMode.kBrake).smartCurrentLimit(stallCurrentLimit, freeCurrentLimit);
    FRConfig.closedLoop.pid(rightPidControllerId[0],rightPidControllerId[1],rightPidControllerId[2]);
    FRConfig.encoder.countsPerRevolution(countsPerRevolutionId);
    FRDrive.configure(FRConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    BLConfig.inverted(false).idleMode(IdleMode.kBrake).smartCurrentLimit(stallCurrentLimit, freeCurrentLimit).follow(FLDrive);
    BLConfig.encoder.countsPerRevolution(countsPerRevolutionId);
    BLDrive.configure(BLConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    BRConfig.inverted(true).idleMode(IdleMode.kBrake).smartCurrentLimit(stallCurrentLimit, freeCurrentLimit).follow(FRDrive);
    BRConfig.encoder.countsPerRevolution(countsPerRevolutionId);
    BRDrive.configure(BRConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    // Encoder Access
    FLEncoder = FLDrive.getEncoder();
    FREncoder = FRDrive.getEncoder();
    BLEncoder = BLDrive.getEncoder();
    BREncoder = BRDrive.getEncoder();

    // PID Access 
    leftPID = FLDrive.getClosedLoopController();
    rightPID = FRDrive.getClosedLoopController();
  }


  public void tankDrive(double left, double right) {
    //TODO: ADD CONTROL
  }

  /** Gets encoder distance 
   * @return the average of the drive encoders
   */
  public double getEncoderDistance() {

    double distance = 0;

    distance += FLEncoder.getPosition();
    distance += FREncoder.getPosition();
    distance += BREncoder.getPosition();
    distance += BLEncoder.getPosition();

    distance /= 4.0;

    return distance;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }


}


