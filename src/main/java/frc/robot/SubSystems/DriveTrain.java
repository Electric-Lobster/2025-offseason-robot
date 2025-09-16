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

public class DriveTrain extends SubsystemBase {
  //motors 
  SparkMax FLDrive = new SparkMax(0, MotorType.kBrushless);
  SparkMax FRDrive = new SparkMax(0, MotorType.kBrushless);
  SparkMax BLDrive = new SparkMax(0, MotorType.kBrushless);
  SparkMax BRDrive = new SparkMax(0, MotorType.kBrushless);

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
    FLConfig.inverted(false).idleMode(IdleMode.kBrake).smartCurrentLimit(55, 40);
    FLConfig.closedLoop.pid(0.1, 0, 0);
    FLConfig.encoder.countsPerRevolution(0);
    FLDrive.configure(FLConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    
    FRConfig.inverted(true).idleMode(IdleMode.kBrake).smartCurrentLimit(55, 40);
    FRConfig.closedLoop.pid(0.1, 0, 0);
    FRConfig.encoder.countsPerRevolution(0);
    FRDrive.configure(FRConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    BLConfig.inverted(false).idleMode(IdleMode.kBrake).smartCurrentLimit(55, 40).follow(FLDrive);
    BLConfig.encoder.countsPerRevolution(0);
    BLDrive.configure(BLConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    BRConfig.inverted(true).idleMode(IdleMode.kBrake).smartCurrentLimit(55, 40).follow(FRDrive);
    BRConfig.encoder.countsPerRevolution(0);
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

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}


