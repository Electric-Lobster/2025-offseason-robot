// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.SubSystems;

import java.util.Set;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Elevator extends SubsystemBase {
  
  //Motors 
  SparkMax stage1LeftM = new SparkMax(4, MotorType.kBrushless);
  SparkMax stage1RightM = new SparkMax(5, MotorType.kBrushless);
  SparkMax stage2M = new SparkMax(14, MotorType.kBrushless);

  //Configuration for motors 

  SparkMaxConfig stage1LMConfig = new SparkMaxConfig();
  SparkMaxConfig stage1RMConfig = new SparkMaxConfig();
  SparkMaxConfig stage2MConfig = new SparkMaxConfig();


  //PIDS
  SparkClosedLoopController stage1PID; 
  SparkClosedLoopController stage2PID;


  //Encoders
  RelativeEncoder stage1LeftEnc;
  RelativeEncoder stage1RightEnc;
  RelativeEncoder stage2Enc;


  /** Creates a new Elevator. */
  public Elevator() {

    stage1LMConfig.inverted(false).smartCurrentLimit(5,5).idleMode(IdleMode.kBrake);
    stage1LMConfig.closedLoop.pid(0.1, 0, 0);
    stage1LMConfig.encoder.positionConversionFactor(0);

    stage1RMConfig.inverted(true).smartCurrentLimit(5,5).idleMode(IdleMode.kBrake).follow(stage1LeftM);
    stage1RMConfig.encoder.positionConversionFactor(0);

    stage2MConfig.inverted(true).smartCurrentLimit(5,5).idleMode(IdleMode.kBrake);
    stage2MConfig.closedLoop.pid(0.1, 0, 0);
    stage2MConfig.encoder.positionConversionFactor(0);

    stage1PID = stage1LeftM.getClosedLoopController();
    stage2PID = stage2M.getClosedLoopController();

    stage1LeftEnc = stage1LeftM.getEncoder();
    stage1RightEnc = stage1RightM.getEncoder();
    stage2Enc = stage2M.getEncoder();

  }

  public double getStage1Position() {

    double position = 0;

    position += stage1LeftEnc.getPosition();

    position += stage1RightEnc.getPosition();

    position /=2.0;

    return position;
  }

  public double getStage2Position() {

    return stage2Enc.getPosition();

  }

  private void setStage1Throttle(double throttle) {

    
    if (getStage1Position() <= 0 && throttle < 0) {
      throttle = 0;
    } else if (getStage1Position() >= 25 && throttle > 0){ // TODO replace with constant 
      throttle = 0;
    } 
    
    stage1LeftM.set(throttle);


  }

  private void setStage2Position(double throttle) {

    if (getStage2Position() <= 0 && throttle < 0) {
      throttle = 0;
    } else if (getStage2Position() >= 16 && throttle > 0) { // TODO replace with constant
      throttle = 0;
    }

    stage2M.set(throttle);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
