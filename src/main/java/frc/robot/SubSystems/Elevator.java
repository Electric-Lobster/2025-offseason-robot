// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.SubSystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Elevator extends SubsystemBase {
  
  //motors 
  SparkMax stage1LeftM = new SparkMax(4, MotorType.kBrushless);
  SparkMax stage1RightM = new SparkMax(5, MotorType.kBrushless);
  SparkMax stage2M = new SparkMax(14, MotorType.kBrushless);



  /** Creates a new Elevator. */
  public Elevator() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
