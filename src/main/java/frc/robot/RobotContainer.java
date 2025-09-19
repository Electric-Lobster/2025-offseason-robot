// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import choreo.auto.AutoChooser;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.SubSystems.DriveTrain;

import static frc.robot.Constants.joystickIds.*;

public class RobotContainer {
  //Subsystems
  DriveTrain m_DriveTrain = new DriveTrain();


  //Auto container
  AutoContainer autoContainer = new AutoContainer(m_DriveTrain);


  //auto selector
  AutoChooser autoChooser = new AutoChooser();

  private final Joystick m_leftStick = new Joystick(leftStickPortId);
  
  private final Joystick m_rightStick = new Joystick(rightStickPortId);
  
  private final Joystick m_operatorStick = new Joystick(operatorStickPortId);
  

  public RobotContainer() {

    configureBindings();

    configureChooser();

    
    }

  private void configureBindings() {}

  private void configureChooser() {

    autoChooser.addRoutine("Test 1: Straight", autoContainer::test1);
    autoChooser.addRoutine("Test 2: Turn Left", autoContainer::test2);

    SmartDashboard.putData(autoChooser);
  }


  public Command getAutonomousCommand() {
    return autoChooser.selectedCommand();
  }
}
