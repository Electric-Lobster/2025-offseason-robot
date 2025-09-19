// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.SubSystems.DriveTrain;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class DefaultDrive extends Command {

  DriveTrain driveTrain;
  Joystick ljoystick;
  Joystick rJoystick;

  
  /** Creates a new DefaultDrive. */
  public DefaultDrive(DriveTrain DrvT,Joystick lJoystick, Joystick rJoystick) {

    // Use addRequirements() here to declare subsystem dependencies.
    
    this.driveTrain = DrvT;
    this.ljoystick = lJoystick;
    this.rJoystick = rJoystick;

    addRequirements(driveTrain);
    
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
