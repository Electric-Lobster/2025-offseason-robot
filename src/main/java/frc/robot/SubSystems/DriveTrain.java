// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.SubSystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import choreo.trajectory.DifferentialSample;
import edu.wpi.first.math.controller.LTVUnicycleController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.ADIS16470_IMU;
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

  //Trajectory controller
  LTVUnicycleController pathController = new LTVUnicycleController(0.02);

  //Gyro
  ADIS16470_IMU Gyro = new ADIS16470_IMU();

  //kinematics
  DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics(0.60325);

  //Pose
  Pose2d initialPose = new Pose2d();


  /** Creates a new DriveTrain. */
  public DriveTrain() {
    FLConfig.inverted(false).idleMode(IdleMode.kBrake).smartCurrentLimit(stallCurrentLimit,freeCurrentLimit);
    FLConfig.closedLoop.pid(leftPidController[0],leftPidController[1],leftPidController[2]);
    FLConfig.encoder.countsPerRevolution(countsPerRevolution);
    FLDrive.configure(FLConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    
    FRConfig.inverted(true).idleMode(IdleMode.kBrake).smartCurrentLimit(stallCurrentLimit, freeCurrentLimit);
    FRConfig.closedLoop.pid(rightPidController[0],rightPidController[1],rightPidController[2]);
    FRConfig.encoder.countsPerRevolution(countsPerRevolution);
    FRDrive.configure(FRConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    BLConfig.inverted(false).idleMode(IdleMode.kBrake).smartCurrentLimit(stallCurrentLimit, freeCurrentLimit).follow(FLDrive);
    BLConfig.encoder.countsPerRevolution(countsPerRevolution);
    BLDrive.configure(BLConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    BRConfig.inverted(true).idleMode(IdleMode.kBrake).smartCurrentLimit(stallCurrentLimit, freeCurrentLimit).follow(FRDrive);
    BRConfig.encoder.countsPerRevolution(countsPerRevolution);
    BRDrive.configure(BRConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    // Encoder Access
    FLEncoder = FLDrive.getEncoder();
    FREncoder = FRDrive.getEncoder();
    BLEncoder = BLDrive.getEncoder();
    BREncoder = BRDrive.getEncoder();

    // PID Access 
    leftPID = FLDrive.getClosedLoopController();
    rightPID = FRDrive.getClosedLoopController();

    Gyro.calibrate();
  }


  /**
   * drives the robot in tank style controls
   * @param left throttle
   * @param right throttle
   */
  public void tankDrive(double left, double right) {
    FLDrive.set(left);
    FRDrive.set(right);
  }


  /**
   * arcade style driving
   * @param throttle the forward throttle
   * @param rotation the rotational throttle
   */
  public void arcadeDrive(double throttle, double rotation) {
    FLDrive.set(throttle + rotation);
    FRDrive.set(throttle - rotation);
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

  
/**Gets motor speed 
 * @return the average speed of motors 
 */
public double getEncoderSpeed() {

double speed = 0;

speed += FLEncoder.getVelocity();
speed += FREncoder.getVelocity();
speed += BREncoder.getVelocity();
speed += BLEncoder.getVelocity();

speed /= 4.0;

return speed;

}


/**Gets left encoder motor speed 
 * @return the average distance of left encoder 
 */
public double getLeftDistance() {

  double distance = 0;
  
  distance += FLEncoder.getPosition();
  distance += BLEncoder.getPosition();
  
  distance /= 2.0;
  
  return distance;
  
  }


  /**Gets left encoder speed  
 * @return the average speed of left encoders
 */
public double getLeftSpeed() {

  double distance = 0;
  
  distance += FLEncoder.getVelocity();
  distance += BLEncoder.getVelocity();
  
  distance /= 2.0;
  
  return distance;
  
  }


  /** Gets right distance encoder distance  
   * @return the average distance of right encoder 
   */
  public double getRightDistance() {

    double distance = 0;

    distance += FREncoder.getVelocity();
    distance += BREncoder.getVelocity();

    distance /= 2.0;

    return distance;
  }
  
  
  public double getRightSpeed() {

    double speed = 0; 

    speed += FREncoder.getVelocity();
    speed += BREncoder.getVelocity();

    speed /= 2.0;

    return speed;
  }

  
  /**
   * follows a trajectory for autonomous programs
   * @param sample the current sample of the trajectory
   */
  public void followTrajectory(DifferentialSample sample) {

    //gets robot pose
    Pose2d pose = getPose();

    // velocity from sample
    ChassisSpeeds ff = sample.getChassisSpeeds();

    //generate next robot speed
    ChassisSpeeds speeds = pathController.calculate(pose, sample.getPose(), ff.vxMetersPerSecond, ff.omegaRadiansPerSecond);

    DifferentialDriveWheelSpeeds wheelSpeeds = kinematics.toWheelSpeeds(speeds);
    velocityDrive(wheelSpeeds.leftMetersPerSecond, wheelSpeeds.rightMetersPerSecond);
  }


  /**
   * Drives the robots using speed for both sides of the drivetrain
   * @param leftSpeed left wheel velocity
   * @param rightSpeed right wheel velocity
   */
  private void velocityDrive(double leftSpeed, double rightSpeed) {
    leftPID.setReference(leftSpeed, ControlType.kVelocity);
    rightPID.setReference(rightSpeed, ControlType.kVelocity);
  }


  /**
   * gets the pose of the robot
   * @return the pose based on the initial pose
   */
  public Pose2d getPose() {
    double angle = Math.toRadians(getYaw());
    //TODO: ADD IN ENCODER DISTANCES
    DifferentialDriveOdometry odometry = new DifferentialDriveOdometry(new Rotation2d(angle), 0, 0, initialPose);
  
    return odometry.getPoseMeters();
  }

  /**
   * resets the inital pose
   * @param newPose the new iinitial pose
   */
  public void resetPose(Pose2d newPose) {
    initialPose = newPose;
    //TODO: Reset Encoder
  }


  /**
   * returns the yaw angle of the robot
   * @return yaw in degrees
   */
  public double getYaw() {
    return Gyro.getAngle(Gyro.getYawAxis());
  }


  /**
   * returns yaw speed of the robot
   * @return degrees per second
   */
  public double getYawRate() {
    return Gyro.getRate(Gyro.getYawAxis());
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }


}


