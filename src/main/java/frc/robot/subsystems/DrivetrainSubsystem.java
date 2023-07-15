// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.Encoder;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.kauailabs.navx.frc.AHRS;

// import edu.wpi.first.wpilibj.examples.ramsetecommand.Constants.DriveConstants;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import frc.robot.subsystems.GyroSubsystem;

public class DrivetrainSubsystem extends SubsystemBase {
  /** Creates a new DrivetrainSubsystem. */
 


  private static DifferentialDrive diffDrive;
 
  private final WPI_VictorSPX leftLead;
  private final WPI_VictorSPX rightLead; 
  private final WPI_VictorSPX leftFollower;
  private final WPI_VictorSPX rightFollower;

  private final Encoder encoderLeft = new Encoder(0,1, false, Encoder.EncodingType.k2X);
  private final Encoder encoderRight = new Encoder(2,3, false, Encoder.EncodingType.k2X);

  private final EncoderSim m_leftEncoderSim = new EncoderSim(encoderLeft);
  private final EncoderSim m_rightEncoderSim = new EncoderSim(encoderRight);

  public final static Gyro navX = new AHRS(SPI.Port.kMXP);
  private final DifferentialDriveOdometry m_odometry; 



  private double encoderLeftDistance(){
    return encoderLeft.getDistance();
  }

  private double encoderRightDistance(){
    return encoderRight.getDistance();
  }

  private double encoderLeftRate(){
    return encoderLeft.getRate();
  }

  private double encoderRightRate(){
    return encoderRight.getRate();
  }

  private boolean encoderLeftDirection(){
    return encoderLeft.getDirection();
  }

  private boolean encoderRightDirection(){
    return encoderRight.getDirection();
  }

  private String driveMode;

  //private final Encoder m_leftEncoder =
  //new Encoder(
    //  DriveConstants.kLeftEncoderPorts[1],
      //DriveConstants.kLeftEncoderPorts[2],
      //DriveConstants.kLeftEncoderReversed);

  // The right-side drive encoder
    //private final Encoder m_rightEncoder =
    //new Encoder(
      //  DriveConstants.kRightEncoderPorts[0],
        //DriveConstants.kRightEncoderPorts[1],
        //DriveConstants.kRightEncoderReversed);

  // WPI_VictorSPX leftLeader = new WPI_VictorSPX(Constants.DrivetrainConstants.leftFrontCANID);
  // WPI_VictorSPX leftFollow = new WPI_VictorSPX(Constants.DrivetrainConstants.leftBackCANID);
  // WPI_VictorSPX rightLeader = new WPI_VictorSPX(Constants.DrivetrainConstants.rightFrontCANID);
  // WPI_VictorSPX rightFollow = new WPI_VictorSPX(Constants.DrivetrainConstants.rightBackCANID);

  public DrivetrainSubsystem() {
    
    m_odometry = new DifferentialDriveOdometry(navX.getRotation2d(), 0, 0);
    encoderLeft.setDistancePerPulse(1./256.);
    encoderRight.setDistancePerPulse(1./256.);

    // Configures the encoder to consider itself stopped after .1 seconds
    //Later

    // Configures the encoder to consider itself stopped when its rate is below 10
    encoderLeft.setMinRate(10);
    encoderRight.setMinRate(10);

    // Configures an encoder to average its period measurement over 5 samples
    // Can be between 1 and 127 samples
    encoderLeft.setSamplesToAverage(5);
    encoderRight.setSamplesToAverage(5);

    leftLead = new WPI_VictorSPX(8);
    rightLead = new WPI_VictorSPX(6);
    leftFollower = new WPI_VictorSPX(7);
    rightFollower = new WPI_VictorSPX(9);



    leftLead.setInverted(true);
    leftFollower.setInverted(true);
    

    leftFollower.follow(leftLead);

    
    rightFollower.follow(rightLead);

    diffDrive = new DifferentialDrive(leftLead,rightLead);

    driveMode = "normal";
  }

  public void updateDriveMode(String mode) {
    driveMode = mode;
  }

  public void teleopDrive(double driveValue, double turnValue) {
    switch(driveMode) {
      case "max":
        diffDrive.arcadeDrive(driveValue, turnValue);
      break;

      case "fast":
        diffDrive.arcadeDrive(driveValue * 0.80, turnValue * 0.80);
      break;

      case "normal":
        diffDrive.arcadeDrive(driveValue * 0.70, turnValue * 0.70);
      break;

      case "creep speed (slow)":
        diffDrive.arcadeDrive(driveValue * 0.60 , turnValue * 0.60);
      break;

      case "no speed":
        diffDrive.arcadeDrive(driveValue * 0 , turnValue * 0);
      break;
    }

  }

  public void setRaw(double driveValue, double turnValue){
    diffDrive.arcadeDrive(driveValue, turnValue);
  }

  public static double getHeading (){
    return navX.getRotation2d().getDegrees();
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Left encoder distance", encoderLeftDistance());
    SmartDashboard.putNumber("Right encoder distance", encoderRightDistance());
    SmartDashboard.putNumber("Left encoder rate", encoderLeftRate());
    SmartDashboard.putNumber("Right encoder rate", encoderRightRate());
    SmartDashboard.putBoolean("Left encoder direction", encoderLeftDirection());
    SmartDashboard.putBoolean("Right encoder direction", encoderRightDirection());

    SmartDashboard.putNumber("Left encoder value meters", encoderLeftDistance());
    m_odometry.update(navX.getRotation2d(), encoderLeft.getDistance(), encoderRight.getDistance());
  }

  public void teleopInit() {
    encoderLeft.reset();
    encoderRight.reset();
  }
  
  public void autonomousInit(){
    encoderLeft.reset();
    encoderRight.reset();
  }

  public void setRaw(double speedForRobot) {
  }
}
