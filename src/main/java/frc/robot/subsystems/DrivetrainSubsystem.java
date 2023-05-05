// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Encoder;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
// import edu.wpi.first.wpilibj.examples.ramsetecommand.Constants.DriveConstants;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class DrivetrainSubsystem extends SubsystemBase {
  /** Creates a new DrivetrainSubsystem. */
 


  private static DifferentialDrive diffDrive;
 
  private final WPI_VictorSPX leftLead;
  private final WPI_VictorSPX rightLead; 
  private final WPI_VictorSPX leftFollower;
  private final WPI_VictorSPX rightFollower;



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

  WPI_VictorSPX leftLeader = new WPI_VictorSPX(Constants.DrivetrainConstants.leftFrontCANID);
  WPI_VictorSPX leftFollow = new WPI_VictorSPX(Constants.DrivetrainConstants.leftBackCANID);
  WPI_VictorSPX rightLeader = new WPI_VictorSPX(Constants.DrivetrainConstants.rightFrontCANID);
  WPI_VictorSPX rightFollow = new WPI_VictorSPX(Constants.DrivetrainConstants.rightBackCANID);

  public DrivetrainSubsystem() {

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
    }

  }

  public void setRaw(double driveValue, double turnValue){
    diffDrive.arcadeDrive(driveValue, turnValue);
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
  
}
