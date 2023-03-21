// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DrivetrainSubsystem extends SubsystemBase {
  /** Creates a new DrivetrainSubsystem. */
 


  private static DifferentialDrive diffDrive;
 
  private final WPI_VictorSPX leftLead;
  private final WPI_VictorSPX rightLead; 
  private final WPI_VictorSPX leftFollower;
  private final WPI_VictorSPX rightFollower;

  private String driveMode;

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
        diffDrive.arcadeDrive(driveValue * 0.75, turnValue * 0.80);
      break;

      case "normal":
        diffDrive.arcadeDrive(driveValue * 0.60, turnValue * 0.65);
      break;

      case "creep speed (slow)":
        diffDrive.arcadeDrive(driveValue * 0.45 , turnValue * 0.50);
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
