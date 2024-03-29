// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class GyroSubsystem extends SubsystemBase {
  /** Creates a new GyroSubsystem. */
  AHRS m_gyro;
  public GyroSubsystem() {
    m_gyro = new AHRS(SPI.Port.kMXP) ;
  }

  public double getAngle(){
    return m_gyro.getAngle() + 45;
  }

  public double getPitch(){
    return m_gyro.getPitch();
  }

  public double getRoll() {
    return m_gyro.getRoll() - 1.4;
  }

  public void resetYaw(){
    m_gyro.zeroYaw();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("roll angle", getRoll());
    SmartDashboard.putNumber("yaw angle", getAngle());
  }
}
