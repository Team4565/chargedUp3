// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class EncodersSubsystem extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
  private final Encoder m_encoderLeft = new Encoder(0,1, false, Encoder.EncodingType.k2X);
  private final Encoder m_encoderRight = new Encoder(2,3, false, Encoder.EncodingType.k2X);
  
  public double encoderLeftDistance(){
    return m_encoderLeft.getDistance();
  }

  public double encoderRightDistance(){
    return m_encoderRight.getDistance();
  }

  public double encoderLeftRate(){
    return m_encoderLeft.getRate();
  }

  public double encoderRightRate(){
    return m_encoderRight.getRate();
  }

  public boolean encoderLeftDirection(){
    return m_encoderLeft.getDirection();
  }

  public boolean encoderRightDirection(){
    return m_encoderRight.getDirection();
  }
  public EncodersSubsystem() {
    m_encoderLeft.setDistancePerPulse(50./256.);
    m_encoderRight.setDistancePerPulse(50./256.);

    // Configures the encoder to consider itself stopped after .1 seconds
    //Later

    // Configures the encoder to consider itself stopped when its rate is below 10
    m_encoderLeft.setMinRate(10);
    m_encoderRight.setMinRate(10);

    // Configures an encoder to average its period measurement over 5 samples
    // Can be between 1 and 127 samples
    m_encoderLeft.setSamplesToAverage(5);
    m_encoderRight.setSamplesToAverage(5);
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
    
  }

  public void teleopInit() {
    m_encoderLeft.reset();
    m_encoderRight.reset();
  }
  
  public void autonomousInit(){
    m_encoderLeft.reset();
    m_encoderRight.reset();
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

  public void reset() {
  }
}
