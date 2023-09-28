// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.ExampleSubsystem;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.EncodersSubsystem;



/** An example command that uses an example subsystem. */
public class DriveWithEncoders extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  public final DrivetrainSubsystem m_drivetrain;
   // private final EncodersSubsystem m_encoderLeft;
  private final EncodersSubsystem m_encoderRight;
  private double distanceForRobot;


  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  
  public DriveWithEncoders(DrivetrainSubsystem drivetrain, EncodersSubsystem encoderRightSubsystem, double distance) {
    
    m_encoderRight = encoderRightSubsystem;
    m_drivetrain = drivetrain;
    distanceForRobot = distance;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  public void autonomousPeriodic(){
    if (m_encoderRight.encoderRightDistance() < distanceForRobot){
      m_drivetrain.setRaw(-0.5, 0);
    }
    else {
      m_drivetrain.setRaw(-0.0, 0);
      m_encoderRight.reset();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
