// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.ExampleSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;
import edu.wpi.first.wpilibj.Encoder;

/** An example command that uses an example subsystem. */
public class DriveWithEncoders extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  public final DrivetrainSubsystem m_drivetrain;
  private double speedForRobot;
  private double distanceForRobot;

  private final Encoder encoderLeft = new Encoder(0,1, false, Encoder.EncodingType.k2X);
  private final Encoder encoderRight = new Encoder(2,3, false, Encoder.EncodingType.k2X);
  
  private double encoderLeftDistance(){
    return encoderLeft.getDistance();
  }

  private double encoderRightDistance(){
    return encoderRight.getDistance();
  }

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  
  public DriveWithEncoders(DrivetrainSubsystem drivetrain, double speed, double distance) {
    m_drivetrain = drivetrain;
    speed = speedForRobot;
    distance = distanceForRobot;

    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (encoderLeft.getDistance() < distanceForRobot && encoderRight.getDistance() < distanceForRobot){
      
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
