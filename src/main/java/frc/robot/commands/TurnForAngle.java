// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.GyroSubsystem;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** An example command that uses an example subsystem. */
public class TurnForAngle extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final GyroSubsystem m_gyro;
  private final DrivetrainSubsystem m_drivetrain;
  private double AngleToTurn;
  private double rotationForRobot;

  private double IntialAngle;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public TurnForAngle(GyroSubsystem gyro, DrivetrainSubsystem subsystem, double Angle, double rotation) {
    m_gyro = gyro;
    m_drivetrain = subsystem;
    AngleToTurn = Angle;
    rotationForRobot = rotation;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    IntialAngle = m_gyro.getAngle();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_drivetrain.setRaw(0, rotationForRobot);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drivetrain.setRaw(0.0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(AngleToTurn > 0) {
      return m_gyro.getAngle() >= IntialAngle + AngleToTurn;
    }
    else {
      return m_gyro.getAngle() <= IntialAngle + AngleToTurn;
    }
  }
}
