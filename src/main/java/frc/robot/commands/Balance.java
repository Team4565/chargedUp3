// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.GyroSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** An example command that uses an example subsystem. */
public class Balance extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})

  private final GyroSubsystem m_gyro;
  private final DrivetrainSubsystem m_DrivetrainSubsystem;
  private double roll;
  private double speed;


  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public Balance (GyroSubsystem subsystem, DrivetrainSubsystem drivetrainSubsystem) {
    m_gyro = subsystem;
    m_DrivetrainSubsystem = drivetrainSubsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem, drivetrainSubsystem);
    
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    if(m_gyro.getRoll() > 4) {
      m_DrivetrainSubsystem.setRaw(-0.43, 0);
    }
    else if(m_gyro.getRoll() < -4) {
      m_DrivetrainSubsystem.setRaw(0.43, 0);
    }
    else {
      m_DrivetrainSubsystem.setRaw(0, 0);
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
