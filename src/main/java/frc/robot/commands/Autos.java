// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.GyroSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public final class Autos {
  /** Example static factory for an autonomous command. */
  public static CommandBase exampleAuto(ExampleSubsystem subsystem) {
    return Commands.sequence(subsystem.exampleMethodCommand(), new ExampleCommand(subsystem));
  }

  public static CommandBase drive5Seconds(DrivetrainSubsystem drivetrain) {
    CommandBase commandGroup = new SequentialCommandGroup();
    commandGroup.andThen(new DriveForTime(drivetrain, 8, -0.5));
    //commandGroup.andThen(new TurnForTime(drivetrain, 1, -0.5));
    return commandGroup;
  }

  public static CommandBase gyro(GyroSubsystem m_gyro, DrivetrainSubsystem drivetrain) {
    return Commands.sequence(new Balance(m_gyro, drivetrain)); 
  }

  public static CommandBase turn(DrivetrainSubsystem drivetrain) {
    return Commands.sequence(new TurnForTime(drivetrain, 1, -0.5));
  }

  private Autos() {
    throw new UnsupportedOperationException("This is a utility class!");
  }
}
