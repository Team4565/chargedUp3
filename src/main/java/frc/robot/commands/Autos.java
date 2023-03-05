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

  public static CommandBase driveForSeconds(DrivetrainSubsystem drivetrain, GyroSubsystem gyro) {
    SequentialCommandGroup group = new SequentialCommandGroup(
      new DriveForTime(drivetrain, 5.3, -0.5),
      new TurnForAngle(gyro, drivetrain, -79, -0.5),
      new DriveForTime(drivetrain, 2.5, -0.5),
      new TurnForAngle(gyro, drivetrain, -77, -0.5),
      new DriveForTime(drivetrain, 1.4, -0.70),
      new Balance(gyro, drivetrain)
    );
    return group;
  }  

  public static CommandBase driveForSecondsTwo(DrivetrainSubsystem drivetrain, GyroSubsystem gyro) {
    SequentialCommandGroup group = new SequentialCommandGroup(
      new DriveForTime(drivetrain, 5.3, -0.5),
      new TurnForAngle(gyro, drivetrain, 70, 0.50),
      new DriveForTime(drivetrain, 2, -0.5),
      new TurnForAngle(gyro, drivetrain, 67, 0.50),
      new DriveForTime(drivetrain, 1.4, -0.70),
      new Balance(gyro, drivetrain)
    );
    return group;
  }

  public static CommandBase driveForSecondsThree(DrivetrainSubsystem drivetrain, GyroSubsystem gyro) {
    SequentialCommandGroup group = new SequentialCommandGroup(
      new DriveForTime(drivetrain, 2.35, -0.66),
      new DriveForTime(drivetrain, 2, 0.66),
      new Balance(gyro, drivetrain)
    );
    return group;
  }  

  public static CommandBase gyro(GyroSubsystem m_gyro, DrivetrainSubsystem drivetrain) {
    SequentialCommandGroup group = new SequentialCommandGroup(
      new DriveForTime(drivetrain, 1.3, -0.77),
      new Balance(m_gyro, drivetrain)
    );
    return group; 
  }

  public static CommandBase turn(DrivetrainSubsystem drivetrain, GyroSubsystem gyro) {
    return Commands.sequence(new TurnForAngle(gyro, drivetrain, 70, 0.5));
  }

  private Autos () {
    throw new UnsupportedOperationException("This is a utility class!");
  }
}
