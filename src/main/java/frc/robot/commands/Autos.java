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
      new DriveForTime(drivetrain, 5.6, -0.5),
      new TurnForAngle(gyro, drivetrain, -74, -0.53),
      new DriveForTime(drivetrain, 2.5, -0.5),
      new TurnForAngle(gyro, drivetrain, -73, -0.53),
      new DriveForTime(drivetrain, 1.2, -0.70),
      new Balance(gyro, drivetrain)
    );
    return group;
  }  

  public static CommandBase driveForSecondsTwo(DrivetrainSubsystem drivetrain, GyroSubsystem gyro) {
    SequentialCommandGroup group = new SequentialCommandGroup(
      new DriveForTime(drivetrain, 5, -0.5),
      new TurnForAngle(gyro, drivetrain, 80, 0.53),
      new DriveForTime(drivetrain, 2, -0.5),
      new TurnForAngle(gyro, drivetrain, 72, 0.53),
      new DriveForTime(drivetrain, 1.2, -0.65),
      new Balance(gyro, drivetrain)
    );
    return group;
  }

  public static CommandBase driveForSecondsThree(DrivetrainSubsystem drivetrain, GyroSubsystem gyro) {
    SequentialCommandGroup group = new SequentialCommandGroup(
      new DriveForTime(drivetrain, 3.3, -0.66),
      new DriveForTime(drivetrain, 1.3, 0.66),
      new Balance(gyro, drivetrain)
    );
    return group;
  }

  public static CommandBase driveForSecondsFour(DrivetrainSubsystem drivetrain, GyroSubsystem gyro) {
    SequentialCommandGroup group = new SequentialCommandGroup(
      new DriveForTime(drivetrain, 1.3, -0.7),
      new DriveForTime(drivetrain, 1.3, 0.7),
      new DriveForTime(drivetrain, 5.7, -0.5),
      new TurnForAngle(gyro, drivetrain, 82, 0.53),
      new DriveForTime(drivetrain, 2, -0.5),
      new TurnForAngle(gyro, drivetrain, 72, 0.53),
      new DriveForTime(drivetrain, 1.3, -0.70),
      new Balance(gyro, drivetrain)
    );
    return group;
  }   

  public static CommandBase driveForSecondsFive(DrivetrainSubsystem drivetrain, GyroSubsystem gyro) {
    SequentialCommandGroup group = new SequentialCommandGroup(
      new DriveForTime(drivetrain, 3.5, -0.6),
      new DriveForTime(drivetrain, 5.8, 0.5),
      new DriveForTime(drivetrain, 3, -0.5)
    );
    return group;
  }

  public static CommandBase driveForSecondsSix(DrivetrainSubsystem drivetrain, GyroSubsystem gyro) {
    SequentialCommandGroup group = new SequentialCommandGroup(
      new DriveForTime(drivetrain, 5, 0.5),
      new TurnForAngle(gyro, drivetrain, -81, -0.53),
      new DriveForTime(drivetrain, 2, 0.5),
      new TurnForAngle(gyro, drivetrain, -72, -0.53),
      new DriveForTime(drivetrain, 1.2, 0.65),
      new Balance(gyro, drivetrain)
    );
    return group;
  }

  public static CommandBase driveForSecondsSeven(DrivetrainSubsystem drivetrain) {
    SequentialCommandGroup group = new SequentialCommandGroup(
      new DriveForTime(drivetrain, 1, 0.65),
      new DriveForTime(drivetrain, 4, -0.55)
    );
    return group;
  }

  public static CommandBase gyro(GyroSubsystem m_gyro, DrivetrainSubsystem drivetrain) {
    SequentialCommandGroup group = new SequentialCommandGroup(
      new Balance(m_gyro, drivetrain)
    );
    return group; 
  }

  public static CommandBase turn(DrivetrainSubsystem drivetrain, GyroSubsystem gyro) {
    return Commands.sequence(new TurnForAngle(gyro, drivetrain, 88, 0.55));
  }


public static CommandBase forwardsBackwardsBalance(DrivetrainSubsystem drivetrain, GyroSubsystem gyro){
  return new SequentialCommandGroup(
    new DriveStraightForTime(drivetrain,.1,.9,gyro),
  new DriveStraightForTime(drivetrain,4,-.8,gyro),
  new DriveStraightForTime(drivetrain,2,.8,gyro)
  , new Balance(gyro, drivetrain)
  );
}


  private Autos () {
    throw new UnsupportedOperationException("This is a utility class!");
  }
}
