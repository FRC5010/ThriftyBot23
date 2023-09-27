// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.revrobotics.RelativeEncoder;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.RobotConstants;
import frc.robot.subsystems.DrivetrainSubsystem;

public class DriveDistance extends CommandBase {
  DrivetrainSubsystem drivetrainSubsystem;

  RelativeEncoder leftEncoder;
  RelativeEncoder rightEncoder;

  double initialEncoderValue;

  double targetDistance;
  double speed;

  /** Creates a new DriveDistance. */
  public DriveDistance(DrivetrainSubsystem drivetrainSubsystem, double speed, double inchDistance) {
    this.drivetrainSubsystem = drivetrainSubsystem;

    leftEncoder = drivetrainSubsystem.getLeftEncoder();
    rightEncoder = drivetrainSubsystem.getRightEncoder();

    targetDistance = inchDistance;
    this.speed = speed;

    initialEncoderValue = leftEncoder.getPosition();

  
    addRequirements(drivetrainSubsystem);

    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
   drivetrainSubsystem.runMotor(speed);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drivetrainSubsystem.stopMotor();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    double currentEncoderPosition = Math.abs(initialEncoderValue-leftEncoder.getPosition());
    double distanceTraveled = currentEncoderPosition*RobotConstants.INCH_WHEEL_DIAMETER;
    SmartDashboard.putNumber("Distance Drive Encoder Position", currentEncoderPosition);
    return distanceTraveled >= targetDistance;
  }
}
