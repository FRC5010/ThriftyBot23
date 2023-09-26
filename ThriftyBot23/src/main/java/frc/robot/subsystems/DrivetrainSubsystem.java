// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.SerialPort.Port;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DrivetrainSubsystem extends SubsystemBase {
  private CANSparkMax motor1;
  private CANSparkMax motor2;
  private DifferentialDrive diffDrive;
  private AHRS gyro;
  /** Creates a new DrivetrainSubsystem. */
  public DrivetrainSubsystem() {
    motor1 = new CANSparkMax(8,MotorType.kBrushless);
    motor1.restoreFactoryDefaults();
    motor2 = new CANSparkMax(9, MotorType.kBrushless);
    motor2.restoreFactoryDefaults();
    //motor1.setInverted(true);
    motor2.setInverted(true);
    diffDrive = new DifferentialDrive(motor1, motor2);
    gyro = new AHRS(Port.kUSB);
}
  public void drive(double throttle, double steer){
    diffDrive.arcadeDrive(throttle, steer);
  }
  public void runMotor(double speed){
    motor1.set(speed);
    motor2.set(speed);
  }

  public void stopMotor(){
    motor1.set(0);
    motor2.set(0);
  }

  public void turnMotors(double speed){
    motor1.set(-speed);
    motor2.set(speed);
  }

  public void resetGyro(double heading){
    System.out.println("Gyro Command Start");
    gyro.reset();
    gyro.setAngleAdjustment(heading);
    System.out.println("Gyro Command End");
  }

  public double getHeading(){
    return gyro.getYaw();
  }

  public double getPitch(){
    return gyro.getPitch();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
