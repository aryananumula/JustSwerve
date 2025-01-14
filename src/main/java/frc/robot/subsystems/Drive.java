// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;

import java.util.ArrayList;
import java.util.List;

import com.ctre.phoenix6.hardware.CANcoder;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

/**
 * Swerve Drive subsystem
 */
public class Drive extends SubsystemBase {
  public static final PIDController pid = new PIDController(2, 1, 0.1);
  // Motors
  public static TalonFX LEFT_FRONT_DRIVE_SPEED_MOTOR;
  public static TalonFX LEFT_BACK_DRIVE_SPEED_MOTOR;
  public static TalonFX RIGHT_FRONT_DRIVE_SPEED_MOTOR;
  public static TalonFX RIGHT_BACK_DRIVE_SPEED_MOTOR;

  public static TalonFX LEFT_FRONT_DRIVE_DIRECTION_MOTOR;
  public static TalonFX LEFT_BACK_DRIVE_DIRECTION_MOTOR;
  public static TalonFX RIGHT_FRONT_DRIVE_DIRECTION_MOTOR;
  public static TalonFX RIGHT_BACK_DRIVE_DIRECTION_MOTOR;

  // Encoders
  public static CANcoder LEFT_FRONT_DRIVE_DISTANCE_ENCODER;
  public static CANcoder LEFT_BACK_DRIVE_DISTANCE_ENCODER;
  public static CANcoder RIGHT_FRONT_DRIVE_DISTANCE_ENCODER;
  public static CANcoder RIGHT_BACK_DRIVE_DISTANCE_ENCODER;
  // public static MedianPIDSource DRIVE_DISTANCE_ENCODERS;

  public static CANcoder LEFT_FRONT_DRIVE_DIRECTION_ENCODER;
  public static CANcoder LEFT_BACK_DRIVE_DIRECTION_ENCODER;
  public static CANcoder RIGHT_FRONT_DRIVE_DIRECTION_ENCODER;
  public static CANcoder RIGHT_BACK_DRIVE_DIRECTION_ENCODER;

  // Gyro
  public static AHRS DRIVE_GYRO;

  public Drive() {
    // Motors
    LEFT_FRONT_DRIVE_SPEED_MOTOR = new TalonFX(Constants.kFrontLeftDriveMotorId);
    LEFT_BACK_DRIVE_SPEED_MOTOR = new TalonFX(Constants.kBackLeftDriveMotorId);
    RIGHT_FRONT_DRIVE_SPEED_MOTOR = new TalonFX(Constants.kFrontRightDriveMotorId);
    RIGHT_BACK_DRIVE_SPEED_MOTOR = new TalonFX(Constants.kBackRightDriveMotorId);

    LEFT_FRONT_DRIVE_DIRECTION_MOTOR = new TalonFX(Constants.kFrontLeftDriveMotorId);
    LEFT_BACK_DRIVE_DIRECTION_MOTOR = new TalonFX(Constants.kBackLeftDriveMotorId);
    RIGHT_FRONT_DRIVE_DIRECTION_MOTOR = new TalonFX(Constants.kFrontRightDriveMotorId);
    RIGHT_BACK_DRIVE_DIRECTION_MOTOR = new TalonFX(Constants.kBackRightDriveMotorId);

    // Encoders
    LEFT_FRONT_DRIVE_DISTANCE_ENCODER = new CANcoder(Constants.kFrontLeftEncoderId, Constants.kCANbusName);
    LEFT_BACK_DRIVE_DISTANCE_ENCODER = new CANcoder(Constants.kBackLeftEncoderId, Constants.kCANbusName);
    RIGHT_FRONT_DRIVE_DISTANCE_ENCODER = new CANcoder(Constants.kFrontRightEncoderId, Constants.kCANbusName);
    RIGHT_BACK_DRIVE_DISTANCE_ENCODER = new CANcoder(Constants.kBackRightEncoderId, Constants.kCANbusName);

    LEFT_FRONT_DRIVE_DIRECTION_ENCODER = new CANcoder(Constants.kFrontLeftEncoderId, Constants.kCANbusName);
    LEFT_BACK_DRIVE_DIRECTION_ENCODER = new CANcoder(Constants.kBackLeftEncoderId, Constants.kCANbusName);
    RIGHT_FRONT_DRIVE_DIRECTION_ENCODER = new CANcoder(Constants.kFrontRightEncoderId, Constants.kCANbusName);
    RIGHT_BACK_DRIVE_DIRECTION_ENCODER = new CANcoder(Constants.kBackRightEncoderId, Constants.kCANbusName);

    // Gyro
    DRIVE_GYRO = new AHRS(NavXComType.kMXP_SPI);
  }

  public void resetGyro() {
    DRIVE_GYRO.reset();
  }

  public double getGyroAngle() {
    return DRIVE_GYRO.getAngle();
  }

  public Rotation2d getLeftFrontAngle() {
    return new Rotation2d(LEFT_FRONT_DRIVE_DIRECTION_ENCODER.getPosition().getValueAsDouble());
  }

  public Rotation2d getLeftBackAngle() {
    return new Rotation2d(LEFT_BACK_DRIVE_DIRECTION_ENCODER.getPosition().getValueAsDouble());
  }

  public Rotation2d getRightFrontAngle() {
    return new Rotation2d(RIGHT_FRONT_DRIVE_DIRECTION_ENCODER.getPosition().getValueAsDouble());
  }

  public Rotation2d getRightBackAngle() {
    return new Rotation2d(RIGHT_BACK_DRIVE_DIRECTION_ENCODER.getPosition().getValueAsDouble());
  }

  public void setLeftFrontAngle(Rotation2d angle) {
    LEFT_FRONT_DRIVE_DIRECTION_MOTOR.set(pid.calculate(getLeftFrontAngle().getRadians(), angle.getRadians()));
  }

  public void setLeftBackAngle(Rotation2d angle) {
    LEFT_BACK_DRIVE_DIRECTION_MOTOR.set(pid.calculate(getLeftBackAngle().getRadians(), angle.getRadians()));
  }

  public void setRightFrontAngle(Rotation2d angle) {
    RIGHT_FRONT_DRIVE_DIRECTION_MOTOR.set(pid.calculate(getRightFrontAngle().getRadians(), angle.getRadians()));
  }

  public void setRightBackAngle(Rotation2d angle) {
    RIGHT_BACK_DRIVE_DIRECTION_MOTOR.set(pid.calculate(getRightBackAngle().getRadians(), angle.getRadians()));
  }

  public void setLeftFrontSpeed(double speed) {
    LEFT_FRONT_DRIVE_SPEED_MOTOR.set(speed);
  }

  public void setLeftBackSpeed(double speed) {
    LEFT_BACK_DRIVE_SPEED_MOTOR.set(speed);
  }

  public void setRightFrontSpeed(double speed) {
    RIGHT_FRONT_DRIVE_SPEED_MOTOR.set(speed);
  }

  public void setRightBackSpeed(double speed) {
    RIGHT_BACK_DRIVE_SPEED_MOTOR.set(speed);
  }

  public void drive(double v1, double v2, double w) {
    w += getGyroAngle();
    List<Double> v = new ArrayList<>();
    v.add(v1);
    v.add(v2);
    List<Double> m1 = new ArrayList<>();
    List<Double> m2 = new ArrayList<>();
    List<Double> m3 = new ArrayList<>();
    List<Double> m4 = new ArrayList<>();

    List<Double> p1 = new ArrayList<>();
    p1.add(-1.0);
    p1.add(0.0);
    List<Double> p2 = new ArrayList<>();
    p2.add(0.0);
    p2.add(1.0);
    List<Double> p3 = new ArrayList<>();
    p3.add(1.0);
    p3.add(0.0);
    List<Double> p4 = new ArrayList<>();
    p4.add(0.0);
    p4.add(-1.0);
    m1.add(v.get(0) + w * p1.get(0));
    m1.add(v.get(1) + w * p1.get(1));
    m2.add(v.get(0) + w * p2.get(0));
    m2.add(v.get(1) + w * p2.get(1));
    m3.add(v.get(0) + w * p3.get(0));
    m3.add(v.get(1) + w * p3.get(1));
    m4.add(v.get(0) + w * p4.get(0));
    m4.add(v.get(1) + w * p4.get(1));
    // Post-Normalization
    double max = Math.max(Math.sqrt(Math.pow(m1.get(0), 2) + Math.pow(m1.get(1), 2)),
        Math.max(Math.sqrt(Math.pow(m2.get(0), 2) + Math.pow(m2.get(1), 2)),
            Math.max(Math.sqrt(Math.pow(m3.get(0), 2) + Math.pow(m3.get(1), 2)),
                Math.sqrt(Math.pow(m4.get(0), 2) + Math.pow(m4.get(1), 2)))));
    LEFT_BACK_DRIVE_SPEED_MOTOR.set(Math.sqrt(Math.pow(m1.get(0), 2) + Math.pow(m1.get(1), 2)) / max);
    LEFT_FRONT_DRIVE_SPEED_MOTOR.set(Math.sqrt(Math.pow(m2.get(0), 2) + Math.pow(m2.get(1), 2)) / max);
    RIGHT_FRONT_DRIVE_SPEED_MOTOR.set(Math.sqrt(Math.pow(m3.get(0), 2) + Math.pow(m3.get(1), 2)) / max);
    RIGHT_BACK_DRIVE_SPEED_MOTOR.set(Math.sqrt(Math.pow(m4.get(0), 2) + Math.pow(m4.get(1), 2)) / max);
    setLeftBackAngle(new Rotation2d(Math.atan2(m1.get(1), m1.get(0))));
    setLeftFrontAngle(new Rotation2d(Math.atan2(m2.get(1), m2.get(0))));
    setRightFrontAngle(new Rotation2d(Math.atan2(m3.get(1), m3.get(0))));
    setRightBackAngle(new Rotation2d(Math.atan2(m4.get(1), m4.get(0))));
  }

  public void initDefaultCommand() {
  }
}