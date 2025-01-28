// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.List;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

/**
 * Swerve Drive subsystem
 */
public final class Drive extends SubsystemBase {
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
    LEFT_FRONT_DRIVE_DISTANCE_ENCODER = new CANcoder(Constants.kFrontLeftEncoderId);
    LEFT_BACK_DRIVE_DISTANCE_ENCODER = new CANcoder(Constants.kBackLeftEncoderId);
    RIGHT_FRONT_DRIVE_DISTANCE_ENCODER = new CANcoder(Constants.kFrontRightEncoderId);
    RIGHT_BACK_DRIVE_DISTANCE_ENCODER = new CANcoder(Constants.kBackRightEncoderId);

    LEFT_FRONT_DRIVE_DIRECTION_ENCODER = new CANcoder(Constants.kFrontLeftEncoderId);
    LEFT_BACK_DRIVE_DIRECTION_ENCODER = new CANcoder(Constants.kBackLeftEncoderId);
    RIGHT_FRONT_DRIVE_DIRECTION_ENCODER = new CANcoder(Constants.kFrontRightEncoderId);
    RIGHT_BACK_DRIVE_DIRECTION_ENCODER = new CANcoder(Constants.kBackRightEncoderId);

    // Gyro
    DRIVE_GYRO = new AHRS(NavXComType.kMXP_SPI);
    resetGyro();

    register();
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
    LEFT_FRONT_DRIVE_DIRECTION_MOTOR
        .set(pid.calculate(getLeftFrontAngle().getRadians(), angle.getRadians() - getGyroAngle()));
  }

  public void setLeftBackAngle(Rotation2d angle) {
    LEFT_BACK_DRIVE_DIRECTION_MOTOR
        .set(pid.calculate(getLeftBackAngle().getRadians(), angle.getRadians() - getGyroAngle()));
  }

  public void setRightFrontAngle(Rotation2d angle) {
    RIGHT_FRONT_DRIVE_DIRECTION_MOTOR
        .set(pid.calculate(getRightFrontAngle().getRadians(), angle.getRadians() - getGyroAngle()));
  }

  public void setRightBackAngle(Rotation2d angle) {
    RIGHT_BACK_DRIVE_DIRECTION_MOTOR
        .set(pid.calculate(getRightBackAngle().getRadians(), angle.getRadians() - getGyroAngle()));
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
    List<Double> v = new ArrayList<>();
    v.add(v1);
    v.add(v2);
    List<Double> m1 = new ArrayList<>();
    List<Double> m2 = new ArrayList<>();
    List<Double> m3 = new ArrayList<>();
    List<Double> m4 = new ArrayList<>();

    List<Double> p1 = new ArrayList<>();
    p1.add(
        -1.0 * Math.cos(Math.toRadians(DRIVE_GYRO.getAngle())) - 0.0 * Math.sin(Math.toRadians(DRIVE_GYRO.getAngle())));
    p1.add(
        -1.0 * Math.sin(Math.toRadians(DRIVE_GYRO.getAngle())) + 0.0 * Math.cos(Math.toRadians(DRIVE_GYRO.getAngle())));
    List<Double> p2 = new ArrayList<>();
    p2.add(0.0);
    p2.add(1.0);
    p2.add(
        0.0 * Math.cos(Math.toRadians(DRIVE_GYRO.getAngle())) - 1.0 * Math.sin(Math.toRadians(DRIVE_GYRO.getAngle())));
    p2.add(
        0.0 * Math.sin(Math.toRadians(DRIVE_GYRO.getAngle())) + 1.0 * Math.cos(Math.toRadians(DRIVE_GYRO.getAngle())));
    List<Double> p3 = new ArrayList<>();
    p3.add(1.0);
    p3.add(0.0);
    p3.add(
        1.0 * Math.cos(Math.toRadians(DRIVE_GYRO.getAngle())) - 0.0 * Math.sin(Math.toRadians(DRIVE_GYRO.getAngle())));
    p3.add(
        1.0 * Math.sin(Math.toRadians(DRIVE_GYRO.getAngle())) + 0.0 * Math.cos(Math.toRadians(DRIVE_GYRO.getAngle())));
    List<Double> p4 = new ArrayList<>();
    p4.add(0.0);
    p4.add(-1.0);
    p4.add(
        0.0 * Math.cos(Math.toRadians(DRIVE_GYRO.getAngle()))
            - -1.0 * Math.sin(Math.toRadians(DRIVE_GYRO.getAngle())));
    p4.add(
        0.0 * Math.sin(Math.toRadians(DRIVE_GYRO.getAngle()))
            + -1.0 * Math.cos(Math.toRadians(DRIVE_GYRO.getAngle())));
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

    double angle1 = Math.atan2(m1.get(1), m1.get(0));
    double angle2 = Math.atan2(m2.get(1), m2.get(0));
    double angle3 = Math.atan2(m3.get(1), m3.get(0));
    double angle4 = Math.atan2(m4.get(1), m4.get(0));

    if (Math.abs(angle1) > Math.PI / 2) {
      angle1 = angle1 > 0 ? angle1 - Math.PI : angle1 + Math.PI;
      LEFT_BACK_DRIVE_SPEED_MOTOR.set(-Math.sqrt(Math.pow(m1.get(0), 2) + Math.pow(m1.get(1), 2)) / max);
    } else {
      LEFT_BACK_DRIVE_SPEED_MOTOR.set(Math.sqrt(Math.pow(m1.get(0), 2) + Math.pow(m1.get(1), 2)) / max);
    }

    if (Math.abs(angle2) > Math.PI / 2) {
      angle2 = angle2 > 0 ? angle2 - Math.PI : angle2 + Math.PI;
      LEFT_FRONT_DRIVE_SPEED_MOTOR.set(-Math.sqrt(Math.pow(m2.get(0), 2) + Math.pow(m2.get(1), 2)) / max);
    } else {
      LEFT_FRONT_DRIVE_SPEED_MOTOR.set(Math.sqrt(Math.pow(m2.get(0), 2) + Math.pow(m2.get(1), 2)) / max);
    }

    if (Math.abs(angle3) > Math.PI / 2) {
      angle3 = angle3 > 0 ? angle3 - Math.PI : angle3 + Math.PI;
      RIGHT_FRONT_DRIVE_SPEED_MOTOR.set(-Math.sqrt(Math.pow(m3.get(0), 2) + Math.pow(m3.get(1), 2)) / max);
    } else {
      RIGHT_FRONT_DRIVE_SPEED_MOTOR.set(Math.sqrt(Math.pow(m3.get(0), 2) + Math.pow(m3.get(1), 2)) / max);
    }

    if (Math.abs(angle4) > Math.PI / 2) {
      angle4 = angle4 > 0 ? angle4 - Math.PI : angle4 + Math.PI;
      RIGHT_BACK_DRIVE_SPEED_MOTOR.set(-Math.sqrt(Math.pow(m4.get(0), 2) + Math.pow(m4.get(1), 2)) / max);
    } else {
      RIGHT_BACK_DRIVE_SPEED_MOTOR.set(Math.sqrt(Math.pow(m4.get(0), 2) + Math.pow(m4.get(1), 2)) / max);
    }

    System.out.println("Max: " + max);

    setLeftBackAngle(new Rotation2d(angle1));
    setLeftFrontAngle(new Rotation2d(angle2));
    setRightFrontAngle(new Rotation2d(angle3));
    setRightBackAngle(new Rotation2d(angle4));
  }

  public void initDefaultCommand() {
  }
}