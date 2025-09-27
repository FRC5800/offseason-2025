// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class XDrive extends SubsystemBase {
    private SparkMax lf = new SparkMax(1, MotorType.kBrushless);
    private SparkMax rf = new SparkMax(2, MotorType.kBrushless);
    private SparkMax rb = new SparkMax(4, MotorType.kBrushless);
    private SparkMax lb = new SparkMax(3, MotorType.kBrushless);

    AHRS gyro = new AHRS(NavXComType.kMXP_SPI);

    double FL = 0;
    double FR = 0;
    double BL = 0;
    double BR = 0;

    double lerp(double a, double b, double f)  {
        return a + f * (b - a);
    }

    public XDrive() {
        gyro.zeroYaw();
        var lfconfig = new SparkMaxConfig();
        lfconfig.idleMode(IdleMode.kBrake);
        lfconfig.smartCurrentLimit(80);
        lfconfig.disableFollowerMode();
        lf.configure(lfconfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);

        var rfconfig = new SparkMaxConfig();
        rfconfig.idleMode(IdleMode.kBrake);
        rfconfig.smartCurrentLimit(80);
        rfconfig.disableFollowerMode();
        rf.configure(rfconfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);

        var rbconfig = new SparkMaxConfig();
        rbconfig.idleMode(IdleMode.kBrake);
        rbconfig.smartCurrentLimit(80);
        rbconfig.disableFollowerMode();
        rb.configure(rbconfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);

        var lbconfig = new SparkMaxConfig();
        lbconfig.idleMode(IdleMode.kBrake);
        lbconfig.smartCurrentLimit(80);
        lbconfig.disableFollowerMode();
        lb.configure(lbconfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);


    }

    public void drive(double y, double x, double r) {
        double botHeading = -Math.toRadians(gyro.getAngle());
        double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
        double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);
        rotX *= 1.1;
        double omega = r;

        double denominator = Math.max(Math.abs(rotX) + Math.abs(rotY) + Math.abs(r), 1);
        FL = lerp(FL, ((rotY + rotX + omega) / denominator) * 0.7, 0.1);
        FR = lerp(FR, ((rotY - rotX - omega) / denominator) * 0.7, 0.1);
        BL = lerp(BL, ((rotY - rotX + omega) / denominator) * 0.7, 0.1);
        BR = lerp(BR, ((rotY + rotX - omega) / denominator) * 0.7, 0.1);

        // Aplicando aos motores
        lf.set(FL);
        rf.set(FR);
        lb.set(BL);
        rb.set(BR);
    }

    public void periodic(){
            
    SmartDashboard.putNumber("gyro", gyro.getAngle());
    
    
        }
}