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

import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class XDrive extends SubsystemBase {
    private SparkMax lf = new SparkMax(1, MotorType.kBrushless);
    private SparkMax rf = new SparkMax(2, MotorType.kBrushless);
    private SparkMax rb = new SparkMax(4, MotorType.kBrushless);
    private SparkMax lb = new SparkMax(3, MotorType.kBrushless);
    
    public XDrive() {
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
        double Vx = x * 1.1;
        double Vy = -y;
        double omega = r;

        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(r), 1);
        double FL = (Vy + Vx + omega) / denominator;
        double FR = (Vy - Vx - omega) / denominator;
        double BL = (Vy - Vx + omega) / denominator;
        double BR = (Vy + Vx - omega) / denominator;

        // Aplicando aos motores
        lf.set(FL*0.7);
        rf.set(FR*0.7);
        lb.set(BL*0.7);
        rb.set(BR*0.7);
    }
}
