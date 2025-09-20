// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj.AnalogInput;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.AbsoluteEncoder;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SwerveModule extends SubsystemBase {
  //variáveis do SwerveModule

  //motores NEO de tração e rotação com controladores SparkMax
  final SparkMax driveMotor;
  final SparkMax turningMotor;

  //pid de rotação
  final PIDController turningPidController;
  
  //encoders dos motores
  final RelativeEncoder turningEncoder;
  final RelativeEncoder driveEncoder;

  
  //encoder absoluto
  final boolean absoluteEncoderReversed;
  final double absoluteEncoderOffsetRad;
  final AnalogInput absoluteEncoder;
  

          //Construtor do módulo
          public SwerveModule(int driveMotorId, int turningMotorId, boolean driveMotorReversed, boolean turningMotorReversed,
          int absoluteEncoderId, double absoluteEncoderOffset, boolean absoluteEncoderReversed)
          
          {
                //id dos motores
                driveMotor = new SparkMax(driveMotorId, MotorType.kBrushless);
                turningMotor = new SparkMax(turningMotorId, MotorType.kBrushless);
                
                //config dos motores, talvez colocar nas constantes
                var driveMotorConfig = new SparkMaxConfig();
                driveMotorConfig.smartCurrentLimit(60);
                driveMotorConfig.idleMode(null);
                driveMotorConfig.inverted(driveMotorReversed);
                driveMotor.configure(driveMotorConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
              
                  
                    //construtores do encoder absoluto, off set e reversed: 
                    this.absoluteEncoder = new AnalogInput(absoluteEncoderId); 
                    this.absoluteEncoderOffsetRad = absoluteEncoderOffset;
                    this.absoluteEncoderReversed = absoluteEncoderReversed;
                    
              

                //método que retorna os valores dos encoders
                driveEncoder = driveMotor.getEncoder();
                turningEncoder = turningMotor.getEncoder();

                //Conversão de unidades
                //obs: module constants precisa ser criado na classe de constants
                driveEncoder.setPositionConversionFactor(ModuleConstants.kDriveEncoderRot2Meter);
                driveEncoder.setVelocityConversionFactor(ModuleConstants.kDriveEncoderRPM2MeterPerSec);
                turningEncoder.setPositionConversionFactor(ModuleConstants.kTurningEncoderRot2Rad);
                turningEncoder.setVelocityConversionFactor(ModuleConstants.kTurningEncoderRPM2RadPerSec);

                turningPidController = new PIDController(ModuleConstants.kPTurning, 0, 0);
                turningPidController.enableContinuousInput(-Math.PI, Math.PI);

                resetEncoders();
            }
      //Posições medidas pelos encoders de tração e rotação na arena
        public double getDrivePosition() {
        return driveEncoder.getPosition();
    }
         public double getTurningPosition() {
        return turningEncoder.getPosition();
    }

      //Velocidades medidas pelos encoders de tração e rotação
        public double getDriveVelocity() {
        return driveEncoder.getVelocity();
    }
        public double getTurningVelocity() {
        return turningEncoder.getVelocity();
    }
   
    
    //cria um método que retorna o valor do encoder absoluto 
          public double getAbsoluteEncoderRad() {
          double angle = absoluteEncoder.getVoltage() / RobotController.getVoltage5V(); //
           angle *= 2.0 * Math.PI;
           angle -= absoluteEncoderOffsetRad; //subtrai do valor do offset
           return angle * (absoluteEncoderReversed ? -1.0 : 1.0); // indica se está positivo 
          } 
    

        public SwerveModuleState getState() {
            return new SwerveModuleState(getDriveVelocity(), new Rotation2d(getTurningPosition()));
        }

        public void setDesiredState(SwerveModuleState state) {
            if (Math.abs(state.speedMetersPerSecond) < 0.001) {
                stop();
                return;
            }
            state = SwerveModuleState.optimize(state, getState().angle);
            driveMotor.set(state.speedMetersPerSecond / DriveConstants.kPhysicalMaxSpeedMetersPerSecond);
            turningMotor.set(turningPidController.calculate(getTurningPosition(), state.angle.getRadians()));
            SmartDashboard.putString("Swerve[" + absoluteEncoder.getChannel() + "] state", state.toString());
        }

        public void stop() {
            driveMotor.set(0);
            turningMotor.set(0);
        }
  }