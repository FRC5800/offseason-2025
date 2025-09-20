// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import frc.robot.Constants.DriveConstants;
import com.ctre.phoenix.sensors.PigeonIMU;
import frc.robot.subsystems.SwerveModule;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SwerveSubsystem extends SubsystemBase {

  //Criando o giroscópio PigeonIMU que recebe a porta do DriveConstants
  private final PigeonIMU gyro = new PigeonIMU(DriveConstants.kGyroPort);

  // Odometria do robô, usando as cinemáticas e a posição inicial
  private final SwerveDriveOdometry odometer = new SwerveDriveOdometry(DriveConstants.kDriveKinematics,
            new Rotation2d(0));

  // Criando os 4 módulos swerve
  private final SwerveModule frontLeft = new SwerveModule(
        DriveConstants.kFrontLeftDriveMotorPort, //id do motor de tração
        DriveConstants.kFrontLeftTurningMotorPort, //id do motor de direção
        DriveConstants.kFrontLeftDriveEncoderReversed, // se o encoder de tração está invertido
        DriveConstants.kFrontLeftTurningEncoderReversed, // se o encoder de direção está invertido
        DriveConstants.kFrontLeftDriveAbsoluteEncoderPort, // id do encoder absoluto de tração
        DriveConstants.kFrontLeftDriveAbsoluteEncoderOffsetRad, // offset do encoder absoluto de tração em radianos
        DriveConstants.kFrontLeftDriveAbsoluteEncoderReversed); // se o encoder absoluto de tração está invertido

  private final SwerveModule frontRight = new SwerveModule(
          DriveConstants.kFrontRightDriveMotorPort,
          DriveConstants.kFrontRightTurningMotorPort,
          DriveConstants.kFrontRightDriveEncoderReversed,
          DriveConstants.kFrontRightTurningEncoderReversed,
          DriveConstants.kFrontRightDriveAbsoluteEncoderPort,
          DriveConstants.kFrontRightDriveAbsoluteEncoderOffsetRad,
          DriveConstants.kFrontRightDriveAbsoluteEncoderReversed);

  private final SwerveModule backLeft = new SwerveModule(
          DriveConstants.kBackLeftDriveMotorPort,
          DriveConstants.kBackLeftTurningMotorPort,
          DriveConstants.kBackLeftDriveEncoderReversed,
          DriveConstants.kBackLeftTurningEncoderReversed,
          DriveConstants.kBackLeftDriveAbsoluteEncoderPort,
          DriveConstants.kBackLeftDriveAbsoluteEncoderOffsetRad,
          DriveConstants.kBacktLeftDriveAbsoluteEncoderReversed);

  private final SwerveModule backRight = new SwerveModule(
            DriveConstants.kBackRightDriveMotorPort,
            DriveConstants.kBackRightTurningMotorPort,
            DriveConstants.kBackRightDriveEncoderReversed,
            DriveConstants.kBackRightTurningEncoderReversed,
            DriveConstants.kBackRightDriveAbsoluteEncoderPort,
            DriveConstants.kBackRightDriveAbsoluteEncoderOffsetRad,
            DriveConstants.kBackRightDriveAbsoluteEncoderReversed);  

   public SwerveSubsystem() {
        // Zera o gyro após 1 segundo para evitar leituras incorretas ao ligar
        new Thread(() -> {
            try {
                Thread.sleep(1000);
                zeroHeading();
            } catch (Exception e) {
                e.printStackTrace();
            }
        }).start();
    }

    // Método para zerar o heading (ângulo) do giroscópio Pigeon
    public void zeroHeading() {
        gyro.setYaw(0);
    }

    // Retorna o ângulo atual em graus dentro do intervalo -180 a 180
    public double getHeading() {
        double[] ypr = new double[3];
        gyro.getYawPitchRoll(ypr);
        // Retorna o yaw normalizado
        return Math.IEEEremainder(ypr[0], 360);
    }
    // descobrir como funciona isso e onde colocar pro código nn reclamar
    public SwerveModulePosition getPosition() {
    double distanceMeters = driveEncoder.getDistance();
    Rotation2d angle = Rotation2d.fromDegrees(turningEncoder.getAngle());
    return new SwerveModulePosition(distanceMeters, angle);
}

    // Retorna o Rotation2d do heading, para uso na odometria
    public Rotation2d getRotation2d() {
        return Rotation2d.fromDegrees(getHeading());
    }

    // Retorna a pose (posição + orientação) atual do robô no campo
    public Pose2d getPose() {
        return odometer.getPoseMeters();
    }

    // Reseta a odometria para uma pose conhecida (útil no início do jogo e auto)

    public void resetOdometry(Pose2d pose) {
      odometer.resetPosition(
          getRotation2d(),
          getModulePosition(), // criar um método que retorna SwerveModulePosition com base na distância e ângulo
          pose
      );
  }
  
    // Método chamado periodicamente 
    @Override
    public void periodic() {
        // Atualiza a odometria com os estados atuais dos módulos e o heading do gyro
        odometer.update(
            getRotation2d(),
            frontLeft.getModulePosition(),
            frontRight.geModuletPosition(),   
            backLeft.getModulePosition()
            backRight.getModulePosition()
        );

        // Exibe dados no SmartDashboard para debug/monitoramento
        SmartDashboard.putNumber("Robot Heading", getHeading());
        SmartDashboard.putString("Robot Location", getPose().getTranslation().toString());
    }

    // Para todos os módulos (motors)
    public void stopModules() {
        frontLeft.stop();
        frontRight.stop();
        backLeft.stop();
        backRight.stop();
    }

    // Define os estados desejados para os 4 módulos (velocidade e ângulo) definidos pelas cinemáticas 
    // e normaliza as velocidades para não exceder a velocidade máxima física do robô 
    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.normalizeWheelSpeeds(
            desiredStates,
            DriveConstants.kPhysicalMaxSpeedMetersPerSecond
        );
        frontLeft.setDesiredState(desiredStates[0]);
        frontRight.setDesiredState(desiredStates[1]);
        backLeft.setDesiredState(desiredStates[2]);
        backRight.setDesiredState(desiredStates[3]);
    }
}
   