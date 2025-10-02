// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class VisionSystem extends SubsystemBase {
  private AprilTagFieldLayout aprilTagFieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded);
  private Transform3d robotToCam = new Transform3d(0.33, -0.165, 0.285, new Rotation3d());
  private PhotonPoseEstimator photonPoseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, robotToCam);
  PhotonCamera camera = new PhotonCamera("Arducam_OV9281_USB_Camera");
  Optional<EstimatedRobotPose> pose = Optional.empty();

  /** Creates a new VisionSystem. */
  public VisionSystem() {}

  public Optional<Pose2d> getEstimatedPose() {
    return pose.map(p -> p.estimatedPose.toPose2d());
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    List<PhotonPipelineResult> results = camera.getAllUnreadResults();
    for(var result: results)
      pose = photonPoseEstimator.update(result);
  }
}
