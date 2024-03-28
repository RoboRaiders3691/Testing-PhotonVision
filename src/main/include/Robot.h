// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <string>

#include <frc/TimedRobot.h>
#include <frc/smartdashboard/SendableChooser.h>

#include <frc/geometry/Rotation2d.h>
#include <frc/geometry/Pose2d.h>
#include <frc/geometry/Pose3d.h>
#include <frc/geometry/Transform3d.h>
#include "frc/Timer.h"
#include <frc/apriltag/AprilTagFieldLayout.h>
#include <frc/apriltag/AprilTagFields.h>


#include <photon/PhotonCamera.h>
#include <photon/PhotonPoseEstimator.h>

class Robot : public frc::TimedRobot {
 public:
  void RobotInit() override;
  void RobotPeriodic() override;
  void AutonomousInit() override;
  void AutonomousPeriodic() override;
  void TeleopInit() override;
  void TeleopPeriodic() override;
  void DisabledInit() override;
  void DisabledPeriodic() override;
  void TestInit() override;
  void TestPeriodic() override;
  void SimulationInit() override;
  void SimulationPeriodic() override;

 private:
  frc::SendableChooser<std::string> m_chooser;
  const std::string kAutoNameDefault = "Default";
  const std::string kAutoNameCustom = "My Auto";
  std::string m_autoSelected;

  //Variable for Pi
  const double Pi = 3.1415926535;

   photon::PhotonCamera pCamera1{"PhotonCam1"};

  frc::Transform3d robotToCam1 =
  frc::Transform3d(frc::Translation3d(0_m, 0_m, 0_m),
                    frc::Rotation3d(0_rad, 0_rad, 0_rad));

  frc::AprilTagFieldLayout aprilTagFieldLayout = frc::LoadAprilTagLayoutField(frc::AprilTagField::k2024Crescendo);

  photon::PhotonPoseEstimator camPoseEstimator{
    aprilTagFieldLayout, photon::MULTI_TAG_PNP_ON_COPROCESSOR, std::move(pCamera1), robotToCam1};


};
