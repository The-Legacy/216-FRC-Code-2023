// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
#include <frc/Joystick.h>
#include <frc/Compressor.h>
#include <frc/Solenoid.h>
#include <frc/XboxController.h>
#include <frc/TimedRobot.h>
#include <frc/drive/DifferentialDrive.h>
#include <frc/motorcontrol/PWMSparkMax.h>
#include <ctre/phoenix.h>
#include <frc/motorcontrol/MotorControllerGroup.h>
#include <frc/motorcontrol/PWMMotorController.h>
#include <frc/DigitalInput.h>
#include <AHRS.h>
#include <cameraserver/CameraServer.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <fmt/core.h>
#include <frc/smartdashboard/SendableChooser.h>
#include <frc/AddressableLED.h>
#include <frc/DriverStation.h>
/**
* This is a demo program showing the use of the DifferentialDrive class.
* Runs the motors with tank steering.
*/
class Robot : public frc::TimedRobot {
//LED's
static constexpr int kLength = 246;
frc::AddressableLED m_led{4};
std::array<frc::AddressableLED::LEDData, kLength> m_ledBuffer;
int firstPixelHue = 0;
//frc::DriverStation::DriverStation:: ds;
//auton chooser
frc::SendableChooser<std::string> m_chooser;
const std::string kAutoNameDefault = "Middle Auton";
const std::string kAutoNameCustom = "Side Auton";
std::string m_autoSelected;
//Falcon motor controllers
ctre::phoenix::motorcontrol::can::WPI_TalonFX m_Elevator{6};
ctre::phoenix::motorcontrol::can::WPI_TalonFX m_FrontRight{7};
ctre::phoenix::motorcontrol::can::WPI_TalonFX m_MidRight{5};
ctre::phoenix::motorcontrol::can::WPI_TalonFX m_BackRight{2};
ctre::phoenix::motorcontrol::can::WPI_TalonFX m_FrontLeft{8};
ctre::phoenix::motorcontrol::can::WPI_TalonFX m_MidLeft{3};
ctre::phoenix::motorcontrol::can::WPI_TalonFX m_BackLeft{4};
ctre::phoenix::motorcontrol::can::WPI_TalonFX m_TwoBar{10};
//sparks
frc::PWMSparkMax m_HorizElevator{0};
frc::PWMSparkMax m_FrontIntake{1};
frc::PWMSparkMax m_BackIntake{2};
//controllers
frc::XboxController Controller{0};
frc::XboxController Scontroller{1};
//limit switches
frc::DigitalInput TopLimitSwitch {0};
frc::DigitalInput BottomLimitSwitch {1};
//Nav X
AHRS NavX{frc::SPI::Port::kMXP};
//pnuematics
frc::Compressor PneumaHub{9, frc::PneumaticsModuleType::CTREPCM};
frc::Solenoid DogShift{9, frc::PneumaticsModuleType::CTREPCM, 5};
frc::Solenoid TwoBarBrake{9, frc::PneumaticsModuleType::CTREPCM, 6};
//motor groups
frc::MotorControllerGroup mg_RightDrive {m_FrontRight, m_MidRight, m_BackRight};
frc::MotorControllerGroup mg_Intake {m_FrontIntake, m_BackIntake};
frc::MotorControllerGroup mg_LeftDrive {m_MidLeft, m_FrontLeft, m_BackLeft};
frc::DifferentialDrive mg_RobotDrive{mg_LeftDrive, mg_RightDrive};
frc::Timer Timer;
//Pre-Sets
bool speedToggle = true;
bool led = true;
double desiredAngle = 0;
double power = 1;
public:
void Rainbow(){

    // For every pixel
    for (int i = 0; i < kLength; i++) {
      // Calculate the hue - hue is easier for rainbows because the color
      // shape is a circle so only one value needs to precess
      const auto pixelHue = (firstPixelHue + (i * 180 / kLength)) % 180;
      // Set the value
      m_ledBuffer[i].SetHSV(pixelHue, 255, 128);
    }
    // Increase by to make the rainbow "move"
    firstPixelHue += 1;
    // Check bounds
    firstPixelHue %= 180;
  }
  void DisabledPeriodic() override{
    Rainbow();
    m_led.SetData(m_ledBuffer);
  }
void RobotInit() override {
  //LED's
  m_led.SetLength(kLength);
  m_led.SetData(m_ledBuffer);
  m_led.Start();
  
  //auton chooser
  m_chooser.SetDefaultOption(kAutoNameDefault, kAutoNameDefault);
  m_chooser.AddOption(kAutoNameCustom, kAutoNameCustom);
  frc::SmartDashboard::PutData("Auto Modes", &m_chooser);
  //camera server
  frc::CameraServer::StartAutomaticCapture();
  //feeding the watchdog
  mg_RobotDrive.SetSafetyEnabled(true);
  mg_RobotDrive.SetSafetyEnabled(false);
  mg_RobotDrive.Feed();
  //inverting motors
  m_BackRight.SetInverted(true);
  m_MidLeft.SetInverted(true);
  m_FrontRight.SetInverted(true);
  //775 releases
  m_HorizElevator.SetSafetyEnabled(false);
  m_FrontIntake.SetSafetyEnabled(false);
  m_BackIntake.SetSafetyEnabled(false);
}

void AutonomousInit() override {
  double NavPitch = double(NavX.GetPitch());
  double NavYaw = double(NavX.GetYaw());
  double YawError = NavYaw / 300;
  Timer.Start();
  NavX.Calibrate();
  NavX.Reset();
  //auton selector
  m_autoSelected = m_chooser.GetSelected();
  fmt::print("Auto selected: {}\n", m_autoSelected);
  //side auton
  if (m_autoSelected == kAutoNameCustom) {
    if ((double)Timer.Get() < .5 ){
      mg_Intake.Set(.8);
      m_HorizElevator.Set(-.7);
    } else if ((double)Timer.Get() > .5 && (double)Timer.Get() < 1.5){
      m_HorizElevator.Set(0);
      mg_Intake.Set(-.95);
    } else if ((double)Timer.Get() > 1.5 && (double)Timer.Get() < 2){
      mg_Intake.Set(0);
    } else if((double)Timer.Get() > 2 && (double)Timer.Get() < 5) {
      mg_LeftDrive.Set(-.7);
      mg_RightDrive.Set(-.7);
    } else{
      mg_LeftDrive.Set(0);
      mg_RightDrive.Set(0);
    }
  //middle auton
    } else {
      if ((double)Timer.Get() < .5 ){
        mg_Intake.Set(.8);
        m_HorizElevator.Set(-.7);
      } else if ((double)Timer.Get() > .5 && (double)Timer.Get() < 1.5){
        m_HorizElevator.Set(0);
        mg_Intake.Set(-.95);
      } else if ((double)Timer.Get() > 1.5 && (double)Timer.Get() < 2){
        mg_Intake.Set(0);
      } else if ((double)Timer.Get() > 2 && (double)Timer.Get() < 4){
        if (NavYaw < 179) {
          mg_RightDrive.Set(-.4);
          mg_LeftDrive.Set(.4);
        } else if (NavYaw > 181){
          mg_RightDrive.Set(.4);
          mg_LeftDrive.Set(-.4);
        } else {
          mg_RightDrive.Set(0);
          mg_LeftDrive.Set(0);
        }
      } else if ((double)Timer.Get() > 4 && (double)Timer.Get() < 6){
        DogShift.Set(Off);
        mg_LeftDrive.Set(.7 + YawError);
        mg_RightDrive.Set(.7 - YawError);
      } else if ((double)Timer.Get() > 6 && (double)Timer.Get() < 6.25){
        mg_LeftDrive.Set(0);
        mg_RightDrive.Set(0);
      } else if ((double)Timer.Get() > 6.25 && NavPitch > 9){
        mg_LeftDrive.Set(.15);
        mg_RightDrive.Set(.15);
      } else if ((double)Timer.Get() > 6.5 && NavPitch < 9 && NavPitch > -9){
        m_FrontRight.SetNeutralMode(NeutralMode::Brake);
        m_MidRight.SetNeutralMode(NeutralMode::Brake);
        m_BackRight.SetNeutralMode(NeutralMode::Brake);
        m_FrontLeft.SetNeutralMode(NeutralMode::Brake);
        m_MidLeft.SetNeutralMode(NeutralMode::Brake);
        m_BackLeft.SetNeutralMode(NeutralMode::Brake);
       } else if ((double)Timer.Get() > 6.5 && NavPitch < -9){
        mg_LeftDrive.Set(-.15);
        mg_RightDrive.Set(-.15);
      }
    }
  }

void AutonomousPeriodic() override {
  DogShift.Set(On);
  double NavPitch = double(NavX.GetPitch());
  double NavYaw = double(NavX.GetYaw());
  double YawError = NavYaw / 300;
  //Changes the lights to red when we are red in match and designated red
    if(led == true && frc::DriverStation::IsTeleopEnabled() && frc::DriverStation::GetAlliance() == frc::DriverStation::Alliance::kRed){
      for (int i = 0; i < kLength; i++) {
        m_ledBuffer[i].SetRGB(255, 0, 0);
      }
      m_led.SetData(m_ledBuffer);
    }
  //Changes the lights to blue when we are blue in match and designated blue
    if(led == true && frc::DriverStation::IsTeleopEnabled() && frc::DriverStation::GetAlliance() == frc::DriverStation::Alliance::kBlue){
      for (int i = 0; i < kLength; i++) {
        m_ledBuffer[i].SetRGB(0, 0, 255);
      }
      m_led.SetData(m_ledBuffer);
    }
  //side auton
  if (m_autoSelected == kAutoNameCustom) {
    if ((double)Timer.Get() < .5 ){
      mg_Intake.Set(.8);
      m_HorizElevator.Set(-.7);
    } else if ((double)Timer.Get() > .5 && (double)Timer.Get() < 1.5){
      m_HorizElevator.Set(0);
      mg_Intake.Set(-.95);
    } else if ((double)Timer.Get() > 1.5 && (double)Timer.Get() < 2){
      mg_Intake.Set(0);
    } else if((double)Timer.Get() > 2 && (double)Timer.Get() < 5) {
      mg_LeftDrive.Set(-.7);
      mg_RightDrive.Set(-.7);
    } else{
      mg_LeftDrive.Set(0);
      mg_RightDrive.Set(0);
    }
  //middle auton
  } else {
    if ((double)Timer.Get() < .5 ){
      mg_Intake.Set(.8);
      m_HorizElevator.Set(-.7);
    } else if ((double)Timer.Get() > .5 && (double)Timer.Get() < 1.5){
      m_HorizElevator.Set(0);
      mg_Intake.Set(-.95);
    } else if ((double)Timer.Get() > 1.5 && (double)Timer.Get() < 2){
      mg_Intake.Set(0);
    } else if ((double)Timer.Get() > 2 && (double)Timer.Get() < 4){
      if (NavYaw < 179) {
        mg_RightDrive.Set(-.4);
        mg_LeftDrive.Set(.4);
      } else if (NavYaw > 181){
        mg_RightDrive.Set(.4);
        mg_LeftDrive.Set(-.4);
      } else {
        mg_RightDrive.Set(0);
        mg_LeftDrive.Set(0);
      }
    } else if ((double)Timer.Get() > 4 && (double)Timer.Get() < 6){
      DogShift.Set(Off);
      mg_LeftDrive.Set(.4 + YawError);
      mg_RightDrive.Set(.4 - YawError);
    } else if ((double)Timer.Get() > 6 && (double)Timer.Get() < 6.25){
      mg_LeftDrive.Set(0);
      mg_RightDrive.Set(0);
    } else if ((double)Timer.Get() > 6.25 && NavPitch > 9){
      mg_LeftDrive.Set(.15);
      mg_RightDrive.Set(.15);
    } else if ((double)Timer.Get() > 6.5 && NavPitch < 9 && NavPitch > -9){
      m_FrontRight.SetNeutralMode(NeutralMode::Brake);
      m_MidRight.SetNeutralMode(NeutralMode::Brake);
      m_BackRight.SetNeutralMode(NeutralMode::Brake);
      m_FrontLeft.SetNeutralMode(NeutralMode::Brake);
      m_MidLeft.SetNeutralMode(NeutralMode::Brake);
      m_BackLeft.SetNeutralMode(NeutralMode::Brake);
      mg_LeftDrive.Set(0);
      mg_RightDrive.Set(0);
    } else if ((double)Timer.Get() > 6.5 && NavPitch < -9){
      mg_LeftDrive.Set(-.15);
      mg_RightDrive.Set(-.15);
    }
  }  
}

void TeleopInit() override{
  NavX.Reset();
  
  
}
void TeleopPeriodic() override {
  //Changes the lights to red when we are red in match and designated red
    if(led == true && frc::DriverStation::IsTeleopEnabled() && frc::DriverStation::GetAlliance() == frc::DriverStation::Alliance::kRed){
      for (int i = 0; i < kLength; i++) {
        m_ledBuffer[i].SetRGB(255, 0, 0);
      }
    }
  //Changes the lights to blue when we are blue in match and designated blue
    if(led == true && frc::DriverStation::IsTeleopEnabled() && frc::DriverStation::GetAlliance() == frc::DriverStation::Alliance::kBlue){
      for (int i = 0; i < kLength; i++) {
        m_ledBuffer[i].SetRGB(0, 0, 255);
      }
    }
  if(frc::DriverStation::GetMatchTime() <= 30.0){
      for (int i = 0; i < kLength; i++) {
        m_ledBuffer[i].SetRGB(0, 255, 0);
      }
    }
  // if(frc::DriverStation::GetMatchTime() <=20.0){
  //   DogShift.Set(On);
  // }
    m_led.SetData(m_ledBuffer);
  double NavPitch = double(NavX.GetPitch());
  //SmartDashboard
  frc::SmartDashboard::PutNumber("Balancing Angle", NavPitch);
  //firstController////////////////////////////////////////////////////////////////
  //Drive with arcade style
  mg_RobotDrive.ArcadeDrive(-Controller.GetLeftY() * power, -Controller.GetRightX() * (.65 * power ));
  //maybe braking
  // if (Scontroller.GetBButtonPressed() == true && speedToggle == true) {
  //   power = .3;
  //   speedToggle = false;
  // } 
  // if (Scontroller.GetBButtonPressed() == true && speedToggle ==false) {
  //   power = .3;
  //   speedToggle = true;
  // }
  //coast to break
  if (Scontroller.GetBButtonPressed()) {
    m_FrontRight.SetNeutralMode(NeutralMode::Brake);
    m_MidRight.SetNeutralMode(NeutralMode::Brake);
    m_BackRight.SetNeutralMode(NeutralMode::Brake);
    m_FrontLeft.SetNeutralMode(NeutralMode::Brake);
    m_MidLeft.SetNeutralMode(NeutralMode::Brake);
    m_BackLeft.SetNeutralMode(NeutralMode::Brake);
    }
  if (Scontroller.GetAButtonPressed()) {
    m_FrontRight.SetNeutralMode(NeutralMode::Coast);
    m_MidRight.SetNeutralMode(NeutralMode::Coast);
    m_BackRight.SetNeutralMode(NeutralMode::Coast);
    m_FrontLeft.SetNeutralMode(NeutralMode::Coast);
    m_MidLeft.SetNeutralMode(NeutralMode::Coast);
    m_BackLeft.SetNeutralMode(NeutralMode::Coast);
    }

  double MaxSpeed = (.85 + (Controller.GetLeftTriggerAxis())*.15);
  mg_RobotDrive.SetMaxOutput(MaxSpeed);
  //shifting
  if (Controller.GetBButtonPressed()){
    DogShift.Toggle();
  
  m_led.SetData(m_ledBuffer);
  }
  // cone intake
  if(Controller.GetLeftBumperReleased() ||
    Controller.GetRightBumperReleased()){
    mg_Intake.Set(0);
  } else if(Controller.GetRightBumperPressed()){
    mg_Intake.Set(0.65);
  } else if(Controller.GetLeftBumperPressed()){
    mg_Intake.Set(-0.65);
  }
  // cube intake
  if(Controller.GetYButtonPressed()){
    m_BackIntake.Set(-.65);
    m_FrontIntake.Set(.65);
  }
  //cube outtake
  if (Controller.GetXButtonPressed()){
    mg_Intake.Set(-.75);
  }
  //kill button
  if (Controller.GetAButtonPressed()) {
    mg_Intake.StopMotor();
  }
  //second controller/////////////////////////////////////////////////
  if (Scontroller.GetStartButtonPressed()) {
    PneumaHub.Disable();
  } else if (Scontroller.GetBackButtonPressed()) {
    PneumaHub.Enabled();
  }
  if (Scontroller.GetYButtonPressed()){
    led = false;
    for (int i = 0; i < kLength; i++) {
      m_ledBuffer[i].SetRGB(255, 255, 0);
}
    m_led.SetData(m_ledBuffer);
  }
  if (Scontroller.GetXButtonPressed()){
    led = false;
    for (int i = 0; i < kLength; i++) {
      m_ledBuffer[i].SetRGB(255, 0, 255);
}
    m_led.SetData(m_ledBuffer);
  }
  if (Scontroller.GetRightBumperPressed()){
    led = true;
    for (int i = 0; i < kLength; i++) {
      m_ledBuffer[i].SetRGB(0, 0, 0);
}
    m_led.SetData(m_ledBuffer);
  }
  //vertical elevator
  m_Elevator.Set(ControlMode::PercentOutput, -Scontroller.GetLeftY());
  if(Scontroller.GetLeftY() > 0 && TopLimitSwitch.Get() == false){
    m_Elevator.Set(ControlMode::PercentOutput, 0);
  }
  if(Scontroller.GetLeftY() < 0 && BottomLimitSwitch.Get() == false){
    m_Elevator.Set(ControlMode::PercentOutput, 0);
  }
  //horizontal elevator
  m_HorizElevator.Set(Scontroller.GetRightX()*.65);
  //Two bar
  if(Scontroller.GetRightTriggerAxis() < .5 and
    Scontroller.GetLeftTriggerAxis() < .5){
    m_TwoBar.Set(ControlMode::PercentOutput, 0);
    TwoBarBrake.Set(false);
  } else if(Scontroller.GetRightTriggerAxis() > .5){
    m_TwoBar.Set(ControlMode::PercentOutput, .24);
    TwoBarBrake.Set(true);
  } else if(Scontroller.GetLeftTriggerAxis() > .5){
    m_TwoBar.Set(ControlMode::PercentOutput, -.24);
    TwoBarBrake.Set(true);
  }
  //kill button
  if (Scontroller.GetAButtonPressed()) {
    m_Elevator.Set(ControlMode::PercentOutput, 0);
    m_HorizElevator.StopMotor();
    m_TwoBar.Set(ControlMode::PercentOutput, 0);
  }

}
};
#ifndef RUNNING_FRC_TESTS
int main() {
return frc::StartRobot<Robot>();
}
#endif
