package frc.robot.subsystems;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkMaxAlternateEncoder.Type;

import edu.wpi.first.wpilibj.DutyCycle;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;
import frc.robot.Constants.TurretConstants;

public class Turret extends SubsystemBase {
  CANSparkMax turretMotor = new CANSparkMax(TurretConstants.turretMotor_ID, MotorType.kBrushless);

  RelativeEncoder throughBoreAlternateEncoder = turretMotor.getAlternateEncoder(Type.kQuadrature, 8192);

  DutyCycleEncoder turrretEncoder = new DutyCycleEncoder(0);

  SparkPIDController turretController = turretMotor.getPIDController();

  double tx = LimelightHelpers.getTX("2");
  double setpoint = (tx / 360.0) * TurretConstants.gearRatio;


  public Turret() {

    turretMotor.restoreFactoryDefaults();
    turretMotor.setSmartCurrentLimit(50);

    turretMotor.setInverted(false);

    turretMotor.setIdleMode(IdleMode.kCoast);

    turretController.setP(TurretConstants.kP);
    turretController.setI(TurretConstants.kI);
    turretController.setD(TurretConstants.kD);

    turretController.setOutputRange(-4, 4);
    turretController.setFeedbackDevice(throughBoreAlternateEncoder);

    turrretEncoder.setDistancePerRotation(360);


  }

  @Override
  public void periodic() {

    /*NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
    NetworkTableEntry tx = table.getEntry("tx");
    NetworkTableEntry ty = table.getEntry("ty");
    NetworkTableEntry ta = table.getEntry("ta");

    //read values periodically
    double x = tx.getDouble(0.0);
    double y = ty.getDouble(0.0);
    double area = ta.getDouble(0.0);

    //post to smart dashboard periodically
    SmartDashboard.putNumber("LimelightX", x);
    SmartDashboard.putNumber("LimelightY", y);
    SmartDashboard.putNumber("LimelightArea", area);*/

    SmartDashboard.putNumber("Through Bore Value", throughborevalue());
    SmartDashboard.putNumber("Through Bore Value2", turrretEncoder.getDistance());
    


    

    double currentPosition = throughBoreAlternateEncoder.getPosition();
    double currentPosition2 = turrretEncoder.getAbsolutePosition();

    if (currentPosition > TurretConstants.maxAngle || currentPosition < TurretConstants.minAngle) {
      setpoint = 0;
    } 

     if (currentPosition2 > TurretConstants.maxAngle || currentPosition2 < TurretConstants.minAngle) {
      setpoint = 0;
    } 

  }

  public double throughborevalue() {
    double encoderValue = throughBoreAlternateEncoder.getPosition();

    return encoderValue;
  }

  public double throughborevalue2() {
    double turrretEncoderValue = turrretEncoder.getAbsolutePosition();

    return turrretEncoderValue;
  }

  public void moveTurret() {
    turretController.setReference(setpoint, CANSparkMax.ControlType.kPosition);
  }

}
