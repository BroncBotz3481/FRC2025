package frc.robot;


import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Rotations;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.ArmConstants;
import org.dyn4j.geometry.Rotation;

public class RobotMath
{
 public static class Arm{
   public static Angle convertArmAngleToSensorUnits(Angle measurement){
     return Rotations.of(measurement.in(Rotations) * ArmConstants.kArmReduction);
   }
   public static Angle convertSensorUnitsToArmAngle(Angle measurement){
     return Rotations.of(measurement.in(Rotations) / ArmConstants.kArmReduction);
   }
 }



  public static class Elevator{

    public static Distance convertRotationsToDistance(Angle rotations){
      return Meters.of(rotations.in(Rotations) *
              (2 * Math.PI * ElevatorConstants.kElevatorDrumRadius) / ElevatorConstants.kElevatorGearing);
    }

    public static Angle convertDistanceToRotations(Distance distance){
      return Rotations.of(distance.in(Meters) /
              (2 * Math.PI * ElevatorConstants.kElevatorDrumRadius) * ElevatorConstants.kElevatorGearing);
    }
  }


}
