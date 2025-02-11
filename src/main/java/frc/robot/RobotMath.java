package frc.robot;


import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Rotations;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import frc.robot.Constants.AlgaeArmConstants;
import frc.robot.Constants.CoralArmConstants;
import frc.robot.Constants.ElevatorConstants;
import org.dyn4j.geometry.Rotation;

public class RobotMath
{

  public static class AlgaeArm
  {

    /**
     * Convert {@link Angle} into motor {@link Angle}
     *
     * @param measurement Angle, to convert.
     * @return {@link Angle} equivalent to rotations of the motor.
     */
    public static Angle convertAlgaeAngleToSensorUnits(Angle measurement)
    {
      return Rotations.of(measurement.in(Rotations) * AlgaeArmConstants.kAlgaeArmReduction);
    }

    /**
     * Convert motor rotations {@link Angle} into usable {@link Angle}
     *
     * @param measurement Motor roations
     * @return Usable angle.
     */
    public static Angle convertSensorUnitsToAlgaeAngle(Angle measurement)
    {
      return Rotations.of(measurement.in(Rotations) / AlgaeArmConstants.kAlgaeArmReduction);

    }
  }

  public static class CoralArm
  {

    /**
     * Convert {@link Angle} into motor {@link Angle}
     *
     * @param measurement Angle, to convert.
     * @return {@link Angle} equivalent to rotations of the motor.
     */
    public static Angle convertCoralAngleToSensorUnits(Angle measurement)
    {
      return Rotations.of(measurement.in(Rotations) * CoralArmConstants.kCoralArmReduction);
    }

    /**
     * Convert motor rotations {@link Angle} into usable {@link Angle}
     *
     * @param measurement Motor roations
     * @return Usable angle.
     */
    public static Angle convertSensorUnitsToCoralAngle(Angle measurement)
    {
      return Rotations.of(measurement.in(Rotations) / CoralArmConstants.kCoralArmReduction);

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
