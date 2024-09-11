#pragma once
#include "rotation2d.h"
#include "vector2d.h"
#include <Arduino.h>
/**
 * definition of "In the x,y plane of the robot" :
 * Where y is the vertical axis centered at the shoulder joint and x is parallel to the lateral movement of the arm.
 */

/**
 * Struct that stores the model of the motor.
 * @author David Ly
 */
struct MotorModel
{
public:
    double stallCurrent, stallTorque, maxVoltage, maxRPM, resistance, Kt, Kv, gearRatio, freeCurrent;
    /**
     * Creates a new MotorModel with given parameters.
     * @param a_stallCurrent The stall current of the motor in Amps.
     * @param a_stallTorque The stall torque of the motor in Nm.
     * @param a_maxVoltage The max voltage of the motor in Volts.
     * @param a_maxRPM The max RPM of the motor in RPM.
     * @param ratio The gear ratio of the motor.
     */
    MotorModel(double a_stallCurrent, double a_stallTorque, double a_maxVoltage, double a_maxRPM, double a_freeCurrent, double ratio)
    {
        stallCurrent = a_stallCurrent;
        stallTorque = a_stallTorque / ratio;
        maxVoltage = a_maxVoltage;
        maxRPM = a_maxRPM * ratio;
        freeCurrent = a_freeCurrent;
        gearRatio = ratio;
        resistance = a_maxVoltage / a_stallCurrent;            // based on ohm's law: in Ohms
        Kt = stallTorque / stallCurrent;                       // Torque constant: Nm/A
        Kv = maxRPM / (maxVoltage - freeCurrent * resistance); // Velocity constant: RPM/V
    }
    /**
     * @return Velocity Constant in RPS/V
     */
    double getKvRPS()
    {
        return Kv / 60.0;
    }
};
/**
 * Struct that stores the state of the motor in terms of velocity and torque.
 * @author David Ly
 */
struct MotorState
{
    /**
     * Creates a new MotorState with given parameters.
     * @param vel The velocity of the motor in RPS.
     * @param torque The torque of the motor in Nm.
     */
    MotorState(double vel, double torque) : vel(vel),
                                            torque(torque)
    {
    }
    double vel;
    double torque;
};
/**
 * Struct that stores the pose of the arm in terms of the angles of the joints.
 * @author David Ly
 */
struct ArmPose
{
    /**
     * Creates a new ArmPose with given parameters.
     * @param shoulder The angle of the shoulder joint.
     * @param elbow The angle of the elbow joint.
     * @param wrist The angle of the wrist joint.
     */
    ArmPose(Rotation2d shoulder, Rotation2d elbow, Rotation2d wrist) : shoulder(shoulder),
                                                                       elbow(elbow),
                                                                       wrist(wrist)
    {
    }
    Rotation2d shoulder;
    Rotation2d elbow;
    Rotation2d wrist;
};
/**
 * Defines the properties of a joint.
 * @author David Ly
 */
struct JointProperties
{
public:
    const double mass, length;
    const Vector2d cm_pos;
    /**
     * Creates a new JointProperties with given parameters.
     * x, y position of the center of mass is relative to the center of rotation of the joint.
     * Where x parallel to the longest side of the joint and y parallel to the shortest side of the joint.
     * @param mass The mass of the joint in kg.
     * @param cm_x The x position of the center of mass of the joint in m.
     * @param cm_y The y position of the center of mass of the joint in m.
     * @param length The length of the joint in m.
     */
    JointProperties(double mass, double cm_x, double cm_y, double length) : mass(mass),
                                                                            cm_pos(cm_x, cm_y),
                                                                            length(length)
    {
    }
    /**
     * Creates a new JointProperties with given parameters.
     * Center of mass position is relative to the center of rotation of the joint.
     * Where x parallel to the longest side of the joint and y parallel to the shortest side of the joint.
     * @param mass The mass of the joint in kg.
     * @param cm The position of the center of mass of the joint in m.
     * @param length The length of the joint in m.
     */
    JointProperties(double mass, Vector2d cm, double length) : mass(mass),
                                                               cm_pos(cm.getX(), cm.getY()),
                                                               length(length)
    {
    }
};

#define GRAVITY 9.81

MotorModel shoulder_motor(111, 5.28, 24, 3210, 0.696, 120);
MotorModel elbow_motor(81.9, 2.49, 24, 4300, 0.497, 1);               // todo: find ratio
MotorModel wrist_motor(24.5, GRAVITY * 90.0 / 100.0, 24, 13, 0.7, 1); // todo: find ratio

// Distance of links in m, CM...
JointProperties shoulderProperties(1, 1, 1, 1);
JointProperties elbowProperties(1, 1, 1, 1);
JointProperties wristProperties(1, 1, 1, 1);

#define ESTIMATED_HELDMASS_OFFSET 0.75
/**
 * Calculates the position of the end of the shoulder joint.
 * In the x,y plane of the robot
 * @param buf The vector to store the position of the end of the shoulder joint.
 * @param armPose The pose of the arm.
 */
void getShoulderEndPosition(Vector2d *buf, ArmPose &armPose)
{
    Vector2d shoulderPositionVector(shoulderProperties.length, armPose.shoulder);
    *buf += shoulderPositionVector;
}

/**
 * Calculates the position of the end of the elbow joint.
 * In the x,y plane of the robot
 * @param buf The vector to store the position of the end of the elbow joint.
 * @param armPose The pose of the arm.
 */
void getElbowEndPosition(Vector2d *buf, ArmPose &armPose)
{
    getShoulderEndPosition(buf, armPose);
    Rotation2d temp(armPose.shoulder.getRad() + armPose.elbow.getRad());
    Vector2d elbowPositionVector(elbowProperties.length, temp);
    *buf += elbowPositionVector;
}

/**
 * Calculates the position of the end of the wrist joint. Asumming an offset for the point mass at the end of the joint
 * In the x,y plane of the robot
 * @param buf The vector to store the position of the end of the wrist joint.
 * @param armPose The pose of the arm.
 */
void getWristEndPosition(Vector2d *buf, ArmPose &armPose)
{
    getElbowEndPosition(buf, armPose);
    Rotation2d temp(armPose.shoulder.getRad() + armPose.elbow.getRad() + armPose.wrist.getRad());
    Vector2d wristPositionVector(wristProperties.length * ESTIMATED_HELDMASS_OFFSET, temp);
    *buf += wristPositionVector;
}
/**
 * Calculates the position of the center of mass of the held mass times the heldmass.
 * In the x,y plane of the robot
 * @param buf The vector to store the position of the center of mass of the held mass.
 * @param armPose The pose of the arm.
 * @param heldMass The mass of the held mass in kg.
 */
void calculateCenterMassofHeldMass(Vector2d *buf, ArmPose &armPose, double heldMass)
{
    getWristEndPosition(buf, armPose);
    *buf *= heldMass;
}

/**
 * Calculates the center of mass * mass of the joint.
 * In the x,y plane of the robot.
 * @param buf OUT The vector to store the position of the center of mass * mass of the joint.
 * @param jointPose The pose of the joint.
 * @param properties The properties of the joint.
 */
void computeJointCM(Vector2d *buf, Rotation2d &jointPose, JointProperties &properties)
{
    // function assumes that buf is already filled with the value of the joint position in x,y
    //(joint position is not hte same as joint cm)
    Vector2d orientedJoint(0, 0);
    orientedJoint += properties.cm_pos;           // copy cm position vector
    Vector2d::rotateBy(orientedJoint, jointPose); // orient cm position vector properly
    *buf += orientedJoint;
    *buf *= properties.mass; // cm * mass of the joint
}

/**
 * Calculates the torque on the wrist joint.
 * In the x,y plane of the robot.
 * @param armPose The pose of the arm.
 * @param heldMass The mass of the held mass in kg.
 * @return The torque on the wrist joint in Nm.
 */
double computeWristTorque(ArmPose &armPose, double heldMass)
{
    // Origin to CM of held mass
    Vector2d heldMassPos(0, 0); //
    calculateCenterMassofHeldMass(&heldMassPos, armPose, heldMass);

    // Origin to CM of wrist
    Vector2d wristMass(0, 0);
    getElbowEndPosition(&wristMass, armPose);
    computeJointCM(&wristMass, armPose.wrist, wristProperties);

    double totalMass = wristProperties.mass + heldMass;
    Vector2d centerMass(0, 0);
    centerMass += wristMass;
    centerMass += heldMassPos;
    centerMass *= 1.0 / totalMass;

    Vector2d wristPosition(0, 0);
    getElbowEndPosition(&wristPosition, armPose);

    // Wrist joint to new CM of wrist with held mass
    Vector2d wristJointToCM(0, 0);
    wristJointToCM += centerMass;
    wristJointToCM -= wristPosition;

    // pendulum torque equation: torque = -mg/L sin(theta), theta is measured from the y-axis
    //  used cos, since angle is measured from the x-axis, so it would be sin(90deg-theta) = cos(theta)
    double torque = -GRAVITY * totalMass / wristJointToCM.getLength() * wristJointToCM.getOrientation().getCos();
    return torque;
}

/**
 * Calculates the torque on the elbow joint.
 * In the x,y plane of the robot.
 * @param armPose The pose of the arm.
 * @param heldMass The mass of the held mass in kg.
 * @return The torque on the elbow joint in Nm.
 */
double computeElbowTorque(ArmPose &armPose, double heldMass)
{
    Vector2d heldMassPos(0, 0);
    calculateCenterMassofHeldMass(&heldMassPos, armPose, heldMass);

    Vector2d wristMass(0, 0);
    getElbowEndPosition(&wristMass, armPose);
    computeJointCM(&wristMass, armPose.wrist, wristProperties);

    Vector2d elbowMass(0, 0);
    getShoulderEndPosition(&elbowMass, armPose);
    computeJointCM(&elbowMass, armPose.elbow, elbowProperties);

    double totalMass = elbowProperties.mass + wristProperties.mass + heldMass;
    Vector2d centerMass(0, 0);
    centerMass += wristMass;
    centerMass += elbowMass;
    centerMass += heldMassPos;
    centerMass *= 1.0 / totalMass;

    Vector2d elbowPosition(0, 0);
    getShoulderEndPosition(&elbowPosition, armPose);

    Vector2d elbowJointToCM(0, 0);
    elbowJointToCM += centerMass;
    elbowJointToCM -= elbowPosition;

    // pendulum torque equation: torque = -mg/L sin(theta), theta is measured from the y-axis
    //  used cos, since angle is measured from the x-axis, so it would be sin(90deg-theta) = cos(theta)
    double torque = -GRAVITY * totalMass / elbowJointToCM.getLength() * elbowJointToCM.getOrientation().getCos();
    return torque;
}
/**
 * Calculates the torque on the shoulder joint.
 * In the x,y plane of the robot.
 * @param armPose The pose of the arm.
 * @param heldMass The mass of the held mass in kg.
 * @return The torque on the shoulder joint in Nm.
 */
double computeShoulderTorque(ArmPose &armPose, double heldMass)
{
    Vector2d heldMassPos(0, 0);
    calculateCenterMassofHeldMass(&heldMassPos, armPose, heldMass);

    Vector2d wristMass(0, 0);
    getElbowEndPosition(&wristMass, armPose);
    computeJointCM(&wristMass, armPose.wrist, wristProperties);

    Vector2d elbowMass(0, 0);
    getShoulderEndPosition(&wristMass, armPose);
    computeJointCM(&elbowMass, armPose.elbow, elbowProperties);

    Vector2d shoulderMass(0, 0);
    computeJointCM(&shoulderMass, armPose.shoulder, shoulderProperties);

    double totalMass = shoulderProperties.mass + elbowProperties.mass + wristProperties.mass + heldMass;
    Vector2d centerMass(0, 0);
    centerMass += wristMass;
    centerMass += elbowMass;
    centerMass += shoulderMass;
    centerMass += heldMassPos;
    centerMass *= 1.0 / totalMass;

    Vector2d shoulderPosition(0, 0);
    Vector2d shoulderJointToCM(0, 0);
    shoulderJointToCM += centerMass;
    shoulderJointToCM -= shoulderPosition;

    // pendulum torque equation: torque = -mg/L sin(theta), theta is measured from the y-axis
    //  used cos, since angle is measured from the x-axis, so it would be sin(90deg-theta) = cos(theta)
    double torque = -GRAVITY * totalMass / shoulderJointToCM.getLength() * shoulderJointToCM.getOrientation().getCos();
    return torque;
}

/**
 * Calculates the voltage required to achieve the given state.
 * @param requiredState The state to achieve.
 * @param model The model of the motor.
 */
double findVoltage(MotorState &requiredState, MotorModel &model)
{
    return requiredState.torque / model.Kt * model.resistance + requiredState.vel * model.Kv;
}

/**
 * Calculates the state of the arm to maintain the given pose.
 * @param pose Current arm pose.
 * @param heldMass The mass of the held mass in kg.
 */
void maintainState(ArmPose &pose, double heldMass)
{
    MotorState wristMotorState(0, computeWristTorque(pose, heldMass));
    MotorState elbowMotorState(0, computeElbowTorque(pose, heldMass));
    MotorState shoulderMotorState(0, computeShoulderTorque(pose, heldMass));

    double wristMotorVoltage = findVoltage(wristMotorState, wrist_motor);
    double elbowMotorVoltage = findVoltage(elbowMotorState, elbow_motor);
    double shoulderMotorVoltage = findVoltage(shoulderMotorState, shoulder_motor);
}

JointProperties proto(.2, .07, 0, .1);

double calculateTorque(const Rotation2d pos)
{
    Vector2d cm_pos(0, 0);
    cm_pos += proto.cm_pos;
    Vector2d::rotateBy(cm_pos, pos);
    return -GRAVITY * proto.mass * cm_pos.getLength() * cm_pos.getOrientation().getSin(); // or getCos()?
}

void maintainStateProto(Rotation2d pos, double *output)
{
    double torque = calculateTorque(pos);
    MotorState motorState(0, torque);
    double voltage = findVoltage(motorState, wrist_motor);
    *output = voltage;
}