within Vehicles.Rdd2;

// SPDX-License-Identifier: Apache-2.0

block GuidanceController
  "Pilot, attitude, and position guidance task"
  import Interfaces = Vehicles.Rdd2.ControllerInterfaces;

  parameter Real samplePeriod(unit = "s") = 0.005;
  parameter Real maximumCollectiveThrust_N = 41.3749272
    "Four RDD2 rotors at maximum speed";
  parameter Real inertia[3](each unit = "kg.m2") = {
    0.02166666666666667, 0.02166666666666667, 0.04000000000000001}
    "Reference-model inertia; composed stacks use the rate allocator's inertia";

  input Integer mode(min = 0, max = 3)
    "0=acro, 1=attitude, 2=mission, 3=pilot position";
  input Boolean armed;
  Interfaces.PilotInput pilot;
  Interfaces.GuidanceStateInput navigation;
  Interfaces.TrajectoryReferenceInput reference;

  Interfaces.PilotCommands pilotCommands;
  Interfaces.RateCommand rateCommand;
  output Real angularVelocitySetpointFlu_rad_s[3];

protected
  CommandMapping pilotMapping(samplePeriod = samplePeriod);
  LogLinearController positionGuidance(samplePeriod = samplePeriod);
  Real navigationEulerB321_rad[3] "{yaw, pitch, roll}";
  Real attitudeReferenceQuaternion[4];
  Real attitudeRateSetpointFlu_rad_s[3];
  Real rateCorrectionFlu_rad_s[3];
  Planning.Bezier.MultirotorTrajectory trajectorySample;
  Planning.Bezier.FlatReference nominalReference;
  Real nominalToActualBody[3, 3];

equation
  pilotMapping.pilot.stick = pilot.stick;
  pilotMapping.pilot.throttle = pilot.throttle;
  connect(pilotMapping.commands, pilotCommands);

  navigationEulerB321_rad = LieGroups.SO3.EulerB321.from_Quat(
    navigation.quaternionWorldBody);
  attitudeReferenceQuaternion = LieGroups.SO3.EulerB321.to_Quat({
    navigationEulerB321_rad[1],
    pilotCommands.attitudeTiltDesired_rad[2],
    pilotCommands.attitudeTiltDesired_rad[1]});
  attitudeRateSetpointFlu_rad_s =
    Control.Multirotor.LogLinear.attitudeControl(
      positionGuidance.attitudeGain,
      navigation.quaternionWorldBody,
      attitudeReferenceQuaternion);

  positionGuidance.positionWorld = navigation.positionWorldEnu_m;
  positionGuidance.velocityWorld = navigation.velocityWorldEnu_m_s;
  positionGuidance.quaternionWorldBody = navigation.quaternionWorldBody;
  positionGuidance.positionReferenceWorld = reference.positionWorld_m;
  positionGuidance.velocityReferenceWorld = reference.velocityWorld_m_s;
  positionGuidance.accelerationReferenceWorld =
    reference.accelerationWorld_m_s2;
  positionGuidance.headingQuaternionReference =
    LieGroups.SO3.EulerB321.to_Quat({reference.yaw_rad, 0.0, 0.0});
  // Mission and pilot position guidance are the same cascade on different
  // reference sources, so both hold the integral and both take the outer-loop
  // thrust and rate command.
  positionGuidance.resetIntegral = not armed or mode < 2;

  // Reconstruct nominal rotation derivatives from the same flat outputs that
  // already supply acceleration feedforward. Position feedback continues to
  // choose the corrected thrust and attitude; nominal rates retain their own
  // reference frame, which can differ from that corrected attitude.
  trajectorySample.position = reference.positionWorld_m;
  trajectorySample.velocity = reference.velocityWorld_m_s;
  trajectorySample.acceleration =
    if armed and mode >= 2 then reference.accelerationWorld_m_s2 else zeros(3);
  trajectorySample.jerk =
    if armed and mode >= 2 then reference.jerkWorld_m_s3 else zeros(3);
  trajectorySample.snap =
    if armed and mode >= 2 then reference.snapWorld_m_s4 else zeros(3);
  trajectorySample.yaw = reference.yaw_rad;
  trajectorySample.yawRate =
    if armed and mode >= 2 then reference.yawRate_rad_s else 0.0;
  trajectorySample.yawAcceleration =
    if armed and mode >= 2 then reference.yawAcceleration_rad_s2 else 0.0;
  nominalReference = Planning.Bezier.flatReference(
    trajectorySample, positionGuidance.mass, positionGuidance.gravity,
    diagonal(inertia));
  nominalToActualBody =
    transpose(LieGroups.SO3.Quat.to_DCM(navigation.quaternionWorldBody))
      * nominalReference.bodyToWorld;
  rateCommand.angularVelocityFeedforwardFlu_rad_s =
    if armed and mode >= 2 then
      nominalToActualBody * nominalReference.angularVelocityBody else zeros(3);
  rateCommand.angularAccelerationFeedforwardFlu_rad_s2 =
    if armed and mode >= 2 then
      nominalToActualBody * nominalReference.angularAccelerationBody else zeros(3);

  rateCommand.thrust_N = if mode >= 2 then
      positionGuidance.thrust
    else
      maximumCollectiveThrust_N * pilotCommands.throttleInput ^ 2;
  angularVelocitySetpointFlu_rad_s = if mode >= 2 then
      positionGuidance.angularVelocitySetpoint
        + rateCommand.angularVelocityFeedforwardFlu_rad_s
    elseif mode == 1 then
      {1.0, 1.0, 0.0} .* attitudeRateSetpointFlu_rad_s
      + {0.0, 0.0, 1.0} .* pilotCommands.acroRateDesired_rad_s
    else
      pilotCommands.acroRateDesired_rad_s;
  rateCorrectionFlu_rad_s = if mode >= 2 then
      positionGuidance.angularVelocityCorrection
    else
      zeros(3);
  rateCommand.angularVelocityCommandFlu_rad_s =
    angularVelocitySetpointFlu_rad_s + rateCorrectionFlu_rad_s;

  annotation(Documentation(info = "<html>
    <p>This block is one deployable RTOS task and one eFMU. It turns pilot,
    navigation, and trajectory messages into the compact rate-command message
    consumed by the fast control task.</p>
    <p>Mission and pilot-position modes add flatness-derived body-rate
    feedforward to the attitude correction. Rates and accelerations are rotated
    from the nominal reference body to the actual body FLU frame. The rate task
    applies the rotating-frame acceleration transport term. Other modes and
    disarmed operation publish zero trajectory feedforward.</p>
    <p>Vector signals stay vector equations throughout the public model.</p>
  </html>"));
end GuidanceController;
