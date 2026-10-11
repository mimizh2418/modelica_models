within Estimation.StrapdownINS.ESKF;

function correctIndependentAiding
  "Fuse flow, height and heading in order while retaining outcome priority"
  input State previous;
  input Tuning tuning;
  input Avionics.OpticalFlowSample opticalFlow;
  input Avionics.BarometerSample barometer;
  input Avionics.MagnetometerSample magnetometer;
  input Real gravityWorldEnu_m_s2[3];
  input Boolean reseedNow;
  input Boolean opticalFlowAttempted;
  input Boolean absoluteAidingAttempted;
  input Boolean barometerNew;
  input Boolean magnetometerNew;
  input Real imuTimestampHeldNext_s;
  input Real imuAngularVelocityHeldNext_rad_s[3];
  input Real imuSpecificForceHeldNext_m_s2[3];
  input Boolean correctionAcceptedPrevious;
  input Integer correctionSourcePrevious;
  input Integer correctionOutcomePrevious;
  input Real normalizedInnovationSquaredPrevious;
  input Real opticalFlowTimestampConsumedPrevious_s;
  input Real barometerTimestampConsumedPrevious_s;
  input Real magnetometerTimestampConsumedPrevious_s;
  output State working;
  output Boolean opticalFlowCorrectionAccepted;
  output Boolean barometerCorrectionAccepted;
  output Boolean magnetometerCorrectionAccepted;
  output Boolean correctionAccepted;
  output Integer correctionSource;
  output Integer correctionOutcome;
  output Real normalizedInnovationSquared;
  output Real opticalFlowTimestampConsumedNext_s;
  output Real barometerTimestampConsumedNext_s;
  output Real magnetometerTimestampConsumedNext_s;
protected
  Integer aidingOutcome;
  Real aidingNis;
algorithm
  working := copyState(previous);
  opticalFlowCorrectionAccepted := false;
  barometerCorrectionAccepted := false;
  magnetometerCorrectionAccepted := false;
  correctionAccepted := correctionAcceptedPrevious;
  correctionSource := correctionSourcePrevious;
  correctionOutcome := correctionOutcomePrevious;
  normalizedInnovationSquared := normalizedInnovationSquaredPrevious;
  opticalFlowTimestampConsumedNext_s := opticalFlowTimestampConsumedPrevious_s;
  barometerTimestampConsumedNext_s := barometerTimestampConsumedPrevious_s;
  magnetometerTimestampConsumedNext_s := magnetometerTimestampConsumedPrevious_s;
  if opticalFlowAttempted then
    (working, opticalFlowCorrectionAccepted, aidingOutcome, aidingNis) :=
      correctOpticalFlow(working, opticalFlow, tuning.innovationGate,
        imuTimestampHeldNext_s - opticalFlow.timestamp_s,
        imuAngularVelocityHeldNext_rad_s,
        imuSpecificForceHeldNext_m_s2, gravityWorldEnu_m_s2,
        tuning.maximumAidingDelay_s,
        tuning.minimumOpticalFlowQuality,
        tuning.minimumOpticalFlowGroundDistance_m,
        tuning.useSemiDirectBias);
    if not absoluteAidingAttempted then
      correctionAccepted := opticalFlowCorrectionAccepted;
      correctionSource := SourceOpticalFlow;
      correctionOutcome := aidingOutcome;
      normalizedInnovationSquared := aidingNis;
    end if;
    opticalFlowTimestampConsumedNext_s := opticalFlow.timestamp_s;
  end if;
  if not reseedNow and barometerNew then
    (working, barometerCorrectionAccepted, aidingOutcome, aidingNis) :=
      correctBarometer(working, barometer,
        tuning.barometerBias_m, tuning.barometerBiasVariance_m2,
        tuning.innovationGate,
        imuTimestampHeldNext_s - barometer.timestamp_s,
        imuAngularVelocityHeldNext_rad_s,
        imuSpecificForceHeldNext_m_s2, gravityWorldEnu_m_s2,
        tuning.maximumAidingDelay_s,
        tuning.useSemiDirectBias, tuning.useBarometerBiasConsider,
        tuning.barometerBiasProcessNoise_m2_s);
    if not absoluteAidingAttempted and not opticalFlowAttempted then
      correctionAccepted := barometerCorrectionAccepted;
      correctionSource := SourceBarometer;
      correctionOutcome := aidingOutcome;
      normalizedInnovationSquared := aidingNis;
    end if;
    barometerTimestampConsumedNext_s := barometer.timestamp_s;
  end if;
  if not reseedNow and magnetometerNew then
    (working, magnetometerCorrectionAccepted, aidingOutcome, aidingNis) :=
      correctMagnetometer(working, magnetometer,
        tuning.localMagneticFieldWorldEnu_T, tuning.innovationGate,
        imuTimestampHeldNext_s - magnetometer.timestamp_s,
        imuAngularVelocityHeldNext_rad_s,
        imuSpecificForceHeldNext_m_s2, gravityWorldEnu_m_s2,
        tuning.maximumAidingDelay_s, tuning.useEquivariantMagnetometer,
        tuning.useSemiDirectBias);
    if not absoluteAidingAttempted and not opticalFlowAttempted
        and not barometerNew then
      correctionAccepted := magnetometerCorrectionAccepted;
      correctionSource := SourceMagnetometer;
      correctionOutcome := aidingOutcome;
      normalizedInnovationSquared := aidingNis;
    end if;
    magnetometerTimestampConsumedNext_s := magnetometer.timestamp_s;
  end if;
  annotation(Inline=false);
end correctIndependentAiding;
