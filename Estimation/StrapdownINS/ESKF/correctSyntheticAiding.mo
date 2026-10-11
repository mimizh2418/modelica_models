within Estimation.StrapdownINS.ESKF;

function correctSyntheticAiding
  "Apply rest and unaided synthetic measurements after physical aiding"
  input State previous;
  input Tuning tuning;
  input Boolean vehicleAtRest;
  input Boolean imuUsable;
  input Boolean reseedNow;
  input Boolean correctionAttempted;
  input Boolean aidingLive;
  input Boolean ladderConfigured;
  input Real rejectionElapsedPrevious_s;
  input Boolean imuQuiet;
  input Real pseudoPositionHoldPrevious_m[3];
  input Boolean correctionAcceptedPrevious;
  input Integer correctionSourcePrevious;
  input Integer correctionOutcomePrevious;
  input Real normalizedInnovationSquaredPrevious;
  output State working;
  output Boolean zeroVelocityCorrectionAccepted;
  output Boolean pseudoPositionCorrectionAccepted;
  output Boolean correctionAccepted;
  output Integer correctionSource;
  output Integer correctionOutcome;
  output Real normalizedInnovationSquared;
  output Real pseudoPositionHoldNext_m[3];
protected
  Boolean pseudoPositionConfigured;
  Boolean zeroVelocityConfigured;
  Integer aidingOutcome;
  Real aidingNis;
algorithm
  working := copyState(previous);
  zeroVelocityCorrectionAccepted := false;
  pseudoPositionCorrectionAccepted := false;
  correctionAccepted := correctionAcceptedPrevious;
  correctionSource := correctionSourcePrevious;
  correctionOutcome := correctionOutcomePrevious;
  normalizedInnovationSquared := normalizedInnovationSquaredPrevious;
  pseudoPositionConfigured := tuning.pseudoPositionVariance_m2 > 0.0
    and tuning.pseudoPositionVariance_m2 < FiniteMagnitudeLimit;
  zeroVelocityConfigured := tuning.zeroVelocityVariance_m2_s2 > 0.0
    and tuning.zeroVelocityVariance_m2_s2 < FiniteMagnitudeLimit;
  if vehicleAtRest and imuUsable and not reseedNow
      and tuning.stationaryVelocityVariance_m2_s2 > 0.0
      and tuning.stationaryVelocityVariance_m2_s2 < FiniteMagnitudeLimit then
    (working, zeroVelocityCorrectionAccepted, aidingOutcome, aidingNis) :=
      correctZeroVelocity(working,
        tuning.stationaryVelocityVariance_m2_s2, 0.0,
        tuning.useSemiDirectBias);
    if not correctionAttempted then
      correctionAccepted := zeroVelocityCorrectionAccepted;
      correctionSource := SourceZeroVelocity;
      correctionOutcome := aidingOutcome;
      normalizedInnovationSquared := aidingNis;
    end if;
  end if;
  if not correctionAttempted and not zeroVelocityCorrectionAccepted
      and (not aidingLive or (ladderConfigured
        and rejectionElapsedPrevious_s
          >= tuning.aidingDivergentWindow_s)) then
    if zeroVelocityConfigured and imuQuiet then
      (working, zeroVelocityCorrectionAccepted, correctionOutcome,
       normalizedInnovationSquared) :=
        correctZeroVelocity(working, tuning.zeroVelocityVariance_m2_s2,
          0.0, tuning.useSemiDirectBias);
      correctionAccepted := zeroVelocityCorrectionAccepted;
      correctionSource := SourceZeroVelocity;
    elseif pseudoPositionConfigured then
      (working, pseudoPositionCorrectionAccepted, correctionOutcome,
       normalizedInnovationSquared) :=
        correctPseudoPosition(working, pseudoPositionHoldPrevious_m,
          tuning.pseudoPositionVariance_m2, 0.0,
          tuning.useSemiDirectBias);
      correctionAccepted := pseudoPositionCorrectionAccepted;
      correctionSource := SourcePseudoPosition;
    end if;
  end if;
  pseudoPositionHoldNext_m := if aidingLive
    then working.positionWorldEnu_m else pseudoPositionHoldPrevious_m;
  annotation(Inline=false);
end correctSyntheticAiding;
