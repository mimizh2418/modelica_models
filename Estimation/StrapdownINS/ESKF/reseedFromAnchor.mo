within Estimation.StrapdownINS.ESKF;

function reseedFromAnchor
  "Apply an admitted re-seed from the current mocap or GPS anchor"
  input State predicted;
  input Boolean reseedNow;
  input Integer anchorPresent;
  input Avionics.MocapSample mocap;
  input Avionics.GpsSample gps;
  input Estimation.StrapdownINS.InitialVariances initialVariances;
  output State working;
  output Boolean reseeded;
  output Integer reseedSource;
protected
  Real reseedPosition[3];
  Real reseedVelocity[3];
algorithm
  working := copyState(predicted);
  reseeded := false;
  reseedSource := SourceNone;
  if reseedNow then
    reseedSource := anchorPresent;
    if anchorPresent == SourceMocap then
      reseedPosition := mocap.positionWorldEnu_m;
      reseedVelocity := working.velocityWorldEnu_m_s;
    else
      reseedPosition := gps.positionWorldEnu_m;
      reseedVelocity := if gps.velocityValid then
        gps.velocityWorldEnu_m_s else working.velocityWorldEnu_m_s;
    end if;
    // Position and velocity, and only their covariance blocks, are replaced;
    // the attitude and bias states and their variances are carried through
    // untouched. The record assembly lives in reseed() so it does not share
    // a common-subexpression temporary with the prediction-side assemblies.
    working := reseed(working, reseedPosition, reseedVelocity,
      initialVariances);
    reseeded := true;
  end if;
  annotation(Inline=false);
end reseedFromAnchor;
