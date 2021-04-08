
void geofenceCheck() {    //Run every time there's new GPS data available
  //Record current latitude and longitude displacement for averaging

  disY = latitude - fixedLat;
  disX = longitude - fixedLong;

  float distance = sqrt(sq(disY) + sq(disX));

  if (distance >= CUTDOWN_DISTANCE) {
    triggerCutdown();
}
 
//  pastLatitudeDisplacement[avgFramePos] = latitude - lastLatitude;
//  pastLongitudeDisplacement[avgFramePos] = longitude - lastLongitude;
//  avgFramePos++;
//  avgFramePos = avgFramePos % DISPLACEMENT_AVG_FRAME;
//
//  //Determine average latitude and longitude drift rate
//  float latSum, longSum;
//  for(int i = 0; i < DISPLACEMENT_AVG_FRAME; i++){
//    latSum = latSum + pastLatitudeDisplacement[i];
//    longSum = longSum + pastLongitudeDisplacement[i];
//  }
//  float latDriftRate = latSum / (DISPLACEMENT_AVG_FRAME * GPS_CHECK_TIME);
//  float longDriftRate = longSum / (DISPLACEMENT_AVG_FRAME * GPS_CHECK_TIME);
//
//  //Find predicted latitude and longitude of landing position
//  float latPredicted = latDriftRate * (altitude / 1000 / DESCENT_RATE); //Altitude is given by the gps in mm
//  float longPredicted = longDriftRate * (altitude / 1000 / DESCENT_RATE);
//
//  //Find if cutdown is required
//  if( (latPredicted / 10000000) < 30.736 ) { //latitude and longitude are given by the GPS in degrees * 10^7
//    triggerCutdown();
//  } else if( (latPredicted / 10000000) < 32.851 ) {
//    if( (longPredicted / 10000000) > ((latPredicted / 10000000 / 2.2175) - 96.017) ) {
//      triggerCutdown();
//    }
//  } else if( (latPredicted / 10000000) < 35.031 ) {
//    if( (longPredicted / 10000000) > ((latPredicted / 10000000 / .63537) - 132.61) ) {
//      triggerCutdown();
//    }
//  } else {
//    if( (longPredicted / 10000000) > -77.468 ) {
//      triggerCutdown();
//    }
//  }
}
