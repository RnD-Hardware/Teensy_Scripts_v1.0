unsigned long estop_FB_counter = millis();
void eStop_freeMotor() {

  if ((estop_FB == 0) && (Estop_FB_flag == false) && ((millis() - estop_FB_counter) > 200)) {
    //    NUC_state = 5;  // for smooth breaking and start accelerating
    quick_stop_response(6); // deaccelarate break
    /* stop */
    estop_FB_to_NUC = 1;
    ErrorCode = 121;
    if ((millis() - estop_FB_counter) > 1000) {
      /* stop */
      stop_motor();
      //      clear_encoder();
      //      NUC_state = 5;  // for smooth breaking and start accelerating
      Estop_FB_flag = true;
      estop_FB_counter = millis();
    }
  }
  else if ((estop_FB == 1) && Estop_FB_flag) {
    /* enable motor */
    en_motor();
    estop_FB_to_NUC = 0;
    Estop_FB_flag = false;
    estop_FB_counter = millis();
  }

  else if (estop_FB == 1) {
    estop_FB_counter = millis();
    estop_FB_to_NUC = 0;
  }
}
