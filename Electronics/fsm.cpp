enum State {
    INIT_STATE,
    RETRIEVE_DATA_STATE,
    ERROR_STATE,
    IDLE_STATE,
};

State currentState = INIT_STATE;

void updateStateMachine() {
    switch (currentState) {
        case INIT_STATE:
            if /*(all sensors initialized)*/ {
                currentState = RETRIEVE_DATA_STATE;
            }
            else {
                currentState = ERROR_STATE;
            }
            break;

        case RETRIEVE_DATA_STATE:
            if /*(all data retrieved successfully within a timeframe
            )*/ {
                currentState = IDLE_STATE;
            } else {
                currentState = ERROR_STATE;
            }
            break;

        case ERROR_STATE:
            currentState = IDLE_STATE;
            if /*(given signal)*/ {
                currentState = INIT_STATE;
            }
            break;

        case IDLE_STATE:
            if /*(given signal)*/ {
                currentState = RETRIEVE_DATA_STATE;
            }
            if /*(error occurs)*/ {
                currentState = ERROR_STATE;
            }
            break;
    }
}

void executeStateActions() {
    switch (currentState) {
        case INIT_STATE:
           /*initialize sensors
            initDWM1000()
            initICM20948()
            initICM42670P()
            initVL53L1L()
            initGPS()
            */
            break;

        case RETRIEVE_DATA_STATE:
            //retrieve data from each sensor
            break;

        case ERROR_STATE:
           //log and handle error
            break;

        case IDLE_STATE:
           //sensors in low power mode, sleep mode
            break;
    }
}

void loop() {
    updateStateMachine();
    executeStateActions();
}
