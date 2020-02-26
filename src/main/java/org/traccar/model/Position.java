/*
 * Copyright 2012 - 2016 Anton Tananaev (anton@traccar.org)
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
package org.traccar.model;

import java.util.Date;

import org.traccar.database.QueryIgnore;

public class Position extends Message {

    public static final String KEY_ORIGINAL = "raw";
    public static final String KEY_INDEX = "index";
    public static final String KEY_HDOP = "hdop";
    public static final String KEY_VDOP = "vdop";
    public static final String KEY_PDOP = "pdop";
    public static final String KEY_SATELLITES = "sat"; // in use
    public static final String KEY_SATELLITES_VISIBLE = "satVisible";
    public static final String KEY_RSSI = "rssi";
    public static final String KEY_GPS = "gps";
    public static final String KEY_ROAMING = "roaming";
    public static final String KEY_EVENT = "event";
    public static final String KEY_ALARM = "alarm";
    public static final String KEY_STATUS = "status";
    public static final String KEY_ODOMETER = "odometer"; // meters
    public static final String KEY_ODOMETER_SERVICE = "serviceOdometer"; // meters
    public static final String KEY_ODOMETER_TRIP = "tripOdometer"; // meters
    public static final String KEY_HOURS = "hours";
    public static final String KEY_STEPS = "steps";
    public static final String KEY_HEART_RATE = "heartRate";
    public static final String KEY_INPUT = "input";
    public static final String KEY_OUTPUT = "output";
    public static final String KEY_IMAGE = "image";
    public static final String KEY_VIDEO = "video";
    public static final String KEY_AUDIO = "audio";

    // The units for the below four KEYs currently vary.
    // The preferred units of measure are specified in the comment for each.
    public static final String KEY_POWER = "power"; // volts
    public static final String KEY_BATTERY = "battery"; // volts
    public static final String KEY_BATTERY_LEVEL = "batteryLevel"; // percentage
    public static final String KEY_FUEL_LEVEL = "fuel"; // liters
    public static final String KEY_FUEL_USED = "fuelUsed"; // liters
    public static final String KEY_FUEL_CONSUMPTION = "fuelConsumption"; // liters/hour
    public static final String KEY_FUEL_TEMPEATURE = "fuelTemperature"; // liters/hour

    public static final String KEY_VERSION_FW = "versionFw";
    public static final String KEY_VERSION_HW = "versionHw";
    public static final String KEY_TYPE = "type";
    public static final String KEY_IGNITION = "ignition";
    public static final String KEY_FLAGS = "flags";
    public static final String KEY_ANTENNA = "antenna";
    public static final String KEY_CHARGE = "charge";
    public static final String KEY_IP = "ip";
    public static final String KEY_ARCHIVE = "archive";
    public static final String KEY_DISTANCE = "distance"; // meters
    public static final String KEY_TOTAL_DISTANCE = "totalDistance"; // meters
    public static final String KEY_RPM = "rpm";
    public static final String KEY_VIN = "vin";
    public static final String KEY_APPROXIMATE = "approximate";
    public static final String KEY_THROTTLE = "throttle";
    public static final String KEY_MOTION = "motion";
    public static final String KEY_ARMED = "armed";
    public static final String KEY_GEOFENCE = "geofence";
    public static final String KEY_ACCELERATION = "acceleration";
    public static final String KEY_DEVICE_TEMP = "deviceTemp"; // celsius
    public static final String KEY_COOLANT_TEMP = "coolantTemp"; // celsius
    public static final String KEY_ENGINE_TEMP = "engineTemp"; // celsius
    public static final String KEY_AIR_TEMP = "airTemp"; // celsius
    public static final String KEY_ENGINE_LOAD = "engineLoad";
    public static final String KEY_OPERATOR = "operator";
    public static final String KEY_COMMAND = "command";
    public static final String KEY_BLOCKED = "blocked";
    public static final String KEY_DOOR = "door";
    public static final String KEY_AXLE_WEIGHT = "axleWeight";
    public static final String KEY_G_SENSOR = "gSensor";
    public static final String KEY_ICCID = "iccid";
    public static final String KEY_PHONE = "phone";

    public static final String KEY_DTCS = "dtcs";
    public static final String KEY_OBD_SPEED = "obdSpeed"; // knots
    public static final String KEY_VEHICLE_SPEED = "vehicleSpeed"; // knots
    public static final String KEY_OBD_ODOMETER = "obdOdometer"; // meters

    public static final String KEY_RESULT = "result";

    public static final String KEY_DRIVER_UNIQUE_ID = "driverUniqueId";
    public static final String KEY_SEAT_BELT_SWITCH = "driverSeatBeltSwitch";
    public static final String KEY_PARKING_STATE = "parkingState";
    public static final String KEY_LIGHT_STATE = "lightState";
    public static final String KEY_CLUTCH_PEDAL = "clutchPedal";
    //region new keys to manager errors
    public static final String KEY_UNKNOWN = "unknown";
    public static final String KEY_ERROR = "error";
    //endregion

    //region new keys for new varids
    public static final String KEY_AC = "A/C";
    public static final String KEY_ACCELERATOR_PEDAL_POSITION = "acceleratorPedalPosition";
    public static final String KEY_ASR_SWITCH = "ASRSwitch";
    public static final String KEY_BATTERY_POTENTIAL = "batteryPotential";
    public static final String KEY_BRAKE_PEDAL_SWITCH = "brakePedalSwitch";
    public static final String KEY_CALCULATED_ENGINE_LOAD_VALUE = "calculatedEngineLoadValue";
    public static final String KEY_CRUISE_CONTROL_ENABLE_SWITCH = "cruiseControlEnableSwitch";
    public static final String KEY_DISTANCE_TRAVELED_SINCE_CODES_CLEARED = "distanceTraveledSinceCodesCleared";
    public static final String KEY_DOOR_DRIVER = "doorDriver";
    public static final String KEY_DOOR_PASSENGER = "doorPassenger";
    public static final String KEY_ENGINE_FUEL_RATE = "engineFuelRate";
    public static final String KEY_ENGINE_INTAKE_MANIFOLD_TEMPERATURE = "engineIntakeManifold1Temperature";
    public static final String KEY_ENGINE_ON_TIME = "engineOnTime";
    public static final String KEY_FUEL_QUANTITY = "fuelQuantity";
    public static final String KEY_HEAD_LIGHT = "headLight";
    public static final String KEY_INTAKE_MANIFOLD_ABSULT_PRESSUR = "intakeManifoldAbsultPressur";
    public static final String KEY_PARKING_BRAKE_SWITCH = "parkingBrakeSwitch";
    public static final String KEY_REAL_GROUND_VEHICLE_SPEED = "realGroundVehicleSpeed";
    public static final String KEY_SEAT_BELT = "seatBelt";
    public static final String KEY_SEAT_BELT_PASSANGER = "seatBeltPassanger";
    //TODO: REMPLAZAR POR OBDODOMETER
    //public static final String KEY_TOTAL_VEHICLE_DISTANCE = "totalVehicleDistance";
    public static final String KEY_TRANSMISSION_SELECTED_GEAR = "transmissionSelectedGear";
    //endregion

    // Start with 1 not 0
    public static final String PREFIX_FUEL_LEVEL = "fuelLevel";
    public static final String PREFIX_TEMP = "temp";
    public static final String PREFIX_ADC = "adc";
    public static final String PREFIX_IO = "io";
    public static final String PREFIX_COUNT = "count";
    public static final String PREFIX_IN = "in";
    public static final String PREFIX_OUT = "out";

    public static final String ALARM_GENERAL = "general";
    public static final String ALARM_SOS = "sos";
    public static final String ALARM_VIBRATION = "vibration";
    public static final String ALARM_MOVEMENT = "movement";
    public static final String ALARM_LOW_SPEED = "lowspeed";
    public static final String ALARM_OVERSPEED = "overspeed";
    public static final String ALARM_FALL_DOWN = "fallDown";
    public static final String ALARM_LOW_POWER = "lowPower";
    public static final String ALARM_LOW_BATTERY = "lowBattery";
    public static final String ALARM_FAULT = "fault";
    public static final String ALARM_POWER_OFF = "powerOff";
    public static final String ALARM_POWER_ON = "powerOn";
    public static final String ALARM_DOOR = "door";
    public static final String ALARM_LOCK = "lock";
    public static final String ALARM_UNLOCK = "unlock";
    public static final String ALARM_GEOFENCE = "geofence";
    public static final String ALARM_GEOFENCE_ENTER = "geofenceEnter";
    public static final String ALARM_GEOFENCE_EXIT = "geofenceExit";
    public static final String ALARM_GPS_ANTENNA_CUT = "gpsAntennaCut";
    public static final String ALARM_ACCIDENT = "accident";
    public static final String ALARM_TOW = "tow";
    public static final String ALARM_IDLE = "idle";
    public static final String ALARM_HIGH_RPM = "highRpm";
    public static final String ALARM_ACCELERATION = "hardAcceleration";
    public static final String ALARM_BRAKING = "hardBraking";
    public static final String ALARM_CORNERING = "hardCornering";
    public static final String ALARM_LANE_CHANGE = "laneChange";
    public static final String ALARM_FATIGUE_DRIVING = "fatigueDriving";
    public static final String ALARM_POWER_CUT = "powerCut";
    public static final String ALARM_POWER_RESTORED = "powerRestored";
    public static final String ALARM_JAMMING = "jamming";
    public static final String ALARM_TEMPERATURE = "temperature";
    public static final String ALARM_PARKING = "parking";
    public static final String ALARM_SHOCK = "shock";
    public static final String ALARM_BONNET = "bonnet";
    public static final String ALARM_FOOT_BRAKE = "footBrake";
    public static final String ALARM_FUEL_LEAK = "fuelLeak";
    public static final String ALARM_TAMPERING = "tampering";
    public static final String ALARM_REMOVING = "removing";

    //region Alarms from type 11 MARX
    public static final String ALARM_EMERGENCY = "emergency";
    public static final String ALARM_ENGINE_ACTIVATED = "engineActivated";
    public static final String ALARM_ROBBERY = "robbery";
    public static final String ALARM_KEYPAD_LOCKED = "keypadLoked";
    public static final String ALARM_CRASH = "crash";
    public static final String ALARM_FUNCTION_BUTTON_PRESSED = "functionButtonPressed";
    public static final String ALARM_VOICE_CALL = "voiceCall";
    public static final String ALARM_COASTING_DETECTION = "coastingDetection";
    public static final String ALARM_VIOLATION_1_FREQUENCY = "violation1Frequency";
    public static final String ALARM_VIOLATION_2_FREQUENCY = "violation2Frequency";
    public static final String ALARM_SPEED_IGNITION_OFF = "speedIgnitionOff";
    public static final String ALARM_REPLY_COMMAND = "replyCommand";
    public static final String ALARM_IP_CHANGED = "ipChanged";
    public static final String ALARM_GPS_NAVIGATION_START = "gpsNavigationStart";
    public static final String ALARM_OVER_SPEED_START = "overSpeedStart";
    public static final String ALARM_IDLE_SPEED_START = "idleSpeedStart";
    public static final String ALARM_GPS_FACTORY_RESET = "gpsFactoryReset";
    public static final String ALARM_GPS_NAVIGATION_END = "gpsNavigationEnd";
    public static final String ALARM_OVER_SPEED_END = "overSpeedEnd";
    public static final String ALARM_IDLE_SPEED_END = "idleSpeedEnd";
    public static final String ALARM_DIVER_AUTHENTICATION_UPDATE = "driverAuthenticationUpdate";
    public static final String ALARM_DRIVING_WITHOUT_AUTHENTICATION = "drivingWithoutAuthentication";
    public static final String ALARM_DOOR_CLOSE = "doorClose";
    public static final String ALARM_SHOCK_UNLOCK2_INACTIVE = "shockUnlock2Inactive";
    public static final String ALARM_CFE_INPUT6_INACTIVE = "cfeInput6Inactive";
    public static final String ALARM_VOLUME_SENSOR_INACTIVE_EVENT = "volumeSensorInactiveEvent";
    public static final String ALARM_DRIVING_STOP = "drivingStop";
    public static final String ALARM_DISTRESS_BUTTON_INACTIVE = "distressButtonInactive";
    public static final String ALARM_UNLOCK_INPUT_INACTIVE = "unlockInputInactive";
    public static final String ALARM_CFE_INPUT1_INACTIVE = "cfeInput1Inactive";
    public static final String ALARM_LOCK_INPUT_INACTIVE = "lockInputInactive";
    public static final String ALARM_CFE_INPUT2_INACTIVE = "cfeInput2Inactive";
    public static final String ALARM_CFE_INPUT3_INACTIVE = "cfeInput3Inactive";
    public static final String ALARM_CFE_INPUT4_INACTIVE = "cfeInput4Inactive";
    public static final String ALARM_CFE_INPUT5_INACTIVE = "cfeInput5Inactive";
    public static final String ALARM_IGNITION_INPUT_INACTIVE = "ignitionInputInactive";
    public static final String ALARM_DOOR_OPEN = "doorOpen";
    public static final String ALARM_SHOCK_UNLOCK2_ACTIVE = "shockUnlock2Active";
    public static final String ALARM_CFE_INPUT6_ACTIVE = "cfeInput6Active";
    public static final String ALARM_VOLUME_SENSOR_ACTIVE = "volumeSensorActive";
    public static final String ALARM_DRIVING_START = "drivingStart";
    public static final String ALARM_UNLOCK_INPUT_ACTIVE = "unlockInputActive";
    public static final String ALARM_CFE_INPUT1_ACTIVE = "cfeInput1Active";
    public static final String ALARM_LOCK_INPUT_ACTIVE = "lockInputActive";
    public static final String ALARM_CFE_INPUT2_ACTIVE = "cfeInput2Active";
    public static final String ALARM_CFE_INPUT3_ACTIVE = "cfeInput3Active";
    public static final String ALARM_CFE_INPUT4_ACTIVE = "cfeInput4Active";
    public static final String ALARM_CFE_INPUT5_ACTIVE = "cfeInput5Active";
    public static final String ALARM_IGNITION_INPUT_ACTIVE = "ignitionInputActive";
    public static final String ALARM_BACKUP_BATTERY_DISCONNECTED = "backupBatteryDisconnected";
    public static final String ALARM_BACKUP_BATTERY_LOW_LEVEL = "backupBatteryLowLevel";
    public static final String ALARM_HALT_MOVEMENT_END = "haltMovementEnd";
    public static final String ALARM_GO_MOVEMENT_START = "goMovementStart";
    public static final String ALARM_MAIN_POWER_CONNECTED = "mainPowerConnected";
    public static final String ALARM_MAIN_POWER_HIGH_LEVEL = "mainPowerHighLevel";
    public static final String ALARM_BACKUP_BATTERY_CONNECTED = "backupBatteryConnected";
    public static final String ALARM_BACKUP_BATTERY_HIGH_LEVEL = "backupBatteryHighLevel";
    public static final String ALARM_MESSAGE_FROM_KEYBOARD = "messageFromKeyboard";
    public static final String ALARM_SATELLITE_COMMUNICATION = "satelliteCommunication";
    public static final String ALARM_HARSH_BRAKING_SENSOR = "harshBrakingSensor";
    public static final String ALARM_SUDDEN_COURSE_CHANGE_SENSOR = "suddenCourseChangeSensor";
    public static final String ALARM_HARSH_ACCELERATION_SENSOR = "harshAccelerationSensor";
    public static final String ALARM_MAIN_POWER_LOW = "mainPowerLow";
    public static final String ALARM_TAMPER_ACTIVE = "tamperActive";
    public static final String ALARM_TAMPER_INACTIVE = "tamperInactive";
    public static final String ALARM_CFE_VENT = "cfeVent";
    public static final String ALARM_UNLOCK_INPUT = "unlockInput";
    public static final String ALARM_ORIENTATION_CHANGE = "orientationChange";
    public static final String ALARM_CAN_GPS_SPEED_CALIBRATION = "canGpsSpeedCalibration";
    public static final String ALARM_NO_MODEM_ZONE_ENTRY = "noModemZoneEntry";
    public static final String ALARM_GEO_HOTSPOT_VIOLATION = "geoHotspotViolation";
    public static final String ALARM_FREQUENCY_MEASUREMENT_THRESHOLD_VIOLATION = "frequencyMeasurementThresholdViolation";
    public static final String ALARM_ANALOG_MEASUREMENT_THRESHOLD_VIOLATION = "analogMeasurementThresholdViolation";
    public static final String ALARM_TRAILER_CONNECTION_STATUS = "trailerConnectionStatus";
    public static final String ALARM_AHR = "ahr";
    public static final String ALARM_PSP = "psp";
    public static final String ALARM_WAKE_UP_EVENT = "wakeUpEvent";
    public static final String ALARM_PRE_HIBERNATION_EVENT = "preHibernationEvent";
    public static final String ALARM_VECTOR_CHANGE_CURVE_SMOOTHING = "vectorChangeCurveSmoothing";
    public static final String ALARM_GARMIN_CONNECTION_STATUS = "garminConnectionStatus";
    public static final String ALARM_RADIO_OFF_MODE = "radioOffMode";
    public static final String ALARM_HEADER_ERROR = "headerError";
    public static final String ALARM_GEO_FENCE_OVER_SPEED_START = "geoFenceOverSpeedStart";
    public static final String ALARM_GEO_FENCE_OVER_SPEED_END = "geoFenceOverSpeedEnd";
    public static final String ALARM_POINTERCEPT_BEACON_START_STOP = "pointerceptBeaconStartStop";
    public static final String ALARM_POINTERCEPT_CPIN_ERROR = "pointerceptCpinError";
    public static final String ALARM_OTA = "ota";
    public static final String ALARM_POINTERCEPT_PERIODIC_BEACON_TRANSMISSION = "pointerceptPeriodicBeaconTransmission";
    public static final String ALARM_FINISH_MODE = "finishMode";
    public static final String ALARM_COM_LOCATION_GLANCING = "comLocationGlancing";
    public static final String ALARM_VIOLATION_KEEP_IN_FENCE = "violationKeepInFence";
    public static final String ALARM_VIOLATION_KEEP_OUT_FENCE = "violationKeepOutFence";
    public static final String ALARM_VIOLATION_WAYPOINT = "violationWaypoint";
    //endregion

    public Position() {
    }

    public Position(String protocol) {
        this.protocol = protocol;
        this.serverTime = new Date();
    }

    private String protocol;

    public String getProtocol() {
        return protocol;
    }

    public void setProtocol(String protocol) {
        this.protocol = protocol;
    }

    private Date serverTime = new Date();

    public Date getServerTime() {
        return serverTime;
    }

    public void setServerTime(Date serverTime) {
        this.serverTime = serverTime;
    }

    private Date deviceTime;

    public Date getDeviceTime() {
        return deviceTime;
    }

    public void setDeviceTime(Date deviceTime) {
        this.deviceTime = deviceTime;
    }

    private Date fixTime;

    public Date getFixTime() {
        return fixTime;
    }

    public void setFixTime(Date fixTime) {
        this.fixTime = fixTime;
    }

    public void setTime(Date time) {
        setDeviceTime(time);
        setFixTime(time);
    }

    private boolean outdated;

    @QueryIgnore
    public boolean getOutdated() {
        return outdated;
    }

    public void setOutdated(boolean outdated) {
        this.outdated = outdated;
    }

    private boolean valid;

    public boolean getValid() {
        return valid;
    }

    public void setValid(boolean valid) {
        this.valid = valid;
    }

    private double latitude;

    public double getLatitude() {
        return latitude;
    }

    public void setLatitude(double latitude) {
        this.latitude = latitude;
    }

    private double longitude;

    public double getLongitude() {
        return longitude;
    }

    public void setLongitude(double longitude) {
        this.longitude = longitude;
    }

    private double altitude; // value in meters

    public double getAltitude() {
        return altitude;
    }

    public void setAltitude(double altitude) {
        this.altitude = altitude;
    }

    private double speed; // value in knots

    public double getSpeed() {
        return speed;
    }

    public void setSpeed(double speed) {
        this.speed = speed;
    }

    private double course;

    public double getCourse() {
        return course;
    }

    public void setCourse(double course) {
        this.course = course;
    }

    private String address;

    public String getAddress() {
        return address;
    }

    public void setAddress(String address) {
        this.address = address;
    }

    private double accuracy;

    public double getAccuracy() {
        return accuracy;
    }

    public void setAccuracy(double accuracy) {
        this.accuracy = accuracy;
    }

    private Network network;

    public Network getNetwork() {
        return network;
    }

    public void setNetwork(Network network) {
        this.network = network;
    }

    @Override
    @QueryIgnore
    public String getType() {
        return super.getType();
    }

}
