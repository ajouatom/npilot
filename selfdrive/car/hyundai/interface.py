#!/usr/bin/env python3
from cereal import car
from panda import Panda
from common.conversions import Conversions as CV
from selfdrive.car.hyundai.values import CAR, DBC, HDA2_CAR, EV_CAR, HYBRID_CAR, LEGACY_SAFETY_MODE_CAR, Buttons, CarControllerParams
from selfdrive.car.hyundai.radar_interface import RADAR_START_ADDR
from selfdrive.car import STD_CARGO_KG, create_button_enable_events, create_button_event, scale_rot_inertia, scale_tire_stiffness, gen_empty_fingerprint, get_safety_config
from selfdrive.car.interfaces import CarInterfaceBase
from selfdrive.car.disable_ecu import disable_ecu
from common.params import Params

ButtonType = car.CarState.ButtonEvent.Type
EventName = car.CarEvent.EventName
ENABLE_BUTTONS = (Buttons.RES_ACCEL, Buttons.SET_DECEL, Buttons.CANCEL)
BUTTONS_DICT = {Buttons.RES_ACCEL: ButtonType.accelCruise, Buttons.SET_DECEL: ButtonType.decelCruise,
                Buttons.GAP_DIST: ButtonType.gapAdjustCruise, Buttons.CANCEL: ButtonType.cancel}

def torque_tune(tune, max_lat_accel=2.5, friction=0.01, kd=0.0, steering_angle_deadzone_deg=0.0):
  tune.init('torque')
  tune.torque.useSteeringAngle = True
  tune.torque.kp = 1.0 / max_lat_accel
  tune.torque.kf = 1.0 / max_lat_accel
  tune.torque.ki = 0.1 / max_lat_accel
  tune.torque.friction = friction
  tune.torque.steeringAngleDeadzoneDeg = steering_angle_deadzone_deg
  tune.torque.kd = kd

class CarInterface(CarInterfaceBase):
  def __init__(self, CP, CarController, CarState):
    super().__init__(CP, CarController, CarState)

  @staticmethod
  def get_pid_accel_limits(CP, current_speed, cruise_speed):

    v_current_kph = current_speed * CV.MS_TO_KPH

    gas_max_bp = [10., 20., 50., 70., 130., 150.]
    gas_max_v = [1.4, 1.2, 0.63, 0.44, 0.15, 0.1]

    return CarControllerParams.ACCEL_MIN, interp(v_current_kph, gas_max_bp, gas_max_v)

  @staticmethod
  def get_params(candidate, fingerprint=gen_empty_fingerprint(), car_fw=[], disable_radar=False):  # pylint: disable=dangerous-default-value
    ret = CarInterfaceBase.get_std_params(candidate, fingerprint)

    ret.carName = "hyundai"
    ret.safetyConfigs = [get_safety_config(car.CarParams.SafetyModel.hyundai, 0)]
    ret.radarOffCan = RADAR_START_ADDR not in fingerprint[1] or DBC[ret.carFingerprint]["radar"] is None

    # WARNING: disabling radar also disables AEB (and we show the same warning on the instrument cluster as if you manually disabled AEB)
    ret.openpilotLongitudinalControl = disable_radar and (candidate not in LEGACY_SAFETY_MODE_CAR)

    ret.pcmCruise = not ret.openpilotLongitudinalControl

    # These cars have been put into dashcam only due to both a lack of users and test coverage.
    # These cars likely still work fine. Once a user confirms each car works and a test route is
    # added to selfdrive/car/tests/routes.py, we can remove it from this list.
    ret.dashcamOnly = candidate in {CAR.KIA_OPTIMA_H, CAR.ELANTRA_GT_I30}

    ret.steerActuatorDelay = 0.1  # Default delay
    ret.steerLimitTimer = 0.4
    tire_stiffness_factor = 1.

    ret.stoppingControl = True
    ret.vEgoStopping = 1.0

    ret.longitudinalTuning.kpV = [0.1]
    ret.longitudinalTuning.kiV = [0.0]
    ret.stopAccel = 0.0

    ret.longitudinalActuatorDelayUpperBound = 1.0  # s
    
    ret.maxSteeringAngleDeg = 1000.

    ret.steerFaultMaxAngle = 85
    ret.steerFaultMaxFrames = 90

    ret.disableLateralLiveTuning = False

    # lateral
    lateral_control = Params().get("LateralControl", encoding='utf-8')
    if lateral_control == 'INDI':
      ret.lateralTuning.init('indi')
      ret.lateralTuning.indi.innerLoopGainBP = [0.]
      ret.lateralTuning.indi.innerLoopGainV = [3.3]
      ret.lateralTuning.indi.outerLoopGainBP = [0.]
      ret.lateralTuning.indi.outerLoopGainV = [2.8]
      ret.lateralTuning.indi.timeConstantBP = [0.]
      ret.lateralTuning.indi.timeConstantV = [1.4]
      ret.lateralTuning.indi.actuatorEffectivenessBP = [0.]
      ret.lateralTuning.indi.actuatorEffectivenessV = [1.8]
    else:
      torque_tune(ret.lateralTuning, 2.5, 0.01)

    ret.steerRatio = 16.5
    ret.steerActuatorDelay = 0.2

    ret.steerLimitTimer = 2.5

    # longitudinal
    ret.longitudinalTuning.kpBP = [0., 5.*CV.KPH_TO_MS, 10.*CV.KPH_TO_MS, 30.*CV.KPH_TO_MS, 130.*CV.KPH_TO_MS]
    ret.longitudinalTuning.kpV = [1.2, 1.0, 0.93, 0.88, 0.5]
    ret.longitudinalTuning.kiBP = [0., 130. * CV.KPH_TO_MS]
    ret.longitudinalTuning.kiV = [0.1, 0.05]
    ret.longitudinalActuatorDelayLowerBound = 0.3
    ret.longitudinalActuatorDelayUpperBound = 0.3

    ret.stopAccel = -2.0
    ret.stoppingDecelRate = 0.4  # brake_travel/s while trying to stop
    ret.vEgoStopping = 0.5
    ret.vEgoStarting = 0.5

    # genesis
    if candidate == CAR.GENESIS:
      ret.mass = 1900. + STD_CARGO_KG
      ret.wheelbase = 3.01
      ret.centerToFront = ret.wheelbase * 0.4
    elif candidate == CAR.GENESIS_G70:
      ret.mass = 1640. + STD_CARGO_KG
      ret.wheelbase = 2.84
      ret.centerToFront = ret.wheelbase * 0.4
    elif candidate == CAR.GENESIS_G80:
      ret.mass = 1855. + STD_CARGO_KG
      ret.wheelbase = 3.01
      ret.centerToFront = ret.wheelbase * 0.4
    elif candidate == CAR.GENESIS_EQ900:
      ret.mass = 2200
      ret.wheelbase = 3.15
      ret.centerToFront = ret.wheelbase * 0.4

      # thanks to 파파
      ret.steerRatio = 16.0
      ret.steerActuatorDelay = 0.075

      if ret.lateralTuning.which() == 'torque':
        torque_tune(ret.lateralTuning, 2.5, 0.01)

    elif candidate == CAR.GENESIS_EQ900_L:
      ret.mass = 2290
      ret.wheelbase = 3.45
      ret.centerToFront = ret.wheelbase * 0.4
    elif candidate == CAR.GENESIS_G90:
      ret.mass = 2150
      ret.wheelbase = 3.16
      ret.centerToFront = ret.wheelbase * 0.4
    # hyundai
    elif candidate in [CAR.SANTA_FE]:
      ret.mass = 1694 + STD_CARGO_KG
      ret.wheelbase = 2.766
      ret.centerToFront = ret.wheelbase * 0.4
    elif candidate in [CAR.SANTA_FE_2022, CAR.SANTA_FE_HEV_2022]:
      ret.mass = 1750 + STD_CARGO_KG
      ret.wheelbase = 2.766
      ret.centerToFront = ret.wheelbase * 0.4
    elif candidate in [CAR.SONATA, CAR.SONATA_HEV, CAR.SONATA21_HEV]:
      ret.mass = 1513. + STD_CARGO_KG
      ret.wheelbase = 2.84
      ret.centerToFront = ret.wheelbase * 0.4
      tire_stiffness_factor = 0.65
    elif candidate in [CAR.SONATA19, CAR.SONATA19_HEV]:
      ret.mass = 4497. * CV.LB_TO_KG
      ret.wheelbase = 2.804
      ret.centerToFront = ret.wheelbase * 0.4
    elif candidate == CAR.SONATA_LF_TURBO:
      ret.mass = 1590. + STD_CARGO_KG
      ret.wheelbase = 2.805
      tire_stiffness_factor = 0.65
      ret.centerToFront = ret.wheelbase * 0.4
    elif candidate == CAR.PALISADE:
      ret.mass = 1999. + STD_CARGO_KG
      ret.wheelbase = 2.90
      ret.centerToFront = ret.wheelbase * 0.4

      # thanks to 지구별(alexhys)
      ret.steerRatio = 16.0
      ret.steerActuatorDelay = 0.075

      if ret.lateralTuning.which() == 'torque':
        torque_tune(ret.lateralTuning, 2.3, 0.01)

    elif candidate in [CAR.ELANTRA, CAR.ELANTRA_GT_I30]:
      ret.mass = 1275. + STD_CARGO_KG
      ret.wheelbase = 2.7
      tire_stiffness_factor = 0.7
      ret.centerToFront = ret.wheelbase * 0.4
    elif candidate == CAR.ELANTRA_2021:
      ret.mass = (2800. * CV.LB_TO_KG) + STD_CARGO_KG
      ret.wheelbase = 2.72
      ret.steerRatio = 13.27 * 1.15   # 15% higher at the center seems reasonable
      tire_stiffness_factor = 0.65
      ret.centerToFront = ret.wheelbase * 0.4
    elif candidate == CAR.ELANTRA_HEV_2021:
      ret.mass = (3017. * CV.LB_TO_KG) + STD_CARGO_KG
      ret.wheelbase = 2.72
      ret.steerRatio = 13.27 * 1.15  # 15% higher at the center seems reasonable
      tire_stiffness_factor = 0.65
      ret.centerToFront = ret.wheelbase * 0.4
    elif candidate == CAR.KONA:
      ret.mass = 1275. + STD_CARGO_KG
      ret.wheelbase = 2.7
      tire_stiffness_factor = 0.7
      ret.centerToFront = ret.wheelbase * 0.4
    elif candidate in [CAR.KONA_HEV, CAR.KONA_EV]:
      ret.mass = 1395. + STD_CARGO_KG
      ret.wheelbase = 2.6
      tire_stiffness_factor = 0.7
      ret.centerToFront = ret.wheelbase * 0.4
    elif candidate in [CAR.IONIQ, CAR.IONIQ_EV_LTD, CAR.IONIQ_EV_2020, CAR.IONIQ_PHEV]:
      ret.mass = 1490. + STD_CARGO_KG
      ret.wheelbase = 2.7
      tire_stiffness_factor = 0.385
      #if candidate not in [CAR.IONIQ_EV_2020, CAR.IONIQ_PHEV]:
      #  ret.minSteerSpeed = 32 * CV.MPH_TO_MS
      ret.centerToFront = ret.wheelbase * 0.4
    elif candidate in [CAR.GRANDEUR_IG, CAR.GRANDEUR_IG_HEV]:
      tire_stiffness_factor = 0.8
      ret.mass = 1570. + STD_CARGO_KG
      ret.wheelbase = 2.845
      ret.centerToFront = ret.wheelbase * 0.385
      ret.steerRatio = 16.

    elif candidate in [CAR.GRANDEUR_IG_FL, CAR.GRANDEUR_IG_FL_HEV]:
      tire_stiffness_factor = 0.8
      ret.mass = 1600. + STD_CARGO_KG
      ret.wheelbase = 2.885
      ret.centerToFront = ret.wheelbase * 0.385
      ret.steerRatio = 17.
    elif candidate == CAR.VELOSTER:
      ret.mass = 3558. * CV.LB_TO_KG
      ret.wheelbase = 2.80
      tire_stiffness_factor = 0.9
      ret.centerToFront = ret.wheelbase * 0.4
    elif candidate == CAR.TUCSON_TL_SCC:
      ret.mass = 1594. + STD_CARGO_KG #1730
      ret.wheelbase = 2.67
      tire_stiffness_factor = 0.7
      ret.centerToFront = ret.wheelbase * 0.4
    # kia
    elif candidate == CAR.SORENTO:
      ret.mass = 1985. + STD_CARGO_KG
      ret.wheelbase = 2.78
      tire_stiffness_factor = 0.7
      ret.centerToFront = ret.wheelbase * 0.4
    elif candidate in [CAR.K5, CAR.K5_HEV]:
      ret.mass = 3558. * CV.LB_TO_KG
      ret.wheelbase = 2.80
      tire_stiffness_factor = 0.7
      ret.centerToFront = ret.wheelbase * 0.4
    elif candidate in [CAR.K5_2021, CAR.K5_HEV_2022]:
      ret.mass = 1515. + STD_CARGO_KG
      ret.wheelbase = 2.85
      tire_stiffness_factor = 0.7
    elif candidate == CAR.STINGER:
      tire_stiffness_factor = 1.125 # LiveParameters (Tunder's 2020)
      ret.mass = 1825.0 + STD_CARGO_KG
      ret.wheelbase = 2.906
      ret.centerToFront = ret.wheelbase * 0.4
    elif candidate == CAR.FORTE:
      ret.mass = 3558. * CV.LB_TO_KG
      ret.wheelbase = 2.80
      tire_stiffness_factor = 0.7
      ret.centerToFront = ret.wheelbase * 0.4
    elif candidate == CAR.CEED:
      ret.mass = 1350. + STD_CARGO_KG
      ret.wheelbase = 2.65
      tire_stiffness_factor = 0.6
      ret.centerToFront = ret.wheelbase * 0.4
    elif candidate == CAR.SPORTAGE:
      ret.mass = 1985. + STD_CARGO_KG
      ret.wheelbase = 2.78
      tire_stiffness_factor = 0.7
      ret.centerToFront = ret.wheelbase * 0.4
    elif candidate in [CAR.NIRO_EV, CAR.NIRO_HEV, CAR.NIRO_HEV_2021]:
      ret.mass = 1737. + STD_CARGO_KG
      ret.wheelbase = 2.7
      tire_stiffness_factor = 0.7
      ret.centerToFront = ret.wheelbase * 0.4
    elif candidate == CAR.SELTOS:
      ret.mass = 1310. + STD_CARGO_KG
      ret.wheelbase = 2.6
      tire_stiffness_factor = 0.7
      ret.centerToFront = ret.wheelbase * 0.4
    elif candidate == CAR.MOHAVE:
      ret.mass = 2285. + STD_CARGO_KG
      ret.wheelbase = 2.895
      ret.centerToFront = ret.wheelbase * 0.5
      tire_stiffness_factor = 0.8
    elif candidate in [CAR.K7, CAR.K7_HEV]:
      tire_stiffness_factor = 0.7
      ret.mass = 1650. + STD_CARGO_KG
      ret.wheelbase = 2.855
      ret.centerToFront = ret.wheelbase * 0.4
      ret.steerRatio = 17.25
    elif candidate == CAR.K9:
      ret.mass = 2075. + STD_CARGO_KG
      ret.wheelbase = 3.15
      ret.centerToFront = ret.wheelbase * 0.4
      tire_stiffness_factor = 0.8

      ret.steerRatio = 14.5

      if ret.lateralTuning.which() == 'torque':
        torque_tune(ret.lateralTuning, 2.3, 0.01)

    elif candidate == CAR.EV6:
      ret.mass = 2055 + STD_CARGO_KG
      ret.wheelbase = 2.9
      ret.steerRatio = 16.
      ret.safetyConfigs = [get_safety_config(car.CarParams.SafetyModel.noOutput),
                           get_safety_config(car.CarParams.SafetyModel.hyundaiHDA2)]
      tire_stiffness_factor = 0.65

      if ret.lateralTuning.which() == 'torque':
        torque_tune(ret.lateralTuning, 3.5, 0.01)


    # these cars require a special panda safety mode due to missing counters and checksums in the messages
    if candidate in LEGACY_SAFETY_MODE_CAR:
      ret.safetyConfigs = [get_safety_config(car.CarParams.SafetyModel.hyundaiLegacy)]

    # set appropriate safety param for gas signal
    if candidate in HYBRID_CAR:
      ret.safetyConfigs[0].safetyParam = 2
    elif candidate in EV_CAR:
      ret.safetyConfigs[0].safetyParam = 1

    ret.centerToFront = ret.wheelbase * 0.4

    # TODO: get actual value, for now starting with reasonable value for
    # civic and scaling by mass and wheelbase
    ret.rotationalInertia = scale_rot_inertia(ret.mass, ret.wheelbase)

    # TODO: start from empirically derived lateral slip stiffness for the civic and scale by
    # mass and CG position, so all cars will have approximately similar dyn behaviors
    ret.tireStiffnessFront, ret.tireStiffnessRear = scale_tire_stiffness(ret.mass, ret.wheelbase, ret.centerToFront,
                                                                         tire_stiffness_factor=tire_stiffness_factor)

    ret.enableBsm = 0x58b in fingerprint[0]

    if ret.openpilotLongitudinalControl:
      ret.safetyConfigs[0].safetyParam |= Panda.FLAG_HYUNDAI_LONG

    return ret

  @staticmethod
  def init(CP, logcan, sendcan):
    if CP.openpilotLongitudinalControl:
      disable_ecu(logcan, sendcan, addr=0x7d0, com_cont_req=b'\x28\x83\x01')

  def _update(self, c):
    ret = self.CS.update(self.cp, self.cp_cam)
    #ajouatom
    if not self.CS.CP.openpilotLongitudinalControl:# and c.enabled:
      ret.cruiseState.enabled = ret.cruiseState.available

    ret.steeringRateLimited = self.CC.steer_rate_limited if self.CC is not None else False

    # On some newer model years, the CANCEL button acts as a pause/resume button based on the PCM state
    # To avoid re-engaging when openpilot cancels, check user engagement intention via buttons
    # Main button also can trigger an engagement on these cars
    allow_enable = any(btn in ENABLE_BUTTONS for btn in self.CS.cruise_buttons) or any(self.CS.main_buttons)
    allow_enable = allow_enable or self.CP.carFingerprint in HDA2_CAR
    events = self.create_common_events(ret, pcm_enable=self.CS.CP.pcmCruise, allow_enable=allow_enable or True)

    if self.CS.brake_error:
      events.add(EventName.brakeUnavailable)

    if self.CS.CP.openpilotLongitudinalControl and self.CS.cruise_buttons[-1] != self.CS.prev_cruise_buttons:
      buttonEvents = [create_button_event(self.CS.cruise_buttons[-1], self.CS.prev_cruise_buttons, BUTTONS_DICT)]
      # Handle CF_Clu_CruiseSwState changing buttons mid-press
      if self.CS.cruise_buttons[-1] != 0 and self.CS.prev_cruise_buttons != 0:
        buttonEvents.append(create_button_event(0, self.CS.prev_cruise_buttons, BUTTONS_DICT))
      ret.buttonEvents = buttonEvents
      events.events.extend(create_button_enable_events(ret.buttonEvents))

      if self.CS.cruise_buttons[-1] != 0:
        # ajouatom
        if self.CS.cruise_buttons[-1] == Buttons.GAP_DIST:
          self.cruiseGap = 1 if self.cruiseGap == 4 else self.cruiseGap + 1
          print("cruiseGap={}", self.cruiseGap )

    if not self.CS.CP.openpilotLongitudinalControl:
      self.cruiseGap = ret.cruiseState.cruiseGap # ajouatom: carstate���� �о��..

    ret.cruiseState.cruiseGap = self.cruiseGap  #ajouatom      
    ret.cruiseGap = self.cruiseGap
    # low speed steer alert hysteresis logic (only for cars with steer cut off above 10 m/s)
    if ret.vEgo < (self.CP.minSteerSpeed + 2.) and self.CP.minSteerSpeed > 10.:
      self.low_speed_alert = True
    if ret.vEgo > (self.CP.minSteerSpeed + 4.):
      self.low_speed_alert = False
    if self.low_speed_alert:
      events.add(car.CarEvent.EventName.belowSteerSpeed)

    ret.events = events.to_msg()

    return ret

  def apply(self, c):
    return self.CC.update(c, self.CS)
