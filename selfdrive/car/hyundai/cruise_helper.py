import copy
import random
import numpy as np
from common.numpy_fast import clip, interp
from cereal import car
from common.realtime import DT_CTRL
from common.conversions import Conversions as CV
from selfdrive.car.hyundai.values import Buttons
from common.params import Params
from selfdrive.controls.lib.drive_helpers import V_CRUISE_MAX, V_CRUISE_MIN, V_CRUISE_DELTA_KM, V_CRUISE_DELTA_MI
from selfdrive.controls.lib.lane_planner import TRAJECTORY_SIZE
from selfdrive.controls.lib.longitudinal_mpc_lib.long_mpc import AUTO_TR_CRUISE_GAP

from selfdrive.ntune import ntune_scc_get
from selfdrive.road_speed_limiter import road_speed_limiter_get_max_speed, road_speed_limiter_get_active, \
  get_road_speed_limiter
import common.loger as trace1

SYNC_MARGIN = 3.

# do not modify
MIN_SET_SPEED_KPH = V_CRUISE_MIN
MAX_SET_SPEED_KPH = V_CRUISE_MAX

ALIVE_COUNT = [6, 8]
WAIT_COUNT = [12, 13, 14, 15, 16]
AliveIndex = 0
WaitIndex = 0

MIN_CURVE_SPEED = 25. * CV.KPH_TO_MS

EventName = car.CarEvent.EventName

ButtonType = car.CarState.ButtonEvent.Type
ButtonPrev = ButtonType.unknown
ButtonCnt = 0
LongPressed = False

class CruiseHelper:

  def __init__(self):
    self.mapValid = 0
    self.v_cruise_kph_backup = 20
    self.v_cruise_kph_current = 20
    self.curve_speed_last = 255
    self.cruiseSuspended = False
    self.preBrakePressed = False
    self.curvature = 0
    self.position_x = 1000.0
    self.position_y = 300.0
    self.activate_E2E = False

    self.update_params(0, True)

  def update_params(self, frame, all):
    if all or frame % 100 == 0:
      self.autoCurveSpeedCtrl = Params().get_bool("AutoCurveSpeedCtrl")
      self.naviDecelMarginDist = float(int(Params().get("NaviDecelMarginDist", encoding="utf8")))
      self.naviDecelRate = float(int(Params().get("NaviDecelRate", encoding="utf8")))
    if all or (frame + 30) % 100 == 0:
      self.autoResumeFromGasSpeed = float(int(Params().get("AutoResumeFromGasSpeed", encoding="utf8")))
      self.autoResumeFromGas = Params().get_bool("AutoResumeFromGas")
      self.autoResumeFromBrakeRelease = Params().get_bool("AutoResumeFromBrakeRelease")
    if all or (frame + 60) % 100 == 0:
      self.autoResumeFromBrakeReleaseDist = float(int(Params().get("AutoResumeFromBrakeReleaseDist", encoding="utf8")))
      self.autoResumeFromBrakeReleaseLeadCar = Params().get_bool("AutoResumeFromBrakeReleaseLeadCar")

  @staticmethod
  def get_lead(sm):

    radar = sm['radarState']
    if radar.leadOne.status:
      return radar.leadOne

    return None

  def get_apply_accel(self, CS, sm, accel, stopping):

    gas_factor = ntune_scc_get("sccGasFactor")
    brake_factor = ntune_scc_get("sccBrakeFactor")

    #lead = self.get_lead(sm)
    #if lead is not None:
    #  if not lead.radar:
    #    brake_factor *= 0.975

    if accel > 0:
      accel *= gas_factor
    else:
      accel *= brake_factor
      
    return accel

# ajouatom
# navi 운영방법
# - 속도제한신호(mapValid)가 들어오면 현재크루즈속도(v_cruise_kph) 백업(v_cruise_kph_backup)
# - 속도제한신호가 나가면 백업된 크르즈속도 복원
# - 운행중 사용자가 속도변경을 하면?
#   - 버튼을 눌렀을때: resume버튼시 백업속도로 변경, 현재크루즈속도를 백업하도록 함.
#   - 엑셀을 밟았을때: 현재크루즈속도변경, 현재크르주속도를 백업하도록 함.
#   - 브레이크를 밟았을때: 현재크루즈속도 변경, resume시 백업속도로 주행
# - 속도제한신호가 들어오면
# - 구간단속은???? 몰라..
# - 크루즈 운용방법(HW)
#   - 브레이크를 밟으면: Cruise disable... 
#   - 브레이클를 밟았다 떼면: Cruise disable... 유지
#   - 엑셀을 밟으면: 30Km/h이상되면 auto Resume
#   - 엑셀을 밟았다 떼면: zz
  def cal_curve_speed(self, controls, v_ego, frame, curve_speed_last):
    curve_speed_ms = curve_speed_last

    if frame % 20 == 0:
      md = controls.sm['modelV2']
      curvature = self.curvature
      if len(md.position.x) == TRAJECTORY_SIZE and len(md.position.y) == TRAJECTORY_SIZE:
        x = md.position.x
        y = md.position.y
        dy = np.gradient(y, x)
        d2y = np.gradient(dy, x)
        curv = d2y / (1 + dy ** 2) ** 1.5

        start = int(interp(v_ego, [10., 27.], [10, TRAJECTORY_SIZE-10]))
        if abs(curvature) > 0.0008: # opkr
          curv = curv[5:TRAJECTORY_SIZE-10]
        else:
          curv = curv[start:min(start+10, TRAJECTORY_SIZE)]
        a_y_max = 2.975 - v_ego * 0.0375  # ~1.85 @ 75mph, ~2.6 @ 25mph
        v_curvature = np.sqrt(a_y_max / np.clip(np.abs(curv), 1e-4, None))
        model_speed = np.mean(v_curvature) * 0.85 * ntune_scc_get("sccCurvatureFactor")
        if np.isnan(model_speed):
          model_speed = 0.0

        curve_speed_ms = float(max(model_speed, MIN_CURVE_SPEED))

        #if model_speed < v_ego:
        #  curve_speed_ms = float(max(model_speed, MIN_CURVE_SPEED))
        #else:
        #  curve_speed_ms = 255.

        if np.isnan(curve_speed_ms):
          curve_speed_ms = 255.

        self.position_x = x[TRAJECTORY_SIZE-1]
        self.position_y = y[TRAJECTORY_SIZE-1]
        str_log1 = 'CURVE={:5.1f},MODEL={:5.1f},POS={:5.1f},{:5.1f}'.format( curve_speed_ms*CV.MS_TO_KPH, model_speed*CV.MS_TO_KPH, x[TRAJECTORY_SIZE-1], y[TRAJECTORY_SIZE-1])
        trace1.printf2( '{}'.format( str_log1 ) )
      else:
        #curve_speed_ms = 255.
        self.position_x = 1000.0
        self.position_y = 300.0
        pass
      #if curve_speed_ms < 255:
      #  print('curve_speed_ms = {:3.1f}'.format(curve_speed_ms*CV.MS_TO_KPH))
  
    return curve_speed_ms

  def cruise_pause(self, controls):
    if controls.enabled and not self.cruiseSuspended:
      controls.events.add(EventName.cruisePaused)
      self.cruiseSuspended = True
    
  def cruise_resume(self, controls, CS):
    if controls.enabled and self.cruiseSuspended:
      controls.LoC.reset(v_pid=CS.vEgo)
      controls.events.add(EventName.cruiseResume)
      self.cruiseSuspended = False

  @staticmethod
  def pcmCruiseControl(v_cruise_kph_last, v_cruise_kph):
    pass

  def update_cruise_navi(self, controls, CS):  # called by controlds's state_transition

    frame = controls.sm.frame
    self.update_params(frame, False)

    v_cruise_kph_last = controls.v_cruise_kph
    vEgo_cruise_kph = int(clip(CS.vEgo * CV.MS_TO_KPH+0.5, 20, 161))
    
    mapValid_last = self.mapValid

    # target속도가 현재설정속도보다 낮으면 바꿈.
    #if self.v_cruise_kph_backup < controls.v_cruise_kph:
    #  self.v_cruise_kph_backup = controls.v_cruise_kph
      
    #cruise_set_speed_kph = controls.v_cruise_kph      
    cruise_set_speed_kph = self.v_cruise_kph_current
    spdTarget = cruise_set_speed_kph #설정속도
    v_ego_kph = CS.vEgo * CV.MS_TO_KPH    #실제속도
    speedLimit = controls.sm['liveNaviData'].speedLimit
    speedLimitDistance = controls.sm['liveNaviData'].arrivalDistance  #speedLimitDistance
    safetySign  = controls.sm['liveNaviData'].safetySign #124 과속방지턱,
    self.mapValid = controls.sm['liveNaviData'].mapValid
    trafficType = controls.sm['liveNaviData'].trafficType
    turnInfo = controls.sm['liveNaviData'].turnInfo
    distanceToTurn = controls.sm['liveNaviData'].distanceToTurn
    #print("get_navi...{} {} {} {} {} {}".format(speedLimit, speedLimitDistance, mapValid, trafficType, turnInfo, distanceToTurn))
    
    #Navi신호가 없다가 생기면 현재속도 백업(맵종류별로 백업을...)
    if not mapValid_last and self.mapValid:
      self.v_cruise_kph_backup = self.v_cruise_kph_current
    #Navi신호가 없어지면 이전속도로 복원
    if mapValid_last and not self.mapValid:
      self.v_cruise_kph_current = self.v_cruise_kph_backup

    naviDecelMarginDist = interp(v_ego_kph, [30, 120], [50, self.naviDecelMarginDist])
    #현재속도유지: navi신호가 없거나 trafficType 이 아무것도 없으면...
    if not self.mapValid or trafficType == 0:      
      spdTarget = cruise_set_speed_kph
    #현재속도유지: 속도제한이 30이하이면
    elif speedLimit < 30:
      spdTarget = cruise_set_speed_kph
    #elif safetySign == 124: #과속방지턱
    #  spdTarget = interp(v_ego_kph , [40, 60, 80], [35, 50, 65])
    elif speedLimitDistance < naviDecelMarginDist:
      spdTarget = speedLimit
    elif cruise_set_speed_kph <= speedLimit:
      spdTarget = cruise_set_speed_kph
    elif vEgo_cruise_kph <= speedLimit:
      spdTarget = speedLimit
    else:
      distMargin = naviDecelMarginDist
      acc = 100./self.naviDecelRate * 3600.  # meter/hour: 0~100이 20초 이정도로 감속..
      dist = (vEgo_cruise_kph + speedLimit)/2. * (vEgo_cruise_kph - speedLimit) / acc * 1000. #현재속도대비 감속에 필요한거리 (M)
      dist = max(dist, 1)
      if speedLimitDistance < dist + distMargin:
        spdTarget = speedLimit + (speedLimitDistance-distMargin)/dist * (vEgo_cruise_kph - speedLimit)
      elif vEgo_cruise_kph < speedLimit:
        spdTarget = speedLimit + 10
      else:
        spdTarget = cruise_set_speed_kph
      if spdTarget < speedLimit:
        spdTarget = speedLimit

    if self.autoCurveSpeedCtrl:
      curve_speed = self.cal_curve_speed(controls, CS.vEgo, controls.sm.frame, self.curve_speed_last)
    else:
      curve_speed = 255
    self.curve_speed_last = curve_speed
    #설정속도, 제한속도, 현재속도중 가장 낮은속도로 변경함..
    cruise_set_speed_kph_navi = min( cruise_set_speed_kph, spdTarget)

    controls.v_cruise_kph = min( cruise_set_speed_kph_navi, curve_speed * CV.MS_TO_KPH)
    #네비 또는 커브에 의해 속도가 변경되면... 
    if cruise_set_speed_kph_navi!= cruise_set_speed_kph or cruise_set_speed_kph_navi != controls.v_cruise_kph:
      #v_cruise_kph_current: 현재설정속도를 변경안함..
      pass
    else:
      self.v_cruise_kph_current = cruise_set_speed_kph

    #self.v_cruise_kph = clip(round(cruise_set_speed_kph, 1), 8, 180)
    #print("get_navi...{} {} {} {}".format(speedLimit, speedLimitDistance, self.mapValid, trafficType))
    #str_log1 = 'NaviSpeed:v:{},t:{},l:{:.1f},d:{:.1f},ss{},t{:.1f}'.format( self.mapValid, trafficType, speedLimit, speedLimitDistance, safetySign, self.v_cruise_kph)
    #trace1.printf2( '{}'.format( str_log1 ) )
    #print("{}".format(str_log1))

    #ajouatom test: 
    #v_cruise_kph => 현재설정속도, navi,curve에 의해 변화됨.
    #v_cruise_kph_current => curve에 대한 백업
    #v_cruise_kph_backup => navi속도 백업
    # 버튼을 안누르고 엑셀로 자동속도변경..
    # 브레이크를 밟거나 엑셀을 밟으면
    blinker = CS.leftBlinker or CS.rightBlinker
    resumeGasPedal = 0.4
    #깜박이OFF, 핸들각도가 10도이내, 핸들에 토크가 없고..
    resume_cond = not blinker and abs(CS.steeringAngleDeg) < 10 and not CS.steeringPressed
    if controls.enabled:              
      if CS.gasPressed:
        if not self.cruiseSuspended and self.activate_E2E == True and CS.gas > resumeGasPedal:
          self.activate_E2E = False
          controls.events.add(EventName.cruiseResume)

        if self.cruiseSuspended:
          # auto resuming 30KM/h가 넘을때..
          if CS.gas > resumeGasPedal and resume_cond and CS.vEgo*CV.MS_TO_KPH >=  self.autoResumeFromGasSpeed and self.autoResumeFromGas:
            controls.v_cruise_kph = vEgo_cruise_kph + 5.0
            self.v_cruise_kph_current = controls.v_cruise_kph
            self.v_cruise_kph_backup = controls.v_cruise_kph
            self.cruise_resume(controls, CS)
        # 엑셀을 밟고 주행속도가 설정속도보다 높아지면..
        elif vEgo_cruise_kph > controls.v_cruise_kph - 10.0:
          #설정속도를 주행속도도 변경함.. (파라미터화?)
          controls.v_cruise_kph = vEgo_cruise_kph + 8.0

        #Sync cruise Speed from Gas: 설정속도가 backup속도보다 높아지면 backup속도 변경
        if controls.v_cruise_kph > self.v_cruise_kph_backup:
            self.v_cruise_kph_backup = controls.v_cruise_kph
            self.v_cruise_kph_current = controls.v_cruise_kph
        #Auto Speed Change pre backup speed:2단계: 설정속도가 current속도보다 높으면 자동으로 backup속도로 변경..
        elif controls.v_cruise_kph > self.v_cruise_kph_current:
            controls.v_cruise_kph = self.v_cruise_kph_backup
            self.v_cruise_kph_current = self.v_cruise_kph_backup
        #Auto Speed Change pre current speed:1단계: 설정속도가  30Km이상이면 자동으로 current속도로 변경..
        elif CS.gas > 0.6 and controls.v_cruise_kph >= self.autoResumeFromGasSpeed and controls.v_cruise_kph<self.v_cruise_kph_current:
            controls.v_cruise_kph = self.v_cruise_kph_current
      #브레이크를 밟으면... cruise해제,...
      elif CS.brakePressed:
        self.cruise_pause(controls)
        pass
      # brake를 밟았다가 떼었을때
      elif not CS.brakePressed and self.preBrakePressed:
        if resume_cond and self.autoResumeFromBrakeRelease:
          lead = self.get_lead(controls.sm)
          dRel = lead.dRel if lead is not None else 0
          #전방레이더가 Params 이상 잡혀있으면 Cruise control 활성화..
          if v_ego_kph > 3.0 and self.autoResumeFromBrakeReleaseDist <  dRel < 100 :
            self.cruise_resume(controls, CS)
            controls.v_cruise_kph = vEgo_cruise_kph + 5.0
            self.v_cruise_kph_current = controls.v_cruise_kph
          # 60km/h 이하.. 직선도로 곡선 5M이내, 150M이내 정지선, 자동E2E모드 전환.
          elif v_ego_kph <= 60.0 and self.position_x < 150.0: # and abs(self.position_y) < 3.0:
            self.cruise_resume(controls, CS)
            self.activate_E2E = True
          # 40km/h이상, 전방에 레이더가 잡히지 않으면, 운전자가 너무빠르다고 판단.. 브레이크를 밟았을것이라고 판단....브레이크를 떼는 순간의 속도로 유지...
          elif v_ego_kph >= 30.0:
            if 0 < dRel < 100.0:
              pass
            else:
              controls.v_cruise_kph = vEgo_cruise_kph + 5.0
              self.v_cruise_kph_current = controls.v_cruise_kph
              self.cruise_resume(controls, CS)
          # 정지상태에서 전방에 10M이내 차가 있으면 Cruise control 활성화
          elif v_ego_kph < 1.0 and resume_cond and 2 < dRel < 10:
            if self.autoResumeFromBrakeReleaseLeadCar:
              self.cruise_resume(controls, CS)

    self.preBrakePressed = CS.brakePressed


  def update_v_cruise2(self, v_cruise_kph, buttonEvents, enabled, metric, controls, CS):

    global ButtonCnt, LongPressed, ButtonPrev
    button_type = 0
    if enabled:
      if ButtonCnt:
        ButtonCnt += 1
      for b in buttonEvents:
        if b.pressed and not ButtonCnt and (b.type == ButtonType.accelCruise or b.type == ButtonType.decelCruise or b.type == ButtonType.gapAdjustCruise):
          ButtonCnt = 1
          ButtonPrev = b.type
        elif not b.pressed and ButtonCnt:
          if not LongPressed and b.type == ButtonType.accelCruise:
            v_cruise_kph += 2 if metric else 1 * CV.MPH_TO_KPH
            button_type = ButtonType.accelCruise
          elif not LongPressed and b.type == ButtonType.decelCruise:
            v_cruise_kph -= 2 if metric else 1 * CV.MPH_TO_KPH
            button_type = ButtonType.decelCruise
          elif not LongPressed and b.type == ButtonType.gapAdjustCruise:
            pass
          LongPressed = False
          ButtonCnt = 0
      if ButtonCnt > 70:
        LongPressed = True
        V_CRUISE_DELTA = V_CRUISE_DELTA_KM if metric else V_CRUISE_DELTA_MI
        if ButtonPrev == ButtonType.accelCruise:
          v_cruise_kph += V_CRUISE_DELTA - v_cruise_kph % V_CRUISE_DELTA
          button_type = ButtonType.accelCruise
        elif ButtonPrev == ButtonType.decelCruise:
          v_cruise_kph -= V_CRUISE_DELTA - -v_cruise_kph % V_CRUISE_DELTA
          button_type = ButtonType.decelCruise
        ButtonCnt %= 70
      v_cruise_kph = clip(v_cruise_kph, MIN_SET_SPEED_KPH, MAX_SET_SPEED_KPH)

      # accel button을 눌렀을때...
      if button_type == ButtonType.accelCruise and not self.cruiseSuspended and self.activate_E2E == True:
          self.activate_E2E = False
          controls.events.add(EventName.cruiseResume)

      elif button_type == ButtonType.accelCruise:
        self.cruise_resume(controls, CS)

        # current속도가 set도보다 느리면 current속도 바꿈. 
        if self.v_cruise_kph_current < v_cruise_kph:
          self.v_cruise_kph_current = v_cruise_kph

        # current속도가 set속도보다 빠르면 current속도로 바꿈.
        if self.v_cruise_kph_current > v_cruise_kph:
          v_cruise_kph = self.v_cruise_kph_current
        # backup속도가 set속도보다 빠르면 backup속도로 바꿈.
        elif self.v_cruise_kph_backup > v_cruise_kph:
          v_cruise_kph = self.v_cruise_kph_backup
          self.v_cruise_kph_current = v_cruise_kph

      # decelbutton 
      if button_type == ButtonType.decelCruise:
        # set속도가 주행속도보다 느리면 설정속도와 backup속도를 주행속도로 바꿈.
        vEgo_cruise_kph = int(clip(CS.vEgo * CV.MS_TO_KPH+0.5, 20, 161))
        # Cruise가 중지된상황이면, E2E로 시작..
        if self.activate_E2E == False:
          self.activate_E2E = True
          controls.events.add(EventName.cruiseResume)
          if self.cruiseSuspended:
            self.cruise_resume(controls, CS)
            if v_cruise_kph < vEgo_cruise_kph:
              v_cruise_kph = vEgo_cruise_kph + 3
            if self.v_cruise_kph_current < v_cruise_kph:
              self.v_cruise_kph_current = v_cruise_kph

        elif self.cruiseSuspended:
          self.cruise_resume(controls, CS)
          if v_cruise_kph < vEgo_cruise_kph:
            v_cruise_kph = vEgo_cruise_kph + 3
          if self.v_cruise_kph_current < v_cruise_kph:
            self.v_cruise_kph_current = v_cruise_kph
        else:
          if vEgo_cruise_kph < v_cruise_kph:
            v_cruise_kph = vEgo_cruise_kph + 3
          self.v_cruise_kph_backup = v_cruise_kph
          self.v_cruise_kph_current = v_cruise_kph

    if self.v_cruise_kph_backup < self.v_cruise_kph_current:
      self.v_cruise_kph_backup = self.v_cruise_kph_current

    str_log1 = 'gas{:5.1f},cruise:v:{},{},c:{},b:{}'.format(CS.gas, v_cruise_kph, controls.v_cruise_kph, self.v_cruise_kph_current, self.v_cruise_kph_backup)
    trace1.printf3( '{}'.format( str_log1 ) )

    return v_cruise_kph

