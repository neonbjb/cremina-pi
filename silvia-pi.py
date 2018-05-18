#!/usr/bin/python

def he_control_loop(dummy,state):
  from time import sleep
  from datetime import datetime, timedelta
  import RPi.GPIO as GPIO
  import config as conf

  GPIO.setmode(GPIO.BCM)
  GPIO.setup(conf.he_pin, GPIO.OUT)
  GPIO.output(conf.he_pin,1)

  heating = False

  try:
    while True:
      if state['snoozeon'] == True :
        now = datetime.now()
        dt = datetime.strptime(state['snooze'],'%H:%M')
        if dt.hour == now.hour and dt.minute == now.minute :
          state['snoozeon'] = False

      avgpid = state['avgpid']

      if state['snoozeon']:
        state['heating'] = False
        GPIO.output(conf.he_pin,1)
        sleep(1)
      else:
        if avgpid >= 100 :
          state['heating'] = True
          GPIO.output(conf.he_pin,0)
          sleep(1)
        elif avgpid > 0 and avgpid < 100:
          state['heating'] = True
          GPIO.output(conf.he_pin,0)
          sleep(avgpid/100.)
          GPIO.output(conf.he_pin,1)
          sleep(1-(avgpid/100.))
          state['heating'] = False
        else:
          GPIO.output(conf.he_pin,1)
          state['heating'] = False
          sleep(1)

  finally:
    GPIO.output(conf.he_pin,1)
    GPIO.cleanup()
  
    
def pid_loop(dummy,state):
  import sys
  from time import sleep, time
  from math import isnan
  from hx711 import HX711
  import Adafruit_GPIO.SPI as SPI
  import Adafruit_MAX31855.MAX31855 as MAX31855
  import PID as PID
  import config as conf
  
  # Append the HX711 module into the path
  sys.path.append('hx711py')

  def c_to_f(c):
    return c * 9.0 / 5.0 + 32.0

  sensor = MAX31855.MAX31855(spi=SPI.SpiDev(conf.spi_port, conf.spi_dev))
  loadcell = HX711(2, 3)
  loadcell.set_reading_format("LSB", "MSB")
  loadcell.set_reference_unit(1)
  loadcell.reset();
  loadcell.tare();
    
  def read_loadcell():
    return loadcell.get_weight(5) / conf.loadcell_calibration
    
  def loadcell_weight_to_bar(p):
    # 9 bar on the Cremina is ~= 40lbs of force on the loadcell.
    return 9 * p / 40
  
  pid = PID.PID(conf.Pc,conf.Ic,conf.Dc)
  pid.SetPoint = state['settemp']
  pid.setSampleTime(conf.sample_time*5)

  nanct=0
  i=0
  j=0
  pidhist = [0.,0.,0.,0.,0.,0.,0.,0.,0.,0.]
  avgpid = 0.
  temphist = [0.,0.,0.,0.,0.]
  avgtemp = 0.
  pressurehist = [0.,0.,0.,0.]
  avgpressure = 0.
  lastsettemp = state['settemp']
  lasttime = time()
  sleeptime = 0
  iscold = True
  iswarm = False
  lastcold = 0
  lastwarm = 0
  
  shot_times = [] # Stores last 10 shot times
  shot_start_time = 0 # Set to seconds since epoch since the last time shot_pressure_threshold was reached
  shot_debounce_count = 0 # Used to debounce pressure readings to determine when shots start and end
  DEBOUNCE_INTERVAL = 4 # Number of readings above the pressure threshold that must be seen before a shot is determined to have started
  shot_in_progress = False # Set to True when debounce algorithm determines that shot has started. Used to detect the end of a shot.

  try:
    while True : # Loops 10x/second
      tempc = sensor.readTempC()
      if isnan(tempc) :
        nanct += 1
        if nanct > 100000 :
          sys.exit
        continue
      else:
        nanct = 0

      tempf = c_to_f(tempc)
      temphist[i%5] = tempf
      avgtemp = sum(temphist)/len(temphist)
      
      # Fetch pressure
      rawpressure = read_loadcell();
      pressurehist[i%len(pressurehist)] = loadcell_weight_to_bar(rawpressure)
      avgpressure = sum(pressurehist)/len(pressurehist)
      
      # Attempt to recognize and time shots based on pressure readings
      if shot_in_progress:
        if avgpressure < conf.shot_pressure_threshold:
          shot_debounce_count += 1
          if shot_debounce_count > DEBOUNCE_INTERVAL:
            # Shot was finished.
            shot_times.append(round(time() - shot_start_time,2))
            shot_in_progress = False
            shot_debounce_count = 0
            shot_start_time = 0
        else:
          shot_debounce_count = 0
      elif avgpressure > conf.shot_pressure_threshold:
        if shot_debounce_count == 0:
          shot_start_time = time()
        shot_debounce_count += 1
        if shot_debounce_count > DEBOUNCE_INTERVAL:
          shot_in_progress = True
          shot_debounce_count = 0
      else:
        shot_start_time = 0
        shot_debounce_count = 0
        
      # Never let the shot times list grow bigger than 10
      while len(shot_times) > 10:
        del shot_times[0]

      if avgtemp < 100 :
        lastcold = i

      if avgtemp > 200 :
        lastwarm = i

      if iscold and (i-lastcold)*conf.sample_time > 60*15 :
        pid = PID.PID(conf.Pw,conf.Iw,conf.Dw)
        pid.SetPoint = state['settemp']
        pid.setSampleTime(conf.sample_time*5)
        iscold = False

      if iswarm and (i-lastwarm)*conf.sample_time > 60*15 : 
        pid = PID.PID(conf.Pc,conf.Ic,conf.Dc)
        pid.SetPoint = state['settemp']
        pid.setSampleTime(conf.sample_time*5)
        iscold = True

      if state['settemp'] != lastsettemp :
        pid.SetPoint = state['settemp']
        lastsettemp = state['settemp']

      if i%10 == 0 :
        pid.update(avgtemp)
        pidout = pid.output
        pidhist[i/10%10] = pidout
        avgpid = sum(pidhist)/len(pidhist)

      state['i'] = i
      state['tempf'] = round(tempf,2)
      state['avgtemp'] = round(avgtemp,2)
      state['rawpressure'] = round(rawpressure,2)
      state['avgpressure'] = round(avgpressure,2)
      state['pidval'] = round(pidout,2)
      state['avgpid'] = round(avgpid,2)
      state['pterm'] = round(pid.PTerm,2)
      if iscold :
        state['iterm'] = round(pid.ITerm * conf.Ic,2)
        state['dterm'] = round(pid.DTerm * conf.Dc,2)
      else :
        state['iterm'] = round(pid.ITerm * conf.Iw,2)
        state['dterm'] = round(pid.DTerm * conf.Dw,2)
      state['iscold'] = iscold
      state['shotactive'] = shot_in_progress
      state['activeshottime'] = 0 if (shot_start_time == 0) else (time() - shot_start_time)
      state['shottimes'] = shot_times
      state['lastshottime'] = 0 if (len(shot_times) == 0) else (shot_times[len(shot_times)-1])

      print time(), state

      sleeptime = lasttime+conf.sample_time-time()
      if sleeptime < 0 :
        sleeptime = 0
      sleep(sleeptime)
      i += 1
      lasttime = time()

  finally:
    pid.clear

def rest_server(dummy,state):
  from bottle import route, run, get, post, request, static_file, abort
  from subprocess import call
  from datetime import datetime
  import config as conf
  import os

  basedir = os.path.dirname(__file__)
  wwwdir = basedir+'/www/'

  @route('/')
  def docroot():
    return static_file('index.html',wwwdir)

  @route('/<filepath:path>')
  def servfile(filepath):
    return static_file(filepath,wwwdir)

  @route('/curtemp')
  def curtemp():
    return str(state['avgtemp'])
  
  @route('/curpressure')
  def curpressure():
    return str(state['avgpressure'])

  @get('/settemp')
  def settemp():
    return str(state['settemp'])

  @post('/settemp')
  def post_settemp():
    try:
      settemp = float(request.forms.get('settemp'))
      if settemp >= 200 and settemp <= 260 :
        state['settemp'] = settemp
        return str(settemp)
      else:
        abort(400,'Set temp out of range 200-260.')
    except:
      abort(400,'Invalid number for set temp.')

  @get('/snooze')
  def get_snooze():
    return str(state['snooze'])

  @post('/snooze')
  def post_snooze():
    snooze = request.forms.get('snooze')
    try:
      datetime.strptime(snooze,'%H:%M')
    except:
      abort(400,'Invalid time format.')
    state['snoozeon'] = True
    state['snooze'] = snooze
    return str(snooze)

  @post('/resetsnooze')
  def reset_snooze():
    state['snoozeon'] = False
    return True

  @get('/allstats')
  def allstats():
    return dict(state)

  @route('/restart')
  def restart():
    call(["reboot"])
    return '';

  @route('/shutdown')
  def shutdown():
    call(["shutdown","-h","now"])
    return '';

  @get('/healthcheck')
  def healthcheck():
    return 'OK'

  run(host='0.0.0.0',port=conf.port,server='cheroot')

if __name__ == '__main__':
  from multiprocessing import Process, Manager
  from time import sleep
  from urllib2 import urlopen
  import config as conf

  manager = Manager()
  pidstate = manager.dict()
  pidstate['snooze'] = conf.snooze 
  pidstate['snoozeon'] = False
  pidstate['i'] = 0
  pidstate['settemp'] = conf.set_temp
  pidstate['avgpid'] = 0.

  p = Process(target=pid_loop,args=(1,pidstate))
  p.daemon = True
  p.start()

  h = Process(target=he_control_loop,args=(1,pidstate))
  h.daemon = True
  h.start()

  r = Process(target=rest_server,args=(1,pidstate))
  r.daemon = True
  r.start()

  #Start Watchdog loop
  piderr = 0
  weberr = 0
  weberrflag = 0
  urlhc = 'http://localhost:'+str(conf.port)+'/healthcheck'

  lasti = pidstate['i']
  sleep(1)

  while p.is_alive() and h.is_alive() and r.is_alive():
    curi = pidstate['i']
    if curi == lasti :
      piderr = piderr + 1
    else :
      piderr = 0

    lasti = curi

    if piderr > 9 :
      print 'ERROR IN PID THREAD, RESTARTING'
      p.terminate()

    try:
      hc = urlopen(urlhc,timeout=2)
    except:
      weberrflag = 1
    else:
      if hc.getcode() != 200 :
        weberrflag = 1

    if weberrflag != 0 :
      weberr = weberr + 1

    if weberr > 9 :
      print 'ERROR IN WEB SERVER THREAD, RESTARTING'
      r.terminate()

    weberrflag = 0

    sleep(1)
