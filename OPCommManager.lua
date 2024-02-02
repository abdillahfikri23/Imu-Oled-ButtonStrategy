--Darwin OP Commmanager for NSL 2011
module(..., package.seeall);

--Added for Hardware config file
local struct = require('struct')
local cwd = unix.getcwd();
package.path = cwd.."/../Config/?.lua;"..package.path;

require('DynamixelPacket');
require('Dynamixel');
require('unix');
require('shm');
require('carray');
require('vector');
require('Config');
require('Transform')

print("Robot ID:",Config.game.robotID);

dirReverse = Config.servo.dirReverse;
posZero=Config.servo.posZero;
gyrZero=Config.gyro.zero;
legBias=Config.walk.servoBias;
armBias=Config.servo.armBias;
idMap = Config.servo.idMap;
nJoint = #idMap;
scale={};
for i=1,nJoint do 
  scale[i]=Config.servo.steps[i]/Config.servo.moveRange[i];
end

tLast=0;
count=1;
battery_warning=0;
battery_led1 = 0;
battery_led2 = 0;
battery_blink = 0;

chk_servo_no=0;
nButton = 0;

-- USB disconnect bug
USB_bug = 1;
USB_bug = 0;
torqueTrigger = 0;
for i = 1,#Config.servo.dirReverse do
  scale[Config.servo.dirReverse[i]] = -scale[Config.servo.dirReverse[i]];
end

-- Setup shared memory
function shm_init()
  shm.destroy('dcmSensor');
  sensorShm = shm.new('dcmSensor');
  sensorShm.kneeCurrent = vector.zeros(2);
  sensorShm.strategy = vector.zeros(2);
  sensorShm.voltage = vector.zeros(1);
  sensorShm.time = vector.zeros(1);
  sensorShm.count = vector.zeros(1);
  sensorShm.position = vector.zeros(nJoint);
  sensorShm.servoposition = vector.zeros(nJoint);
  sensorShm.button = vector.zeros(2); --OP has TWO buttons

  sensorShm.imuAngle = vector.zeros(3);
  sensorShm.imuAcc = vector.zeros(3);
  sensorShm.imuGyr = vector.zeros(3);
  sensorShm.imuAccRaw = vector.zeros(3);
  sensorShm.imuGyrRaw = vector.zeros(3);
  sensorShm.imuGyrBias=vector.zeros(3); --rate gyro bias
  sensorShm.temperature=vector.zeros(nJoint);
  sensorShm.battery=vector.zeros(1); --Now only use cm730 value
  sensorShm.updatedCount =vector.zeros(1);   --Increases at every cycle

  shm.destroy('dcmActuator');
  actuatorShm = shm.new('dcmActuator');
  print(nJoint)
  actuatorShm.command = vector.zeros(nJoint);
  actuatorShm.velocity = vector.zeros(nJoint);
  actuatorShm.hardness = vector.zeros(nJoint);
  actuatorShm.offset = vector.zeros(nJoint); --in rads
  actuatorShm.bias = vector.zeros(nJoint); --in clicks
  actuatorShm.led = vector.zeros(1);

  actuatorShm.torqueEnable = vector.zeros(1); --Global torque on.off
  -- Gain 0: normal gain 1: Kick gain (more stiff)
  actuatorShm.gain=vector.zeros(nJoint); 
  actuatorShm.gainChanged=vector.ones(1);  --set compliance once
  actuatorShm.velocityChanged=vector.zeros(1);
  actuatorShm.hardnessChanged=vector.zeros(1);
  actuatorShm.torqueEnableChanged=vector.zeros(1);

  actuatorShm.backled = vector.zeros(3);  --red blue green
  actuatorShm.eyeled = vector.zeros(3);   --RGB15 eye led
  actuatorShm.headled = vector.zeros(3);  --RGB15 head led
  actuatorShm.headledChanged = vector.zeros(1);

  --Dummy variable (for compatibility with nao)
  actuatorShm.ledFaceRight=vector.zeros(24);
  actuatorShm.ledFaceLeft=vector.zeros(24);
  actuatorShm.ledChest=vector.zeros(24);

  --New PID parameters variables
  --Default value is (32,0,0)
  actuatorShm.p_param=vector.ones(nJoint)*32; 
  actuatorShm.i_param=vector.ones(nJoint)*0; 
  actuatorShm.d_param=vector.ones(nJoint)*0; 

  --SJ: list of servo IDs to read
  --0: Head only 1: All servos 2: Head+Leg
  --readID: 1 for readable, 0 for non-readable
  actuatorShm.readType=vector.zeros(1);   
  actuatorShm.readID=vector.zeros(nJoint); 

  --SJ: battery testing mode (read voltage from all joints)
  actuatorShm.battTest=vector.zeros(1);   
end

function entry()
  Dynamixel.open();
  --   Dynamixel.ping_probe();
  --We have to manually turn on the MC for OP   
  -- Dynamixel.dynamixel_power(200,1);
  unix.usleep(200000);
  -- Dynamixel.ping_probe();
  shm_init();
  carray_init();
  -- Read head and not legs
  actuator.readType[1]=1;
  -- Read only kankles
  actuator.readType[1]=3;

  -- Read initial leg bias from config
  for i=1,12 do 	
    actuator.bias[i+5]=legBias[i];
  end

  sync_gain(); --Initial PID setting

  --Setting arm bias
  for i=1,3 do
    actuator.offset[i+2]=armBias[i];
  end
  for i=4,6 do
    actuator.offset[i+14]=armBias[i];
  end

end

-- Setup CArray mappings into shared memory
function carray_init()
  sensor = {};
  for k,v in sensorShm.next, sensorShm do
    sensor[k] = carray.cast(sensorShm:pointer(k));
  end

  actuator = {};
  for k,v in actuatorShm.next, actuatorShm do
    actuator[k] = carray.cast(actuatorShm:pointer(k));
  end
end


function sync_command()
  local addr = 116;
  local ids = {};
  local data = {};
  local n = 0;
  for i = 1,#idMap do
    if (actuator.hardness[i] > 0) then
      n = n+1;
      ids[n] = idMap[i];
      local word=0;
      word = posZero[i] + actuator.bias[i] + scale[i]*
      (actuator.command[i]+actuator.offset[i]);
      data[n] = math.min(math.max(word, 0), Config.servo.steps[i]-1);
    else
      actuator.command[i] = sensor.position[i];
    end
  end
  if (n > 0) then
    Dynamixel.sync_write_word(ids, addr, data);
  end
end

function sync_hardness()
  local addr=100; --hardness is working with RX28
  local ids = {};
  local data = {};
  local n = 0;
  for i = 1,#idMap do
    n = n+1;
    ids[n] = idMap[i];
    data[n] = 1023*actuator.hardness[i];
    -- print("data hardness"..i.."= "..data[n])
  end
  if (n > 0) then
    Dynamixel.sync_write_word(ids, 100, data);
    -- Dynamixel.sync_write_word(ids, 36, data);
  end
end

function torque_enable(valueTorque)
  local addr = 64;
  local ids = {};
  local data = {};
  local n = 0;
  for i = 1,#idMap do
    n = n+1;
    ids[n] = idMap[i];
    data[n] = valueTorque;--actuator.torqueEnable[1];
  end
  if (n > 0) then
    Dynamixel.sync_write_byte(ids, addr, data);
  end   
  -- print("Torque enable changed")
end

--Servo feedback param for servomotors
--Used to stiffen support foot during kicking

function sync_gain()
  if Config.servo.pid==0 then --Old firmware.. compliance slope
    --28,29: Compliance slope positive / negative
    local addr={84,85};
    local ids = {};
    local data = {};
    local n = 0;
    for i = 1,#idMap do
      n = n+1;
      ids[n] = idMap[i];
      if actuator.gain[i]>0 then
        data[n] = Config.servo.slope_param[2];
      else
        data[n] = Config.servo.slope_param[1];
      end
    end
    Dynamixel.sync_write_byte(ids, addr[1], data);
    Dynamixel.sync_write_byte(ids, addr[2], data);
  else --New firmware: PID parameters
    -- P: 28, I: 27, D: 26
    local addr={84,82,80};

    local ids = {};
    local data_p = {};
    local data_i = {};
    local data_d = {};
    local n = 0;
    for i = 1,#idMap do
      n = n+1;
      ids[n] = idMap[i];
      if actuator.gain[i]>0 then
        data_p[n] = Config.servo.pid_param[2][1];
        data_i[n] = Config.servo.pid_param[2][2];
        data_d[n] = Config.servo.pid_param[2][3];
      else
        data_p[n] = Config.servo.pid_param[1][1];
        data_i[n] = Config.servo.pid_param[1][2];
        data_d[n] = Config.servo.pid_param[1][3];
      end
    end
    Dynamixel.sync_write_byte(ids, addr[1], data_p);
    Dynamixel.sync_write_byte(ids, addr[2], data_i);
    Dynamixel.sync_write_byte(ids, addr[3], data_d);
  end
end

function sync_battery()
    --battery test mode... read from ALL servos
  if actuator.battTest[1]==1 then 
    chk_servo_no=(chk_servo_no%nJoint)+1;
    sensor.temperature[chk_servo_no]=Dynamixel.get_temperature(idMap[chk_servo_no]);
  else
    chk_servo_no=(chk_servo_no%12)+1;
    sensor.temperature[chk_servo_no+5]=0;
  end

  local battery=Dynamixel.read_data(200,31,4);
  if battery ~= nil then
    sensor.battery[1]=battery[1];
    if battery[1]<Config.bat_low then battery_warning=1;
    else battery_warning=0;
    end

    if battery[1]<Config.bat_led[1] then
      battery_led1 = 0;
      battery_led2 = 0;
    elseif battery[1]<Config.bat_led[2] then
      battery_led1 = 1;
      battery_led2 = 0;
    elseif battery[1]<Config.bat_led[3] then
      battery_led1 = 1;
      battery_led2 = 1;
    elseif battery[1]<Config.bat_led[4] then
      battery_led1 = 1+2;
      battery_led2 = 1;
    elseif battery[1]<Config.bat_led[5] then
      battery_led1 = 1+2;
      battery_led2 = 1+2;
    elseif battery[1]<Config.bat_led[6] then
      battery_led1 = 1+2+4;
      battery_led2 = 1+2;
    else
      battery_led1 = 1+2+4;
      battery_led2 = 1+2+4;
    end

  end
end


function sync_led()
  --New function to turn on status LEDs
  local packet;

  if count%20==0 then --5 fps eye led refresh rate
    packet=actuator.eyeled[1]+32*actuator.eyeled[2]+1024*actuator.eyeled[3];
    Dynamixel.sync_write_word({200},25,{packet});
    unix.usleep(100);
  end

  if battery_warning==1 then
    packet=5;
    Dynamixel.sync_write_word({200},26,{packet});
    unix.usleep(100);
    battery_warning=0;

  elseif count%20==10 then --5 fps head led refresh rate
    packet=actuator.headled[1]+32*actuator.headled[2]+1024*actuator.headled[3];
    Dynamixel.sync_write_word({200},26,{packet});
    unix.usleep(100);
  end

  if count%100==25 then --1 fps back led refresh rate
--    packet =  
-- actuator.backled[1]+2*actuator.backled[2]+4*actuator.backled[3];

    battery_blink = 1-battery_blink;
    if battery_blink == 1 then
      packet=battery_led1;
    else
      packet=battery_led2;
    end

    Dynamixel.sync_write_byte({200},25,{packet});
    unix.usleep(100);
  end
end

function bulk_read()
--[[
  --146: bulk read instruction
  --36: Position address
  print("Bulk read test");
  data=Dynamixel.bulk_read_data(200,{1,2,3},36,2); 
  print("Received data size:",#data);
  print("Received data:",unpack(data));
--]]
end


function nonsync_read()

  --Position reading
  local idToRead={1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20};   --Head only reading
  if actuator.readType[1]==1 then --All servo reading
    for i=1,#idMap do 
      idToRead[i]=i;
    end
  elseif actuator.readType[1]==3 then -- Read ankles only
    idToRead = {10,16}; --kankle ids
    for i = 1,#idMap do
      sensor.position[i] = actuator.command[i];
    end;
  else --if actuator.readType[1]==0 then --Head only reading
    for i = 3,#idMap do
      sensor.position[i] = actuator.command[i];
    end;

  end
  -- Update the readings
  for i = 1,#idToRead do
    local id = idMap[idToRead[i]];
    --Sometimes DCM crashes here... maybe a bug
    local raw=null;
    if id then
      raw=Dynamixel.get_position(id);
      if raw then
	sensor.servoposition[idToRead[i]] = raw;
        sensor.position[idToRead[i]] = 
		(raw-posZero[i]-actuator.bias[i])/scale[i] - 
		 actuator.offset[i];
      end
    end
  end

  --IMU reading

  -- local data=Dynamixel.read_data(200,38,12);
  -- local offset=1;

  -- -- for x = 0, #data do
  -- --   print(x,data[x])
  -- -- end
  -- if data and #data>11 then
  --   for i=1,3 do
  --     sensor.imuGyr[Config.gyro.rpy[i]] =
  --     Config.gyro.sensitivity[i]*
  --     (DynamixelPacket.byte_to_word(data[offset],data[offset+1])-gyrZero[i]);

  --     sensor.imuAcc[Config.acc.xyz[i]] = 
  --     Config.acc.sensitivity[i]*
  --     (DynamixelPacket.byte_to_word(data[offset+6],data[offset+7])-Config.acc.zero[i]);

  --     sensor.imuGyrRaw[Config.gyro.rpy[i]]=DynamixelPacket.byte_to_word(data[offset],data[offset+1]);
  --     sensor.imuAccRaw[Config.acc.xyz[i]]=DynamixelPacket.byte_to_word(data[offset+6],data[offset+7]);
  --     offset = offset + 2;
  --   end
  -- end

  --Button reading
  -- data=Dynamixel.read_data(200,30,1);
  -- if data then
  --   sensor.button[1]=math.floor(data[1]/2);
  --   sensor.button[2]=data[1]%2;
  -- end
end

function getLowByte16( value )
  local high_value = math.floor(value/256)
  high_value = high_value*256
  local low_value = value - high_value
  return low_value
end

function getHighByte16( value )
  local high_value = math.floor(value/256)
  return high_value
end

function imu_rpy(id,address,len)
  local data = 0
  local dataFix = 0
  local b = 0
  data = Dynamixel.read_data(id,address,len);
  if data ~= nil then
    dataFix = DynamixelPacket.byte_to_word(data[1],data[2])
    b = string.format("%X", getLowByte16(dataFix))
    if (dataFix ~= 0 and dataFix > 180) then 
      dataFix = tonumber(b,16)-255
    else
      -- print(dataFix)
    end
  end
  return dataFix
end

function gyr_rpy(id,address,len)
  local data = 0
  local dataFix = 0
  local temp = 0
  local b = 0
  data = Dynamixel.read_data(id,address,len);
  if data ~= nil then
    dataFix = DynamixelPacket.byte_to_word(data[1],data[2])
    b = string.format("%X", getLowByte16(dataFix))
    temp = (dataFix/64);
    if (temp ~= 0 and temp > 180) then
      temp = temp - 1023;
    end
  end
  return temp;
end

--function update_imu()
--[[   t=unix.time();
  if tLast==0 then tLast=t; end
  tPassed=t-tLast;z
  tLast=t;

  iAngle=vector.new({sensor.imuAngle[1],sensor.imuAngle[2],sensor.imuAngle[3]});
  gyrDelta = vector.new({sensor.imuGyr[1],sensor.imuGyr[2],sensor.imuGyr[3]})
  *math.pi/180 * tPassed; --dps to rps conversion

  --Angle transformation: yaw -> pitch -> roll
  local tTrans=Transform.rotZ(iAngle[3]);
  tTrans=tTrans*Transform.rotY(iAngle[2]);
  tTrans=tTrans*Transform.rotX(iAngle[1]);

  local tTransDelta=Transform.rotZ(gyrDelta[3]);
  tTransDelta=tTransDelta*Transform.rotY(gyrDelta[2]);
  tTransDelta=tTransDelta*Transform.rotX(gyrDelta[1]);
  tTrans=tTrans*tTransDelta;
  iAngle=Transform.getRPY(tTrans);

  local accMag=sensor.imuAcc[1]^2+sensor.imuAcc[2]^2+sensor.imuAcc[3]^2;
  --print("AccMag:",accMag)
  if accMag>Config.angle.gMin and accMag<Config.angle.gMax then
    local angR=math.atan2(-sensor.imuAcc[2], 
    math.sqrt(sensor.imuAcc[1]^2+sensor.imuAcc[3]^2) );
    local angP=math.atan2(sensor.imuAcc[1], 
    math.sqrt(sensor.imuAcc[2]^2+sensor.imuAcc[3]^2) );
    iAngle[1], iAngle[2] =
    (1-Config.angle.accFactor)*iAngle[1]+Config.angle.accFactor*angR,
    (1-Config.angle.accFactor)*iAngle[2]+Config.angle.accFactor*angP; ]]
  -- end
  --Roll = imu_rpy(200,44,4) * -1
  --Pitch = imu_rpy(200,46,4) * -1
  --Yaw = imu_rpy(200,48,4) * -1
  -- print(Yaw)
  --gyrX = gyr_rpy(200,32,4) * -1 --Config.gyro.sensitivity[1] * ((gyr_rpy(200,32,4) * -1)-gyrZero[1])
  --gyrY = gyr_rpy(200,34,4) * -1 --Config.gyro.sensitivity[2] * ((gyr_rpy(200,34,4) * -1)-gyrZero[2])
  --gyrZ = gyr_rpy(200,36,4) * -1 --Config.gyro.sensitivity[3] * ((gyr_rpy(200,36,4) * -1)-gyrZero[3])
  --sensor.imuAngle[1],sensor.imuAngle[2],sensor.imuAngle[3] = Roll,Pitch,Yaw;
  --sensor.imuGyr[1],sensor.imuGyr[2],sensor.imuGyr[3] = gyrX,gyrY,gyrZ;

function update_imu()
  -- Nama shared memory yang sama dengan yang ditulis oleh program Python
  local shm_rpy = "/dev/shm/imu_bno"
  local shm_gyr = "/dev/shm/gyr_bno"
  local data_size = 3 * 8  -- Ukuran data (3 float64, setiap float64 memiliki ukuran 8 byte)

  -- Loop untuk membaca data dari shared memory
  --while true do
  -- Buka shared memory
  local file1 = io.open(shm_rpy, "rb")
  local file2 = io.open(shm_gyr, "rb")
  if not file1 then
      print("Failed to open shared memory")
      success = false
  else
      -- Baca data dari file
      local memory_data1 = file1:read(data_size)
      local memory_data2 = file2:read(data_size)
      file1:close()
      file2:close()
      --print("Raw Data:", memory_data)

      -- Pastikan data yang dibaca memiliki panjang yang sesuai
      if not memory_data1 or not memory_data2 or #memory_data1 ~= data_size or #memory_data2 ~= data_size then
         --print("Failed to read valid data from shared memory")
         --success = false
      else
         -- Unpack data from binary string only if there are no nil values
         local Roll, Pitch, Yaw = struct.unpack('ddd', memory_data1)
         local gyrX, gyrY, gyrZ = struct.unpack('ddd', memory_data2)
         
         sensor.imuAngle[1],sensor.imuAngle[2],sensor.imuAngle[3] = Roll,Pitch,Yaw;
         sensor.imuGyr[1],sensor.imuGyr[2],sensor.imuGyr[3] = gyrX,gyrY,gyrZ;

         -- Display IMU data only if there are no nil values
         if Roll ~= nil and Pitch ~= nil and Yaw ~= nil and gyrX ~= nil and gyrY ~= nil and gyrZ ~= nil then
             --print(string.format("Data IMU: Roll=%.2f, Pitch=%.2f, Yaw=%.2f", Roll, Pitch, Yaw))
             --print(string.format("Data GYRO: Gyr_x=%.2f, Gyr_y=%.2f, Gyr_z=%.2f", gyrX, gyrY, gyrZ))
         else
             print("Received nil values. Skipping unpack.")
         end
      end

  end
  --end

  local strategy = 0
  strategy = Dynamixel.read_data(200,51,4)
    if (strategy ~= nil) then
      if (strategy[2]==1) then
        torqueTrigger=0;
      else
        torqueTrigger=0;
      end
      sensor.strategy[1],sensor.strategy[2] = strategy[1],strategy[2];
    elseif (strategy == nil) then
      torqueTrigger=0;
    end

  local voltage = 0
  voltage = Dynamixel.read_data(200,31,4)
  if voltage ~= nil then
    sensor.voltage[1] = voltage[1]+5
  end

  sensor.kneeCurrent[1], sensor.kneeCurrent[2] = gyr_rpy(14,126,4), gyr_rpy(13,126,4); 
end

function update()
  -- Dynamixel.ping_probe();

  -- Grab all tty's and see if different than
  -- the previous tty (Felix's USB bug)
  if( USB_bug == 1 ) then
    local ttyname = 'none'
    local ttys = unix.readdir("/dev");
    for i=1,#ttys do
      if (string.find(ttys[i], "tty.usb") or
        string.find(ttys[i], "ttyUSB")) then
        ttyname = "/dev/"..ttys[i];
        break;
      end 
    end
    if( ttyname ~= Dynamixel.dttyname ) then
      if( Dynamixel.dttyname ~= nil ) then
        print('bug detected! Found '..ttyname..', but connected original is: '.. Dynamixel.dttyname);
        -- Close, then wait .2 seconds, then try opening again
        Dynamixel.close();
        unix.sleep( 200000 ); -- .2 seconds
        Dynamixel.open();
      end
    end
  end

  bulk_read();

  nonsync_read();
  update_imu();

  count=count+1;
  sensor.updatedCount[1]=count%100; --This count indicates whether DCM has processed current reading or not

  if actuator.battTest[1]==1 then --in test mode, refresh faster
    sync_battery();
  else
    if count%100==0 then sync_battery();end
  end
  if actuator.hardnessChanged[1]==1 then
    sync_hardness();
    unix.usleep(100);
    actuator.hardnessChanged[1]=0;
  end
  if actuator.gainChanged[1]==1 then
    sync_gain();
    actuator.gainChanged[1]=0;
    unix.usleep(100);
  end
  if actuator.velocityChanged[1]==1 then
    sync_velocity();
    actuator.velocityChanged[1]=0;
    unix.usleep(100);
  end
  if actuator.torqueEnableChanged[1]==1 then
    torque_enable();
    actuator.torqueEnableChanged[1]=0;
    unix.usleep(100);
  end
  -- torque_enable(1);
  -- sync_gain();
  if (torqueTrigger==1) then
    torque_enable(0);
  else
    unix.usleep(2000);
    torque_enable(1);
    -- print("aaaa");
  end
  -- print("bbbb");
  sync_hardness();
  sync_command();
  unix.usleep(100);

  sync_led();

  --[[ print(
    string.format("IMU Acc: %.2f %.2f %.2f ",unpack(sensorShm:get('imuAcc')))..
    string.format("Gyr: %.1f %.1f %.1f ",unpack(sensorShm:get('imuGyr')))..
    string.format("Angle: %.1f %.1f %.1f ",unpack(sensorShm:get('imuAngle')))..
    string.format("Strategy: %d %d ",unpack(sensorShm:get('strategy')))
    ) ]]
end

function exit()
  Dynamixel.close();
end

