--[[
 fast tuning of VTOL gains for multirotors and quadplanes

 This should be used in QLOITER mode for quadplanes and LOITER mode
 for copters, although it will work in other VTOL modes

--]]

local PARAM_TABLE_KEY = 8
local PARAM_TABLE_PREFIX = "QUIK_"

local is_quadplane = false
local atc_prefix = "ATC"

-- bind a parameter to a variable
function bind_param(name)
   local p = Parameter()
   assert(p:init(name), string.format('could not find %s parameter', name))
   return p
end

-- add a parameter and bind it to a variable
function bind_add_param(name, idx, default_value)
   assert(param:add_param(PARAM_TABLE_KEY, idx, name, default_value), string.format('could not add param %s', name))
   return bind_param(PARAM_TABLE_PREFIX .. name)
end

-- setup quicktune specific parameters
assert(param:add_table(PARAM_TABLE_KEY, PARAM_TABLE_PREFIX, 7), 'could not add param table')

local QUIK_ENABLE      = bind_add_param('ENABLE',         1, 0)
local QUIK_AXES        = bind_add_param('AXES',           2, 7)
local QUIK_DOUBLE_TIME = bind_add_param('DOUBLE_TIME',    3, 10)
local QUIK_GAIN_MARGIN = bind_add_param('GAIN_MARGIN',    4, 70)
local QUIK_OSC_SMAX    = bind_add_param('OSC_SMAX',       5, 5)
local QUIK_YAW_P_MAX   = bind_add_param('YAW_P_MAX',      6, 0.5)
local QUIK_YAW_D_MAX   = bind_add_param('YAW_D_MAX',      7, 0.01)

local INS_GYRO_FILTER  = bind_param("INS_GYRO_FILTER")

local UPDATE_RATE_HZ = 40
local STAGE_DELAY = 4.0

local YAW_FLTE_MAX = 2.0
local FLTD_MUL = 0.5
local FLTT_MUL = 0.5

-- SMAX to set if none set at start
local DEFAULT_SMAX = 50.0

-- work out vehicle type
if param:get("Q_A_RAT_RLL_SMAX") then
   is_quadplane = true
   atc_prefix = "Q_A"
   gcs:send_text(0, "Quicktune for quadplane loaded")
elseif param:get("ATC_RAT_RLL_SMAX") then
   is_quadplane = false
   gcs:send_text(0, "Quicktune for multicopter loaded")
else
   gcs:send_text(0, "Quicktune unknown vehicle")
   return
end

-- get time in sections since boot
function get_time()
   return millis():tofloat() * 0.001
end

local axis_names = { "RLL", "PIT", "YAW" }
local param_suffixes = { "FF", "P", "I", "D", "SMAX", "FLTT", "FLTD", "FLTE" }
local stages = { "D", "P" }
local stage = stages[1]
local last_stage_change = get_time()
local last_gain_report = get_time()
local slew_parm = nil
local slew_target = 0
local slew_delta = 0

local axes_done = {}


-- create params dictionary indexed by name, such as "RLL_P"
local params = {}
local param_saved = {}
local param_changed = {}
local need_restore = false

for i, axis in ipairs(axis_names) do
   for j, suffix in ipairs(param_suffixes) do
      local pname = axis .. "_" .. suffix
      params[pname] = bind_param(atc_prefix .. "_" .. "RAT_" .. pname)
      param_changed[pname] = false
   end
end

-- reset to start
function reset_axes_done()
   for i, axis in ipairs(axis_names) do
      axes_done[axis] = false
   end
   stage = stages[1]
end

-- get all current param values into param_saved dictionary
function get_all_params()
   for pname in pairs(params) do
      param_saved[pname] = params[pname]:get()
   end
end

-- restore all param values from param_saved dictionary
function restore_all_params()
   for pname in pairs(params) do
      local changed = param_changed[pname] and 1 or 0
      if param_changed[pname] then
         params[pname]:set(param_saved[pname])
         param_changed[pname] = false
      end
   end
end

-- save all param values to storage
function save_all_params()
   for pname in pairs(params) do
      if param_changed[pname] then
         params[pname]:set_and_save(params[pname]:get())
         param_saved[pname] = params[pname]:get()
         param_changed[pname] = false
      end
   end
end

-- setup a default SMAX if zero
function setup_SMAX()
   for i, axis in ipairs(axis_names) do
      local smax = axis .. "_SMAX"
      if params[smax]:get() <= 0 then
         adjust_gain(smax, DEFAULT_SMAX)
      end
   end
end

-- setup filter frequencies
function setup_filters()
   for i, axis in ipairs(axis_names) do
      local fltd = axis .. "_FLTD"
      local fltt = axis .. "_FLTT"
      local flte = axis .. "_FLTE"
      adjust_gain(fltt, INS_GYRO_FILTER:get() * FLTT_MUL)
      adjust_gain(fltd, INS_GYRO_FILTER:get() * FLTD_MUL)
      if axis == "YAW" then
         local FLTE = params[flte]
         if FLTE:get() <= 0.0 or FLTE:get() > YAW_FLTE_MAX then
            adjust_gain(flte, YAW_FLTE_MAX)
         end
      end
   end
end

reset_axes_done()
get_all_params()

-- 3 position switch
local quick_switch = nil

-- get the axis name we are working on, or nil for all done
function get_current_axis()
   local axes = QUIK_AXES:get()
   for i = 1, #axis_names do
      local mask = (1 << (i-1))
      local axis_name = axis_names[i]
      if (mask & axes) ~= 0 and axes_done[axis_name] == false then
         return axis_names[i]
      end
   end
   return nil
end

-- get slew rate for an axis
function get_slew_rate(axis)
   local roll_srate, pitch_srate, yaw_srate = AC_AttitudeControl:get_rpy_srate()
   if axis == "RLL" then
      return roll_srate
   end
   if axis == "PIT" then
      return pitch_srate
   end
   if axis == "YAW" then
      return yaw_srate
   end
   return 0.0
end

-- move to next stage of tune
function advance_stage(axis)
   if stage == "D" then
      stage = "P"
   else
      axes_done[axis] = true
      stage = "D"
   end
end

-- get param name, such as RLL_P, used as index into param dictionaries
function get_pname(axis, stage)
   return axis .. "_" .. stage
end

-- change a gain
function adjust_gain(pname, value)
   local P = params[pname]
   need_restore = true
   param_changed[pname] = true
   P:set(value)
   if string.sub(pname, -2) == "_P" then
      -- also change I gain
      local iname = string.gsub(pname, "_P", "_I")
      local ffname = string.gsub(pname, "_P", "_FF")
      local I = params[iname]
      local FF = params[ffname]
      if FF:get() > 0 then
         -- if we have any FF on an axis then we don't couple I to P,
         -- usually we want I = FF for a one sectond time constant for trim
         return
      end
      param_changed[iname] = true
      if string.sub(pname, 1, 3) == "YAW" then
         -- for yaw, I is 1/10 of P
         I:set(value*0.1)
      else
         -- otherwise I == P
         I:set(value)
      end
   end
end

-- return gain multipler for one loop
function get_gain_mul()
   return math.exp(math.log(2.0)/(UPDATE_RATE_HZ*QUIK_DOUBLE_TIME:get()))
end

function setup_slew_gain(pname, gain)
   slew_parm = pname
   slew_target = gain
   slew_steps = UPDATE_RATE_HZ/2
   slew_delta = (gain - params[pname]:get()) / slew_steps
end

function update_slew_gain()
   if slew_parm ~= nil then
      local P = params[slew_parm]
      local axis = string.sub(slew_parm, 1, 3)
      local ax_stage = string.sub(slew_parm, -1)
      adjust_gain(slew_parm, P:get()+slew_delta)
      slew_steps = slew_steps - 1
      logger.write('QUIK','SRate,Gain,Param', 'ffn', get_slew_rate(axis), P:get(), axis .. ax_stage)
      if slew_steps == 0 then
         gcs:send_text(0, string.format("%s %.4f", slew_parm, P:get()))
         slew_parm = nil
      end
   end
end

-- return gain limits on a parameter, or 0 for no limit
function gain_limit(pname)
   if string.sub(pname, 1, 3) == "YAW" then
      local suffix = string.sub(pname, -2)
      if suffix == "_P" then
         return QUIK_YAW_P_MAX:get()
      end
      if suffix == "_D" then
         return QUIK_YAW_D_MAX:get()
      end
   end
   return 0.0
end

function reached_limit(pname, gain, srate)
   if srate > QUIK_OSC_SMAX:get() then
      return true
   end
   local limit = gain_limit(pname)
   if limit > 0.0 and gain >= limit then
      return true
   end
   return false
end

-- main update function
function update()
   if quick_switch == nil then
      quick_switch = rc:find_channel_for_option(300)
   end
   if quick_switch == nil or QUIK_ENABLE:get() < 1 then
      return
   end
   local sw_pos = quick_switch:get_aux_switch_pos()
   if sw_pos == 0 or not arming:is_armed() or not vehicle:get_likely_flying() then
      -- abort, revert parameters
      if need_restore then
         need_restore = false
         restore_all_params()
         gcs:send_text(0, string.format("Tuning: reverted"))
      end
      reset_axes_done()
      return
   end
   if sw_pos == 2 then
      -- save all params
      if need_restore then
         need_restore = false
         save_all_params()
         gcs:send_text(0, string.format("Tuning: saved"))
      end
   end
   if sw_pos ~= 1 then
      return
   end

   if get_time() - last_stage_change < STAGE_DELAY then
      update_slew_gain()
      return
   end

   axis = get_current_axis()
   if axis == nil then
      -- nothing left to do
      return
   end

   if not need_restore then
      -- we are just starting tuning, get current values
      gcs:send_text(0, string.format("Tuning: starting tune"))
      get_all_params()
      setup_SMAX()
      setup_filters()
   end

   local srate = get_slew_rate(axis)
   local pname = get_pname(axis, stage)
   local P = params[pname]
   if reached_limit(pname, P:get(), srate) then
      local reduction = (100.0-QUIK_GAIN_MARGIN:get())*0.01
      local new_gain = P:get() * reduction
      local limit = gain_limit(pname)
      if limit > 0.0 and new_gain > limit then
         new_gain = limit
      end
      setup_slew_gain(pname, new_gain)
      logger.write('QUIK','SRate,Gain,Param', 'ffn', srate, P:get(), axis .. stage)
      gcs:send_text(0, string.format("Tuning: %s done", pname))
      advance_stage(axis)
      last_stage_change = get_time()
      if get_current_axis() == nil then
         gcs:send_text(0, string.format("Tuning: DONE"))
      end
   else
      local new_gain = P:get()*get_gain_mul()
      if new_gain <= 0.0001 then
         new_gain = 0.001
      end
      adjust_gain(pname, new_gain)
      logger.write('QUIK','SRate,Gain,Param', 'ffn', srate, P:get(), axis .. stage)
      if get_time() - last_gain_report > 3 then
         last_gain_report = get_time()
         gcs:send_text(0, string.format("%s %.4f sr:%.2f", pname, new_gain, srate))
      end
   end
end

-- wrapper around update(). This calls update() at 10Hz,
-- and if update faults then an error is displayed, but the script is not
-- stopped
function protected_wrapper()
  local success, err = pcall(update)
  if not success then
     gcs:send_text(0, "Internal Error: " .. err)
     -- when we fault we run the update function again after 1s, slowing it
     -- down a bit so we don't flood the console with errors
     --return protected_wrapper, 1000
     return
  end
  return protected_wrapper, 1000/UPDATE_RATE_HZ
end

-- start running update loop
return protected_wrapper()
