-- This script loads the 6DoF mixer matrix for a 6 motor frame
-- https://youtu.be/QUDhnYvH66k

Motors_6DoF:add_motor(0,  0.000000,  0.000000,  0.000000,  0.000000,  1.000000,  0.000000, false, 1)
Motors_6DoF:add_motor(1,  0.000000, -0.000000,  0.000000,  0.000000, -0.000000,  1.000000, false, 2)
Motors_6DoF:add_motor(2,  0.000000, -0.000000, -0.000000,  0.000000, -1.000000, -0.000000, false, 3)
Motors_6DoF:add_motor(3,  0.000000, -0.000000, -0.000000,  0.000000,  0.000000, -1.000000, false, 4)
--[[ Motors_6DoF:add_motor(4,  0.000000, -0.000000,  0.000000, -0.000000,  0.010753, -1.013924, true, 5)
Motors_6DoF:add_motor(5,  0.000000, -0.000000,  0.000000, -0.000000,  0.811593,  0.431718, true, 6)
JV: Comment out motor 5 and 6 to test only lateral movement first and changed init(6) to init(4) motors
]]
assert(Motors_6DoF:init(4),'unable to setup 4 motors')

--[[JV: I added the line below since it has been add on Github's version of the script ]]
motors:set_frame_string("6DoF example")
