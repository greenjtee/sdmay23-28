# SPDX-FileCopyrightText: 2020 Efabless Corporation
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#      http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
# SPDX-License-Identifier: Apache-2.0

set ::env(PDK) "sky130A"
set ::env(STD_CELL_LIBRARY) "sky130_fd_sc_hd"

set script_dir [file dirname [file normalize [info script]]]

set ::env(DESIGN_NAME) user_adder

set ::env(VERILOG_FILES) "\
	$::env(CARAVEL_ROOT)/verilog/rtl/defines.v \
	$script_dir/../../verilog/rtl/user_adder.v"

set ::env(DESIGN_IS_CORE) 0

set ::env(CLOCK_PORT) "wb_clk_i"
set ::env(CLOCK_NET) "add_sub_accum.clk"
set ::env(CLOCK_PERIOD) "10"

# always got: "There are hold violations in the design at the typical corner" when FP_SIZING was absolute... 
# no matter what PL or GLB parameters I set. tried increasing both HOLD_MAX_BUFFER_PERCENT and HOLD_SLACK_MARGIN to 80% and 0.3ns
set ::env(FP_SIZING) relative
# max area in wrapper: 0 0 2920 3520
set ::env(DIE_AREA) "0 0 900 600"

set ::env(FP_PIN_ORDER_CFG) $script_dir/pin_order.cfg

set ::env(PL_BASIC_PLACEMENT) 0
set ::env(PL_TARGET_DENSITY) 0.6
set ::env(FP_CORE_UTIL) 5
# with 5%: area = 200352 u^2 @ 98% utilization
# with 10%: ?
# Not sure how FP_SIZING absolute and relative works excatly and how DIE_AREA affects the overall size and constraints

# set ::env(PL_RANDOM_GLB_PLACEMENT) 0
# set ::env(PL_RESIZER_ALLOW_SETUP_VIOS) 1
# set ::env(GLB_RESIZER_ALLOW_SETUP_VIOS) 1

# set ::env(PL_RESIZER_HOLD_MAX_BUFFER_PERCENT) 80
# set ::env(GLB_RESIZER_HOLD_MAX_BUFFER_PERCENT) 80
# set ::env(PL_RESIZER_HOLD_SLACK_MARGIN) 0.1ns
# set ::env(GLB_RESIZER_HOLD_SLACK_MARGIN) 0.1ns

# set ::nev(PL_RESIZER_SETUP_MAX_BUFFER_PERCENT) 80
# set ::nev(GLB_RESIZER_SETUP_MAX_BUFFER_PERCENT) 80
# set ::env(PL_RESIZER_SETUP_SLACK_MARGIN) 0.05ns
# set ::env(GLB_RESIZER_SETUP_SLACK_MARGIN) 0.05ns

# set ::anv(CTS_TARGET_SKEW) 200

# Maximum layer used for routing is metal 4.
# This is because this macro will be inserted in a top level (user_project_wrapper) 
# where the PDN is planned on metal 5. So, to avoid having shorts between routes
# in this macro and the top level metal 5 stripes, we have to restrict routes to metal4.  
# 
# set ::env(GLB_RT_MAXLAYER) 5

set ::env(RT_MAX_LAYER) {met4}

# You can draw more power domains if you need to 
set ::env(VDD_NETS) [list {vccd1}]
set ::env(GND_NETS) [list {vssd1}]

set ::env(DIODE_INSERTION_STRATEGY) 4 
# If you're going to use multiple power domains, then disable cvc run.
set ::env(RUN_CVC) 1
