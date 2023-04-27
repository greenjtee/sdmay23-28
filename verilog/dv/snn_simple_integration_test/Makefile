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
#
# SPDX-License-Identifier: Apache-2.0

SRAM := ../../../dependencies/pdks/sky130A/libs.ref/sky130_sram_macros/verilog/sky130_sram_1kbyte_1rw1r_8x1024_8.v

default: test

test: snn_simple_integration_test_tb.v $(SRAM) ../../rtl/queue.v ../../rtl/neuron.v ../../rtl/snn.v ../../rtl/rand_gen.v
	iverilog -o snn_simple_integration_test snn_simple_integration_test_tb.v ../../rtl/queue.v ../../rtl/neuron.v  ../../rtl/snn.v ../../rtl/rand_gen.v $(SRAM)
	vvp snn_simple_integration_test
