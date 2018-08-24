PATH_TBN=../../tbn
PATH_RTL=../../rtl
PATH_OUT=xsim.dir/work

.PHONY: clean show

pid_tb.vcd: $(PATH_OUT)/pid_tb.sdb $(PATH_OUT)/sys_bus_model.sdb $(PATH_OUT)/red_pitaya_pid.sdb $(PATH_OUT)/red_pitaya_pid_block.sdb $(PATH_OUT)/pid_relock.sdb $(PATH_OUT)/red_pitaya_limit.sdb $(PATH_OUT)/red_pitaya_limit_block.sdb
	xelab --debug typical --snapshot pid_tb work.pid_tb
	xsim pid_tb --runall

$(PATH_OUT)/pid_tb.sdb: $(PATH_TBN)/red_pitaya_pid_tb.sv
	xvlog -sv $<

$(PATH_OUT)/red_pitaya_pid.sdb: $(PATH_RTL)/classic/red_pitaya_pid.v
	xvlog $<

$(PATH_OUT)/red_pitaya_pid_block.sdb: $(PATH_RTL)/classic/red_pitaya_pid_block.v
	xvlog $<

$(PATH_OUT)/pid_relock.sdb: $(PATH_RTL)/classic/pid_relock.v
	xvlog $<

$(PATH_OUT)/sys_bus_model.sdb: $(PATH_TBN)/sys_bus_model_old.sv
	xvlog -sv $<

$(PATH_OUT)/red_pitaya_limit.sdb: $(PATH_RTL)/classic/red_pitaya_limit.v
	xvlog $<

$(PATH_OUT)/red_pitaya_limit_block.sdb: $(PATH_RTL)/classic/red_pitaya_limit_block.v
	xvlog $<

show: pid_tb.vcd
	gtkwave wave_config.gtkw

clean:
	rm -rf xsim.dir pid_tb.vcd *.pb *.log *.jou *.wdb *.str
