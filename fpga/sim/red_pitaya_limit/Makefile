PATH_TBN=../../tbn
PATH_RTL=../../rtl
PATH_OUT=xsim.dir/work

.PHONY: clean show

limit_tb.vcd: $(PATH_OUT)/limit_tb.sdb $(PATH_OUT)/sys_bus_model.sdb $(PATH_OUT)/red_pitaya_limit.sdb $(PATH_OUT)/red_pitaya_limit_block.sdb
	xelab --debug typical --snapshot limit_tb work.limit_tb
	xsim limit_tb --runall

$(PATH_OUT)/limit_tb.sdb: $(PATH_TBN)/red_pitaya_limit_tb.sv
	xvlog -sv $<

$(PATH_OUT)/red_pitaya_limit.sdb: $(PATH_RTL)/classic/red_pitaya_limit.v
	xvlog $<

$(PATH_OUT)/red_pitaya_limit_block.sdb: $(PATH_RTL)/classic/red_pitaya_limit_block.v
	xvlog $<

$(PATH_OUT)/sys_bus_model.sdb: $(PATH_TBN)/sys_bus_model_old.sv
	xvlog -sv $<

show: limit_tb.vcd
	gtkwave wave_config.gtkw

clean:
	rm -rf xsim.dir limit_tb.vcd *.pb *.log *.jou *.wdb *.str
