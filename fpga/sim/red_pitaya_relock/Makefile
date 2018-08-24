PATH_TBN=../../tbn
PATH_RTL=../../rtl
PATH_OUT=xsim.dir/work

.PHONY: clean show

relock_tb.vcd: $(PATH_OUT)/relock_tb.sdb $(PATH_OUT)/pid_relock.sdb $(PATH_OUT)/red_pitaya_limit.sdb $(PATH_OUT)/red_pitaya_limit_block.sdb
	xelab --debug typical --snapshot relock_tb work.relock_tb
	xsim relock_tb --runall

$(PATH_OUT)/relock_tb.sdb: $(PATH_TBN)/relock_tb.sv
	xvlog -sv $<

$(PATH_OUT)/pid_relock.sdb: $(PATH_RTL)/classic/pid_relock.v
	xvlog $<

$(PATH_OUT)/red_pitaya_limit.sdb: $(PATH_RTL)/classic/red_pitaya_limit.v
	xvlog $<

$(PATH_OUT)/red_pitaya_limit_block.sdb: $(PATH_RTL)/classic/red_pitaya_limit_block.v
	xvlog $<

show: relock_tb.vcd
	gtkwave wave_config.gtkw

clean:
	rm -rf xsim.dir relock_tb.vcd *.pb *.log *.jou *.wdb *.str
