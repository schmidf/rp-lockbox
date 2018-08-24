PATH_TBN=../../tbn
PATH_RTL=../../rtl
PATH_OUT=xsim.dir/work

.PHONY: clean show

limit_block_tb.vcd: $(PATH_OUT)/red_pitaya_limit_block.sdb $(PATH_OUT)/testbench.sdb
	xelab --debug typical --snapshot testbench work.testbench
	xsim testbench --runall

$(PATH_OUT)/testbench.sdb: $(PATH_TBN)/red_pitaya_limit_block_tb.v
	xvlog $<

$(PATH_OUT)/red_pitaya_limit_block.sdb: $(PATH_RTL)/classic/red_pitaya_limit_block.v
	xvlog $<

show: limit_block_tb.vcd
	gtkwave wave_config.gtkw

clean:
	rm -rf xsim.dir limit_block_tb.vcd *.pb *.log *.jou *.wdb *.str
