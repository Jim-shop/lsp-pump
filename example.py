from lsp import LSP

pump = LSP("COM3", LSP.BAUD._9600, LSP.ADDR._1)
pump.set_syringe_spec(14.37)
pump.set_mode(
    LSP.MODE.PUSH,
    0xe8,
    LSP.VOLUME_UNIT._0_01ML,
    0xe9,
    LSP.SPEED_UNIT._1UL_MIN,
)
print(pump.read_mode())
pump.start()
