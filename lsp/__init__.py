from enum import IntEnum, Enum
from typing import overload, Literal, Tuple
from functools import reduce
from time import sleep
import serial
import serial.tools.list_ports


class LSP:
    class BAUD(IntEnum):
        _9600 = 9600
        _2400 = 2400
        _1200 = 1200

    class ADDR(IntEnum):
        _1 = 1
        _2 = 2
        _3 = 3
        _4 = 4
        _5 = 5
        _6 = 6
        _7 = 7
        _8 = 8
        _9 = 9
        _10 = 10
        _11 = 11
        _12 = 12
        _13 = 13
        _14 = 14
        _15 = 15
        _16 = 16
        _17 = 17
        _18 = 18
        _19 = 19
        _20 = 20
        _21 = 21
        _22 = 22
        _23 = 23
        _24 = 24
        _25 = 25
        _26 = 26
        _27 = 27
        _28 = 28
        _29 = 29
        _30 = 30
        _31 = 31

    class SYRINGE(Enum):
        AIR_TITE_1ML = (b"A", 1)
        AIR_TITE_2_5ML = (b"A", 2)
        AIR_TITE_5ML = (b"A", 3)
        AIR_TITE_10ML = (b"A", 4)
        AIR_TITE_20ML = (b"A", 5)
        AIR_TITE_30ML = (b"A", 6)
        AIR_TITE_50ML = (b"A", 7)

        BECTON_DICKSON_1ML_PLASTIC = (b"B", 1)
        BECTON_DICKSON_3ML_PLASTIC = (b"B", 2)
        BECTON_DICKSON_5ML_PLASTIC = (b"B", 3)
        BECTON_DICKSON_10ML_PLASTIC = (b"B", 4)
        BECTON_DICKSON_20ML_PLASTIC = (b"B", 5)
        BECTON_DICKSON_30ML_PLASTIC = (b"B", 6)
        BECTON_DICKSON_60ML_PLASTIC = (b"B", 7)

        BECTON_DICKSON_0_5ML_GLASS = (b"C", 1)
        BECTON_DICKSON_1ML_GLASS = (b"C", 2)
        BECTON_DICKSON_2_5ML_GLASS = (b"C", 3)
        BECTON_DICKSON_5ML_GLASS = (b"C", 4)
        BECTON_DICKSON_10ML_GLASS = (b"C", 5)
        BECTON_DICKSON_20ML_GLASS = (b"C", 6)
        BECTON_DICKSON_30ML_GLASS = (b"C", 7)
        BECTON_DICKSON_60ML_GLASS = (b"C", 8)

        HAMILTON_10UL = (b"H", 1)
        HAMILTON_25UL = (b"H", 2)
        HAMILTON_50UL = (b"H", 3)
        HAMILTON_100UL = (b"H", 4)
        HAMILTON_250UL = (b"H", 5)
        HAMILTON_500UL = (b"H", 6)
        HAMILTON_1ML = (b"H", 7)
        HAMILTON_2_5ML = (b"H", 8)
        HAMILTON_5ML = (b"H", 9)
        HAMILTON_10ML = (b"H", 10)
        HAMILTON_25ML = (b"H", 11)
        HAMILTON_50ML = (b"H", 12)

        POPPER_AND_SONS_0_25ML = (b"P", 1)
        POPPER_AND_SONS_0_5ML = (b"P", 2)
        POPPER_AND_SONS_1ML = (b"P", 3)
        POPPER_AND_SONS_2ML = (b"P", 4)
        POPPER_AND_SONS_3ML = (b"P", 5)
        POPPER_AND_SONS_5ML = (b"P", 6)
        POPPER_AND_SONS_10ML = (b"P", 7)
        POPPER_AND_SONS_20ML = (b"P", 8)
        POPPER_AND_SONS_30ML = (b"P", 9)
        POPPER_AND_SONS_50ML = (b"P", 10)

        RANFAC_2ML = (b"R", 1)
        RANFAC_5ML = (b"R", 2)
        RANFAC_10ML = (b"R", 3)
        RANFAC_20ML = (b"R", 4)
        RANFAC_30ML = (b"R", 5)
        RANFAC_50ML = (b"R", 6)

        SCIENTIFIC_GLASS_ENGINEERING_25UL = (b"S", 1)
        SCIENTIFIC_GLASS_ENGINEERING_50UL = (b"S", 2)
        SCIENTIFIC_GLASS_ENGINEERING_100UL = (b"S", 3)
        SCIENTIFIC_GLASS_ENGINEERING_250UL = (b"S", 4)
        SCIENTIFIC_GLASS_ENGINEERING_500UL = (b"S", 5)
        SCIENTIFIC_GLASS_ENGINEERING_1ML = (b"S", 6)
        SCIENTIFIC_GLASS_ENGINEERING_2_5ML = (b"S", 7)
        SCIENTIFIC_GLASS_ENGINEERING_5ML = (b"S", 8)
        SCIENTIFIC_GLASS_ENGINEERING_10ML = (b"S", 9)

        SHERWOOD_MONOJET_PLASTIC_1ML = (b"M", 1)
        SHERWOOD_MONOJET_PLASTIC_3ML = (b"M", 2)
        SHERWOOD_MONOJET_PLASTIC_6ML = (b"M", 3)
        SHERWOOD_MONOJET_PLASTIC_12ML = (b"M", 4)
        SHERWOOD_MONOJET_PLASTIC_20ML = (b"M", 5)
        SHERWOOD_MONOJET_PLASTIC_35ML = (b"M", 6)
        SHERWOOD_MONOJET_PLASTIC_50ML = (b"M", 7)

        TERUMO_1ML_PLASTIC = (b"T", 1)
        TERUMO_3ML_PLASTIC = (b"T", 2)
        TERUMO_5ML_PLASTIC = (b"T", 3)
        TERUMO_10ML_PLASTIC = (b"T", 4)
        TERUMO_20ML_PLASTIC = (b"T", 5)
        TERUMO_30ML_PLASTIC = (b"T", 6)
        TERUMO_60ML_PLASTIC = (b"T", 7)

        UNIMETRICS_10UL = (b"U", 1)
        UNIMETRICS_25UL = (b"U", 2)
        UNIMETRICS_50UL = (b"U", 3)
        UNIMETRICS_100UL = (b"U", 4)
        UNIMETRICS_250UL = (b"U", 5)
        UNIMETRICS_500UL = (b"U", 6)
        UNIMETRICS_1000UL = (b"U", 7)

    class MODE(IntEnum):
        PUSH = 1
        PULL = 2
        PUSH_PULL = 3
        PULL_PUSH = 4
        CONTINUE = 5

    class VOLUME_UNIT(IntEnum):
        _0_001UL = 1
        _0_01UL = 2
        _0_1UL = 3
        _1UL = 4
        _0_01ML = 5
        _0_1ML = 6
        _1ML = 7

    class SPEED_UNIT(IntEnum):
        _0_001UL_H = 1
        _0_01UL_H = 2
        _0_1UL_H = 3
        _1UL_H = 4
        _0_001UL_MIN = 5
        _0_01UL_MIN = 6
        _0_1UL_MIN = 7
        _1UL_MIN = 8
        _0_01ML_H = 9
        _0_1ML_H = 10
        _1ML_H = 11
        _0_01ML_MIN = 12
        _0_1ML_MIN = 13
        _1ML_MIN = 14

    class TIME_UNIT(IntEnum):
        _0_1S = 0
        _1S = 1

    class ACTION(IntEnum):
        STOP = 0
        START = 1
        PAUSE = 2

    class STATUS(IntEnum):
        STOPPED = 0
        RUNNING = 1
        PAUSING = 2

    class DIRECTION(IntEnum):
        PULL = 0
        PUSH = 1

    def __encode(self, data: bytes) -> bytes:
        flag = bytes.fromhex("e9")

        addr = self.addr.to_bytes(length=1)
        len_ = len(data).to_bytes(length=1)
        pdu = data
        fcs = reduce(lambda pre, cur: pre ^ cur, addr + len_ + pdu).to_bytes(length=1)

        replaced = (
            (addr + len_ + pdu + fcs)
            .replace(bytes.fromhex("e8"), bytes.fromhex("e800"))
            .replace(bytes.fromhex("e9"), bytes.fromhex("e801"))
        )
        return flag + replaced

    def __decode(self, data: bytes) -> bytes:
        flag = data[0]
        if flag != 0xE9:
            raise ValueError("帧头错误 flag != 0xE9")

        replaced = (
            (data)
            .replace(bytes.fromhex("e801"), bytes.fromhex("e9"))
            .replace(bytes.fromhex("e800"), bytes.fromhex("e8"))
        )
        addr = replaced[1]
        if addr != self.addr:
            raise ValueError("非本设备 addr != self.addr")
        len_ = replaced[2]
        if len(replaced) != 4 + len_:
            raise ValueError("长度错误 4 + len_ != len(replace(data))")
        pdu = replaced[3:-1]
        xor = reduce(lambda pre, cur: pre ^ cur, replaced[1:-1])
        fcs = replaced[-1]
        if xor != fcs:
            raise ValueError("数据校验出错 fcs != xor(addr + len_ + pdu)")

        return pdu

    def __command(self, instruction: bytes) -> str:
        encoded = self.__encode(instruction)
        print("send\t", encoded)
        self.serial.write(encoded)
        buf = self.serial.read(4)  # 小心 len 是 e8 或 e9
        len_ = buf[2] if buf[2] != 0xE8 else buf[2] + buf[3]
        buf += self.serial.read(len_)  # 总长度是 4 + len_ + 整个帧出现的 e8 次数（也包括len和fcs）
        buf += self.serial.read(buf.count(b"\xe8"))
        print("receive\t", buf)
        return self.__decode(buf)

    @overload
    def set_syringe_spec(self, syringe: SYRINGE) -> None:
        ...

    @overload
    def set_syringe_spec(self, diameter: float) -> None:
        ...

    def set_syringe_spec(self, syringe_or_diameter: SYRINGE | float) -> None:
        """设置注射器规格（单位mm）"""
        cmd = b"CWD"
        if type(syringe_or_diameter) == __class__.SYRINGE:
            syringe = syringe_or_diameter.value
            cmd += b"M" + syringe[0] + syringe[1].to_bytes(length=1)
        elif type(syringe_or_diameter) == float or type(syringe_or_diameter) == int:
            diameter = int(syringe_or_diameter * 100)
            if not 1 <= diameter <= 5000:
                raise ValueError("直径超出范围 0.01mm ~ 50.00mm")
            cmd += b"U" + diameter.to_bytes(length=2, byteorder="little")
        if self.__command(instruction=cmd) != b"Y":
            raise RuntimeError("意外回应")

    def read_syringe_spec(self) -> SYRINGE | float:
        """读取注射器规格"""
        resp = self.__command(instruction=b"CRD")
        if resp[0:2] != b"RD":
            raise RuntimeError("意外回复")
        if chr(resp[2]) == "M":  # 厂商
            syringe = (resp[3].to_bytes(length=1), resp[4])
            for i in __class__.SYRINGE:
                if i.value == syringe:
                    return i
            raise RuntimeError("意外的注射器型号")
        elif chr(resp[2]) == "U":  # 自定义
            return int.from_bytes(resp[3:], byteorder="little") / 100
        else:
            raise RuntimeError("意外回复")

    @overload
    def set_mode(
        self,
        mode: Literal[MODE.PUSH, MODE.PULL],
        volume: int,
        volume_unit: VOLUME_UNIT,
        speed: int,
        speed_unit: SPEED_UNIT,
    ) -> None:
        ...

    @overload
    def set_mode(
        self,
        mode: Literal[MODE.PUSH_PULL, MODE.PULL_PUSH],
        push_volume: int,
        push_volumn_unit: VOLUME_UNIT,
        pull_volume: int,
        pull_volumn_unit: VOLUME_UNIT,
        time_gap: int,
        time_gap_unit: TIME_UNIT,
        push_speed: int,
        push_speed_unit: SPEED_UNIT,
        pull_speed: int,
        pull_speed_unit: SPEED_UNIT,
    ) -> None:
        ...

    @overload
    def set_mode(
        self,
        mode: Literal[MODE.CONTINUE],
        volume: int,
        volume_unit: VOLUME_UNIT,
        delay_before_pull: int,
        delay_before_pull_unit: TIME_UNIT,
        delay_before_push: int,
        delay_before_push_unit: TIME_UNIT,
        push_speed: int,
        push_speed_unit: SPEED_UNIT,
        pull_speed: int,
        pull_speed_unit: SPEED_UNIT,
    ) -> None:
        ...

    def set_mode(self, mode: MODE, *args, **kvargs) -> None:
        """设置工作模式"""

        def set_push_or_pull_mode_cmd(
            volume: int,
            volume_unit: __class__.VOLUME_UNIT,
            speed: int,
            speed_unit: __class__.SPEED_UNIT,
        ) -> bytes:
            if not 0 <= volume <= 9999:
                raise ValueError("分配液量不在范围 0 ~ 9999 中")
            _volume = volume.to_bytes(length=2, byteorder="little")
            _volume_unit = volume_unit.to_bytes(length=1)
            if not 1 <= speed <= 9999:
                raise ValueError("速度不在范围 1 ~ 9999 中")
            _speed = speed.to_bytes(length=2, byteorder="little")
            _speed_unit = speed_unit.to_bytes(length=1)
            return _volume + _volume_unit + _speed + _speed_unit

        def set_push_and_pull_mode_cmd(
            push_volume: int,
            push_volumn_unit: __class__.VOLUME_UNIT,
            pull_volume: int,
            pull_volumn_unit: __class__.VOLUME_UNIT,
            time_gap: int,
            time_gap_unit: __class__.TIME_UNIT,
            push_speed: int,
            push_speed_unit: __class__.SPEED_UNIT,
            pull_speed: int,
            pull_speed_unit: __class__.SPEED_UNIT,
        ) -> bytes:
            if not 0 <= push_volume <= 9999:
                raise ValueError("灌注的分配液量不在范围 0 ~ 9999 中")
            _push_volume = push_volume.to_bytes(length=2, byteorder="little")
            _push_volume_unit = push_volumn_unit.to_bytes(length=1)
            if not 0 <= pull_volume <= 9999:
                raise ValueError("抽取的分配液量不在范围 0 ~ 9999 中")
            _pull_volume = pull_volume.to_bytes(length=2, byteorder="little")
            _pull_volume_unit = pull_volumn_unit.to_bytes(length=1)
            if not 0 <= time_gap <= 9999:
                raise ValueError("时间间隔不在范围 0 ~ 9999 中")
            _time_gap = ((time_gap_unit << 14) & time_gap).to_bytes(
                length=2, byteorder="little"
            )
            if not 1 <= push_speed <= 9999:
                raise ValueError("灌注速度不在范围 1 ~ 9999 中")
            _push_speed = push_speed.to_bytes(length=2, byteorder="little")
            _push_speed_unit = push_speed_unit.to_bytes(length=1)
            if not 1 <= pull_speed <= 9999:
                raise ValueError("抽取速度不在范围 1 ~ 9999 中")
            _pull_speed = pull_speed.to_bytes(length=2, byteorder="little")
            _pull_speed_unit = pull_speed_unit.to_bytes(length=1)
            return (
                _push_volume
                + _push_volume_unit
                + _pull_volume
                + _pull_volume_unit
                + _time_gap
                + _push_speed
                + _push_speed_unit
                + _pull_speed
                + _pull_speed_unit
            )

        def set_continue_mode_cmd(
            volume: int,
            volume_unit: __class__.VOLUME_UNIT,
            delay_before_pull: int,
            delay_before_pull_unit: __class__.TIME_UNIT,
            delay_before_push: int,
            delay_before_push_unit: __class__.TIME_UNIT,
            push_speed: int,
            push_speed_unit: __class__.SPEED_UNIT,
            pull_speed: int,
            pull_speed_unit: __class__.SPEED_UNIT,
        ) -> bytes:
            if not 0 <= volume <= 9999:
                raise ValueError("分配液量不在范围 0 ~ 9999 中")
            _volume = volume.to_bytes(length=2, byteorder="little")
            _volume_unit = volume_unit.to_bytes(length=1)
            if not 0 <= delay_before_pull <= 9999:
                raise ValueError("灌注后执行抽取的时间间隔不在范围 0 ~ 9999 中")
            _delay_before_pull = (
                (delay_before_pull_unit << 14) & delay_before_pull
            ).to_bytes(length=2, byteorder="little")
            if not 0 <= delay_before_push <= 9999:
                raise ValueError("抽取后执行灌注的时间间隔不在范围 0 ~ 9999 中")
            _delay_before_push = (
                (delay_before_push_unit << 14) & delay_before_push
            ).to_bytes(length=2, byteorder="little")
            if not 1 <= push_speed <= 9999:
                raise ValueError("灌注速度不在范围 1 ~ 9999 中")
            _push_speed = push_speed.to_bytes(length=2, byteorder="little")
            _push_speed_unit = push_speed_unit.to_bytes(length=1)
            if not 1 <= pull_speed <= 9999:
                raise ValueError("抽取速度不在范围 1 ~ 9999 中")
            _pull_speed = pull_speed.to_bytes(length=2, byteorder="little")
            _pull_speed_unit = pull_speed_unit.to_bytes(length=1)
            return (
                _volume
                + _volume_unit
                + _delay_before_pull
                + _delay_before_push
                + _push_speed
                + _push_speed_unit
                + _pull_speed
                + _pull_speed_unit
            )

        cmd = b"CWT" + mode.to_bytes(length=1)
        if mode == __class__.MODE.PUSH or mode == __class__.MODE.PULL:
            cmd += set_push_or_pull_mode_cmd(*args, **kvargs)
        elif mode == __class__.MODE.PUSH_PULL or mode == __class__.MODE.PULL_PUSH:
            cmd += set_push_and_pull_mode_cmd(*args, **kvargs)
        elif mode == __class__.MODE.CONTINUE:
            cmd += set_continue_mode_cmd(*args, **kvargs)
        else:
            raise ValueError("意外的模式")
        if self.__command(instruction=cmd) != b"Y":
            raise RuntimeError("意外回应")

    @overload
    def read_mode(
        self,
    ) -> Tuple[
        Literal[MODE.PUSH] | Literal[MODE.PULL], int, VOLUME_UNIT, int, SPEED_UNIT
    ]:
        ...

    @overload
    def read_mode(
        self,
    ) -> Tuple[
        Literal[MODE.PUSH_PULL] | Literal[MODE.PULL_PUSH],
        int,
        VOLUME_UNIT,
        int,
        VOLUME_UNIT,
        int,
        TIME_UNIT,
        int,
        SPEED_UNIT,
        int,
        SPEED_UNIT,
    ]:
        ...

    @overload
    def read_mode(
        self,
    ) -> Tuple[
        Literal[MODE.CONTINUE],
        int,
        VOLUME_UNIT,
        int,
        TIME_UNIT,
        int,
        TIME_UNIT,
        int,
        SPEED_UNIT,
        int,
        SPEED_UNIT,
    ]:
        ...

    def read_mode(self) -> Tuple[MODE, ...]:
        """读取工作模式"""

        def read_push_or_pull_mode_cmd(
            resp: bytes,
        ) -> Tuple[int, __class__.VOLUME_UNIT, int, __class__.SPEED_UNIT]:
            volume = int.from_bytes(resp[0:2], byteorder="little")
            volume_unit = __class__.VOLUME_UNIT(resp[2])
            speed = int.from_bytes(resp[3:5], byteorder="little")
            speed_unit = __class__.SPEED_UNIT(resp[5])
            return (volume, volume_unit, speed, speed_unit)

        def read_push_and_pull_mode_cmd(
            resp: bytes,
        ) -> Tuple[
            int,
            __class__.VOLUME_UNIT,
            int,
            __class__.VOLUME_UNIT,
            int,
            __class__.TIME_UNIT,
            int,
            __class__.SPEED_UNIT,
            int,
            __class__.SPEED_UNIT,
        ]:
            push_volume = int.from_bytes(resp[0:2], byteorder="little")
            push_volume_unit = __class__.VOLUME_UNIT(resp[2])
            pull_volume = int.from_bytes(resp[3:5], byteorder="little")
            pull_volume_unit = __class__.VOLUME_UNIT(resp[5])
            _time_gap = int.from_bytes(resp[6:8], byteorder="little")
            time_gap = _time_gap & 0x3FFF
            time_gap_unit = __class__.TIME_UNIT(time_gap >> 14)
            push_speed = int.from_bytes(resp[8:10], byteorder="little")
            push_speed_unit = __class__.SPEED_UNIT(resp[10])
            pull_speed = int.from_bytes(resp[11:13], byteorder="little")
            pull_speed_unit = __class__.SPEED_UNIT(resp[13])
            return (
                push_volume,
                push_volume_unit,
                pull_volume,
                pull_volume_unit,
                time_gap,
                time_gap_unit,
                push_speed,
                push_speed_unit,
                pull_speed,
                pull_speed_unit,
            )

        def read_continue_mode_cmd(
            resp: bytes,
        ) -> Tuple[
            int,
            __class__.VOLUME_UNIT,
            int,
            __class__.TIME_UNIT,
            int,
            __class__.TIME_UNIT,
            int,
            __class__.SPEED_UNIT,
            int,
            __class__.SPEED_UNIT,
        ]:
            volume = int.from_bytes(resp[0:2], byteorder="little")
            volume_unit = __class__.VOLUME_UNIT(resp[2])
            _delay_before_pull = int.from_bytes(resp[3:5], byteorder="little")
            delay_before_pull = _delay_before_pull & 0x3FFF
            delay_before_pull_unit = __class__.TIME_UNIT(_delay_before_pull >> 14)
            _delay_before_push = int.from_bytes(resp[5:7], byteorder="little")
            delay_before_push = _delay_before_push & 0x3FFF
            delay_before_push_unit = __class__.TIME_UNIT(_delay_before_push >> 14)
            push_speed = int.from_bytes(resp[7:9], byteorder="little")
            push_speed_unit = __class__.SPEED_UNIT(resp[9])
            pull_speed = int.from_bytes(resp[10:12], byteorder="little")
            pull_speed_unit = __class__.SPEED_UNIT(resp[12])

            return (
                volume,
                volume_unit,
                delay_before_pull,
                delay_before_pull_unit,
                delay_before_push,
                delay_before_push_unit,
                push_speed,
                push_speed_unit,
                pull_speed,
                pull_speed_unit,
            )

        resp = self.__command(instruction=b"CRT")
        mode = __class__.MODE(resp[2])
        if mode == __class__.MODE.PUSH or mode == __class__.MODE.PULL:
            data = read_push_or_pull_mode_cmd(resp[3:])
        elif mode == __class__.MODE.PUSH_PULL or mode == __class__.MODE.PULL_PUSH:
            data = read_push_and_pull_mode_cmd(resp[3:])
        elif mode == __class__.MODE.CONTINUE:
            data = read_continue_mode_cmd(resp[3:])
        else:
            raise RuntimeError("意外回复")
        return (mode, *data)

    def action(self, action=ACTION) -> None:
        """控制启动暂停停止"""
        cmd = b"CWX" + action.to_bytes(length=1)
        if self.__command(instruction=cmd) != b"Y":
            raise RuntimeError("意外回应")
        
    def start(self) -> None:
        """启动"""
        self.action(__class__.ACTION.START)
    
    def pause(self) -> None:
        """暂停"""
        self.action(__class__.ACTION.PAUSE)

    def stop(self) -> None:
        """终止"""
        self.action(__class__.ACTION.STOP)

    def switch_direction(self) -> None:
        """反向（只用在灌注/抽取模式中）"""
        if self.__command(instruction=b"CWF") != b"Y":
            raise RuntimeError("意外回应")

    def status(self) -> STATUS:
        """读取运行状态信息"""
        resp = self.__command(instruction=b"CRX")
        if resp[0:2] != b"RX":
            raise RuntimeError("意外回应")
        return __class__.STATUS(resp[2])
    
    def read_direction(self) -> DIRECTION:
        """读取方向信息"""
        resp = self.__command(instruction=b"CRF")
        if resp[0:2] != b"RF":
            raise RuntimeError("意外回应")
        return __class__.DIRECTION(resp[2])
    
    def __init__(
        self,
        port: str,
        baud: BAUD,
        addr: ADDR,
    ) -> None:
        self.serial = serial.Serial(
            port=port,
            baudrate=baud,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_EVEN,
            stopbits=serial.STOPBITS_ONE,
            timeout=5,
            xonxoff=False,
            rtscts=False,
            dsrdtr=False,
        )
        self.addr = addr

    def __del__(self):
        self.serial.close()
