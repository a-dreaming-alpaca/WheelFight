FD = 350
RD = 250
BD = 280
LD = 200


def read_sensor_snapshot(uptech):
    raw_io = {str(i): uptech.ADC_IO_GetInputLevel(i) for i in range(8)}
    raw_adc = {str(i): uptech.ADC_Get_Channel(i) for i in range(7)}
    return {
        "raw_io": raw_io,
        "raw_adc": raw_adc,
    }


def _io_values(snapshot):
    return [snapshot["raw_io"][str(i)] for i in range(8)]


def _adc_values(snapshot):
    return [snapshot["raw_adc"][str(i)] for i in range(7)]


def detect_platform(snapshot):
    adc = _adc_values(snapshot)
    ad_4 = adc[4]
    ad_5 = adc[5]
    if ad_4 + ad_5 > 7000:
        return 0
    if (ad_4 <= 3500) != (ad_5 <= 3500):
        return 2
    return 1


def detect_fence(snapshot):
    io = _io_values(snapshot)
    adc = _adc_values(snapshot)
    io_0, io_1, io_2, io_3 = io[0], io[1], io[2], io[3]
    ad_0, ad_1, ad_2, ad_3 = adc[0], adc[1], adc[2], adc[3]

    if io_2 == 0 and io_1 == 1 and io_3 == 1 and ad_0 > FD and ad_1 < RD and ad_2 < BD and ad_3 < LD:
        return 1
    if io_3 == 0 and io_0 == 1 and io_2 == 1 and ad_0 < FD and ad_1 > RD and ad_2 < BD and ad_3 < LD:
        return 2
    if io_0 == 0 and io_1 == 1 and io_3 == 1 and ad_0 < FD and ad_1 < RD and ad_2 > BD and ad_3 < LD:
        return 3
    if io_1 == 0 and io_0 == 1 and io_2 == 1 and ad_0 < FD and ad_1 < RD and ad_2 < BD and ad_3 > LD:
        return 4
    if io_1 == 1 and io_2 == 1 and ad_0 > FD and ad_1 < RD and ad_2 < BD and ad_3 > LD:
        return 5
    if io_2 == 1 and io_3 == 1 and ad_0 > FD and ad_1 > RD and ad_2 < BD and ad_3 < LD:
        return 6
    if io_0 == 1 and io_3 == 1 and ad_0 < FD and ad_1 > RD and ad_2 > BD and ad_3 < LD:
        return 7
    if io_0 == 1 and io_1 == 1 and ad_0 < FD and ad_1 < RD and ad_2 > BD and ad_3 > LD:
        return 8
    if ad_0 > FD and ad_1 < RD and ad_2 > BD and ad_3 < LD:
        return 9
    if ad_0 < FD and ad_1 > RD and ad_2 < BD and ad_3 > LD:
        return 10
    if ad_0 > FD and ad_1 > RD and ad_2 < BD and ad_3 > LD:
        return 11
    if ad_0 > FD and ad_1 > RD and ad_2 > BD and ad_3 < LD:
        return 12
    if ad_0 > FD and ad_1 < RD and ad_2 > BD and ad_3 > LD:
        return 13
    if ad_0 < FD and ad_1 > RD and ad_2 > BD and ad_3 > LD:
        return 14
    if io_0 == 0 and io_1 == 0 and ad_0 < FD and ad_1 < RD:
        return 15
    if io_0 == 0 and io_3 == 0 and ad_0 < FD and ad_3 < LD:
        return 16
    if io_1 == 0 and io_2 == 0 and ad_1 < FD and ad_2 < RD:
        return 17
    if io_2 == 0 and io_3 == 0 and ad_2 < FD and ad_3 < LD:
        return 18
    return 101


def detect_edge(snapshot):
    io = _io_values(snapshot)
    io_4, io_5, io_6, io_7 = io[4], io[5], io[6], io[7]
    if io_4 == 0 and io_5 == 0 and io_6 == 0 and io_7 == 0:
        return 0
    if io_4 == 1 and io_5 == 0 and io_6 == 0 and io_7 == 0:
        return 1
    if io_4 == 0 and io_5 == 1 and io_6 == 0 and io_7 == 0:
        return 2
    if io_4 == 0 and io_5 == 0 and io_6 == 1 and io_7 == 0:
        return 3
    if io_4 == 0 and io_5 == 0 and io_6 == 0 and io_7 == 1:
        return 4
    if io_4 == 1 and io_5 == 1 and io_6 == 0 and io_7 == 0:
        return 5
    if io_4 == 0 and io_5 == 0 and io_6 == 1 and io_7 == 1:
        return 6
    if io_4 == 1 and io_5 == 0 and io_6 == 0 and io_7 == 1:
        return 7
    if io_4 == 0 and io_5 == 1 and io_6 == 1 and io_7 == 0:
        return 8
    return 102


def detect_enemy(snapshot, tag_id):
    io = _io_values(snapshot)
    adc = _adc_values(snapshot)
    io_0, io_1, io_2, io_3 = io[0], io[1], io[2], io[3]
    ad_0 = adc[0]

    if io_0 == 1 and io_1 == 1 and io_2 == 1 and io_3 == 1:
        return 0
    if io_0 == 0:
        if tag_id != 2:
            if ad_0 < 1000:
                return 1
            return 11
        return 5
    if io_1 == 0:
        return 2
    if io_2 == 0:
        return 3
    if io_3 == 0:
        return 4
    return 103


def detect_slip(snapshot):
    io = _io_values(snapshot)
    adc = _adc_values(snapshot)
    io_4, io_5, io_6, io_7 = io[4], io[5], io[6], io[7]
    ad_4 = adc[4]
    ad_5 = adc[5]

    if io_4 == 1 and io_7 == 1 and io_5 == 0 and io_6 == 0:
        return 0
    if io_5 == 1 and io_6 == 1 and io_4 == 0 and io_7 == 0:
        return 1
    if ad_5 < 3500:
        return 2
    if ad_4 < 3500:
        return 3
    return 105


def detect_all(snapshot, tag_id):
    return {
        "fence": detect_fence(snapshot),
        "edge": detect_edge(snapshot),
        "enemy": detect_enemy(snapshot, tag_id),
        "stage": detect_platform(snapshot),
        "slip": detect_slip(snapshot),
    }
