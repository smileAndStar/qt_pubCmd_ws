uint8 header        # 0xFF数据包头
int8[64] acdata     # 心律波形数据 (有符号)
uint8 heartrate     # 心率
uint8 spo2          # 血氧
uint8 bk            # 微循环
uint8[8] rsv1       # 保留数据 1
uint8 sdnn          # 心率变异性
uint8 rmssd
uint8 nn50
uint8 pnn50
uint8[6] rra        # RR间期
uint8 rsv2          # 保留数据 2
uint8 state         # 模块状态