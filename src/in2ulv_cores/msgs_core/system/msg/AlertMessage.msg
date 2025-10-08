uint8 INFO=0
uint8 WARNING=1
uint8 ERROR=2
uint8 CRITICAL=3

uint8 severity  # 警告级别: INFO/WARNING/ERROR/CRITICAL
string source  # 告警来源
string message  # 告警信息
uint16 error_code  # 错误码: 1001=超时/2001=状态异常/3001=频率异常