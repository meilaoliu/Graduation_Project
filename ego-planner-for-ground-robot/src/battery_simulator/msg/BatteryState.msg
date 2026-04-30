Header header
float32 percentage          # 0 - 100
bool charging
float32 estimated_remaining_distance_m   # 基于当前 v 与衰减率的简单估计
string status               # "ok" | "low" | "critical" | "charging"
