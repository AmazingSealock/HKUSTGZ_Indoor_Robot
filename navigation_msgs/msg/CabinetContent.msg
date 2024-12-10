# CabinetContent.msg
Header header
string cabinet_id         # 柜子唯一标识符
BoxInfo box              # 箱子信息（如果柜子为空，则box_id为空字符串）
time last_updated        # 最后更新时间