import re

def convert_log_to_csv(input_file, output_file):
    # 匹配帧号的正则：Frame n. 915 @ 0 fps -> 915
    frame_pattern = re.compile(r"Frame n\. (\d+) @")
    # 匹配点数据的正则：(1,0): x=-1364, y=-1102, z=2719, conf=0 -> 1, 0, -1364, -1102, 2719, 0
    point_pattern = re.compile(r"\((\d+),(\d+)\): x=(-?\d+), y=(-?\d+), z=(-?\d+), conf=(\d+)")
    
    current_frame = None
    csv_lines = ["frame,row,col,x,y,z,conf"]  # CSV表头

    # 读取原始日志文件
    with open(input_file, 'r', encoding='utf-8') as f:
        for line in f:
            line = line.strip()
            if not line:
                continue
            
            # 匹配帧号
            frame_match = frame_pattern.search(line)
            if frame_match:
                current_frame = int(frame_match.group(1))
                continue
            
            # 匹配点数据
            if current_frame is not None:
                point_match = point_pattern.search(line)
                if point_match:
                    row = int(point_match.group(1))
                    col = int(point_match.group(2))
                    x = int(point_match.group(3))
                    y = int(point_match.group(4))
                    z = int(point_match.group(5))
                    conf = int(point_match.group(6))
                    # 拼接CSV行
                    csv_lines.append(f"{current_frame},{row},{col},{x},{y},{z},{conf}")

    # 写入CSV文件
    with open(output_file, 'w', encoding='utf-8') as f:
        f.write('\n'.join(csv_lines))
    
    print(f"转换完成！已生成CSV文件：{output_file}")
    print(f"共处理 {len(csv_lines)-1} 个点数据（表头不计）")

if __name__ == "__main__":
    INPUT_LOG = "./L9-dataset/2026-04-02 010429.XDat"   # 你的原始日志文件
    OUTPUT_CSV = "./L9-dataset/parsed_data.csv" # 输出的CSV文件
    convert_log_to_csv(INPUT_LOG, OUTPUT_CSV)