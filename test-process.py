import re
import argparse
from collections import defaultdict, Counter

def process_text(lines):
    # 用于存储所有符合条件的输出行
    valid_lines = []
    
    # 匹配 "[数字]retired pc 后面的内容" 的正则表达式
    pattern = r'\[\s*(\d+)\]retired pc ([0-9a-f]+) (.+)'
        # 遍历每一行文本，提取符合条件的行
    for line in lines:
        match = re.match(pattern, line.strip())
        if match:
            # 提取括号内的数字（序列号）、pc后面的数字、以及其它内容
            seq_number = int(match.group(1))
            pc_value = match.group(2)
            valid_lines.append((seq_number, pc_value, line.strip()))
    
    # 以 pc_value 为键组织成不同组
    groups = defaultdict(list)
    for seq_number, pc_value, line in valid_lines:
        groups[pc_value].append(seq_number)
    
    # 结果存储列表
    results = []
    
    # 处理每一组
    for pc_value, seq_numbers in groups.items():
        # 计算每条数据与前一条数据的差值
        diffs = [seq_numbers[i] - seq_numbers[i - 1] for i in range(1, len(seq_numbers))]
        
        if diffs:
            # 统计差值的众数
            counter = Counter(diffs)
            mode = counter.most_common(1)[0][0]
            results.append(f"Group with pc {pc_value}: mode = {mode}")
    
    return results

def read_file(file_path):
    # 读取文件内容
    with open(file_path, 'r') as file:
        lines = file.readlines()
    return lines

def main():
    # 使用 argparse 解析命令行参数
    parser = argparse.ArgumentParser(description="Process a log file and extract information.")
    parser.add_argument("file", help="Path to the input file")
    args = parser.parse_args()
    
    # 从文件中读取内容
    file_path = args.file
    lines = read_file(file_path)
    
    # 处理文件内容并输出结果
    results = process_text(lines)
    for result in results:
        print(result)

if __name__ == "__main__":
    main()


# import re
# import argparse
# from collections import defaultdict

# def process_text(lines):
#     # 用于存储所有符合条件的输出行
#     valid_lines = []
    
#     # 匹配 "[数字]retired pc 后面的内容" 的正则表达式
#     pattern = r'\[\s*(\d+)\]retired pc ([0-9a-f]+) (.+)'

#     # 遍历每一行文本，提取符合条件的行
#     for line in lines:
#         match = re.match(pattern, line.strip())
#         if match:
#             # 提取括号内的数字（序列号）、pc后面的数字、以及其它内容
#             seq_number = int(match.group(1))
#             pc_value = match.group(2)
#             valid_lines.append((seq_number, pc_value, line.strip()))
    
#     # 以pc_value为键组织成不同组
#     groups = defaultdict(list)
#     for seq_number, pc_value, line in valid_lines:
#         groups[pc_value].append((seq_number, line))
    
#     # 结果存储列表
#     results = []
    
#     # 处理每一组
#     for pc_value, group_lines in groups.items():
#         # 获取每组的第一个和最后一个输出行
#         first_line = group_lines[0][1]
#         last_line = group_lines[-1][1]
#         # 提取括号内的数字
#         first_seq_number = group_lines[0][0]
#         last_seq_number = group_lines[-1][0]

#         # 计算间隔差，并处理重复次数
#         repeat_count = len(group_lines)
#         if repeat_count > 1:
#             # 执行相减除法运算
#             result = (last_seq_number - first_seq_number) / (repeat_count - 1)
#             results.append(f"Group with pc {pc_value} in {repeat_count - 1}: first {first_seq_number}, last {last_seq_number}, result {result}")
    
#     return results

# def read_file(file_path):
#     # 读取文件内容
#     with open(file_path, 'r') as file:
#         lines = file.readlines()
#     return lines

# # 主程序：从文件读取数据并处理
# def main():
#     # 使用 argparse 解析命令行参数
#     parser = argparse.ArgumentParser(description="Process a log file and extract information.")
#     parser.add_argument("file", help="Path to the input file")
#     args = parser.parse_args()
    
#     # 从文件中读取内容
#     file_path = args.file
#     lines = read_file(file_path)
    
#     # 处理文件内容并输出结果
#     results = process_text(lines)
#     for result in results:
#         print(result)

# if __name__ == "__main__":
#     main()