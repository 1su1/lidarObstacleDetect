import re
import argparse
import pandas as pd

# 解析命令行参数
parser = argparse.ArgumentParser(description='Parse logfile and generate time summary.')
parser.add_argument('logfile', type=str, help='Path to the logfile')
args = parser.parse_args()

# 读取文本文件
with open(args.logfile, 'r') as file:
    data = file.read()

# 使用正则表达式提取不同类型的耗时
pattern = r'\[ INFO\] \[\d+\.\d+\]: (.*?) cost time:(\d+) ms'
matches = re.findall(pattern, data)

# 创建一个字典来存储不同类型的耗时数据
time_dict = {
    'ROI_Clip': [],
    'remove_ground': [],
    'euclidean_cluster': [],
    'total': []
}

# 将匹配的耗时数据添加到对应的列表中
for match in matches:
    process, time = match
    if 'ROI_Clip' in process:
        time_dict['ROI_Clip'].append(int(time))
    elif 'remove ground' in process:
        time_dict['remove_ground'].append(int(time))
    elif 'euclidean cluster' in process:
        time_dict['euclidean_cluster'].append(int(time))
    elif 'total' in process:
        time_dict['total'].append(int(time))

# 计算平均值、最大值和最小值
summary = {}
for process, times in time_dict.items():
    # 将毫秒转换为秒，并计算赫兹
    hz_values = [1000 / time for time in times]
    summary[process] = {
        'Mean/Hz': sum(hz_values) / len(hz_values),
        'Min/Hz': max(hz_values),
        'Max/Hz': min(hz_values)
    }

# 将结果转换为 DataFrame
df = pd.DataFrame(summary)

# 添加标明列
df['Metric'] = ['Mean/Hz', 'Min/Hz', 'Max/Hz']

# 重新排列列的顺序
df = df[['Metric', 'ROI_Clip', 'remove_ground', 'euclidean_cluster', 'total']]

# 输出到 Excel 表格
output_filename = args.logfile.replace('.txt', '_summary.xlsx')
df.to_excel(output_filename, index=False)

print(f"Summary saved to {output_filename}")
