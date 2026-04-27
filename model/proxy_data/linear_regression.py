import os
import pandas as pd
from scipy.stats import linregress

# 获取 python 文件所在目录
folder = os.path.dirname(os.path.abspath(__file__))

# 筛选文件：以“稳态”开头的csv
files = [f for f in os.listdir(folder) if f.startswith("稳态") and f.endswith(".csv")]

if not files:
    print("未找到符合条件的CSV文件")
    exit()

for file in files:
    path = os.path.join(folder, file)

    try:
        # 从第6行开始读取（跳过前5行）
        df = pd.read_csv(path, skiprows=5)

        # 默认取前两列
        x = df.iloc[:, 0]
        y = df.iloc[:, 1]

        # 去除NaN
        valid = ~(x.isna() | y.isna())
        x = x[valid]
        y = y[valid]

        # 线性回归
        result = linregress(x, y)

        slope = result.slope
        intercept = result.intercept
        r = result.rvalue
        r_squared = r ** 2

        # 输出
        print(f"\n文件: {file}")
        print(f"回归方程: y = {slope:.6f} * x + {intercept:.6f}")
        print(f"相关系数 r = {r:.6f}")
        print(f"决定系数 R² = {r_squared:.6f}")

    except Exception as e:
        print(f"\n文件: {file} 处理失败: {e}")