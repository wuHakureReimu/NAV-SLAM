import pandas as pd
import plotly.graph_objects as go
from plotly.subplots import make_subplots

# 读取数据
df = pd.read_csv('point_cloud_data_with_pose.csv')

# 创建带时间轴的动画可视化
fig = go.Figure()

# 全局地图点云
fig.add_trace(go.Scatter3d(
    x=df['x'],
    y=df['y'],
    z=df['z'],
    mode='markers',
    marker=dict(
        size=1.5,
        color=df['z'],
        colorscale='Plasma',
        opacity=0.6
    ),
    name='全局地图点云'
))

# 添加IMU轨迹（带时间序列）
fig.add_trace(go.Scatter3d(
    x=df['IMU_x'],
    y=df['IMU_y'], 
    z=df['IMU_z'],
    mode='lines',
    line=dict(
        color='red',
        width=3
    ),
    name='IMU轨迹'
))

# 添加修正轨迹
fig.add_trace(go.Scatter3d(
    x=df['Modified_x'],
    y=df['Modified_y'],
    z=df['Modified_z'],
    mode='lines',
    line=dict(
        color='green',
        width=3
    ),
    name='修正轨迹'
))



# 更新布局
fig.update_layout(
    title={
        'text': 'IMU+激光SLAM建图可视化',
        'x': 0.5,
        'xanchor': 'center'
    },
    scene=dict(
        xaxis_title='X坐标',
        yaxis_title='Y坐标', 
        zaxis_title='Z坐标',
        aspectmode='data',
        camera=dict(
            up=dict(x=0, y=0, z=1),
            center=dict(x=0, y=0, z=0),
            eye=dict(x=1.5, y=1.5, z=1.5)
        )
    ),
    autosize=True,
    margin=dict(l=20, r=20, t=50, b=20),  # 均衡的边距
    legend=dict(
        x=0.02,
        y=0.98,
        traceorder='normal',
        bgcolor='rgba(255,255,255,0.8)'
    )
)

# 显示图表
fig.show()