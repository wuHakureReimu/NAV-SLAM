import pandas as pd
import plotly.graph_objects as go

def visualize_point_cloud(csv_data_path):
    # 1. 读取所有数据
    df = pd.read_csv(csv_data_path)
    # 按帧号排序，确保动画顺序正确
    unique_frames = sorted(df["frame"].unique())
    print(f"检测到所有帧: {unique_frames}")

    # ===================== 计算全数据集范围 =====================
    x_min, x_max = df["x"].min(), df["x"].max()
    y_min, y_max = df["y"].min(), df["y"].max()
    z_min, z_max = df["z"].min(), df["z"].max()
    # ============================================================

    # 2. 获取第一帧的数据，用于初始化图形
    first_frame = unique_frames[0]
    df_first = df[df["frame"] == first_frame]

    # 3. 创建单个轨迹，颜色根据 conf 值连续变化，透明度固定
    trace = go.Scatter3d(
        x=df_first["x"],
        y=df_first["y"],
        z=df_first["z"],
        mode="markers",
        name="Point Cloud",
        marker=dict(
            color=df_first["conf"],          # 颜色映射依据 conf 数值（连续）
            colorscale="viridis",            # 颜色映射方案
            size=1.0,
            opacity=0.7,                    # 全局透明度，所有点相同
            colorbar=dict(title="conf")      # 颜色条标题
        ),
        visible=True
    )

    # 4. 构建每一帧的动画数据
    frames = []
    for frame_id in unique_frames:
        df_frame = df[df["frame"] == frame_id]

        # 当前帧的散点图数据
        frame_data = go.Scatter3d(
            x=df_frame["x"],
            y=df_frame["y"],
            z=df_frame["z"],
            mode="markers",
            marker=dict(
                color=df_frame["conf"],
                colorscale="viridis",
                size=1.0,
                opacity=0.7
            )
        )
        # 创建 Frame，指定该帧需要更新的轨迹索引（0）
        frames.append(go.Frame(
            data=[frame_data],
            name=str(frame_id)
        ))

    # 5. 创建图形对象，包含初始轨迹和 frames
    fig = go.Figure(
        data=[trace],
        frames=frames,
        layout=go.Layout(
            sliders=[{
                "active": 0,
                "yanchor": "top",
                "xanchor": "left",
                "currentvalue": {
                    "font": {"size": 16},
                    "prefix": "当前帧: ",
                    "visible": True,
                    "xanchor": "right"
                },
                "transition": {"duration": 0, "easing": "linear"},
                "pad": {"b": 8, "t": 8},
                "len": 0.9,
                "x": 0.1,
                "y": 0,
                "steps": [
                    {
                        "args": [
                            [frame.name],
                            {"frame": {"duration": 0, "redraw": True},
                             "mode": "immediate"}
                        ],
                        "label": frame.name,
                        "method": "animate"
                    }
                    for frame in frames
                ]
            }],
            scene=dict(
                xaxis_title='X',
                yaxis_title='Y',
                zaxis_title='Z',
                aspectmode='cube',   # 强制 X/Y/Z 轴单位长度完全一致
                camera=dict(
                    up=dict(x=0, y=0, z=1),
                    center=dict(x=0, y=0, z=0),
                    eye=dict(x=1.8, y=1.8, z=1.2)
                ),
                # 固定坐标轴范围，不随帧变化
                xaxis=dict(range=[x_min, x_max], autorange=False),
                yaxis=dict(range=[y_min, y_max], autorange=False),
                zaxis=dict(range=[z_min, z_max], autorange=False),
            ),
            autosize=True,
            margin=dict(l=20, r=20, t=80, b=20),
            legend=dict(
                x=0.02,
                y=0.98,
                traceorder='normal',
                bgcolor='rgba(255,255,255,0.8)',
                font=dict(size=15)
            )
        )
    )

    fig.show()

if __name__ == "__main__":
    visualize_point_cloud("./L9-dataset/L9dataset2_parsed.csv")