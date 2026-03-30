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
    df_first_edge_point = df_first[df_first["is_edge"] == 1]
    df_first_planar_point = df_first[df_first["is_planar"] == 1]
    df_first_common_point = df_first[(df_first["is_edge"] == 0) & (df_first["is_planar"] == 0)]

    # 3. 创建三个基础轨迹（第一帧）
    trace_common = go.Scatter3d(
        x=df_first_common_point["x"], y=df_first_common_point["y"], z=df_first_common_point["z"],
        mode="markers",
        name=f"common point (Frame {first_frame})",
        marker=dict(color="gray", size=1.0, opacity=0.4),
        visible=True
    )
    trace_planar = go.Scatter3d(
        x=df_first_planar_point["x"], y=df_first_planar_point["y"], z=df_first_planar_point["z"],
        mode="markers",
        name=f"planar point (Frame {first_frame})",
        marker=dict(color="yellow", size=1.0, opacity=0.8),
        visible=True
    )
    trace_edge = go.Scatter3d(
        x=df_first_edge_point["x"], y=df_first_edge_point["y"], z=df_first_edge_point["z"],
        mode="markers",
        name=f"edge point (Frame {first_frame})",
        marker=dict(color="red", size=1.0, opacity=0.8),
        visible=True
    )

    # 4. 构建每一帧的动画数据
    frames = []
    for frame_id in unique_frames:
        df_frame = df[df["frame"] == frame_id]
        df_edge_point = df_frame[df_frame["is_edge"] == 1]
        df_planar_point = df_frame[df_frame["is_planar"] == 1]
        df_common_point = df_frame[(df_frame["is_edge"] == 0) & (df_frame["is_planar"] == 0)]

        frame_trace_common = go.Scatter3d(
            x=df_common_point["x"], y=df_common_point["y"], z=df_common_point["z"],
            mode="markers",
            name=f"common point (Frame {frame_id})",
            marker=dict(color="gray", size=1.0, opacity=0.4),
            visible=True
        )
        frame_trace_planar = go.Scatter3d(
            x=df_planar_point["x"], y=df_planar_point["y"], z=df_planar_point["z"],
            mode="markers",
            name=f"planar point (Frame {frame_id})",
            marker=dict(color="yellow", size=1.0, opacity=0.8),
            visible=True
        )
        frame_trace_edge = go.Scatter3d(
            x=df_edge_point["x"], y=df_edge_point["y"], z=df_edge_point["z"],
            mode="markers",
            name=f"edge point (Frame {frame_id})",
            marker=dict(color="red", size=1.0, opacity=0.8),
            visible=True
        )

        frames.append(go.Frame(
            data=[frame_trace_common, frame_trace_planar, frame_trace_edge],
            name=str(frame_id)
        ))

    # 5. 创建图形对象，包含初始轨迹和 frames
    fig = go.Figure(
        data=[trace_common, trace_planar, trace_edge],
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
                aspectmode='cube',
                camera=dict(
                    up=dict(x=0, y=0, z=1),
                    center=dict(x=0, y=0, z=0),
                    eye=dict(x=1.8, y=1.8, z=1.2)
                ),
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
    visualize_point_cloud("./L9-dataset/L9dataset1_feature.csv")