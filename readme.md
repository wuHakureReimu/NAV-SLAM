

# NAV-SLAM

## RUN

```
mkdir build & cd build
cp ../dataset/xxx.json . # or xxx.csv
cmake ..
./main.exe
```
For L5 datasets, the program reads the depth mat data from L5 dataset `xxx.json`, then outputs `feature_data.csv` for analysis and `point_cloud_data.csv` as the final slam result.

For L9 datasets, several preprocessing steps should be taken with tools in `./visualization`

**steps**

1. implement `parse_dataset.py` to parse XDat formed pointcloud data into csv format `XXX.csv`.
2. check `main.c` to confirm that you are using the L9 data handler, rather L5
3. check `pointcloud.h` to confirm that you have correctly set up the pointcloud parameters, `MAX_COLS` and `MAX_ROWS`
4. run as mentioned above
5. check the two outputed results

## visualization

**Requirements: plotly & pandas**

1. use `L9_data_analysis_ver2.py` to visualize the **parsed** original pointcloud data `xxx.csv`
2. use `L9_feature_analysis.py` to visualize `feature_data.csv` that stores pointcloud after feature extraction
3. use `visualization.py` to visualize `point_cloud_data.csv` which is the SLAM result
