import pyproj
import pandas as pd
from ortools.constraint_solver import pywrapcp

url = 'https://www.city.kawasaki.jp/170/cmsfiles/contents/0000116/116229/141305_public_facility_library.csv'
df = pd.read_csv(url, encoding='cp932')

# index managerを作成
manager = pywrapcp.RoutingIndexManager(len(df), 2, 0)

# routing modelを作成
routing = pywrapcp.RoutingModel(manager)

# 移動距離のcallbackを作成、登録
def distance_callback(from_index, to_index):
    """Returns the distance between the two nodes."""
    # Indexの変換
    from_node = manager.IndexToNode(from_index)
    to_node = manager.IndexToNode(to_index)

    # 2点間の距離を計算
    g = pyproj.Geod(ellps='WGS84')
    distance = g.inv(
        df['経度'][from_node], df['緯度'][from_node], 
        df['経度'][to_node], df['緯度'][to_node]
    )[2]
    return distance

transit_callback_index = routing.RegisterTransitCallback(distance_callback)

# 移動コストを設定
routing.SetArcCostEvaluatorOfAllVehicles(transit_callback_index)

# 配送量のcallbackを作成、登録
def demand_callback(index):
    """Returns the demand of the node."""
    node = manager.IndexToNode(index)
    return df['絵本の蔵書数'][node] / 100

demand_callback_index = routing.RegisterUnaryTransitCallback(demand_callback)

# 積載量の制約
routing.AddDimensionWithVehicleCapacity(
    demand_callback_index,
    0,  # null capacity slack
    [1500, 500],  # vehicle maximum capacities
    True,  # start cumul to zero
    'Capacity')
capacity = routing.GetDimensionOrDie('Capacity')

# 最適化の実行
search_parameters = pywrapcp.DefaultRoutingSearchParameters()
search_parameters.time_limit.seconds = 1
solution = routing.SolveWithParameters(search_parameters)

# 解を確認
print(solution.ObjectiveValue())