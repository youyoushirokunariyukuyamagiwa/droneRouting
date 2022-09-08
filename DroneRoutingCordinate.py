"""Capacited Drone Routing Problem (CDRP)."""
from ortools.constraint_solver import pywrapcp
import pyproj
import pandas as pd

url = 'https://www.city.kawasaki.jp/170/cmsfiles/contents/0000116/116229/141305_public_facility_library.csv'
df = pd.read_csv(url, encoding='cp932')

# index managerを作成
# 引数は順に（地点数、乗り物の数、デポのNode）
manager = pywrapcp.RoutingIndexManager(len(df), 2, 0)

# routing modelを作成
routing = pywrapcp.RoutingModel(manager)

# １＝VTOL機、２＝マルチコプター
drone_type = 1

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

# RegisterTransitCallbackで先ほど作成した距離callbackをRoutingModelに登録します
transit_callback_index = routing.RegisterTransitCallback(distance_callback)

# 移動コストを設定
# SetArcCostEvaluatorOfAllVehiclesで登録した距離callbackをコスト（最適化の目的関数）として
# セットします。実際に解く際に、ここでセットしたもの（ここでは距離）を最小にするようなルートを
# ソルバーは見つけにいきます。
# 今回最小化したいのは配達完了時刻だが、ドローンの速度を一定とするので結局距離を最短にすることと同じ
routing.SetArcCostEvaluatorOfAllVehicles(transit_callback_index)

# 配送量のcalbackを作成、登録
def demand_callback(index):
    """Returns the demand of the node."""
    node = manager.IndexToNode(index)
    return df['絵本の蔵書数'][node] / 100

demand_callback_index = routing.RegisterUnaryTransitCallback(demand_callback)

# 積載量の制約
routing.AddDimensionWithVehicleCapacity(
    demand_callback_index,
    0,  # null capacity slack
    [1500, 1500],  # vehicle maximum capacities
    True,  # start cumulative to zero
    'Capacity')

#capacity = routing.GetDimensionOrDie('Capacity')

def vtol_f_energy_callback(payload):
    return payload*0.2 + 3

def vtol_h_energy_callback(payload):
    return payload*1.5 + 20

def multi_f_energy_callback(payload):
    return payload*1.0 + 10

def multi_h_energy_callback(payload):
    return payload*1.0 + 10

# 配送量のcalbackを作成、登録
def energy_callback(from_index, to_index,payload,drone_type):
    """Returns the energy consumption between two nodes."""
    distance = distance_callback(from_index,to_index)
    energy_consumption

    if drone_type == 1:
        energy_consumption = vtol_f_energy_callback(payload)
    else:
        energy_consumption = multi_f_energy_callback(payload)

    return distance*energy_consumption

energy_callback_index = routing.RegisterUnaryTransitCallback(energy_callback)

# 積載量の制約
routing.AddDimensionWithVehicleCapacity(
    energy_callback_index,
    0,  # null capacity slack
    [1500, 1500],  # vehicle maximum capacities
    True,  # start cumulative to zero
    'Energy')

# 最適化の実行
search_parameters = pywrapcp.DefaultRoutingSearchParameters()
search_parameters.time_limit.seconds = 1
solution = routing.SolveWithParameters(search_parameters)

# 解を確認
print(solution.ObjectiveValue())