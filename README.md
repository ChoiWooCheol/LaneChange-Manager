# LaneChange-Manager
* autoware에 구현되어있지 않은 기능 중 하나인 차선변경기능을 가능하게 해주는 패키지를 작성한 것 입니다.

# Method
* 주변 Lane을 찾는 기능구현은 저의 레포지토리 중 find arround lanes 와 같습니다.
* 현재 차량이 주행하고 있는 current pose를 누적시켜 차량의 방향 벡터를 계산합니다.
* 차량의 state중 차선변경을 위해 변경 할 차선을 검사하는 state를 갖는다면 차량의 양옆 lane을 저장하고 있는 queue를 검색합니다.
* 왼쪽으로 차선변경을 한다고 했을 때, 차량의 방향 벡터와, 현재위치에서 변경할 차선의 방향벡터를 외적합니다.
* 외적을 결과를 통해 queue에 변경 가능한 차선이 있는지 찾고, 있다면 로드합니다.
* 최종적으로 laneChange state를 갖는다면 차선을 변경하게 됩니다.

# Run
```sh
$ roslaunch lanechange_manager lanechange_manager.launch
``` 

# Result
1. 차량 초기 상태, 감지된 waypoints
<img src="lanechange_manager/img/init_car.png" width="100%" height="100%">

2. 차량이 주행하면서 lane change 할 수 있는 waypoints 감지
<img src="calc_around_waypoints/images/rviz2.png" width="100%" height="100%">
<img src="calc_around_waypoints/images/terminal2.png" width="100%" height="100%">
