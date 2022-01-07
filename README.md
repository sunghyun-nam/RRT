# RRT and RRT STAR and DRT
obstacle origin RRT / obstacle modified RRT
obstacle origin RRTSTAR / obstacle modified RRTSTAR
obstacle DRT
obstacle modified RRT(RRTSTAR) : add 'autonomous generation of waypoint' function
obstacle DRT : it based on modified RRTSTAR, just add theta 

수정해야 할것 
origin RRTSTAR 
- cost function을 수정함
- 주변 노드의 거리 평균을 구하고 평균보다 짧은 거리에 있는 노드들 중에서 각도가 최소인 주변 노드를 
- new point의 부모 노드로 지정

modified RRTSTAR 
- cost function 준 부분을 수정해야함 
- 거리와 각도의 차원이 다른데 그냥 더해버림..ㅜ
