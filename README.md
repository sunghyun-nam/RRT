# RRT and RRT STAR
obstacle origin RRT / obstacle modified RRT

obstacle origin RRTSTAR / obstacle modified RRTSTAR

obstacle DRT
obstacle modified RRT(RRTSTAR) : add 'autonomous generation of waypoint' function

obstacle DRT : it based on modified RRTSTAR, just add theta 

origin RRTSTAR 
- cost function을 수정함
- 부모 노드 선정 : 주변 노드의 거리 평균을 구하고 평균보다 짧은 거리에 있는 노드들 중에서 각도가 최소인 주변 노드를 new point의 부모 노드로 지정
- 트리 재구성 : 주변 노드의 부모 노드가 new point일 때의 각도가 기존 트리일 때의 각도보다 작고, 주변 노드의 부모 노드가 new point일 때의 거리가 기존 트리일 때의 거리보다 작을 때 주변 노드를 new point의 자식 노드로 

modified RRTSTAR / DRT
- cost function 준 부분을 수정해야함(아직 안함) -> origin RRTSTAR 참고해서 하시면 되요 
- 거리와 각도의 차원이 다른데 그냥 더해버림..ㅜ
