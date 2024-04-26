1 while (!openList.isEmpty()) {
2     point = openList.getFirst();
3     expand(point);
4 }
5  void expand(currentPoint) {
6      boolean isRaise = isRaise(currentPoint);
7      double cost;
8      for each (neighbor in currentPoint.getNeighbors()) {
9          if (isRaise) {
10              if (neighbor.nextPoint == currentPoint) {
11                 neighbor.setNextPointAndUpdateCost(currentPoint);
12                 openList.add(neighbor);
13             } else {
14                  cost = neighbor.calculateCostVia(currentPoint);
15                  if (cost < neighbor.getCost()) {
16                      currentPoint.setMinimumCostToCurrentCost();
17                      openList.add(currentPoint);
18                 }
19             }
20          } else {
21              cost = neighbor.calculateCostVia(currentPoint);
22              if (cost < neighbor.getCost()) {
23                  neighbor.setNextPointAndUpdateCost(currentPoint);
24                  openList.add(neighbor);
25              }
26          }
27      }
28  }
29  boolean isRaise(point) {
30      double cost;
31      if (point.getCurrentCost() > point.getMinimumCost()) {
32          for each(neighbor in point.getNeighbors()) {
33              cost = point.calculateCostVia(neighbor);
34              if (cost < point.getCurrentCost()) {
35                  point.setNextPointAndUpdateCost(neighbor);
36              }
37          }
38      }
39      return point.getCurrentCost() > point.getMinimumCost();
40  }
