## 目前的进度
完成了与semantic的结合。
## 接下来要做的
现在需要实现room detection的内容，首先应该理清楚room detection的逻辑。 因为我们的扫描是实时扫描的，扫描开始，那么semantic segmentation也就同时开始进行了。对于划分为wall的点，我们用region growing的方法，进行平面的拟合。这里遇到的两个问题：
1. 如何应对label的变化
2. 何时开始进行第二个平面的拟合。

在进行patch detection时，我们使用chunk聚合后的点，这样还能加快速度。如何进行region growing？每个submap维护了一个hash_aggregated表，我们对新加入的点进行knnSearch，得到它的法向量（法向量也许可以通过给定的点的法向量加权和计算得到，可节省速度）。对新加入的点（wall）看能否加入到之前拟合的平面中，如果不行，添加一个新的平面进来。这个方法的缺陷在于无法合适地处理动态的调整，比如检测到了回环。



对于第二个问题，定义一个标准，当region不再growing（新的点大部分是outlier），就停止这个region的growing。此外，可能还需要一个平面合并的问题，对于距离太近而且法向差别很小的平面，合并为一个平面。

对于第一个问题，目前更依赖于label分得准。


出现平面后，就会出现直线，从而进行二维平面上直线的映射。我们预先为dcel设定一个足够大的bounding。接着，将直线插入到dcel中，为每个直线计算weight。接着就开始提取房间了。对于提取房间还是有几个问题的：
1. 如何判断没有房间的情况？
没有房间的情况下，每一个face与outside的距离应该都是很近的，在这里可以制定一个检测的标准。

提取出房间后，摄像机离开了该房间，我们就将在这个房间内的submap构成的model认为是一个room的model。同时开始新的semantic segmentation，因为在一个room中，semantic分割的结果应该是比较好的，所以以room划分比较合理。此时room_id += 1。 另一个问题，需不需要reset dcel？