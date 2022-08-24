# コストマップに付与されるコストを変える
生成したコストマップに付与されるコストを変えたい場合，costmap_common_params.yamlの"cost_scaling_factor", "inflation_radius"を調整します．
  - `cost_scaling_factor`
    - コストマップは，設定されたコストから，inflation_radiusまで指数関数的に減衰します．<br>
        大きいほど減衰が早くなります．
    - 体感ですが，大きすぎると移動中にぶつかってしまいます．
  - `inflation_radius`
    - コスト(黒い部分)が強調されてコストになるときの距離
    - 値が大きいほどコストが大きくなり，狭い場所での経路生成が困難になります．
<br>このパラメータがlocalコストマップ，globalコストマップそれぞれに適用されます．


> 例：[costmap_common_params.yaml](sobit_navigation/param/sobit_turtlebot/costmap_common_params.yaml)
> ```
> #cost_scaling_factor and inflation_radius were now moved to the inflation_layer ns
> inflation_layer:
>  enabled:              true
>  cost_scaling_factor:  5.0  # exponential rate at which the obstacle cost drops off (default: 10)
>  inflation_radius:     0.5  # max. distance from an obstacle at which costs are incurred for planning paths.
> ```