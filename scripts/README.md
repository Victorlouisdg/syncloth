scripts
=======

* `towel_sample.py`: generate a single sample
* `towel_dataset.py`: generate a towel dataset

Usage
-----

`towel_sample.py`

```
blender -P towel_sample.py -- --seed 42
```

`towel_dataset.py`
```
mkdir dataset0
cd dataset0
blender -b -P ../towel_dataset.py -- --dataset_size 50
```