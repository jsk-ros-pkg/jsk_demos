# How to use labelme

## convert HEIF -> jpg
```bash
for file in *.HEIC; do heif-convert ${file} ${file/%.heic/}.jpg; done
```

```bash
yoshiki@yoshiki:~/research_ws/src/jsk_demos/obinata_research/obinata_pr2_73b2_cleanup/labelme$ python3 instancelabelme2voc.py dataset/test/ dataset/test/dataset_voc --labels dataset/labels.txt
```

```bash
yoshiki@yoshiki:~/research_ws/src/jsk_demos/obinata_research/obinata_pr2_73b2_cleanup/labelme/dataset$ rosrun jsk_perception train_ssd.py --train-dataset-dir ./train/dataset_voc/ --val-dataset-dir ./test/dataset_voc/
```