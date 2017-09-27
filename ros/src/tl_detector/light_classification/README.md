
## Generate dataset record

python create_traffic_light_tl_record.py --data_dir=`pwd`/data/bag_dump_just_traffic_light  --output_path=data/traffic_simulator_dataset.record


## Train

python object_detection/train.py --pipeline_config_path=./model/ssd_mobilenet_v1_coco_simulator.config --train_dir=data/run_simulator                                   


## Evaluate

python object_detection/eval.py --pipeline_config_path=./model/ssd_mobilenet_v1_coco.config --eval_dir=data/run --checkpoint_dir=data/run --logtostderr


### Display data on tensorboard

tensorboard --logdir=data/run


## Export

python object_detection/export_inference_graph.py --input_type=image_tensor --pipeline_config_path=model/ssd_mobilenet_v1_coco.config --trained_checkpoint_prefix=./data/run/model.ckpt-20000 --output_directory=data/trained
