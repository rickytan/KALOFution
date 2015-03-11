KALOFution
-------------

This project is a implementation of the rigid part of <http://www.stanford.edu/~qianyizh/projects/elasticreconstruction.html>,
can be used for **global optimization** of point clouds registration.

The origin implementation can be found here: <https://github.com/qianyizh/ElasticReconstruction>

The paper: [Elastic Fragments for Dense Scene Reconstruction](http://stanford.edu/~sdmiller/octo/files/iccv2013.pdf)

## Dependences
- PCL 1.6.x (<http://www.pointclouds.org/>)
- Boost 1.5.x (<http://www.boost.org/>)
- SuiteSparse (<http://faculty.cse.tamu.edu/davis/suitesparse.html>)
- METIS 4.x.x (<http://glaros.dtc.umn.edu/gkhome/fsroot/sw/metis/OLD>)
- Eigne 3 (<http://eigen.tuxfamily.org/index.php?title=Main_Page>)

This project runs on Windows 64bit System, all the lib above need to be `vs2010-64bit`.

## License
**MIT**

## Steps

1. Dump Depth Map

    In this step, we convert the `*.vmap` and `*.nmap` file to `*.pcd`, with **filtering** and **clipping**. Their's a bug in `PCL 1.6.*`, **DON'T** dump to `PLY` format.


        set BASE_DIR=.
        set POS_FILE=szs_pos.txt
        KALOFution.exe -run dumpmap -pos-file %BASE_DIR%\%POS_FILE% --dump-dir %BASE_DIR%\cloud-filtered --filter --data-dir %BASE_DIR%\dump --min-clip 0.5 --max-clip 2.4 --step 45

    * `--pos-file`: if provided, the output cloud will be translated according to camera poses. **Usually used for debugging**.
    * `--dump-dir`: where to store the cloud files. if the `DIR` doesn't exists, it will be created automatically.
    * `--filter`: if provided, the output cloud will be filtered using outlier removal.
    * `--data-dir`: where to load `*.vmap` and `*.nmap`.
    * `--min-clip` `--max-clip`: limit the points `Z-axis` to **min** and **max**.
    * `--step`:  the dump step


2. Build Correspondence

    In this step, we find out all possible cloud pairs which have overlap to each other, and save their correnpondence point indexes. If *cloud_x.pcd* and *cloud_y.pcd* have correnspondence points, the result will be stored in file `corres_x_y.txt`

        set BASE_DIR=.
        set POS_FILE=szs_pos.txt
        KALOFution.exe -run buildcorres --pos-file %BASE_DIR%\%POS_FILE% --data-dir %BASE_DIR%\cloud-filtered --better-corres 1 --need-align 1 --step 45 --cloud-num 149 --save-to %BASE_DIR%\corres-better --dist-thres 0.05 --angle-thres 15 --valid-pair-dist-thres 0.03 --align-sampling-limit 30000 --min-corres-num 8000 --align-translation-thres 0.3

    * `--pos-file`: the camera poses.
    * `--data-dir`: where to load `*.pcd`.
    * `--better-corres`: use better correspondence measurement (slower) or not (faster).
    * `--need-align`: use **ICP** point-to-plane alignment before finding correspondent points (much slower) or not (faster).
    * `--cloud-num`: how many clouds to load.
    * `--step`: the loading step, the same as ***Dump Depth Map***.
    * `--save-to`: where to store `corres_x_y.txt`.
    * `--dist-thres`: the maximum correspondent points distance for alignment.
    * `--angle-thres`: the maximum correspondent points norm angle (in degree) for alignment.
    * `--valid-pair-dist-thres`: the maximum distance for a valid  correspondence pair.
    * `--align-sampling-limit`: down sample number for alignment.
    * `--min-corres-num`: the minimum points pairs for a valid cloud pair.
    * `--align-translation-thres`: the minimum translation distance for a valid alignment.

3. Global Optimization

    In the step, we fill a sparse matrix according to the conrespondence relations, and solve the equation to minimize the global alignment error.

        set BASE_DIR=.
        set POS_FILE=szs_pos.txt
        KALOFution.exe -run optimize --pos-file %BASE_DIR%\%POS_FILE% --data-dir %BASE_DIR%\cloud-filtered --step 45 --cloud-num 149 --corres-dir %BASE_DIR%\corres-better --save-to %BASE_DIR%\optimized-better

    * `--pos-file`: the camera poses.
    * `--data-dir`: where to load `*.pcd`.
    * `--corres-dir`: where to load `cloud_x_y.txt`.
    * `--save-to`: where to store the optimized cloud files (in `PLY` format).

