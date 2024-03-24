# Changelog

All notable changes to this project will be documented in this file. See [standard-version](https://github.com/conventional-changelog/standard-version) for commit guidelines.

## [2.17.6](https://github.com/rickstaa/panda-gazebo/compare/v2.17.5...v2.17.6) (2024-03-24)


### Bug Fixes

* Update 'franka_ros' branch ([2aeb759](https://github.com/rickstaa/panda-gazebo/commit/2aeb759535c4c0643547f1a7c2174752a9dee112))

## [2.17.5](https://github.com/rickstaa/panda-gazebo/compare/v2.17.4...v2.17.5) (2024-02-05)


### Documentation

* **SetGripperWidth:** improve 'grasping' description ([#225](https://github.com/rickstaa/panda-gazebo/issues/225)) ([7584984](https://github.com/rickstaa/panda-gazebo/commit/75849841f7c8d770487ae95e161a9edcec1c2543))

## [2.17.4](https://github.com/rickstaa/panda-gazebo/compare/v2.17.3...v2.17.4) (2024-02-04)


### Documentation

* improve common issues documentation ([#222](https://github.com/rickstaa/panda-gazebo/issues/222)) ([4859382](https://github.com/rickstaa/panda-gazebo/commit/48593828aeffcbec04dcfe0d4d5c9a2da2ac464a))

## [2.17.3](https://github.com/rickstaa/panda-gazebo/compare/v2.17.2...v2.17.3) (2024-02-04)


### Documentation

* **panda:** update worlds description ([#220](https://github.com/rickstaa/panda-gazebo/issues/220)) ([a3d297f](https://github.com/rickstaa/panda-gazebo/commit/a3d297f9b87d33a3ac18a8a0a0e553b4b481be71))

## [2.17.2](https://github.com/rickstaa/panda-gazebo/compare/v2.17.1...v2.17.2) (2024-01-03)


### Bug Fixes

* **joint_locker:** fix joint locker unlock bug ([#212](https://github.com/rickstaa/panda-gazebo/issues/212)) ([3251cb3](https://github.com/rickstaa/panda-gazebo/commit/3251cb3447a9b9a2e663d10f85e037dee7819331))
* **joint_locker:** fix string concat error ([#214](https://github.com/rickstaa/panda-gazebo/issues/214)) ([8ad1d8f](https://github.com/rickstaa/panda-gazebo/commit/8ad1d8fcd992cd533a46d74243db7fb2f90bb95e))

## [2.17.1](https://github.com/rickstaa/panda-gazebo/compare/v2.17.0...v2.17.1) (2024-01-01)


### Documentation

* fix index syntax errors ([#210](https://github.com/rickstaa/panda-gazebo/issues/210)) ([be06ddd](https://github.com/rickstaa/panda-gazebo/commit/be06ddd60accb1f9d85c5669cf10a5be5d117ae6))

## [2.17.0](https://github.com/rickstaa/panda-gazebo/compare/v2.16.1...v2.17.0) (2024-01-01)


### Features

* add Panda joint locker service ([#209](https://github.com/rickstaa/panda-gazebo/issues/209)) ([b0845a5](https://github.com/rickstaa/panda-gazebo/commit/b0845a5e1853c9336733746d02b69897488b42e5))


### Bug Fixes

* **control_switcher:** fix control switcher type error ([#206](https://github.com/rickstaa/panda-gazebo/issues/206)) ([13d4052](https://github.com/rickstaa/panda-gazebo/commit/13d4052d10086d4b072bd120709d7244387abae4))
* ensure srv headers are build before gazebo plugin ([9c4ef85](https://github.com/rickstaa/panda-gazebo/commit/9c4ef858502ab8236d3ead733ae59c3d4682adfc))

## [2.16.1](https://github.com/rickstaa/panda-gazebo/compare/v2.16.0...v2.16.1) (2023-12-23)


### Bug Fixes

* fix 'set_joint_commands' service return values ([#205](https://github.com/rickstaa/panda-gazebo/issues/205)) ([e2df695](https://github.com/rickstaa/panda-gazebo/commit/e2df69549538ec3e32859e96c2356df9b36e5a95))
* fix random joint positions service ([#204](https://github.com/rickstaa/panda-gazebo/issues/204)) ([44d6855](https://github.com/rickstaa/panda-gazebo/commit/44d68552849bd2f35264e6e9e87bafb5d253cff7))


### Documentation

* fix broken links ([b99e1b1](https://github.com/rickstaa/panda-gazebo/commit/b99e1b10dd16f9fd38fcb9c10f084a9884938965))
* fix broken README links ([484c224](https://github.com/rickstaa/panda-gazebo/commit/484c224c71ffb8e338fa3e2363b1daeb24251a72))
* improve issues section ([8a610b3](https://github.com/rickstaa/panda-gazebo/commit/8a610b3c7dfca40f626dff2acb191c06e400747b))

## [2.16.0](https://github.com/rickstaa/panda-gazebo/compare/v2.15.4...v2.16.0) (2023-12-20)


### Features

* improve grasping cube ([#195](https://github.com/rickstaa/panda-gazebo/issues/195)) ([3441187](https://github.com/rickstaa/panda-gazebo/commit/3441187551846c0896d7735a45f4b99d76726837))
* make code compatible with latest franka_ros version ([#183](https://github.com/rickstaa/panda-gazebo/issues/183)) ([b466168](https://github.com/rickstaa/panda-gazebo/commit/b46616842d7215b58c1c203f2585d6e9529cf09c))
* remove 'panda_moveit_config' submodule ([#185](https://github.com/rickstaa/panda-gazebo/issues/185)) ([39c8f12](https://github.com/rickstaa/panda-gazebo/commit/39c8f129806921ae9df03187a3b1a518b5e9163b))


### Bug Fixes

* **control_switcher:** fix control switcher ([#187](https://github.com/rickstaa/panda-gazebo/issues/187)) ([43fd75c](https://github.com/rickstaa/panda-gazebo/commit/43fd75cdbf3958f895f85143e4b7e0b0612a6b1f))
* fix 'set_logging_level' launch file arguments ([#186](https://github.com/rickstaa/panda-gazebo/issues/186)) ([2bc9a26](https://github.com/rickstaa/panda-gazebo/commit/2bc9a26e7f1c513bab9ab39f5d97d56a092b7900))
* fix moveit launch argument ([#188](https://github.com/rickstaa/panda-gazebo/issues/188)) ([a03b978](https://github.com/rickstaa/panda-gazebo/commit/a03b9783cb42f8b3b23aa706da1121c3ebc5249d))
* **moveit_server:** fix add_plane service ([#189](https://github.com/rickstaa/panda-gazebo/issues/189)) ([424fd4e](https://github.com/rickstaa/panda-gazebo/commit/424fd4ef3cf05e72ea2c27c140a10f7284314960))


### Documentation

* add known issues to docs ([35c77f8](https://github.com/rickstaa/panda-gazebo/commit/35c77f8b4a0aeb266f86bf5863ad0ffb02c4f1f8))
* add known issues to docs ([#197](https://github.com/rickstaa/panda-gazebo/issues/197)) ([4cac087](https://github.com/rickstaa/panda-gazebo/commit/4cac08709490ed986854aa14ac12b6141436fa47))

## [2.15.4](https://github.com/rickstaa/panda-gazebo/compare/v2.15.3...v2.15.4) (2023-10-04)


### Documentation

* apply small doc syntax improvement ([70a22fa](https://github.com/rickstaa/panda-gazebo/commit/70a22fa4e1bc96a84c9f664c4031d18b5a8f5d01))

## [2.15.3](https://github.com/rickstaa/panda-gazebo/compare/v2.15.2...v2.15.3) (2023-08-31)


### Documentation

* update documentation ([#159](https://github.com/rickstaa/panda-gazebo/issues/159)) ([c938ed9](https://github.com/rickstaa/panda-gazebo/commit/c938ed96d6cf337aeead8b5ae608e7354debaeb6))

## [2.15.2](https://github.com/rickstaa/panda-gazebo/compare/v2.15.1...v2.15.2) (2023-08-29)


### Documentation

* fix broken 'ros_gazebo_gym' reference ([#156](https://github.com/rickstaa/panda-gazebo/issues/156)) ([8a87a15](https://github.com/rickstaa/panda-gazebo/commit/8a87a1592e8206ea6896725430909b44f5f27a69))
* improve docs ([#158](https://github.com/rickstaa/panda-gazebo/issues/158)) ([4277075](https://github.com/rickstaa/panda-gazebo/commit/42770756ceefc900267771c7d276541402004cdb))

## [2.15.1](https://github.com/rickstaa/panda-gazebo/compare/v2.15.0...v2.15.1) (2023-08-28)


### Documentation

* fix doc release docs ([#152](https://github.com/rickstaa/panda-gazebo/issues/152)) ([748d734](https://github.com/rickstaa/panda-gazebo/commit/748d7345e3f0d850a1b5f94201842ee51bf39fa3))
* improve config ([#154](https://github.com/rickstaa/panda-gazebo/issues/154)) ([29d2518](https://github.com/rickstaa/panda-gazebo/commit/29d251855975120148d1b9ab7cfb5960ff052971))

## [2.15.0](https://github.com/rickstaa/panda-gazebo/compare/v2.14.16...v2.15.0) (2023-08-28)


### Features

* improve ros shutdown and codebase ([#150](https://github.com/rickstaa/panda-gazebo/issues/150)) ([1c04095](https://github.com/rickstaa/panda-gazebo/commit/1c04095245de29b50953968ff403805d705fc06c))


### Documentation

* add pre-commit note ([#147](https://github.com/rickstaa/panda-gazebo/issues/147)) ([2a7b99a](https://github.com/rickstaa/panda-gazebo/commit/2a7b99ae23cb67fc22657998b8b0eb8521265f41))

## [2.14.16](https://github.com/rickstaa/panda-gazebo/compare/v2.14.15...v2.14.16) (2023-08-24)


### Bug Fixes

* fix incorrect 'helpers' import ([#144](https://github.com/rickstaa/panda-gazebo/issues/144)) ([aaa1144](https://github.com/rickstaa/panda-gazebo/commit/aaa1144881f7a0b3979c6c91d191da01e1ee578a))

## [2.14.15](https://github.com/rickstaa/panda-gazebo/compare/v2.14.14...v2.14.15) (2023-08-11)


### Documentation

* improve documentation admonitions ([#131](https://github.com/rickstaa/panda-gazebo/issues/131)) ([ab53eae](https://github.com/rickstaa/panda-gazebo/commit/ab53eaec83ca6e1ac58c4e09d66bfacf79b30743))

## [2.14.14](https://github.com/rickstaa/panda-gazebo/compare/v2.14.13...v2.14.14) (2023-08-02)


### Documentation

* improve documentation urls ([#129](https://github.com/rickstaa/panda-gazebo/issues/129)) ([3331227](https://github.com/rickstaa/panda-gazebo/commit/3331227a6c746b197e156f6d5227a7524428cfa9))

## [2.14.13](https://github.com/rickstaa/panda-gazebo/compare/v2.14.12...v2.14.13) (2023-07-15)


### Documentation

* improve code API docstrings ([#125](https://github.com/rickstaa/panda-gazebo/issues/125)) ([73ce2e2](https://github.com/rickstaa/panda-gazebo/commit/73ce2e227a47805d588233632769d139aa69d44b))

## [2.14.12](https://github.com/rickstaa/panda-gazebo/compare/v2.14.11...v2.14.12) (2023-07-10)


### Documentation

* add pkg test badge ([#117](https://github.com/rickstaa/panda-gazebo/issues/117)) ([71511ab](https://github.com/rickstaa/panda-gazebo/commit/71511ab60e09fb143e8c04b9041e7e6c3abd7ae0))
* update python badge ([#120](https://github.com/rickstaa/panda-gazebo/issues/120)) ([b98a1b8](https://github.com/rickstaa/panda-gazebo/commit/b98a1b83c807e4c993cce3abf2cc34cf94388fc2))

## [2.14.11](https://github.com/rickstaa/panda-gazebo/compare/v2.14.10...v2.14.11) (2023-07-02)


### Documentation

* add sphinx-autoapi ROS doc dependency ([#113](https://github.com/rickstaa/panda-gazebo/issues/113)) ([0dd2db1](https://github.com/rickstaa/panda-gazebo/commit/0dd2db15cc2edbbf4e69205358fbb7669da713bd))

## [2.14.10](https://github.com/rickstaa/panda-gazebo/compare/v2.14.9...v2.14.10) (2023-06-30)


### Documentation

* improve docs ([#110](https://github.com/rickstaa/panda-gazebo/issues/110)) ([9efc08d](https://github.com/rickstaa/panda-gazebo/commit/9efc08d8526ab65245ae53324722ec77412aebc8))

## [2.14.9](https://github.com/rickstaa/panda-gazebo/compare/v2.14.8...v2.14.9) (2023-06-29)


### Documentation

* fix urls ([#108](https://github.com/rickstaa/panda-gazebo/issues/108)) ([5629236](https://github.com/rickstaa/panda-gazebo/commit/56292363fbbaa34b8e85af07dd3b4f3f676d1ade))

## [2.14.8](https://github.com/rickstaa/panda-gazebo/compare/v2.14.7...v2.14.8) (2023-06-23)


### Documentation

* add recursive cloning remark ([a585705](https://github.com/rickstaa/panda-gazebo/commit/a585705d67620cec6cacd8c70ce7572d16cbee21))

## [2.14.7](https://github.com/rickstaa/panda-gazebo/compare/v2.14.6...v2.14.7) (2023-06-21)


### Documentation

* fix small typo ([7e87ca0](https://github.com/rickstaa/panda-gazebo/commit/7e87ca003561f40e3575a8f5932f375d6892ad59))

## [2.14.6](https://github.com/rickstaa/panda-gazebo/compare/v2.14.5...v2.14.6) (2023-06-19)


### Documentation

* fix edit on github button ([8cf0532](https://github.com/rickstaa/panda-gazebo/commit/8cf0532c756462fa9f85aa9f6b93112cbc1d4a78))

## [2.14.5](https://github.com/rickstaa/panda-gazebo/compare/v2.14.4...v2.14.5) (2023-06-16)


### Documentation

* fix doc release documentation ([ba16d38](https://github.com/rickstaa/panda-gazebo/commit/ba16d38306fb129f7b383006966de69c79d8da6a))
* improve documentation ([d8cb2bc](https://github.com/rickstaa/panda-gazebo/commit/d8cb2bc288b6b7c66df3f913724f6001ef0fdb88))

## [2.14.4](https://github.com/rickstaa/panda-gazebo/compare/v2.14.3...v2.14.4) (2023-06-16)


### Documentation

* fix doc release documentation ([ba16d38](https://github.com/rickstaa/panda-gazebo/commit/ba16d38306fb129f7b383006966de69c79d8da6a))
* improve documentation ([d8cb2bc](https://github.com/rickstaa/panda-gazebo/commit/d8cb2bc288b6b7c66df3f913724f6001ef0fdb88))

### [2.14.3](https://github.com/rickstaa/panda-gazebo/compare/v2.14.2...v2.14.3) (2023-06-14)

### [2.14.2](https://github.com/rickstaa/panda-gazebo/compare/v2.14.1...v2.14.2) (2023-06-13)

### [2.14.1](https://github.com/rickstaa/panda-gazebo/compare/v2.14.0...v2.14.1) (2023-06-13)

### [2.13.30](https://github.com/rickstaa/panda-gazebo/compare/v2.14.0...v2.13.30) (2023-06-13)

### [2.13.28](https://github.com/rickstaa/panda-gazebo/compare/v2.13.27...v2.13.28) (2023-06-10)


### Bug Fixes

* fix package.json syntax error ([b9e4297](https://github.com/rickstaa/panda-gazebo/commit/b9e42978a4cd3b72ca765f852a7ffb6d7313d834))
* fix versioning and package.json ([a4913fc](https://github.com/rickstaa/panda-gazebo/commit/a4913fcceaae7b47519c30ef6ec1ee9327b76d17))

### [2.13.5](https://github.com/rickstaa/panda-gazebo/compare/v2.13.27...v2.13.5) (2023-06-10)


### Bug Fixes

* fix package.json syntax error ([b9e4297](https://github.com/rickstaa/panda-gazebo/commit/b9e42978a4cd3b72ca765f852a7ffb6d7313d834))

## [2.13.0](https://github.com/rickstaa/panda-gazebo/compare/v2.12.2...v2.13.0) (2022-02-22)


### Features

* update 'franka_ros' submodule ([e655663](https://github.com/rickstaa/panda-gazebo/commit/e6556631de54a87bc56aff1b38603de58c7cec51))

### [2.12.2](https://github.com/rickstaa/panda-gazebo/compare/v2.12.1...v2.12.2) (2022-02-18)

### [2.12.1](https://github.com/rickstaa/panda-gazebo/compare/v2.12.0...v2.12.1) (2022-02-16)

## [2.12.0](https://github.com/rickstaa/panda-gazebo/compare/v2.11.0...v2.12.0) (2022-02-15)


### Features

* **moveit_server:** enable 'set_joint_positions' service by default ([71722bc](https://github.com/rickstaa/panda-gazebo/commit/71722bc684185e730965317af263cc6b2e99bdb7))

## [2.11.0](https://github.com/rickstaa/panda-gazebo/compare/v2.10.1...v2.11.0) (2022-02-15)


### Features

* **moveit_server:** handle incorrectly defined bounding regions ([669384a](https://github.com/rickstaa/panda-gazebo/commit/669384a081d6e0cf31d9afb60a922ba930517858))

## [2.10.0](https://github.com/rickstaa/panda-gazebo/compare/v2.9.4...v2.10.0) (2022-02-09)


### Features

* **moveit_server:** improve moveit services ([caae4ad](https://github.com/rickstaa/panda-gazebo/commit/caae4ad20419ec7cb4c5954e1959490635294018))

### [2.9.4](https://github.com/rickstaa/panda-gazebo/compare/v2.9.3...v2.9.4) (2022-02-09)


### Bug Fixes

* **moveit_server:** fixes 'get_random_joint_positions' bug ([fec512f](https://github.com/rickstaa/panda-gazebo/commit/fec512f734aef89c9295d6cb5c4c1921c8c061d5))

### [2.9.3](https://github.com/rickstaa/panda-gazebo/compare/v2.9.2...v2.9.3) (2022-02-07)

### [2.9.2](https://github.com/rickstaa/panda-gazebo/compare/v2.9.1...v2.9.2) (2022-02-07)

### [2.9.1](https://github.com/rickstaa/panda-gazebo/compare/v2.9.0...v2.9.1) (2022-02-07)

## [2.9.0](https://github.com/rickstaa/panda-gazebo/compare/v2.8.6...v2.9.0) (2022-02-07)


### Features

* **moveit_server:** add random ee/joint pose attempts parameter ([fab0243](https://github.com/rickstaa/panda-gazebo/commit/fab02433e067f99cf6488cd9163ea8563ca6e725))

### [2.8.6](https://github.com/rickstaa/panda-gazebo/compare/v2.8.5...v2.8.6) (2022-02-05)

### [2.8.5](https://github.com/rickstaa/panda-gazebo/compare/v2.8.4...v2.8.5) (2022-02-04)

### [2.8.4](https://github.com/rickstaa/panda-gazebo/compare/v2.8.3...v2.8.4) (2022-02-03)

### [2.8.3](https://github.com/rickstaa/panda-gazebo/compare/v2.8.2...v2.8.3) (2022-02-01)

### [2.8.2](https://github.com/rickstaa/panda-gazebo/compare/v2.8.1...v2.8.2) (2022-01-29)

### [2.8.1](https://github.com/rickstaa/panda-gazebo/compare/v2.8.0...v2.8.1) (2022-01-29)

## [2.8.0](https://github.com/rickstaa/panda-gazebo/compare/v2.7.11...v2.8.0) (2022-01-26)


### Features

* add ability to disable franka_gazebo logs ([99aed6e](https://github.com/rickstaa/panda-gazebo/commit/99aed6e2461fe6b5303c51437c24f0f21d085309))

### [2.7.11](https://github.com/rickstaa/panda-gazebo/compare/v2.7.10...v2.7.11) (2022-01-24)


### Bug Fixes

* fix dynamic reconfigure test scripts ([029d2f0](https://github.com/rickstaa/panda-gazebo/commit/029d2f07d0c7db47f4146be15c6b223a281e9f39))

### [2.7.8](https://github.com/rickstaa/panda-gazebo/compare/v2.7.7...v2.7.8) (2022-01-22)

### [2.7.7](https://github.com/rickstaa/panda-gazebo/compare/v2.7.6...v2.7.7) (2022-01-10)

### [2.7.6](https://github.com/rickstaa/panda-gazebo/compare/v2.7.5...v2.7.6) (2022-01-10)

### [2.7.5](https://github.com/rickstaa/panda-gazebo/compare/v2.7.4...v2.7.5) (2022-01-10)

### [2.7.4](https://github.com/rickstaa/panda-gazebo/compare/v2.7.3...v2.7.4) (2022-01-07)

### [2.7.3](https://github.com/rickstaa/panda-gazebo/compare/v2.7.2...v2.7.3) (2022-01-07)


### Bug Fixes

* fix 'franka_gripper' log level setter script bug ([d678551](https://github.com/rickstaa/panda-gazebo/commit/d67855133322f14decf186e591e5bd2993b09992))

### [2.7.2](https://github.com/rickstaa/panda-gazebo/compare/v2.7.1...v2.7.2) (2022-01-06)

### [2.7.1](https://github.com/rickstaa/panda-gazebo/compare/v2.7.0...v2.7.1) (2021-12-29)


### Bug Fixes

* **moveit_server:** fixes 'add_plane' bug ([a6a800b](https://github.com/rickstaa/panda-gazebo/commit/a6a800b36040e8d55d33558ad3c4ef452a7c6624))

## [2.7.0](https://github.com/rickstaa/panda-gazebo/compare/v2.6.0...v2.7.0) (2021-12-23)


### Features

* **moveit_server:** add 'add_plane' service ([c95885b](https://github.com/rickstaa/panda-gazebo/commit/c95885b55b1411b4e1640c6a3fb8ceda0140b795))

## [2.6.0](https://github.com/rickstaa/panda-gazebo/compare/v2.5.0...v2.6.0) (2021-12-17)


### Features

* **moveit_server:** add 'get_ee_pose_joint_config' service ([9e2580f](https://github.com/rickstaa/panda-gazebo/commit/9e2580f508e34d49d187310aa8acf72fef4e5aff))

## [2.5.0](https://github.com/rickstaa/panda-gazebo/compare/v2.4.2...v2.5.0) (2021-12-17)


### Features

* update packge gazebo env variables ([5f94b85](https://github.com/rickstaa/panda-gazebo/commit/5f94b85eaf0ae93b944ee3710b27a9d122212f47))

### [2.4.2](https://github.com/rickstaa/panda-gazebo/compare/v2.4.1...v2.4.2) (2021-12-16)

### [2.4.1](https://github.com/rickstaa/panda-gazebo/compare/v2.4.0...v2.4.1) (2021-12-14)

## [2.4.0](https://github.com/rickstaa/panda-gazebo/compare/v2.3.13...v2.4.0) (2021-12-14)


### Features

* add 'max_effort' gripper command parameter ([777f003](https://github.com/rickstaa/panda-gazebo/commit/777f003b3ed26e20dc42122c938cf8986e67d780))

### [2.3.11](https://github.com/rickstaa/panda-gazebo/compare/v2.3.10...v2.3.11) (2021-12-13)

### [2.3.10](https://github.com/rickstaa/panda-gazebo/compare/v2.3.9...v2.3.10) (2021-12-13)

### [2.3.9](https://github.com/rickstaa/panda-gazebo/compare/v2.3.8...v2.3.9) (2021-12-13)

### [2.3.8](https://github.com/rickstaa/panda-gazebo/compare/v2.3.7...v2.3.8) (2021-12-13)

### [2.3.7](https://github.com/rickstaa/panda-gazebo/compare/v2.3.6...v2.3.7) (2021-12-13)

### [2.3.6](https://github.com/rickstaa/panda-gazebo/compare/v2.3.0...v2.3.6) (2021-12-13)

### [2.3.6](https://github.com/rickstaa/panda-gazebo/compare/v2.3.0...v2.3.6) (2021-12-13)

### [2.3.5](https://github.com/rickstaa/panda-gazebo/compare/v2.3.0...v2.3.5) (2021-12-13)

### [2.3.4](https://github.com/rickstaa/panda-gazebo/compare/v2.3.0...v2.3.4) (2021-12-12)

### [2.3.3](https://github.com/rickstaa/panda-gazebo/compare/v2.3.0...v2.3.3) (2021-12-12)

### [2.3.2](https://github.com/rickstaa/panda-gazebo/compare/v2.3.0...v2.3.2) (2021-12-12)

### [2.3.3](https://github.com/rickstaa/panda-gazebo/compare/v2.3.0...v2.3.3) (2021-12-12)

### [2.3.2](https://github.com/rickstaa/panda-gazebo/compare/v2.3.0...v2.3.2) (2021-12-12)

### [2.3.1](https://github.com/rickstaa/panda-gazebo/compare/v2.3.0...v2.3.1) (2021-12-11)

## [2.3.0](https://github.com/rickstaa/panda-gazebo/compare/v2.2.4...v2.3.0) (2021-12-10)


### Features

* add brute force grasping ([e89ebd2](https://github.com/rickstaa/panda-gazebo/commit/e89ebd230c9fd00bb12bcf1a4dccb54c17e8dfb9))
* change physics engine to DART ([25d1e61](https://github.com/rickstaa/panda-gazebo/commit/25d1e61aefc4f191ca33bc4b07e50785db461917)), closes [/github.com/frankaemika/franka_ros/issues/160#issuecomment-989173918](https://github.com/rickstaa//github.com/frankaemika/franka_ros/issues/160/issues/issuecomment-989173918)
* **panda_control_server:** improve `set_gripper_width` action ([496332f](https://github.com/rickstaa/panda-gazebo/commit/496332fc8fa23a5266450c2ea1453bf9ca81a29c))


### Bug Fixes

* add franka_ros[#211](https://github.com/rickstaa/panda-gazebo/issues/211) ([0a21fb3](https://github.com/rickstaa/panda-gazebo/commit/0a21fb302303995928b8dec54cbf7424dc1f2a8f)), closes [/github.com/frankaemika/franka_ros/issues/160#issuecomment-989173918](https://github.com/rickstaa//github.com/frankaemika/franka_ros/issues/160/issues/issuecomment-989173918)

### [2.2.4](https://github.com/rickstaa/panda-gazebo/compare/v2.2.3...v2.2.4) (2021-11-26)

### [2.2.3](https://github.com/rickstaa/panda-gazebo/compare/v2.2.2...v2.2.3) (2021-11-26)

### [2.2.2](https://github.com/rickstaa/panda-gazebo/compare/v2.2.1...v2.2.2) (2021-11-26)


### Bug Fixes

* **control_server:** fix control server 'wait_till_done' method ([6f3153a](https://github.com/rickstaa/panda-gazebo/commit/6f3153a33da3fef0765f6c2a1ede66364c619170))

### [2.2.1](https://github.com/rickstaa/panda-gazebo/compare/v2.2.0...v2.2.1) (2021-11-24)


### Bug Fixes

* **control_server:** move 'gripper_action' execution after other actions ([5c06047](https://github.com/rickstaa/panda-gazebo/commit/5c06047ae7cfa91581ee3a997b53e53d1deba0b0))

## [2.2.0](https://github.com/rickstaa/panda-gazebo/compare/v2.1.4...v2.2.0) (2021-11-24)


### Features

* **panda_control_service:** add gripper grasp feature ([820b176](https://github.com/rickstaa/panda-gazebo/commit/820b176f98052e36333b73403e55b6c513f2b3cf))

### [2.1.4](https://github.com/rickstaa/panda-gazebo/compare/v2.1.3...v2.1.4) (2021-11-23)


### Bug Fixes

* fix franka_ros `develop` branch compatiblity issues ([68a5172](https://github.com/rickstaa/panda-gazebo/commit/68a5172d715c5a7ef10963f6ff9a86dc58724546))

### [2.1.3](https://github.com/rickstaa/panda-gazebo/compare/v2.1.2...v2.1.3) (2021-11-22)


### Bug Fixes

* add 'AddBox' srv to CMakeList ([a46eaeb](https://github.com/rickstaa/panda-gazebo/commit/a46eaeb8559e6195f6769c5244354e405e12c092))

### [2.1.1](https://github.com/rickstaa/panda-gazebo/compare/v2.1.0...v2.1.1) (2021-11-18)


### Bug Fixes

* fix slide world puck z-position ([6e3b8e0](https://github.com/rickstaa/panda-gazebo/commit/6e3b8e03e7c66470bc155600f43c47f82fe534f3))

## [2.1.0](https://github.com/rickstaa/panda-gazebo/compare/v2.0.24...v2.1.0) (2021-11-18)


### Features

* add `planning_scene/add_box` service ([0a6c432](https://github.com/rickstaa/panda-gazebo/commit/0a6c4324aaba15a4b697be189068a1978c5ab743))
* add ability to disable extra services ([2692b48](https://github.com/rickstaa/panda-gazebo/commit/2692b48cb9a9986860c17e8bcd4e9897e8367f08))
* add panda slide env launch script ([9a3da80](https://github.com/rickstaa/panda-gazebo/commit/9a3da8023a24c3693e8763484e9fe1b3d2db4511))


### Bug Fixes

* improve cube/puck models ([bac3a12](https://github.com/rickstaa/panda-gazebo/commit/bac3a12c5b23f63c66fa83905949e4c491a30486))
* remove gravity compensation hotfix ([30adcfe](https://github.com/rickstaa/panda-gazebo/commit/30adcfecf741a124273cbb7e70c1216fb1622fff))
* remove unused Quaternion class ([652cc7b](https://github.com/rickstaa/panda-gazebo/commit/652cc7be6e333204dfc3d7b78812ff09ff7f4090))

### [2.0.20](https://github.com/rickstaa/panda-gazebo/compare/v2.0.19...v2.0.20) (2021-10-29)


### Bug Fixes

* add gravity compensation hotfix ([6113c9f](https://github.com/rickstaa/panda-gazebo/commit/6113c9f045468da34ba1759db07a9dae52028b61))
* fix gravity compensation fix ([6c4bbb5](https://github.com/rickstaa/panda-gazebo/commit/6c4bbb5d65734326a393f34360ad7627887d7b44))

### [2.0.12](https://github.com/rickstaa/panda-gazebo/compare/v2.0.11...v2.0.12) (2021-10-20)

### [2.0.11](https://github.com/rickstaa/panda-gazebo/compare/v2.0.10...v2.0.11) (2021-10-08)

### [2.0.10](https://github.com/rickstaa/panda-gazebo/compare/v2.0.9...v2.0.10) (2021-10-08)


### Bug Fixes

* **readme.md:** restores readme ([2db1f36](https://github.com/rickstaa/panda-gazebo/commit/2db1f36fec8c989fcbfa3d45cdf74369381f88c9))

### [2.0.9](https://github.com/rickstaa/panda-gazebo/compare/v2.0.8...v2.0.9) (2021-10-08)

### [2.0.7](https://github.com/rickstaa/panda-gazebo/compare/v2.0.6...v2.0.7) (2021-10-07)

### [2.0.6](https://github.com/rickstaa/panda-gazebo/compare/v2.0.5...v2.0.6) (2021-10-07)

### [2.0.5](https://github.com/rickstaa/panda-gazebo/compare/v2.0.4...v2.0.5) (2021-10-06)

### [2.0.3](https://github.com/rickstaa/panda-gazebo/compare/v2.0.2...v2.0.3) (2021-10-04)

### [2.0.1](https://github.com/rickstaa/panda-gazebo/compare/v2.0.0...v2.0.1) (2021-09-27)

### [1.0.18](https://github.com/rickstaa/panda-gazebo/compare/v1.0.17...v1.0.18) (2021-09-20)

### [1.0.17](https://github.com/rickstaa/panda-gazebo/compare/v1.0.16...v1.0.17) (2021-09-13)

### [1.0.16](https://github.com/rickstaa/panda-gazebo/compare/v1.0.15...v1.0.16) (2021-09-13)

### [1.0.15](https://github.com/rickstaa/panda-gazebo/compare/v1.0.14...v1.0.15) (2021-09-13)

### [1.0.14](https://github.com/rickstaa/panda-gazebo/compare/v1.0.13...v1.0.14) (2021-09-13)

### [1.0.13](https://github.com/rickstaa/panda-gazebo/compare/v1.0.12...v1.0.13) (2021-09-13)

### [1.0.12](https://github.com/rickstaa/panda-gazebo/compare/v1.0.11...v1.0.12) (2021-09-13)

### [1.0.11](https://github.com/rickstaa/panda-gazebo/compare/v1.0.10...v1.0.11) (2021-09-13)

### [1.0.10](https://github.com/rickstaa/panda-gazebo/compare/v1.0.9...v1.0.10) (2021-09-13)

### [1.0.9](https://github.com/rickstaa/panda-gazebo/compare/v1.0.8...v1.0.9) (2021-09-13)

### [1.0.8](https://github.com/rickstaa/panda-gazebo/compare/v1.0.7...v1.0.8) (2021-09-13)

### [1.0.7](https://github.com/rickstaa/panda-gazebo/compare/v1.0.6...v1.0.7) (2021-09-13)

### [1.0.6](https://github.com/rickstaa/panda-gazebo/compare/v1.0.5...v1.0.6) (2021-09-13)

### [1.0.5](https://github.com/rickstaa/panda-gazebo/compare/v1.0.4...v1.0.5) (2021-09-13)

### [1.0.4](https://github.com/rickstaa/panda-gazebo/compare/v1.0.3...v1.0.4) (2021-09-13)

### [1.0.3](https://github.com/rickstaa/panda-gazebo/compare/v1.0.2...v1.0.3) (2021-09-13)

### [1.0.2](https://github.com/rickstaa/panda-gazebo/compare/v1.0.1...v1.0.2) (2021-09-13)

### [1.0.1](https://github.com/rickstaa/panda-gazebo/compare/v1.0.0...v1.0.1) (2021-09-13)
